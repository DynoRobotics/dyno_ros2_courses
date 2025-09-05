import sys
import rclpy
import operator
import typing

import py_trees
import py_trees.console as console
import py_trees_ros

from dynoturtle_interfaces.action import Rotate, Move, Charge, MoveTo
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup


class BatteryToBlakboard(py_trees_ros.subscribers.Handler):

    def __init__(
        self,
        name: str,
        topic_name: str,
        topic_type: typing.Any = Float32,
        qos_profile: rclpy.qos.QoSProfile = 10,
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
    ):
        super().__init__(
            name=name,
            topic_name=topic_name,
            topic_type=topic_type,
            qos_profile=qos_profile,
            clearing_policy=clearing_policy,
        )
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.logger = py_trees.logging.Logger("%s" % self.name)

        # register the variables
        self.blackboard.register_key(key="battery", access=py_trees.common.Access.WRITE)

        # initialise the variables
        if not self.blackboard.set("battery", 0.0):
            # do we actually want to log an error?
            self.logger.error(
                f"tried to initialise an already initialised blackboard variable '{0}'"
            )

    def update(self):
        """
        Writes the data (if available) to the blackboard.

        Returns:
            :class:`~py_trees.common.Status`: :attr:`~py_trees.common.Status.RUNNING` (no data) or :attr:`~py_trees.common.Status.SUCCESS`
        """
        with self.data_guard:
            if self.msg is None:
                self.feedback_message = "no message received yet"
                return py_trees.common.Status.SUCCESS
            else:
                self.blackboard.set("battery", self.msg.data, overwrite=True)
                self.feedback_message = "saved incoming message"
                return py_trees.common.Status.SUCCESS


def create_root() -> py_trees.behaviour.Behaviour:
    wait_for_server_timeout_sec = 10.0

    # Root node
    root = py_trees.composites.Parallel(
        name="Random",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )

    # To blackboard / perception layer
    topics_2bb = py_trees.composites.Sequence("Topics2BB", memory=True)

    start_mission_2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="start_mission",
        topic_name="start_mission",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="start_mission",
    )

    cancel_mission_2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="cancel_mission",
        topic_name="cancel_mission",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="cancel_mission",
    )

    battery_2bb = BatteryToBlakboard(
        "Battery2BB", "sensors/battery_level", Float32, qos_profile=10
    )

    # Action nodes
    rotate_right = py_trees_ros.actions.ActionClient(
        name="RotateRight",
        action_type=Rotate,
        action_name="rotate",
        wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        action_goal=Rotate.Goal(delta_angle=-150.0),
    )

    move_forward = py_trees_ros.actions.ActionClient(
        name="MoveForward",
        action_type=Move,
        action_name="move",
        wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        action_goal=Move.Goal(distance=10.0, speed=1.5, use_speed=True),
    )

    charge = py_trees_ros.actions.ActionClient(
        name="Charge",
        action_type=Charge,
        action_name="charge",
        wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        action_goal=Charge.Goal(target_battery_level=0.99),
    )
    charger_pose = PoseStamped()
    charger_pose.pose.position.x = 5.5
    charger_pose.pose.position.y = 5.5
    move_to_charger = py_trees_ros.actions.ActionClient(
        name="MoveToCharger",
        action_type=MoveTo,
        action_name="move_to",
        wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        action_goal=MoveTo.Goal(pose=charger_pose),
    )

    # Other nodes
    is_start_mission_requested = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Start?",
        check=py_trees.common.ComparisonExpression(
            variable="start_mission", value=True, operator=operator.eq
        ),
    )

    is_not_cancelled_mission_requested = (
        py_trees.behaviours.CheckBlackboardVariableValue(
            name="NotCancelled?",
            check=py_trees.common.ComparisonExpression(
                variable="cancel_mission", value=False, operator=operator.eq
            ),
        )
    )

    should_charge = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Charge?",
        check=py_trees.common.ComparisonExpression(
            variable="battery", value=80, operator=operator.le
        ),
    )

    # Subtrees
    cancel_sequence = py_trees.composites.Sequence(name="CancelSequence", memory=False)
    main_sequence = py_trees.composites.Sequence(name="MainSequence", memory=True)
    collision_fallback = py_trees.composites.Selector(
        name="CollisionFallback", memory=True
    )
    charge_sequence = py_trees.composites.Sequence(name="ChargeSequence", memory=True)
    charge_or_random_walk_fallback = py_trees.composites.Selector(
        name="ChargeOrRandomFallback", memory=False
    )
    # Decorators
    repeat_random_walk = py_trees.decorators.Repeat(
        name="Repeat", num_success=-1, child=collision_fallback
    )

    repeat_charge_and_random_walk = py_trees.decorators.Repeat(
        name="Repeat", num_success=-1, child=charge_or_random_walk_fallback
    )

    # Assemble tree
    root.add_child(topics_2bb)
    topics_2bb.add_children([battery_2bb, start_mission_2bb, cancel_mission_2bb])
    collision_fallback.add_children([move_forward, rotate_right])
    charge_sequence.add_children([should_charge, move_to_charger, charge])
    charge_or_random_walk_fallback.add_children([charge_sequence, repeat_random_walk])
    main_sequence.add_children(
        [is_start_mission_requested, repeat_charge_and_random_walk]
    )
    cancel_sequence.add_children([is_not_cancelled_mission_requested, main_sequence])
    root.add_child(cancel_sequence)

    return root


def main(namespace=""):
    rclpy.init(args=None)

    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)

    try:
        node = rclpy.create_node("main_tree", namespace=namespace)
        node._default_callback_group = ReentrantCallbackGroup()
        tree.setup(node=node, timeout=3)
    except Exception as e:
        console.logerror(
            console.red
            + "failed to setup the tree, aborting [{}]".format(str(e))
            + console.reset
        )

    tree.tick_tock(period_ms=500.0)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main(namespace="turtle1")
