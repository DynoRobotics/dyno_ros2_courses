import sys
import rclpy
import operator

import py_trees
import py_trees.console as console
import py_trees_ros

from dynoturtle_interfaces.action import Rotate, Move


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

    # Other nodes
    is_start_mission_requested = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Start?",
        check=py_trees.common.ComparisonExpression(
            variable="start_mission", value=True, operator=operator.eq
        ),
    )

    is_cancel_mission_requested = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Cancel?",
        check=py_trees.common.ComparisonExpression(
            variable="cancel_mission", value=False, operator=operator.eq
        ),
    )

    # Subtrees
    cancel_sequence = py_trees.composites.Sequence(name="CancelSequence", memory=False)
    main_sequence = py_trees.composites.Sequence(name="MainSequence", memory=True)
    collision_fallback = py_trees.composites.Selector(
        name="CollisionFallback", memory=True
    )

    # Decorators
    repeat_random_walk = py_trees.decorators.Repeat(
        name="Repeat", num_success=-1, child=collision_fallback
    )

    # Assemble tree
    root.add_child(topics_2bb)
    topics_2bb.add_children([start_mission_2bb, cancel_mission_2bb])
    collision_fallback.add_children([move_forward, rotate_right])
    main_sequence.add_children([is_start_mission_requested, repeat_random_walk])
    cancel_sequence.add_children([is_cancel_mission_requested, main_sequence])
    root.add_child(cancel_sequence)

    return root


def main(namespace=""):
    rclpy.init(args=None)

    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)

    try:
        node = rclpy.create_node("main_tree", namespace=namespace)
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
