import sys
import rclpy

import py_trees
import py_trees.console as console
import py_trees_ros

from my_first_interfaces.action import Rotate


def create_root() -> py_trees.behaviour.Behaviour:
    wait_for_server_timeout_sec = 10.0

    # Root node
    root = py_trees.composites.Parallel(
        name="SimpleDelivery",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )

    # To blackboard / perception layer
    topics_2bb = py_trees.composites.Sequence("Topics2BB", memory=True)

    start_mission_2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="start_mission",
        topic_name="/start_mission",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="start_mission",
    )

    cancel_mission_2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="cancel_mission",
        topic_name="/cancel_mission",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name="cancel_mission",
    )

    # Action nodes
    rotate_right = py_trees_ros.actions.ActionClient(
        name="RotateRight",
        action_type=Rotate,
        action_name="rotate",
        wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        action_goal=Rotate.Goal(delta_angle=-90.0),
    )

    rotate_left = py_trees_ros.actions.ActionClient(
        name="RotateLeft",
        action_type=Rotate,
        action_name="rotate",
        wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        action_goal=Rotate.Goal(delta_angle=90.0),
    )

    # Subtrees
    main_sequence = py_trees.composites.Sequence(name="MainSequence", memory=True)

    # Assemble tree
    root.add_child(topics_2bb)
    topics_2bb.add_children([start_mission_2bb, cancel_mission_2bb])

    main_sequence.add_children([rotate_left, rotate_right])

    root.add_child(main_sequence)

    return root


def main(namespace=""):
    rclpy.init(args=None)

    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)

    try:
        node = rclpy.create_node("simple_behavior", namespace=namespace)
        tree.setup(node=node, timeout=3)
    except Exception as e:
        console.logerror(
            console.red
            + "failed to setup the tree, aborting [{}]".format(str(e))
            + console.reset
        )
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main(namespace="turtle1")
