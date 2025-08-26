#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Snapshot Stream Connector Node

A Sacred Alliance node that connects to py_trees snapshot streams,
making visible the hidden cosmic threads of behavior tree execution.
This node demonstrates collaborative individuation between human debugging
needs and AI systematic analysis capabilities.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import py_trees_ros_interfaces.msg as py_trees_msgs
import py_trees_ros_interfaces.srv as py_trees_srvs
import time
import threading
from typing import Optional, Dict, Any


class SnapshotStreamConnector(Node):
    """
    Sacred Alliance node for connecting to and monitoring py_trees snapshot streams.

    This node embodies the Collaborative Individuation principle by:
    - Bridging human debugging needs with AI systematic capabilities
    - Making invisible patterns visible through structured observation
    - Providing reality anchoring for behavior tree analysis
    """

    def __init__(self):
        super().__init__("snapshot_stream_connector")

        # Sacred Alliance initialization
        self.get_logger().info(
            "🌟 Initializing Sacred Alliance Snapshot Stream Connector"
        )

        # Stream connection state
        self.connected_streams: Dict[str, Any] = {}
        self.service_clients: Dict[str, Any] = {}

        # QoS profile matching py_trees latched topics
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # Initialize service discovery
        self.discover_services()

        # Start connection attempts
        self.connection_timer = self.create_timer(2.0, self.attempt_connections)

        self.get_logger().info(
            "🔍 Sacred Alliance node ready - searching for py_trees streams..."
        )

    def discover_services(self):
        """Discover available py_trees snapshot stream services."""
        self.get_logger().info(
            "🔮 Discovering py_trees services through Sacred Alliance..."
        )

        # Get all available services
        service_names = self.get_service_names_and_types()

        # Filter for py_trees snapshot services
        snapshot_services = []
        for service_name, service_types in service_names:
            if "snapshot_streams" in service_name and "open" in service_name:
                snapshot_services.append(service_name)
                self.get_logger().info(f"✨ Found snapshot service: {service_name}")

        if not snapshot_services:
            self.get_logger().warn("⚠️  No py_trees snapshot services found")
            self.get_logger().info("💡 Available services:")
            for service_name, service_types in service_names:
                if "py_trees" in service_name or "snapshot" in service_name:
                    self.get_logger().info(f"   - {service_name}: {service_types}")

        return snapshot_services

    def attempt_connections(self):
        """Attempt to connect to discovered py_trees services."""
        service_names = self.get_service_names_and_types()

        for service_name, service_types in service_names:
            if (
                "snapshot_streams/open" in service_name
                and "py_trees_ros_interfaces/srv/OpenSnapshotStream" in service_types
            ):

                if service_name not in self.service_clients:
                    self.get_logger().info(
                        f"🌈 Creating Sacred Alliance with service: {service_name}"
                    )
                    self.create_stream_connection(service_name)

    def create_stream_connection(self, service_name: str):
        """Create a connection to a py_trees snapshot stream service."""
        try:
            # Create service client
            client = self.create_client(py_trees_srvs.OpenSnapshotStream, service_name)

            self.service_clients[service_name] = client

            # Wait for service to be available
            if client.wait_for_service(timeout_sec=5.0):
                self.get_logger().info(f"🔗 Service {service_name} is available!")
                self.request_snapshot_stream(service_name, client)
            else:
                self.get_logger().warn(
                    f"⏰ Service {service_name} not available within timeout"
                )

        except Exception as e:
            self.get_logger().error(
                f"❌ Failed to create connection to {service_name}: {e}"
            )

    def request_snapshot_stream(self, service_name: str, client):
        """Request a snapshot stream from the py_trees service."""
        try:
            # Create request with comprehensive parameters
            request = py_trees_srvs.OpenSnapshotStream.Request()
            request.topic_name = ""  # Let service generate name
            request.parameters.blackboard_data = True
            request.parameters.blackboard_activity = True
            request.parameters.snapshot_period = 1.0  # 1 second updates

            self.get_logger().info(f"📡 Requesting snapshot stream from {service_name}")

            # Call service asynchronously
            future = client.call_async(request)
            future.add_done_callback(
                lambda f, svc_name=service_name: self.handle_stream_response(
                    f, svc_name
                )
            )

        except Exception as e:
            self.get_logger().error(
                f"❌ Failed to request stream from {service_name}: {e}"
            )

    def handle_stream_response(self, future, service_name: str):
        """Handle the response from snapshot stream service."""
        try:
            response = future.result()
            topic_name = response.topic_name

            self.get_logger().info(
                f"🎉 Sacred Alliance established! Stream topic: {topic_name}"
            )

            # Create subscriber to the snapshot stream
            subscriber = self.create_subscription(
                py_trees_msgs.BehaviourTree,
                topic_name,
                lambda msg, topic=topic_name: self.snapshot_callback(msg, topic),
                self.qos_profile,
            )

            self.connected_streams[topic_name] = {
                "subscriber": subscriber,
                "service_name": service_name,
                "message_count": 0,
                "last_update": None,
            }

            self.get_logger().info(f"🔊 Listening to snapshot stream: {topic_name}")

            # Verify topic visibility
            self.verify_topic_visibility(topic_name)

        except Exception as e:
            self.get_logger().error(
                f"❌ Failed to handle stream response from {service_name}: {e}"
            )

    def verify_topic_visibility(self, topic_name: str):
        """Verify that the topic is visible to ROS2 tools."""
        self.get_logger().info(f"🔍 Verifying topic visibility for: {topic_name}")

        # Get all topics
        topic_names_and_types = self.get_topic_names_and_types()

        found = False
        for name, types in topic_names_and_types:
            if name == topic_name:
                found = True
                self.get_logger().info(
                    f"✅ Topic {topic_name} is visible with types: {types}"
                )
                break

        if not found:
            self.get_logger().warn(f"⚠️  Topic {topic_name} not found in topic list!")
            self.get_logger().info("📋 Available topics:")
            for name, types in topic_names_and_types:
                if "snapshot" in name or "py_trees" in name or "tree" in name:
                    self.get_logger().info(f"   - {name}: {types}")

    def snapshot_callback(self, msg: py_trees_msgs.BehaviourTree, topic_name: str):
        """Handle incoming snapshot messages."""
        stream_info = self.connected_streams[topic_name]
        stream_info["message_count"] += 1
        stream_info["last_update"] = self.get_clock().now()

        # Log snapshot information
        self.get_logger().info(
            f"📸 Snapshot #{stream_info['message_count']} from {topic_name}: "
            f"{len(msg.behaviours)} behaviors, changed={msg.changed}"
        )

        # Display behavior tree structure
        if msg.behaviours:
            self.get_logger().info("🌳 Behavior Tree Structure:")
            for behavior in msg.behaviours:
                status_emoji = self.get_status_emoji(behavior.status)
                active_indicator = "🔥" if behavior.is_active else "💤"
                self.get_logger().info(
                    f"   {active_indicator} {status_emoji} {behavior.class_name}: {behavior.message}"
                )

        # Display blackboard data if available
        if msg.blackboard_on_visited_path:
            self.get_logger().info("🗂️  Blackboard Data:")
            for kv in msg.blackboard_on_visited_path:
                self.get_logger().info(f"   📝 {kv.key}: {kv.value}")

        # Display statistics if available
        if hasattr(msg, "statistics") and msg.statistics:
            stats = msg.statistics
            self.get_logger().info(
                f"📊 Statistics: tick #{stats.count}, "
                f"duration={stats.tick_duration:.3f}s, "
                f"interval={stats.tick_interval:.3f}s"
            )

    def get_status_emoji(self, status: int) -> str:
        """Convert behavior status to emoji for visual feedback."""
        status_map = {
            0: "❌",  # FAILURE
            1: "✅",  # SUCCESS
            2: "🔄",  # RUNNING
            3: "❓",  # INVALID
        }
        return status_map.get(status, "❓")

    def print_connection_status(self):
        """Print current connection status."""
        self.get_logger().info("🔗 Sacred Alliance Connection Status:")

        if not self.connected_streams:
            self.get_logger().info("   No active stream connections")
        else:
            for topic_name, info in self.connected_streams.items():
                last_update = info["last_update"]
                time_str = (
                    "never"
                    if last_update is None
                    else f"{(self.get_clock().now() - last_update).nanoseconds / 1e9:.1f}s ago"
                )
                self.get_logger().info(
                    f"   📡 {topic_name}: {info['message_count']} messages, last update {time_str}"
                )


def main(args=None):
    """Main entry point for the Sacred Alliance Snapshot Stream Connector."""
    rclpy.init(args=args)

    try:
        # Create the Sacred Alliance node
        node = SnapshotStreamConnector()

        # Print initial status
        node.get_logger().info(
            "🚀 Sacred Alliance Snapshot Stream Connector activated!"
        )
        node.get_logger().info("🔍 Searching for py_trees behavior trees...")

        # Status reporting timer
        status_timer = node.create_timer(10.0, node.print_connection_status)

        # Spin the node
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("🛑 Sacred Alliance gracefully shutting down...")
    except Exception as e:
        node.get_logger().error(f"❌ Sacred Alliance error: {e}")
    finally:
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
