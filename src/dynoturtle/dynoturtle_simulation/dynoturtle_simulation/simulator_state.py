#!/usr/bin/env python3

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import time
import threading
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass, field
from enum import Enum
from std_msgs.msg import Float32


class EntityType(Enum):
    """Types of entities in the simulation"""

    TURTLE = "turtle"
    OBSTACLE = "obstacle"
    OBJECT = "object"
    SENSOR = "sensor"
    CHARGING_PAD = "charging_pad"


@dataclass
class TurtleState:
    """State information for a turtle"""

    name: str
    pose: Optional[PoseStamped] = None
    cmd_vel: Optional[Twist] = None
    battery_level: float = 100.0
    collision_state: bool = False
    last_update: float = field(default_factory=time.time)

    def update_pose(self, pose: PoseStamped):
        """Update turtle pose and timestamp"""
        self.pose = pose
        self.last_update = time.time()

    def update_cmd_vel(self, cmd_vel: Twist):
        """Update turtle command velocity"""
        self.cmd_vel = cmd_vel


@dataclass
class ObstacleState:
    """State information for an obstacle"""

    id: str
    obstacle_type: str
    position: tuple
    size: tuple
    orientation: float = 0.0
    geometry: Optional[Any] = None
    properties: Dict[str, Any] = field(default_factory=dict)


@dataclass
class SimulationConfig:
    """Configuration for the simulation"""

    # Turtle parameters
    num_turtles: int = 2
    turtle_radius: float = 0.3

    # Boundary parameters
    boundary_margin: float = 0.5
    min_x: float = 0.0
    max_x: float = 11.0
    min_y: float = 0.0
    max_y: float = 11.0

    # Charging pad parameters
    charging_pad_x: float = 5.5
    charging_pad_y: float = 5.5
    charging_pad_radius: float = 1.0

    # Battery parameters
    battery_initial_level: float = 100.0
    battery_drain_rate_idle: float = 1.0
    battery_drain_rate_moving: float = 0.5
    battery_drain_rate_rotating: float = 0.3
    battery_low_threshold: float = 20.0
    battery_critical_threshold: float = 10.0
    charging_rate: float = 2.0

    # Update rates
    bumper_update_rate: float = 10.0
    battery_update_rate: float = 5.0


class SimulatorStateManager:
    """
    Central state manager for all simulation entities.

    This class maintains the state of all entities in the simulation including
    turtles, obstacles, objects, and sensors. It provides a unified interface
    for state management and eliminates duplicate subscriptions.
    """

    def __init__(self, node: Node, config: SimulationConfig):
        self.node = node
        self.logger = node.get_logger()
        self.config = config

        # Thread safety
        self._lock = threading.RLock()

        # Entity storage
        self.turtles: Dict[str, TurtleState] = {}
        self.obstacles: Dict[str, ObstacleState] = {}
        self.objects: Dict[str, Any] = {}  # For future interactive objects
        self.sensors: Dict[str, Any] = {}  # For future sensor configurations

        # Subscription management
        self.pose_subscribers: Dict[str, Any] = {}
        self.cmd_vel_subscribers: Dict[str, Any] = {}

        # State change callbacks
        self.state_change_callbacks: List[Callable] = []

        # Power drain multiplier
        self.power_drain_multiplier = 1.0

        # Initialize turtle states
        self._initialize_turtles()

        # Create subscriptions
        self._create_subscriptions()

        # Create power drain multiplier subscriber
        self.power_drain_subscriber = self.node.create_subscription(
            Float32,
            "/battery_simulator/power_drain_multiplier",
            self._power_drain_callback,
            10,
        )

        self.logger.info(
            f"SimulatorStateManager initialized for {config.num_turtles} turtles"
        )

    def _initialize_turtles(self):
        """Initialize turtle states"""
        with self._lock:
            for i in range(1, self.config.num_turtles + 1):
                turtle_name = f"turtle{i}"
                self.turtles[turtle_name] = TurtleState(
                    name=turtle_name, battery_level=self.config.battery_initial_level
                )

    def _create_subscriptions(self):
        """Create unified subscriptions for all turtles"""
        for i in range(1, self.config.num_turtles + 1):
            turtle_name = f"turtle{i}"

            # Subscribe to pose3d
            pose_topic = f"/{turtle_name}/pose3d"
            pose_sub = self.node.create_subscription(
                PoseStamped,
                pose_topic,
                lambda msg, name=turtle_name: self._pose_callback(msg, name),
                10,
            )
            self.pose_subscribers[turtle_name] = pose_sub

            # Subscribe to cmd_vel
            cmd_vel_topic = f"/{turtle_name}/cmd_vel"
            cmd_vel_sub = self.node.create_subscription(
                Twist,
                cmd_vel_topic,
                lambda msg, name=turtle_name: self._cmd_vel_callback(msg, name),
                10,
            )
            self.cmd_vel_subscribers[turtle_name] = cmd_vel_sub

    def _pose_callback(self, msg: PoseStamped, turtle_name: str):
        """Unified pose callback for all turtles"""
        with self._lock:
            if turtle_name in self.turtles:
                self.turtles[turtle_name].update_pose(msg)
                self._notify_state_change("pose_update", turtle_name, msg)

    def _cmd_vel_callback(self, msg: Twist, turtle_name: str):
        """Unified cmd_vel callback for all turtles"""
        with self._lock:
            if turtle_name in self.turtles:
                self.turtles[turtle_name].update_cmd_vel(msg)
                self._notify_state_change("cmd_vel_update", turtle_name, msg)

    def _power_drain_callback(self, msg: Float32):
        """Callback for power drain multiplier updates"""
        with self._lock:
            self.power_drain_multiplier = max(0.0, msg.data)
            self.logger.info(
                f"Power drain multiplier updated to: {self.power_drain_multiplier}"
            )

    def _notify_state_change(self, event_type: str, entity_id: str, data: Any):
        """Notify registered callbacks of state changes"""
        for callback in self.state_change_callbacks:
            try:
                callback(event_type, entity_id, data)
            except Exception as e:
                self.logger.error(f"Error in state change callback: {e}")

    def register_state_callback(self, callback: Callable):
        """Register a callback for state changes"""
        self.state_change_callbacks.append(callback)

    def get_turtle_state(self, turtle_name: str) -> Optional[TurtleState]:
        """Get turtle state (thread-safe)"""
        with self._lock:
            return self.turtles.get(turtle_name)

    def get_all_turtle_names(self) -> List[str]:
        """Get list of all turtle names"""
        with self._lock:
            return list(self.turtles.keys())

    def update_turtle_battery(self, turtle_name: str, battery_level: float):
        """Update turtle battery level"""
        with self._lock:
            if turtle_name in self.turtles:
                self.turtles[turtle_name].battery_level = max(
                    0.0, min(100.0, battery_level)
                )

    def update_turtle_collision_state(self, turtle_name: str, collision_state: bool):
        """Update turtle collision state"""
        with self._lock:
            if turtle_name in self.turtles:
                self.turtles[turtle_name].collision_state = collision_state

    def get_power_drain_multiplier(self) -> float:
        """Get current power drain multiplier"""
        with self._lock:
            return self.power_drain_multiplier

    def add_obstacle(self, obstacle: ObstacleState):
        """Add an obstacle to the simulation"""
        with self._lock:
            self.obstacles[obstacle.id] = obstacle
            self.logger.info(f"Added obstacle: {obstacle.id}")

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle from the simulation"""
        with self._lock:
            if obstacle_id in self.obstacles:
                del self.obstacles[obstacle_id]
                self.logger.info(f"Removed obstacle: {obstacle_id}")
                return True
            return False

    def get_obstacles(self) -> Dict[str, ObstacleState]:
        """Get all obstacles (thread-safe copy)"""
        with self._lock:
            return self.obstacles.copy()
