# Module 4: Robot Modeling and Visualization for Underwater Systems

**Duration**: 60 minutes  
**Prerequisites**: Modules 1, 2 & 3 completed  
**Learning Objectives**: Model underwater robots in URDF, understand coordinate frames, visualize robot state, control robot joints

---

## Learning Objectives

By the end of this module, participants will be able to:

1. **Create URDF models** for complex underwater robots with manipulators
2. **Understand TF2 coordinate transforms** for marine robotics applications
3. **Visualize robot state** using Rviz2 for underwater environments
4. **Implement ros2_control** for joint control and hardware interfaces

---

## Section 1: URDF Modeling for Underwater Robots (20 minutes)

### URDF Overview for Marine Robotics

URDF (Unified Robot Description Format) describes the physical structure of robots, including links, joints, sensors, and visual/collision geometry.

#### Key Components for Underwater Robots:

- **Hull geometry** with hydrodynamic properties
- **Thruster placement** and orientations
- **Manipulator arm** with 6 degrees of freedom
- **Sensor mounting** (cameras, sonar, IMU)
- **Buoyancy and mass properties**

### Underwater Robot URDF Structure

#### 1. Base Vehicle URDF

```xml
<!-- File: auv_description/urdf/auv_vehicle.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="auv_vehicle">

  <!-- Vehicle Parameters -->
  <xacro:property name="vehicle_length" value="2.0" />
  <xacro:property name="vehicle_width" value="0.6" />
  <xacro:property name="vehicle_height" value="0.4" />
  <xacro:property name="vehicle_mass" value="45.0" />

  <!-- Base Link (Vehicle Hull) -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${vehicle_length} ${vehicle_width} ${vehicle_height}"/>
      </geometry>
      <material name="vehicle_blue">
        <color rgba="0.0 0.3 0.8 1.0"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${vehicle_length} ${vehicle_width} ${vehicle_height}"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.02" rpy="0 0 0"/>
      <mass value="${vehicle_mass}"/>
      <inertia ixx="1.2" ixy="0.0" ixz="0.0"
               iyy="4.5" iyz="0.0"
               izz="4.5"/>
    </inertial>
  </link>

  <!-- Thruster Definitions -->
  <xacro:macro name="thruster" params="name x y z roll pitch yaw">
    <link name="${name}_link">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.05" length="0.15"/>
        </geometry>
        <material name="thruster_black">
          <color rgba="0.2 0.2 0.2 1.0"/>
        </material>
      </visual>

      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.05" length="0.15"/>
        </geometry>
      </collision>

      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="2.0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0"
                 iyy="0.01" iyz="0.0"
                 izz="0.005"/>
      </inertial>
    </link>

    <joint name="${name}_joint" type="fixed">
      <parent link="base_link"/>
      <child link="${name}_link"/>
      <origin xyz="${x} ${y} ${z}" rpy="${roll} ${pitch} ${yaw}"/>
    </joint>
  </xacro:macro>

  <!-- Instantiate Thrusters -->
  <xacro:thruster name="thruster_front_right" x="0.8" y="0.25" z="0.0"
                  roll="0" pitch="0" yaw="0"/>
  <xacro:thruster name="thruster_front_left" x="0.8" y="-0.25" z="0.0"
                  roll="0" pitch="0" yaw="0"/>
  <xacro:thruster name="thruster_rear_right" x="-0.8" y="0.25" z="0.0"
                  roll="0" pitch="0" yaw="3.14159"/>
  <xacro:thruster name="thruster_rear_left" x="-0.8" y="-0.25" z="0.0"
                  roll="0" pitch="0" yaw="3.14159"/>
  <xacro:thruster name="thruster_vertical_top" x="0.0" y="0.0" z="0.25"
                  roll="0" pitch="1.5708" yaw="0"/>
  <xacro:thruster name="thruster_vertical_bottom" x="0.0" y="0.0" z="-0.25"
                  roll="0" pitch="-1.5708" yaw="0"/>

</robot>
```

#### 2. Manipulator Arm URDF

```xml
<!-- File: auv_description/urdf/manipulator_arm.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="manipulator_arm">

  <!-- Arm Parameters -->
  <xacro:property name="joint_damping" value="0.1" />
  <xacro:property name="joint_friction" value="0.1" />
  <xacro:property name="joint_effort_limit" value="100.0" />
  <xacro:property name="joint_velocity_limit" value="2.0" />

  <!-- Arm Base Link -->
  <link name="arm_base_link">
    <visual>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.08" length="0.15"/>
      </geometry>
      <material name="arm_gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.08" length="0.15"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
      <mass value="3.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0"
               iyy="0.02" iyz="0.0"
               izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint 1: Base Rotation -->
  <joint name="arm_joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_base_link"/>
    <origin xyz="0.6 0.0 0.0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159"
           effort="${joint_effort_limit}"
           velocity="${joint_velocity_limit}"/>
    <dynamics damping="${joint_damping}" friction="${joint_friction}"/>
  </joint>

  <!-- Link 1: Shoulder -->
  <link name="arm_link_1">
    <visual>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.25"/>
      </geometry>
      <material name="arm_gray"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.25"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <mass value="2.5"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0"
               iyy="0.015" iyz="0.0"
               izz="0.008"/>
    </inertial>
  </link>

  <!-- Joint 2: Shoulder Pitch -->
  <joint name="arm_joint_2" type="revolute">
    <parent link="arm_base_link"/>
    <child link="arm_link_1"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708"
           effort="${joint_effort_limit}"
           velocity="${joint_velocity_limit}"/>
    <dynamics damping="${joint_damping}" friction="${joint_friction}"/>
  </joint>

  <!-- Link 2: Upper Arm -->
  <link name="arm_link_2">
    <visual>
      <origin xyz="0.125 0 0" rpy="0 1.5708 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <material name="arm_gray"/>
    </visual>

    <collision>
      <origin xyz="0.125 0 0" rpy="0 1.5708 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0.125 0 0" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0"
               iyy="0.012" iyz="0.0"
               izz="0.012"/>
    </inertial>
  </link>

  <!-- Joint 3: Elbow Pitch -->
  <joint name="arm_joint_3" type="revolute">
    <parent link="arm_link_1"/>
    <child link="arm_link_2"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0944" upper="2.0944"
           effort="${joint_effort_limit}"
           velocity="${joint_velocity_limit}"/>
    <dynamics damping="${joint_damping}" friction="${joint_friction}"/>
  </joint>

  <!-- Continue with joints 4, 5, 6 and gripper... -->
  <!-- (Additional joints follow similar pattern) -->

</robot>
```

#### 3. Sensor Suite URDF

```xml
<!-- File: auv_description/urdf/sensor_suite.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="sensor_suite">

  <!-- Stereo Camera -->
  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.15 0.05"/>
      </geometry>
      <material name="camera_black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.15 0.05"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="1.0 0.0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Forward-Looking Sonar -->
  <link name="sonar_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.12"/>
      </geometry>
      <material name="sonar_yellow">
        <color rgba="1.0 1.0 0.0 1.0"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.12"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0"
               iyy="0.002" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <joint name="sonar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sonar_link"/>
    <origin xyz="0.9 0.0 -0.1" rpy="0 0 0"/>
  </joint>

  <!-- IMU -->
  <link name="imu_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.02"/>
      </geometry>
      <material name="imu_green">
        <color rgba="0.0 1.0 0.0 1.0"/>
      </material>
    </visual>

    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
               iyy="0.0001" iyz="0.0"
               izz="0.0001"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
  </joint>

</robot>
```

### Hands-on: Create URDF Model

#### Step 1: Create Basic Vehicle URDF

```bash
cd /workspace/src/auv_description/urdf

cat > simple_auv.urdf.xacro << 'EOF'
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_auv">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="2.0 0.6 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.3 0.8 1.0"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="2.0 0.6 0.4"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="45.0"/>
      <inertia ixx="1.2" ixy="0.0" ixz="0.0"
               iyy="4.5" iyz="0.0"
               izz="4.5"/>
    </inertial>
  </link>

</robot>
EOF
```

#### Step 2: Test URDF Syntax

```bash
# Check URDF syntax
check_urdf simple_auv.urdf.xacro

# Convert xacro to URDF
xacro simple_auv.urdf.xacro > simple_auv.urdf
```

---

## Section 2: TF2 Coordinate Transforms for Marine Robotics (15 minutes)

### TF2 Overview for Underwater Systems

TF2 manages coordinate frame relationships, essential for underwater robots operating in 3D space with multiple sensors and manipulators.

#### Key Coordinate Frames for Underwater Robots:

- **world**: Global reference frame (often surface GPS coordinates)
- **map**: Local navigation frame (mission area)
- **odom**: Odometry frame (dead reckoning)
- **base_link**: Robot center
- **sensors**: Camera, sonar, IMU frames
- **manipulator**: Arm joint frames

### TF2 Tree Structure

```
world
└── map
    └── odom
        └── base_link
            ├── camera_link
            ├── sonar_link
            ├── imu_link
            └── arm_base_link
                └── arm_link_1
                    └── arm_link_2
                        └── ... (arm joints)
                            └── gripper_link
```

### TF2 Publisher Implementation

```python
# File: auv_localization/auv_localization/tf_publisher.py
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import math

class AUVTFPublisher:
    def __init__(self, node: Node):
        self.node = node

        # TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)

        # Subscribe to odometry
        self.odom_sub = self.node.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10
        )

        # Timer for static transforms
        self.static_timer = self.node.create_timer(1.0, self.publish_static_transforms)

    def odom_callback(self, msg):
        """Publish dynamic transforms from odometry."""
        # Publish map -> odom transform
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        # For simplicity, assume map and odom are aligned
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

        # Publish odom -> base_link transform
        t2 = TransformStamped()
        t2.header.stamp = msg.header.stamp
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_link'

        t2.transform.translation = msg.pose.pose.position
        t2.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t2)

    def publish_static_transforms(self):
        """Publish static sensor transforms."""
        transforms = []

        # Camera transform
        t_camera = TransformStamped()
        t_camera.header.stamp = self.node.get_clock().now().to_msg()
        t_camera.header.frame_id = 'base_link'
        t_camera.child_frame_id = 'camera_link'
        t_camera.transform.translation.x = 1.0
        t_camera.transform.translation.y = 0.0
        t_camera.transform.translation.z = 0.1
        t_camera.transform.rotation.w = 1.0
        transforms.append(t_camera)

        # Sonar transform
        t_sonar = TransformStamped()
        t_sonar.header.stamp = self.node.get_clock().now().to_msg()
        t_sonar.header.frame_id = 'base_link'
        t_sonar.child_frame_id = 'sonar_link'
        t_sonar.transform.translation.x = 0.9
        t_sonar.transform.translation.y = 0.0
        t_sonar.transform.translation.z = -0.1
        t_sonar.transform.rotation.w = 1.0
        transforms.append(t_sonar)

        # IMU transform
        t_imu = TransformStamped()
        t_imu.header.stamp = self.node.get_clock().now().to_msg()
        t_imu.header.frame_id = 'base_link'
        t_imu.child_frame_id = 'imu_link'
        t_imu.transform.translation.x = 0.0
        t_imu.transform.translation.y = 0.0
        t_imu.transform.translation.z = 0.0
        t_imu.transform.rotation.w = 1.0
        transforms.append(t_imu)

        # Broadcast all static transforms
        for transform in transforms:
            self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('auv_tf_publisher')
    tf_publisher = AUVTFPublisher(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Hands-on: TF2 Visualization

#### Step 1: Create TF Publisher Node

```bash
cd /workspace/src/auv_localization/auv_localization

# Create basic TF publisher
cat > tf_publisher.py << 'EOF'
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math

class SimpleTFPublisher:
    def __init__(self, node: Node):
        self.node = node
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)
        self.timer = self.node.create_timer(0.1, self.publish_transforms)

    def publish_transforms(self):
        # Publish base_link transform
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = -10.0  # 10m underwater
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = rclpy.create_node('simple_tf_publisher')
    tf_pub = SimpleTFPublisher(node)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
EOF
```

#### Step 2: Test TF Tree

```bash
# Run TF publisher
ros2 run auv_localization tf_publisher

# View TF tree (in another terminal)
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo world base_link
```

---

## Section 3: Rviz2 Visualization for Underwater Environments (15 minutes)

### Rviz2 Configuration for Marine Robotics

Rviz2 provides 3D visualization essential for understanding underwater robot behavior and debugging.

#### Key Visualization Elements:

- **Robot Model**: URDF visualization
- **TF Frames**: Coordinate frame relationships
- **Sensor Data**: Camera images, sonar scans
- **Navigation**: Paths, waypoints, obstacles
- **Environment**: Seafloor, obstacles, mission area

### Rviz2 Configuration File

```yaml
# File: auv_description/config/auv_rviz.rviz
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /RobotModel1
        - /TF1
      Splitter Ratio: 0.5
    Tree Height: 557
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Pose Estimate1
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Preferences:
  PromptSaveOnExit: true
Toolbars:
  toolButtonStyle: 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 100
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Mass Properties:
        Inertia: false
        Mass: false
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Alpha: 1
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree: {}
      Update Interval: 0
      Value: true
  Enabled: true
  Global Options:
    Background Color: 0; 43; 87
    Fixed Frame: world
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.4853981137275696
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz_default_plugins)
      Yaw: 0.7853981852531433
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002b0fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006
```
