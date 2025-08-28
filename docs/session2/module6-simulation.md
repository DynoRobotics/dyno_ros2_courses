# Module 6: Underwater Robot Simulation

**Duration**: 60 minutes  
**Prerequisites**: Modules 1-5 completed  
**Learning Objectives**: Simulate underwater robots with manipulator arms using Gazebo and ROS2

---

## Learning Objectives

By the end of this module, participants will be able to:

1. **Set up underwater simulation environments** in Gazebo
2. **Create underwater robot models** with manipulator arms
3. **Control robot movement and arm joints** with teleop
4. **Simulate underwater sensors** (cameras, pressure, IMU)
5. **Understand underwater robotics challenges** in simulation

---

## Section 1: Underwater Simulation Setup (15 minutes)

### Why Underwater Simulation?

Underwater robotics presents unique challenges:

- **Buoyancy forces** - Robots float or sink based on density
- **Fluid dynamics** - Water resistance affects movement
- **Limited visibility** - Cameras have reduced range
- **Pressure effects** - Depth affects sensor readings
- **Communication constraints** - Limited bandwidth underwater

### Gazebo Underwater Physics

Gazebo can simulate underwater conditions using:

- **Buoyancy plugin** - Simulates floating/sinking forces
- **Fluid dynamics** - Water resistance and currents
- **Underwater lighting** - Reduced visibility effects
- **Pressure simulation** - Depth-dependent sensor behavior

### Setting Up Underwater World

#### Step 1: Create Underwater World Package

```bash
cd /workspace/src

# Create underwater simulation package
ros2 pkg create --build-type ament_cmake underwater_robot_simulation

# Create directory structure
cd underwater_robot_simulation
mkdir -p urdf launch worlds config meshes
```

#### Step 2: Create Underwater World File

```xml
<!-- File: underwater_robot_simulation/worlds/underwater.world -->
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="underwater_world">

    <!-- Physics with underwater properties -->
    <physics name="default_physics" default="0" type="ode">
      <gravity>0 0 -9.81</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.400000</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
          <contact_surface_layer>0.01000</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>

    <!-- Underwater lighting -->
    <scene>
      <ambient>0.2 0.4 0.6 1</ambient>
      <background>0.1 0.3 0.5 1</background>
      <shadows>1</shadows>
    </scene>

    <!-- Sun (filtered through water) -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ocean floor -->
    <model name="ocean_floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Sand</name>
            </script>
          </material>
        </visual>
      </link>
      <pose>0 0 -10 0 0 0</pose>
    </model>

    <!-- Underwater obstacles/structures -->
    <model name="pipe_section">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>4.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>4.0</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
      <pose>3 0 -5 0 1.57 0</pose>
    </model>

    <!-- Water volume for buoyancy -->
    <model name="water">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>100 100 20</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Blue</name>
            </script>
          </material>
          <transparency>0.7</transparency>
        </visual>
      </link>
      <pose>0 0 0 0 0 0</pose>
    </model>

  </world>
</sdf>
```

### Hands-on: Launch Underwater World

#### Exercise 1: Test Underwater Environment

```bash
# Launch underwater world
gazebo --verbose /workspace/src/underwater_robot_simulation/worlds/underwater.world
```

**Observe:**

- Blue-tinted underwater lighting
- Sandy ocean floor
- Pipe section (simulating underwater infrastructure)
- Semi-transparent water volume

---

## Section 2: Underwater Robot with Manipulator (25 minutes)

### AUV (Autonomous Underwater Vehicle) Design

Our robot will feature:

- **Torpedo-shaped hull** for hydrodynamic efficiency
- **Thrusters** for 6-DOF movement (surge, sway, heave, roll, pitch, yaw)
- **Manipulator arm** with multiple joints
- **Sensors** (camera, pressure, IMU)
- **Buoyancy control** for depth management

### Create AUV URDF

```xml
<!-- File: underwater_robot_simulation/urdf/auv_with_arm.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="auv_with_arm">

  <!-- Properties -->
  <xacro:property name="hull_length" value="2.0"/>
  <xacro:property name="hull_radius" value="0.3"/>
  <xacro:property name="hull_mass" value="50.0"/>

  <!-- Main hull -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="${hull_radius}" length="${hull_length}"/>
      </geometry>
      <material name="yellow">
        <color rgba="1.0 1.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${hull_radius}" length="${hull_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${hull_mass}"/>
      <inertia ixx="2.0" iyy="2.0" izz="0.5" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Arm base -->
  <link name="arm_base">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.3"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Arm link 1 -->
  <link name="arm_link1">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.6"/>
      </geometry>
      <material name="orange">
        <color rgba="1.0 0.5 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.05" iyy="0.05" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Arm link 2 -->
  <link name="arm_link2">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.5"/>
      </geometry>
      <material name="orange">
        <color rgba="1.0 0.5 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.03" iyy="0.03" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- End effector/gripper -->
  <link name="gripper">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Camera -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Joints -->

  <!-- Arm base to hull -->
  <joint name="arm_base_joint" type="fixed">
    <parent link="base_link"/>
    <child link="arm_base"/>
    <origin xyz="0.8 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Arm joint 1 (shoulder) -->
  <joint name="arm_joint1" type="revolute">
    <parent link="arm_base"/>
    <child link="arm_link1"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <!-- Arm joint 2 (elbow) -->
  <joint name="arm_joint2" type="revolute">
    <parent link="arm_link1"/>
    <child link="arm_link2"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
  </joint>

  <!-- Gripper joint -->
  <joint name="gripper_joint" type="revolute">
    <parent link="arm_link2"/>
    <child link="gripper"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1.0"/>
  </joint>

  <!-- Camera joint -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="1.0 0 0.2" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo materials -->
  <gazebo reference="base_link">
    <material>Gazebo/Yellow</material>
  </gazebo>

  <gazebo reference="arm_base">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="arm_link1">
    <material>Gazebo/Orange</material>
  </gazebo>

  <gazebo reference="arm_link2">
    <material>Gazebo/Orange</material>
  </gazebo>

  <gazebo reference="gripper">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="camera_link">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- Buoyancy plugin -->
  <gazebo>
    <plugin name="buoyancy" filename="libBuoyancyPlugin.so">
      <fluid_density>1025</fluid_density> <!-- Seawater density -->
      <fluid_level>10.0</fluid_level>
      <linear_drag>25.0</linear_drag>
      <angular_drag>2.0</angular_drag>
      <buoyancy name="base_link_buoyancy">
        <link_name>base_link</link_name>
        <geometry>
          <cylinder>
            <radius>${hull_radius}</radius>
            <length>${hull_length}</length>
          </cylinder>
        </geometry>
      </buoyancy>
    </plugin>
  </gazebo>

  <!-- Thruster plugins for movement -->
  <gazebo>
    <plugin name="thruster_plugin" filename="libgazebo_ros_force.so">
      <ros>
        <namespace>/auv</namespace>
        <remapping>~/force:=thruster_force</remapping>
      </ros>
      <link_name>base_link</link_name>
    </plugin>
  </gazebo>

  <!-- Joint state publisher -->
  <gazebo>
    <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
      <ros>
        <namespace>/auv</namespace>
        <remapping>~/joint_states:=joint_states</remapping>
      </ros>
      <update_rate>50</update_rate>
      <joint_name>arm_joint1</joint_name>
      <joint_name>arm_joint2</joint_name>
      <joint_name>gripper_joint</joint_name>
    </plugin>
  </gazebo>

  <!-- Joint position control -->
  <gazebo>
    <plugin name="joint_position_controller" filename="libgazebo_ros_joint_pose_trajectory.so">
      <ros>
        <namespace>/auv</namespace>
        <remapping>~/joint_trajectory:=arm_controller/joint_trajectory</remapping>
      </ros>
      <update_rate>50</update_rate>
      <joint_name>arm_joint1</joint_name>
      <joint_name>arm_joint2</joint_name>
      <joint_name>gripper_joint</joint_name>
    </plugin>
  </gazebo>

  <!-- Underwater camera -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="underwater_camera">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>600</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>10</far> <!-- Limited underwater visibility -->
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/auv</namespace>
          <remapping>~/image_raw:=camera/image_raw</remapping>
          <remapping>~/camera_info:=camera/camera_info</remapping>
        </ros>
        <camera_name>underwater_camera</camera_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Pressure sensor -->
  <gazebo>
    <plugin name="pressure_sensor" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/auv</namespace>
        <remapping>~/imu:=pressure_sensor</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
      <frame_name>base_link</frame_name>
      <update_rate>50.0</update_rate>
    </plugin>
  </gazebo>

</robot>
```

### Launch Files

```python
# File: underwater_robot_simulation/launch/auv_simulation.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('underwater_robot_simulation')

    # World file
    world_file = os.path.join(pkg_dir, 'worlds', 'underwater.world')

    # URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'auv_with_arm.urdf.xacro')

    # Launch Gazebo with underwater world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )

    # Spawn AUV in underwater world
    spawn_auv = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_auv',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'auv_with_arm',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '-2.0'  # Start underwater
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_auv
    ])
```

### Hands-on: Build and Test AUV

#### Exercise 1: Launch AUV Simulation

```bash
# Build package
cd /workspace
colcon build --packages-select underwater_robot_simulation

# Source workspace
source install/setup.bash

# Launch AUV simulation
ros2 launch underwater_robot_simulation auv_simulation.launch.py
```

#### Exercise 2: Inspect AUV

1. **Verify AUV spawned underwater**:
   - Look for yellow torpedo-shaped hull
   - Orange manipulator arm
   - Red gripper end effector

2. **Check ROS2 topics**:

   ```bash
   ros2 topic list | grep auv
   # Should see: /auv/camera/image_raw, /auv/joint_states, etc.
   ```

3. **Monitor joint states**:
   ```bash
   ros2 topic echo /auv/joint_states
   ```

---

## Section 3: AUV Control and Manipulation (15 minutes)

### AUV Movement Control

Create a simple teleop controller for the AUV.

```python
# File: underwater_robot_simulation/underwater_robot_simulation/auv_teleop.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys, select, termios, tty

class AUVTeleop:
    def __init__(self, node: Node):
        self.node = node
        self.node.get_logger().info("AUV Teleop Controller Started")

        # Publishers
        self.force_pub = self.node.create_publisher(Wrench, '/auv/thruster_force', 10)
        self.arm_pub = self.node.create_publisher(JointTrajectory, '/auv/arm_controller/joint_trajectory', 10)

        # Subscribers
        self.joint_sub = self.node.create_subscription(
            JointState, '/auv/joint_states', self.joint_callback, 10
        )

        # Current joint positions
        self.current_joints = [0.0, 0.0, 0.0]  # arm_joint1, arm_joint2, gripper_joint

        # Movement parameters
        self.linear_scale = 50.0   # Force magnitude
        self.angular_scale = 10.0  # Torque magnitude
        self.joint_step = 0.1      # Joint movement step

    def joint_callback(self, msg):
        """Update current joint positions."""
        if len(msg.position) >= 3:
            self.current_joints = list(msg.position[:3])

    def move_auv(self, linear_x=0, linear_y=0, linear_z=0, angular_x=0, angular_y=0, angular_z=0):
        """Send movement command to AUV."""
        wrench = Wrench()
        wrench.force.x = linear_x * self.linear_scale
        wrench.force.y = linear_y * self.linear_scale
        wrench.force.z = linear_z * self.linear_scale
        wrench.torque.x = angular_x * self.angular_scale
        wrench.torque.y = angular_y * self.angular_scale
        wrench.torque.z = angular_z * self.angular_scale

        self.force_pub.publish(wrench)

    def move_arm(self, joint_deltas):
        """Move arm joints by specified deltas."""
        # Update joint positions
        new_positions = [
            self.current_joints[i] + joint_deltas[i]
            for i in range(len(joint_deltas))
        ]

        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = ['arm_joint1', 'arm_joint2', 'gripper_joint']

        point = JointTrajectoryPoint()
        point.positions = new_positions
        point.time_from_start.sec = 1

        trajectory.points = [point]
        self.arm_pub.publish(trajectory)

    def print_instructions(self):
        """Print control instructions."""
        print("""
AUV Underwater Control:
---------------------------
AUV Movement:
   w/s : forward/backward
   a/d : left/right
   q/e : up/down
   z/c : roll left/right
   r/f : pitch up/down
   t/g : yaw left/right

Arm Control:
   1/2 : shoulder joint +/-
   3/4 : elbow joint +/-
   5/6 : gripper +/-

   SPACE : stop all movement
   ESC   : quit

Current Joint Positions: {:.2f}, {:.2f}, {:.2f}
        """.format(*self.current_joints))

def get_key():
    """Get keyboard input."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = rclpy.create_node('auv_teleop')
    teleop = AUVTeleop(node)

    teleop.print_instructions()

    try:
        while True:
            key = get_key()

            # AUV movement
            if key == 'w':
                teleop.move_auv(linear_x=1)
            elif key == 's':
                teleop.move_auv(linear_x=-1)
            elif key == 'a':
                teleop.move_auv(linear_y=1)
            elif key == 'd':
                teleop.move_auv(linear_y=-1)
            elif key == 'q':
                teleop.move_auv(linear_z=1)
            elif key == 'e':
                teleop.move_auv(linear_z=-1)
            elif key == 'z':
                teleop.move_auv(angular_x=1)
            elif key == 'c':
                teleop.move_auv(angular_x=-1)
            elif key == 'r':
                teleop.move_auv(angular_y=1)
            elif key == 'f':
                teleop.move_auv(angular_y=-1)
            elif key == 't':
                teleop.move_auv(angular_z=1)
            elif key == 'g':
                teleop.move_auv(angular_z=-1)

            # Arm control
            elif key == '1':
                teleop.move_arm([teleop.joint_step, 0, 0])
            elif key == '2':
                teleop.move_arm([-teleop.joint_step, 0, 0])
            elif key == '3':
                teleop.move_arm([0, teleop.joint_step, 0])
            elif key == '4':
                teleop.move_arm([0, -teleop.joint_step, 0])
            elif key == '5':
                teleop.move_arm([0, 0, teleop.joint_step])
            elif key == '6':
                teleop.move_arm([0, 0, -teleop.joint_step])

            # Stop
            elif key == ' ':
                teleop.move_auv()  # All zeros

            # Quit
            elif key == '\x1b':  # ESC
                break

            # Update display
            teleop.print_instructions()

            # Spin once to process callbacks
            rclpy.spin_once(node, timeout_sec=0.01)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Hands-on: Control the AUV

#### Exercise 1: AUV Teleop Control

```bash
# Terminal 1: Launch AUV simulation
ros2 launch underwater_robot_simulation auv_simulation.launch.py

# Terminal 2: Run teleop controller
ros2 run underwater_robot_simulation auv_teleop
```

#### Exercise 2: Test Movement and Manipulation

1. **Test AUV movement**:
   - **w/s** - Move forward/backward through water
   - **q/e** - Ascend/descend in water column
   - **a/d** - Strafe left/right

2. **Test arm manipulation**:
   - **1/2** - Move shoulder joint
   - **3/4** - Move elbow joint
   - **5/6** - Open/close gripper

3. **Observe underwater physics**:
   - Notice buoyancy effects
   - Water resistance on movement
   - Arm dynamics underwater

---

## Section 4: Sensors and Underwater Visualization (5 minutes)

### RViz Underwater Visualization

```python
# File: underwater_robot_simulation/launch/auv_with_rviz.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Include AUV simulation
    auv_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('underwater_robot_simulation'),
            '/launch/auv_simulation.launch.py'
        ])
    )

    # RViz with underwater robot visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        auv_simulation,
        rviz
    ])
```

### Hands-on: Complete Underwater System

#### Exercise 1: Full System Launch

```bash
# Launch AUV with RViz
ros2 launch underwater_robot_simulation auv_with_rviz.launch.py
```

#### Exercise 2: RViz Configuration

1. **Set Fixed Frame** to "base_link"
2. **Add displays**:
   - **RobotModel** - See AUV structure
   - **TF** - Coordinate frames
   - **Image** - Underwater camera feed
   - **JointState** - Arm joint positions

3. \*\*Configure
