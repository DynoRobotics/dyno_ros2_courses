import os
import os.path

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    pkg_dynobot_gazebo_worlds = get_package_share_directory("dynobot_gazebo_worlds")

    pkg_dynobot_bringup = get_package_share_directory("dynobot_bringup")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    # Launch args
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    world_name = LaunchConfiguration(
        "world_name",
        default=["kontor", ".world"],  ## OFFICE SCAN
        # default=["basic", ".world"],  ## EMPTY WORLD
    )

    gazebo_gui = LaunchConfiguration("headless", default=1)

    # # Set the path to the world file
    world_file = PathJoinSubstitution(
        [pkg_dynobot_gazebo_worlds, "worlds", world_name]  ## OFFICE SCAN
        # [
        #     "/opt/dependencies_ws/src/dyno_gazebo_worlds",
        #     "worlds",
        #     world_name,
        # ]  ## EMPTY WORLD
    )
    world_launch_configuration = SetLaunchConfiguration(name="world", value=world_file)

    # Set the path to the SDF model files.
    gazebo_models_path = os.path.join(pkg_dynobot_gazebo_worlds, "models")
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # Set Gazebo clock to higher rate to avoid slowing down the rest of the system
    gazebo_params_path = os.path.join(pkg_dynobot_bringup, "params", "gazebo.yaml")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "gui": gazebo_gui,
            "gdb": "false",
            "verbose": "false",
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_path,
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "robot",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.1",
            "-Y",
            "0.0",
            "-topic",
            "robot_description",
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(world_launch_configuration)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)

    return ld


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
