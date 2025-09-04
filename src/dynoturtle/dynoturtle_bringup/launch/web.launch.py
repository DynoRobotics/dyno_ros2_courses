import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    web_bridge = Node(package="web_bridge", executable="web_bridge", name="web_bridge")

    web_gui = ExecuteProcess(
        cmd=["bash", "-c", "npm install && npm run dev"],
        cwd="/home/ubuntu/ws/src/web-ui/react_web_ui",
        output="screen",
    )
    return LaunchDescription([web_bridge, web_gui])


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
