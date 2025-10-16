set shell := ["bash", "-c"]

build:
    ./check_in_container.sh && \
    cd .. && colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

docker_build_base:
    docker buildx build -f .devcontainer/Dockerfile --target local-base -t dynorobotics/ros2-base-dev:jazzy .devcontainer

docker_push_base:
    docker push dynorobotics/ros2-base-dev:jazzy

docker_build:
    docker compose build

install_dependencies:
    ./check_in_container.sh && \
    bash -c 'sudo apt-get update && rosdep install --from-paths src --ignore-src -y' && \
    python3 -m pip install --break-system-packages -r requirements.txt

build_debug:
    ./check_in_container.sh && \
    cd .. && colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug

build_verbose:
    ./check_in_container.sh && \
    cd .. && colcon build --merge-install --symlink-install --event-handlers console_cohesion+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

run_tests:
    ./check_in_container.sh && \
    cd .. && \
    colcon test --merge-install --executor sequential --pytest-args --verbose && \
    colcon test-result --verbose
rebuild:
    ./check_in_container.sh && \
    cd .. && \
    rm -rf build/ install/ && \
    source /opt/dependencies_ws/install/setup.bash && \
    colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Rebuild the ROS 2 package.xml cache
pkg-cache-update:
    rm -rf .devcontainer/pkg_cache/src
    mkdir -p .devcontainer/pkg_cache/src
    cd src && \
      find . -type f -name package.xml -print0 | \
      while IFS= read -r -d '' f; do \
        install -Dm0644 "$f" "../.devcontainer/pkg_cache/src/${f#./}"; \
      done
    echo ">>> pkg_cache refreshed with package.xml files from ./src"

pkg-cache-remove:
    rm -rf .devcontainer/pkg_cache/src

lint:
    ./check_in_container.sh && \
    ament_flake8 && \
    ament_cpplint

zenohd:
    ros2 run rmw_zenoh_cpp rmw_zenohd

web:
    ros2 launch dynoturtle_bringup web.launch.py

teleop:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args- --remap  __ns:=/turtle1 --remap cmd_vel:=safe_cmd_vel

teleop_2:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args- --remap  __ns:=/turtle2 --remap cmd_vel:=safe_cmd_vel

start_mission:
    ros2 topic pub /turtle1/start_mission std_msgs/msg/Empty --once

cancel_mission:
    ros2 topic pub /turtle1/cancel_mission std_msgs/msg/Empty --once

l_turtle:
    ros2 launch dynoturtle_bringup full.launch.py

l_turtle_2:
    ros2 launch dynoturtle_bringup full.launch.py num_turtles:=2

l_turtle_random:
    ros2 launch dynoturtle_bringup full.launch.py behavior_tree:=random_tree

l_turtle_charge:
    ros2 launch dynoturtle_bringup full.launch.py behavior_tree:=charge_tree

l_dynobot:
    ros2 launch dynobot_bringup full.launch.py
