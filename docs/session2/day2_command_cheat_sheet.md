# Session 2 - Useful Commands

## Start the simulation
```bash
just l_dynobot
```

## Start Gazebo Harmonic with an empty world
```bash
gz sim
```

## Publish to /cmd_vel
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

## RQT (General Debugging Tool)
```bash
rqt
```
An empty window will pop-up, from here open the `Plugins` drop-down menu and try out the different tools.

Some recommendations are:
* `Introspection/Node Graph`
* `Logging/Console`
* `Topics/Message Publisher`
* `Topics/Topic Monitor`
* `Visualization/Plot`

## Rosbag Record /cmd_vel topic
```bash
ros2 bag record /cmd_vel
```
This will output a directory called `rosbag2_<yyyy_mm_dd-hh_mm_ss>`, containing a .db3 file which is all the collected data and a `metadata.yaml` file containing information about the bag.

## Rosbag play
After recording one or many topics, the bag can be replayed using the following command:  
```bash
ros2 bag play <rosbag2_directory_name>
```

## Convert Xacro/URDF -> .sdf
Gazebo can not interpret mathematical expressions in the .xacro file, therefore it first needs to be processed with xacro.

```bash
xacro src/dynobot/dynobot_description/urdf/dynobot.xacro -o dynobot.urdf
```

```bash
gz sdf -p dynobot.urdf > dynobot.sdf
```
> Note: This conversion is normally performed automatically by launch files, so this is purely used for debugging purposes.