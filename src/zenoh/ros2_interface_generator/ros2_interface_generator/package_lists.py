"""
Package list configurations for interface generation.

Provides predefined sets of ROS2 packages to generate.
"""

# Essential packages for basic ROS2 functionality
ESSENTIAL_PACKAGES = [
    'builtin_interfaces',
    'std_msgs',
    'std_srvs',  # Standard service definitions (Empty, SetBool, Trigger)
    'geometry_msgs',
    'rcl_interfaces',  # Includes parameter services, logging, etc.
]

# Common packages for robotics applications
COMMON_PACKAGES = ESSENTIAL_PACKAGES + [
    'sensor_msgs',
    'nav_msgs',
    'trajectory_msgs',
    'action_msgs',
    'actionlib_msgs',
    'tf2_msgs',
    'unique_identifier_msgs',  # Required by action_msgs
]

# Standard ROS2 message packages (comprehensive set)
STANDARD_PACKAGES = COMMON_PACKAGES + [
    'diagnostic_msgs',
    'shape_msgs',
    'stereo_msgs',
    'visualization_msgs',
    'map_msgs',
    'lifecycle_msgs',
    'service_msgs',
    'statistics_msgs',
    'rosgraph_msgs',
    'unique_identifier_msgs',
    'type_description_interfaces',
]

# All available packages (very comprehensive)
ALL_PACKAGES = STANDARD_PACKAGES + [
    'example_interfaces',  # Includes example services like AddTwoInts
    'gps_msgs',
    'pcl_msgs',
    'pendulum_msgs',
    'rmw_dds_common',
    'ros_gz_interfaces',
    'rosbag2_interfaces',
    'theora_image_transport',
    'turtlesim',  # Includes spawn, teleport services
    'vision_msgs',
    'py_trees_ros_interfaces',
]

# Package presets
PRESETS = {
    'essential': ESSENTIAL_PACKAGES,
    'common': COMMON_PACKAGES,
    'standard': STANDARD_PACKAGES,
    'all': ALL_PACKAGES,
}


def get_package_list(preset='essential', include=None, exclude=None):
    """
    Get a list of packages to generate.
    
    Args:
        preset: Preset name ('essential', 'common', 'standard', 'all')
        include: Additional packages to include
        exclude: Packages to exclude
    
    Returns:
        List of package names
    """
    if preset not in PRESETS:
        raise ValueError(f"Unknown preset '{preset}'. Choose from: {list(PRESETS.keys())}")
    
    packages = set(PRESETS[preset])
    
    if include:
        packages.update(include)
    
    if exclude:
        packages.difference_update(exclude)
    
    return sorted(packages)

