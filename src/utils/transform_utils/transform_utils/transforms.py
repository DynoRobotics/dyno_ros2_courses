import transforms3d
import geometry_msgs.msg
import math


def euler_from_quaternion(quaternion: geometry_msgs.msg.Quaternion):
    q = [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
    return transforms3d.euler.quat2euler(q)


def yaw_from_quaternion(quaternion: geometry_msgs.msg.Quaternion):
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]
    return yaw


def quaternion_from_euler(
    roll: float, pitch: float, yaw: float
) -> geometry_msgs.msg.Quaternion:
    w, x, y, z = transforms3d.euler.euler2quat(roll, pitch, yaw, "rxyz")
    return geometry_msgs.msg.Quaternion(x=x, y=y, z=z, w=w)


def inverse_transform_point(
    transform: geometry_msgs.msg.Pose, world_point: tuple
) -> tuple:
    """
    Convert world space point to local space relative to transform

    Unity equivalent: Transform.InverseTransformPoint()

    Args:
        transform: The reference transform (like Unity Transform)
        world_point: Point in world coordinates (x, y) or (x, y, z)

    Returns:
        tuple: Point in transform's local space (x, y) or (x, y, z)
    """
    # Handle both 2D and 3D points
    if len(world_point) == 2:
        world_x, world_y = world_point
        world_z = 0.0
        is_2d = True
    else:
        world_x, world_y, world_z = world_point
        is_2d = False

    # Get transform position and rotation
    transform_x = transform.position.x
    transform_y = transform.position.y
    transform_z = transform.position.z
    transform_yaw = yaw_from_quaternion(transform.orientation)

    # Translate to transform origin
    relative_x = world_x - transform_x
    relative_y = world_y - transform_y
    relative_z = world_z - transform_z

    # Rotate by negative transform yaw to get local coordinates
    cos_yaw = math.cos(-transform_yaw)
    sin_yaw = math.sin(-transform_yaw)

    local_x = relative_x * cos_yaw - relative_y * sin_yaw
    local_y = relative_x * sin_yaw + relative_y * cos_yaw
    local_z = relative_z  # Z remains unchanged for 2D transforms

    if is_2d:
        return (local_x, local_y)
    else:
        return (local_x, local_y, local_z)


def inverse_transform_pose(
    transform: geometry_msgs.msg.Pose, world_pose: geometry_msgs.msg.Pose
) -> geometry_msgs.msg.Pose:
    """
    Convert world space pose to local space relative to transform

    Unity equivalent: Transform.InverseTransformPoint() + rotation handling

    Args:
        transform: The reference transform
        world_pose: Pose in world coordinates

    Returns:
        geometry_msgs.msg.Pose: Pose in transform's local space
    """
    # Transform position
    world_point = (world_pose.position.x, world_pose.position.y, world_pose.position.z)
    local_point = inverse_transform_point(transform, world_point)

    # Transform orientation (relative rotation)
    transform_yaw = yaw_from_quaternion(transform.orientation)
    world_yaw = yaw_from_quaternion(world_pose.orientation)
    local_yaw = normalize_angle(world_yaw - transform_yaw)

    # Create local pose
    local_pose = geometry_msgs.msg.Pose()
    local_pose.position.x = local_point[0]
    local_pose.position.y = local_point[1]
    local_pose.position.z = local_point[2]

    # Convert local yaw back to quaternion (simplified for 2D)
    local_pose.orientation.x = 0.0
    local_pose.orientation.y = 0.0
    local_pose.orientation.z = math.sin(local_yaw / 2.0)
    local_pose.orientation.w = math.cos(local_yaw / 2.0)

    return local_pose


def transform_point(transform: geometry_msgs.msg.Pose, local_point: tuple) -> tuple:
    """
    Convert local space point to world space relative to transform

    Unity equivalent: Transform.TransformPoint()

    Args:
        transform: The reference transform
        local_point: Point in local coordinates (x, y) or (x, y, z)

    Returns:
        tuple: Point in world space (x, y) or (x, y, z)
    """
    # Handle both 2D and 3D points
    if len(local_point) == 2:
        local_x, local_y = local_point
        local_z = 0.0
        is_2d = True
    else:
        local_x, local_y, local_z = local_point
        is_2d = False

    # Get transform position and rotation
    transform_x = transform.position.x
    transform_y = transform.position.y
    transform_z = transform.position.z
    transform_yaw = yaw_from_quaternion(transform.orientation)

    # Rotate by transform yaw
    cos_yaw = math.cos(transform_yaw)
    sin_yaw = math.sin(transform_yaw)

    rotated_x = local_x * cos_yaw - local_y * sin_yaw
    rotated_y = local_x * sin_yaw + local_y * cos_yaw
    rotated_z = local_z

    # Translate to world coordinates
    world_x = rotated_x + transform_x
    world_y = rotated_y + transform_y
    world_z = rotated_z + transform_z

    if is_2d:
        return (world_x, world_y)
    else:
        return (world_x, world_y, world_z)


def transform_pose(
    transform: geometry_msgs.msg.Pose, local_pose: geometry_msgs.msg.Pose
) -> geometry_msgs.msg.Pose:
    """
    Convert local space pose to world space relative to transform

    Unity equivalent: Transform.TransformPoint() + rotation handling

    Args:
        transform: The reference transform
        local_pose: Pose in local coordinates

    Returns:
        geometry_msgs.msg.Pose: Pose in world space
    """
    # Transform position
    local_point = (local_pose.position.x, local_pose.position.y, local_pose.position.z)
    world_point = transform_point(transform, local_point)

    # Transform orientation (add rotations)
    transform_yaw = yaw_from_quaternion(transform.orientation)
    local_yaw = yaw_from_quaternion(local_pose.orientation)
    world_yaw = normalize_angle(transform_yaw + local_yaw)

    # Create world pose
    world_pose = geometry_msgs.msg.Pose()
    world_pose.position.x = world_point[0]
    world_pose.position.y = world_point[1]
    world_pose.position.z = world_point[2]

    # Convert world yaw back to quaternion (simplified for 2D)
    world_pose.orientation.x = 0.0
    world_pose.orientation.y = 0.0
    world_pose.orientation.z = math.sin(world_yaw / 2.0)
    world_pose.orientation.w = math.cos(world_yaw / 2.0)

    return world_pose


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi] range

    Args:
        angle: Angle in radians

    Returns:
        float: Normalized angle in [-pi, pi] range
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def angle_difference(target: float, current: float) -> float:
    """
    Calculate the shortest angular difference between two angles

    Args:
        target: Target angle in radians
        current: Current angle in radians

    Returns:
        float: Shortest angular difference in radians
    """
    diff = target - current
    return normalize_angle(diff)


def distance_2d(pose1: geometry_msgs.msg.Pose, pose2: geometry_msgs.msg.Pose) -> float:
    """
    Calculate 2D Euclidean distance between two poses

    Args:
        pose1: First pose
        pose2: Second pose

    Returns:
        float: Distance in meters
    """
    dx = pose2.position.x - pose1.position.x
    dy = pose2.position.y - pose1.position.y
    return math.sqrt(dx * dx + dy * dy)


def distance_3d(pose1: geometry_msgs.msg.Pose, pose2: geometry_msgs.msg.Pose) -> float:
    """
    Calculate 3D Euclidean distance between two poses

    Args:
        pose1: First pose
        pose2: Second pose

    Returns:
        float: Distance in meters
    """
    dx = pose2.position.x - pose1.position.x
    dy = pose2.position.y - pose1.position.y
    dz = pose2.position.z - pose1.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)
