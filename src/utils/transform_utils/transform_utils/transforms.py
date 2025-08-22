import transforms3d
import geometry_msgs.msg


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
