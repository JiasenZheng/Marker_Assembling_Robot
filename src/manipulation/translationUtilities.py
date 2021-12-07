import numpy as np
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from tf_conversions import transformations

def poseToXYZRPYList(pose):
    """
    Converts a pose to an XYZ, RPY pose
    """
    x = pose.point.x
    y = pose.point.y
    z = pose.point.z
    rot_x = pose.orientation.x
    rot_y = pose.orientation.y
    rot_z = pose.orientation.z
    poseAsList = [x, y, z, rot_x, rot_y, rot_z]
    return poseAsList

def rpyVec3ToQuaternion(rpyVec):
    """
    Returns the quaternion of an RPY orientation
    """
    q0, q1, q2, q3 = transformations.quaternion_from_euler(rpyVec.x, rpyVec.y, rpyVec.z)
    return Quaternion(q0, q1, q2, q3)

def quaternionToRPYVec3(q):
    """
    returns an RPY orientation from a quaternion
    """
    r, p, y = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return Vector3(r, p, y)

    