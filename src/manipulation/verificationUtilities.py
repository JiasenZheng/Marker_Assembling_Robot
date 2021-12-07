import numpy as np


def poseToArrays(pose):
    """
    Converts geometry_msg/Pose to position and orientation numpy arrays for 
    subsequent operations 
    """
    a1 = pose.position.x
    a2 = pose.position.y
    a3 = pose.position.z
    pos = np.array([a1, a2, a3])
    a4 = pose.orientation.x
    a5 = pose.orientation.y
    a6 = pose.orientation.z
    a7 = pose.orientation.w
    orient = np.array([a4, a5, a6, a7])
    return pos, orient

def isPoseClose(currentPose, targetPose, posTol=0.002, orientTol=0.005):
    """
    Compares two poses for closesness within specified position and 
    orientation tolerances
    """
    pos1, orient1 = poseToArrays(currentPose)
    pos2, orient2 = poseToArrays(targetPose)
    matchPose = np.allclose(pos1, pos2, atol=posTol)
    matchOrient = np.allclose(np.abs(orient1), np.abs(orient2), atol=orientTol)
    match =  matchPose and matchOrient
    if match:
        msg = f"[Success] Current pose matches the Target pose {targetPose}"
    else:
        msg = f"[Error] Current pose {currentPose} does not match the Target pose {targetPose}"
    return match, msg

