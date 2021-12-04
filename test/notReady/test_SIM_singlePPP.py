#! /usr/bin/env python

import unittest
import rospy
from group4.srv import *
from group4.msg import *
from geometry_msgs.msg import Point, Vector3, Quaternion, Pose
from std_srvs.srv import Trigger


class TestSinglePPPSeqNode(unittest.TestCase):
    def __init__(self, *args):
        super(TestSinglePPPSeqNode, self).__init__(*args)

        ## Initializing test node
        rospy.init_node("test_singlePPPSequence")

        ## Retrieve params from param server

        ## Preparing services to be called in the testing sequence
        rospy.wait_for_service('go_to_cap')
        self.go_to_caps = rospy.ServiceProxy('go_to_cap', GotoPos)

        rospy.wait_for_service('go_to_disposal')
        self.go_to_disposal = rospy.ServiceProxy('go_to_disposal', GotoPos)

        rospy.wait_for_service('go_to_marker')
        self.go_to_markers = rospy.ServiceProxy('go_to_marker', GotoPos)

        rospy.wait_for_service('go_to_master')
        self.go_to_assembly = rospy.ServiceProxy('go_to_master', GotoPos)

        rospy.wait_for_service('go_to_trayLocation')
        self.go_to_trayLoc = rospy.ServiceProxy('go_to_trayLocation', TrayLocationMove)

        rospy.wait_for_service('getMax_F_ext')
        self.listen_for_max_ext_F = rospy.ServiceProxy('getMax_F_ext', Trigger)

        rospy.wait_for_service('stop_recording_F_ext')
        self.get_local_max_ext_F = rospy.ServiceProxy('stop_recording_F_ext', GetMaxWrenches)

    def test_moveToCaps(self):
        result = self.go_to_caps()
        self.assertEqual(result.result, "Done")

    def test_moveToMarkers(self):
        result = self.go_to_markers()
        self.assertEqual(result.result, "Done")

    def test_moveToAssembly(self):
        result = self.go_to_assembly()
        self.assertEqual(result.result, "Done")

    def test_moveToDisposal(self):
        result = self.go_to_disposal()
        self.assertEqual(result.result, "Done")
        

    # def test_pickPlaceMarkerLoc1(self):
    #     pass

    # def test_pickPlaceCapLoc1(self):
    #     pass

    # def test_pressLoc1(self):
    #     pass

if __name__ == "__main__":
    import rostest
    rostest.rosrun('group4', 'test_SinglePPPSequence_node', TestSinglePPPSeqNode)
