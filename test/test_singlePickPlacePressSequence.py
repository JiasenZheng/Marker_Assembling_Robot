#! /usr/bin/env python

import unittest
import rospy
from group4.srv import *
from group4.msg import *
from geometry_msgs.msg import Point, Vector3, Quaternion, Pose
from rostest import rosrun
from std_srvs.srv import Trigger


class TestSinglePPPSeqNode(unittest.TestCase):
    def __init__(self, *args):
        super(TestSinglePPPSeqNode, self).__init__(*args)

        ## Initializing test node
        rospy.init_node("test_singlePPPSequence")

        ## Retrieve params from param server
        self.sim = rospy.get_param("sim_test")
        rospy.loginfo('self.sim **&*(*&%^*((&%^&*()&^%&*(&^%&*(&^%&*(&')
        rospy.loginfo(self.sim)

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

        # rospy.wait_for_service('press')
        # self.press = rospy.ServiceProxy('press', Press)

        rospy.wait_for_service('getMax_F_ext')
        self.listen_for_max_ext_F = rospy.ServiceProxy('getMax_F_ext', Trigger)

        rospy.wait_for_service('stop_recording_F_ext')
        self.get_local_max_ext_F = rospy.ServiceProxy('stop_recording_F_ext', GetMaxWrenches)

        # if not self.sim:
        #     rospy.wait_for_service('grip_cap')
        #     self.grip = rospy.ServiceProxy('grip_cap', Trigger)

        #     rospy.wait_for_service('release_cap')
        #     self.ungrip = rospy.ServiceProxy('release_cap', Trigger)


        #     rospy.wait_for_service('coll_hi')
        #     self.set_coll_limit_high = rospy.ServiceProxy('coll_hi', CollisionSet)

        #     rospy.wait_for_service('coll_lo')
        #     self.set_coll_limit_low = rospy.ServiceProxy('coll_lo', CollisionSet)

    # def test_setCollLimitHigh(self):
    #     if not self.sim:
    #         result = self.set_coll_limit_high()
    #         self.assertEqual(result.result, "Done")
    #     else:
    #         self.assertEqual(self.sim, True)

    # def test_moveToCaps(self):
    #     result = self.go_to_caps()
    #     self.assertEqual(result.result, "Done")

    # def test_moveToMarkers(self):
    #     result = self.go_to_markers()
    #     self.assertEqual(result.result, "Done")

    # def test_moveToAssembly(self):
    #     result = self.go_to_assembly()
    #     self.assertEqual(result.result, "Done") 

    # def test_moveToDisposal(self):
    #     result = self.go_to_disposal()
    #     self.assertEqual(result.result, "Done")

    def test_pickPlaceMarkerLoc0(self):
        ## go to clearance
        result = self.go_to_trayLoc(2, 0, 2, 1)
        success = result.success
        msg = result.error_msg

        ## go to standoff
        if success == True:
            result = self.go_to_trayLoc(2, 0, 1, 1)
            success = result.success
            msg = result.error_msg

        ## go to pick height
        if success == True:
            result = self.go_to_trayLoc(2, 0, 0, 0)
            success = result.success
            msg = result.error_msg

        # ## grip , iff not simulation test
        # if success == True:
        #     if not self.sim:
        #         result = self.grip()
        #         result = result.success
        #         msg = result.message

        ## withdraw from tray to standoff
        if success == True:
            result = self.go_to_trayLoc(2, 0, 1, 0)
            success = result.success
            msg = result.error_msg
        
        ## go to clearance
        if success:
            result = self.go_to_trayLoc(2, 0, 2, 1)
            success = result.success
            msg = result.error_msg

        ## go to assembly clearance
        if success:
            result = self.go_to_trayLoc(1, 0, 4, 1)
            success = result.success
            msg = result.error_msg

        ## go to assembly standoff
        if success:
            result = self.go_to_trayLoc(1, 0, 4, 1)
            success = result.success
            msg = result.error_msg

        ## insert to marker place height
        if success:
            result = self.go_to_trayLoc(1, 0, 0, 0)
            success = result.success
            msg = result.error_msg

        # ## ungrip, iff not simulation test
        # if success:
        #     if not self.sim:
        #         result = self.ungrip()
        #         result = result.success
        #         msg = result.message

        ## withdraw to standoff
        if success:
            result = self.go_to_trayLoc(1, 0, 3, 1)
            success = result.success
            msg = result.error_msg

        # ## move to assembly clearance
        # if success:
        #     result = self.go_to_trayLoc(1, 0, 4, 1)
        #     success = result.success
        #     msg = result.error_msg

        self.assertEqual(success, True)

    # def test_pickPlaceCapLoc0(self):
    #     ## go to clearance
    #     result = self.go_to_trayLoc(3, 0, 2, 1)
    #     success = result.success
    #     msg = result.error_msg

    #     ## go to standoff
    #     if success == True:
    #         result = self.go_to_trayLoc(3, 0, 1, 1)
    #         success = result.success
    #         msg = result.error_msg

    #     ## go to pick height
    #     if success == True:
    #         result = self.go_to_trayLoc(3, 0, 0, 0)
    #         success = result.success
    #         msg = result.error_msg

    #     # ## grip , iff not simulation test
    #     # if success == True:
    #     #     if not self.sim:
    #     #         result = self.grip()
    #     #         result = result.success
    #     #         msg = result.message

    #     ## withdraw from tray to standoff
    #     if success == True:
    #         result = self.go_to_trayLoc(3, 0, 1, 0)
    #         success = result.success
    #         msg = result.error_msg
        
    #     ## go to clearance
    #     if success == True:
    #         result = self.go_to_trayLoc(3, 0, 2, 1)
    #         success = result.success
    #         msg = result.error_msg

    #     ## go to assembly clearance
    #     if success:
    #         result = self.go_to_trayLoc(1, 0, 4, 1)
    #         success = result.success
    #         msg = result.error_msg

    #     ## go to assembly standoff
    #     if success:
    #         result = self.go_to_trayLoc(1, 0, 3, 1)
    #         success = result.success
    #         msg = result.error_msg

    #     ## insert to cap place height
    #     if success:
    #         result = self.go_to_trayLoc(1, 0, 1, 0)
    #         success = result.success
    #         msg = result.error_msg

    #     # ## ungrip, iff not simulation test
    #     # if success:
    #     #     if not self.sim:
    #     #         result = self.ungrip()
    #     #         result = result.success
    #     #         msg = result.message

    #     ## withdraw to standoff
    #     if success:
    #         result = self.go_to_trayLoc(1, 0, 3, 1)
    #         success = result.success
    #         msg = result.error_msg

    #     # ## move to assembly clearance
    #     # if success:
    #     #     result = self.go_to_trayLoc(1, 0, 4, 1)
    #     #     success = result.success
    #     #     msg = result.error_msg
            
    #     self.assertEqual(success, True)

    # def test_pressLoc0(self):
    #     ## go to clearance
    #     result = self.go_to_trayLoc(1, 0, 4, 1)
    #     success = result.success
    #     msg = result.error_msg

    #     # ## grip , iff not simulation test
    #     # if success == True:
    #     #     if not self.sim:
    #     #         result = self.grip()
    #     #         success = result.success
    #     #         msg = result.message

    #     ## go to press height
    #     if success == True:
    #         result = self.go_to_trayLoc(1, 0, 2, 0)
    #         success = result.success
    #         msg = result.error_msg
        
    #     ### CALL PRESS HERE : ImplementHere
        
    #     ## go to standoff
    #     if success == True:
    #         result = self.go_to_trayLoc(1, 0, 3, 0)
    #         success = result.success
    #         msg = result.error_msg

    #     ## go to clearance
    #     if success == True:
    #         result = self.go_to_trayLoc(1, 0, 4, 1)
    #         success = result.success
    #         msg = result.error_msg
        
    #     self.assertEqual(success, True)


if __name__ == "__main__":
    import rostest
    rostest.rosrun('group4', 'test_SinglePPPSequence_node', TestSinglePPPSeqNode)
