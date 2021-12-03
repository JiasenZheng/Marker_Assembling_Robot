import unittest
import manipulation
from manipulation.trayPositions import TrayPositions
from manipulation.translationUtilities import *
from manipulation.verificationUtilities import *

class ManipulationTestCase(unittest.TestCase):
    def test_trayPositionsGenerator(self):
        pass
        # self.assertEquals()

    def test_getLocation(self):
        pass
        # self.assertEquals()

    def test_getRpy(self):
        pass
        # self.assertEquals()

    def test_cvtPoseToList():
        pass
        # self.assertEquals()
        
    def test_cvtRPYToQuaternion():
        pass
        # self.assertEqual()

    def test_cvtQuaternionToRPY():
        pass
        # self.assertEqual()


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(manipulation, 'Manipulation Pkg Tests', ManipulationTestCase)