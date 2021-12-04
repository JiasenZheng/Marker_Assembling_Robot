import unittest
import manager
from manager.manager import manage
import numpy as np

class ManageTestCase(unittest.TestCase):
    def __init__(self, *args):
        super(ManageTestCase, self).__init__(*args)

        ## Tests for a number of different permuations of marker and cap arrangements
        
        self.t1 = { "assembly": np.array([10,176, 0, 117, 79, 0, 29, 106, 116, 0]), 
                    "caps": np.array([118, 76, 27, 174, 30, 11, 99, 81, 100]), 
                    "markers": np.array([29,29,174,117,81,98,78,11,97]),
                    "truth": np.array([])
                     }

        self.t2 = { "assembly": np.array([0,0, 0, 26, 0, 29, 0, 80, 0]), 
                    "caps": np.array([5,99,179,79,79,173,117,116,28]), 
                    "markers": np.array([78,78,28,26,11,120,121,11,99]),
                    "truth": np.array([])
                     }

        self.t3 = { "assembly": np.array([0,0,28,0,12,26,118,80,0]), 
                    "caps": np.array([5,100,73,79,174,118,174,118,28]), 
                    "markers": np.array([78,27,122, 77, 11, 121, 25, 11, 100]),
                    "truth": np.array([])
                     }

        self.t4 = { "assembly": np.array([0,117, 30, 179, 79, 29, 118, 77, 0]), 
                    "caps": np.array([101,100,74,29,12,118,174,116]), 
                    "markers": np.array([116,27,78,121,11,174,24,11,99]),
                    "truth": np.array([])
                     }

        self.t5 = { "assembly": np.array([10, 0, 118, 74, 0, 29, 0, 26, 0]), 
                    "caps": np.array([118, 76, 27, 174, 30, 11, 99, 81, 100]), 
                    "markers": np.array([29,29,174,117,81,98,78,11,97]),
                    "truth": np.array([])
                     }

        self.t6 = { "assembly": np.array([0, 176, 27, 117, 79, 0, 0, 106, 116, 100]), 
                    "caps": np.array([5,99,179,79,79,173,117,116,28]), 
                    "markers": np.array([78,78,28,26,11,120,121,11,99]),
                    "truth": np.array([])
                     }

        self.t7 = { "assembly": np.array([0,178, 118, 26, 6, 29, 118, 80, 100]), 
                    "caps": np.array([5,100,73,79,174,118,174,118,28]), 
                    "markers": np.array([116,27,78,121,11,174,24,11,99]),
                    "truth": np.array([])
                     }

        self.t8 = { "assembly": np.array([0,117,28,176,12,0,0,0,0]), 
                    "caps": np.array([101,100,74,29,12,118,174,116]), 
                    "markers": np.array([116,27,78,121,11,174,24,11,99]),
                    "truth": np.array([])
                     }

        self.t9 = { "assembly": np.array([0,117, 30, 179, 79, 0, 0, 0, 0]), 
                    "caps": np.array([118, 76, 27, 174, 30, 11, 99, 81, 100]), 
                    "markers": np.array([29,29,174,117,81,98,78,11,97]),
                    "truth": np.array([])
                     }

        self.t10 = { "assembly": np.array([10, 0, 118, 74, 0, 29, 0, 26, 0]), 
                    "caps": np.array([5,99,179,79,79,173,117,116,28]), 
                    "markers": np.array([78,78,28,26,11,120,121,11,99]),
                    "truth": np.array([])
                     }

    def create_matchTest(self, test_dict):
        ## call match function
        result = match(test_dict["caps"], test_dict["markers"])
        return result, test_dict["truth"]

    def create_destinationTest(self, test_dict):
        result = destination(test_dict["assemmbly"])
        return result, test_dict["truth_destination"]
        

    def test_t1MatchCapsAndMarkers(self):
        result, expected = self.create_matchTest(self.t1)
        self.assertEquals(result, expected)

    def test_t1DestinationDecision(self):
        result, expected = self.create_destinationTest(self.t1)
        self.assertEquals(result, expected)        


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(manage, 'Manage Pkg Tests', ManageTestCase)