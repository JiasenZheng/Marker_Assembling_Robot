import unittest
import manager
from manager.manager import manage
import numpy as np

class ManageTestCase(unittest.TestCase):
    def __init__(self, *args):
        super(ManageTestCase, self).__init__(*args)

        ## Tests
        self.t1 = { "markers": np.array([]), 
                    "caps": np.array([]), 
                    "assembly": np.array([]),
                    "truth": np.array([])
                     }

        self.t2 = { "markers": np.array([]), 
                    "caps": np.array([]), 
                    "assembly": np.array([]),
                    "truth": np.array([])
                     }

        self.t3 = { "markers": np.array([]), 
                    "caps": np.array([]), 
                    "assembly": np.array([]),
                    "truth": np.array([])
                     }

        self.t4 = { "markers": np.array([]), 
                    "caps": np.array([]), 
                    "assembly": np.array([]),
                    "truth": np.array([])
                     }

        self.t5 = { "markers": np.array([]), 
                    "caps": np.array([]), 
                    "assembly": np.array([]),
                    "truth": np.array([])
                     }

        self.t6 = { "markers": np.array([]), 
                    "caps": np.array([]), 
                    "assembly": np.array([]),
                    "truth": np.array([])
                     }

        self.t7 = { "markers": np.array([]), 
                    "caps": np.array([]), 
                    "assembly": np.array([]),
                    "truth": np.array([])
                     }

        self.t8 = { "markers": np.array([]), 
                    "caps": np.array([]), 
                    "assembly": np.array([]),
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