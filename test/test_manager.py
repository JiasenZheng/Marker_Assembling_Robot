import unittest
import manager
from manager.manager import manage
import numpy as np

class ManageTestCase(unittest.TestCase):
    def __init__(self, *args):
        super(ManageTestCase, self).__init__(*args)

        ## Tests for a number of different permuations of marker and cap arrangements
        
        self.t1 = { "assembly": np.array([10,176, 0, 117, 79, 0, 29, 106, 0]), 
                    "caps": np.array([118, 76, 27, 174, 30, 11, 99, 81, 100]), 
                    "markers": np.array([29,29,174,117,81,98,78,11,97]),
                    "truth": np.array([[0,2], [1,4], [2,3], [3,0], [4,1], [5,6], [6,7], [7,5], [8,8]]),
                    "truth_destination": [2,5,8],
                    "truth_sort": np.array([[7, 5], [0, 2], [1, 4], [6, 7], [4, 1], [8, 8], [5, 6], [3, 0], [2, 3]])
                     }

        self.t2 = { "assembly": np.array([0,0, 0, 26, 0, 29, 0, 80, 0]), 
                    "caps": np.array([5,99,179,79,79,173,117,116,28]), 
                    "markers": np.array([78,78,28,26,11,120,121,11,99]),
                    "truth": np.array([[0,2], [1,3],[2,8],[4,0],[5,5],[8,1]]),
                    "truth_destination": [0,1,2,4,6,8],
                    "truth_sort": np.array([[4, 0], [2, 8], [0, 2], [1, 3], [8, 1], [5, 5]])
                     }

        self.t3 = { "assembly": np.array([0,0,28,0,12,26,118,80,0]), 
                    "caps": np.array([5,100,73,79,174,118,174,118,28]), 
                    "markers": np.array([78,27,122, 77, 11, 121, 25, 11, 100]),
                    "truth": np.array([[0,2],[1,8],[2,5],[3,3],[4,0],[5,7],[8,1]]),
                    "truth_destination": [0,1,3,8],
                    "truth_sort": np.array([[4, 0], [1, 8], [3, 3], [0, 2], [8, 1], [5, 7], [2, 5]])
                     }

        self.t4 = { "assembly": np.array([0,117, 30, 179, 79, 29, 118, 77, 0]), 
                    "caps": np.array([101,100,74,29,12,118,174,116, 22]), 
                    "markers": np.array([116,27,78,121,11,174,24,11,99]),
                    "truth": np.array([[0,5],[1,3],[2,2],[3,7],[4,4],[5,6],[6,8],[8,0]]),
                    "truth_destination": [0,8],
                    "truth_sort": np.array([[4, 4], [6, 8], [1, 3], [2, 2], [8, 0], [0, 5], [3, 7], [5, 6]])
                     }

        self.t5 = { "assembly": np.array([10, 0, 118, 74, 0, 29, 0, 26, 0]), 
                    "caps": np.array([118, 76, 27, 174, 30, 11, 99, 81, 100]), 
                    "markers": np.array([29,29,174,117,81,98,78,11,97]),
                    "truth": np.array([[0,2],[1,4],[2,3],[3,0],[4,1],[5,6],[6,7],[7,5],[8,8]]),
                    "truth_destination": [1,4,6,8],
                    "truth_sort": np.array([[7, 5], [0, 2], [1, 4], [6, 7], [4, 1], [8, 8], [5, 6], [3, 0], [2, 3]])
                     }

        self.t6 = { "assembly": np.array([0, 176, 27, 117, 79, 0, 0, 106, 116, 100]), 
                    "caps": np.array([5,99,179,79,79,173,117,116,28]), 
                    "markers": np.array([78,78,28,26,11,120,121,11,99]),
                    "truth": np.array([[0,3],[1,4],[2,8],[4,0],[5,6],[6,7],[8,1]]),
                    "truth_destination": [0,5,6],
                    "truth_sort": np.array([[4, 0], [2, 8], [0, 3], [1, 4], [8, 1], [5, 6], [6, 7]])
                     }

        self.t7 = { "assembly": np.array([26,178, 118, 26, 6, 29, 118, 80, 100]), 
                    "caps": np.array([5,100,73,79,174,118,174,118,28]), 
                    "markers": np.array([116,27,78,121,11,174,24,11,99]),
                    "truth": np.array([[0,5],[1,8],[2,2],[3,7],[4,0],[5,4],[8,1]]),
                    "truth_destination": [],
                    "truth_sort": np.array([[4, 0], [2, 8], [0, 3], [1, 4], [8, 1], [5, 6], [6, 7]])
                     }

        self.t8 = { "assembly": np.array([0,117,28,176,12,0,0,0,0]), 
                    "caps": np.array([101,100,74,29,12,118,174,116,22]), 
                    "markers": np.array([116,27,78,121,11,174,24,11,99]),
                    "truth": np.array([[0,5],[1,3],[2,2],[3,7],[4,4],[5,6],[6,8],[8,0]]),
                    "truth_destination": [0,5,6,7,8],
                    "truth_sort": np.array([[4, 4], [6, 8], [1, 3], [2, 2], [8, 0], [0, 5], [3, 7], [5, 6]])
                     }

        self.t9 = { "assembly": np.array([0,0, 0, 0, 0, 0, 0, 0, 0]), 
                    "caps": np.array([118, 76, 27, 174, 30, 11, 99, 81, 100]), 
                    "markers": np.array([29,29,174,117,81,98,78,11,97]),
                    "truth": np.array([[0,2],[1,4],[2,3],[3,0],[4,1],[5,6],[6,7],[7,5],[8,8]]),
                    "truth_destination": [0,1,2,3,4,5,6,7,8],
                    "truth_sort": np.array([[4, 4], [6, 8], [1, 3], [2, 2], [8, 0], [0, 5], [3, 7], [5, 6]])
                     }

        self.t10 = { "assembly": np.array([10, 10, 118, 74, 74, 29, 0, 26, 26]), 
                    "caps": np.array([5,99,179,79,79,173,117,116,28]), 
                    "markers": np.array([78,78,28,26,11,120,121,11,99]),
                    "truth": np.array([[0,3],[1,4],[2,8],[4,0],[5,6],[6,7],[8,1]]),
                    "truth_destination": [6],
                    "truth_sort": np.array([[4, 0], [2, 8], [0, 3], [1, 4], [8, 1], [5, 6], [6, 7]])
                     }

    def create_matchTest(self, test_dict):
        ## call match function
        man = manage()
        result = man.thoroughMatching(test_dict["markers"],test_dict["caps"])
        print("result: ", result)
        print("expects: ", test_dict["truth"].tolist())
        return result, test_dict["truth"].tolist()

    def create_destinationTest(self, test_dict):
        man = manage()
        result = man.fullSearch(test_dict["assembly"], 0)
        print("result: ", result)
        print("expects: ", test_dict["truth_destination"])
        return result, test_dict["truth_destination"]
    
    def create_sortTest(self, test_dict):
        man = manage()
        result = man.sort(test_dict["markers"], test_dict["truth"])
        print("result: ", result)
        print("expects: ", test_dict["truth_sort"].tolist())
        return result, test_dict["truth_sort"].tolist()
        
    def test_t1MatchCapsAndMarkers(self):
        result, expected = self.create_matchTest(self.t1)
        self.assertEquals(result, expected)

    # def test_t2MatchCapsAndMarkers(self):
    #     result, expected = self.create_matchTest(self.t2)
    #     self.assertEquals(result, expected)

    def test_t3MatchCapsAndMarkers(self):
        result, expected = self.create_matchTest(self.t3)
        self.assertEquals(result, expected)

    def test_t4MatchCapsAndMarkers(self):
        result, expected = self.create_matchTest(self.t4)
        self.assertEquals(result, expected)

    def test_t5MatchCapsAndMarkers(self):
        result, expected = self.create_matchTest(self.t5)
        self.assertEquals(result, expected)

    def test_t6MatchCapsAndMarkers(self):
        result, expected = self.create_matchTest(self.t6)
        self.assertEquals(result, expected)

    def test_t7MatchCapsAndMarkers(self):
        result, expected = self.create_matchTest(self.t7)
        self.assertEquals(result, expected)

    def test_t8MatchCapsAndMarkers(self):
        result, expected = self.create_matchTest(self.t8)
        self.assertEquals(result, expected)

    def test_t9MatchCapsAndMarkers(self):
        result, expected = self.create_matchTest(self.t9)
        self.assertEquals(result, expected)

    def test_t10MatchCapsAndMarkers(self):
        result, expected = self.create_matchTest(self.t10)
        self.assertEquals(result, expected)

    def test_t1DestinationDecision(self):
        result, expected = self.create_destinationTest(self.t1)
        self.assertEquals(result, expected) 

    def test_t2DestinationDecision(self):
        result, expected = self.create_destinationTest(self.t2)
        self.assertEquals(result, expected) 

    def test_t3DestinationDecision(self):
        result, expected = self.create_destinationTest(self.t3)
        self.assertEquals(result, expected) 

    def test_t4DestinationDecision(self):
        result, expected = self.create_destinationTest(self.t4)
        self.assertEquals(result, expected) 

    def test_t5DestinationDecision(self):
        result, expected = self.create_destinationTest(self.t5)
        self.assertEquals(result, expected) 
    
    def test_t6DestinationDecision(self):
        result, expected = self.create_destinationTest(self.t6)
        self.assertEquals(result, expected) 

    def test_t7DestinationDecision(self):
        result, expected = self.create_destinationTest(self.t7)
        self.assertEquals(result, expected) 

    def test_t8DestinationDecision(self):
        result, expected = self.create_destinationTest(self.t8)
        self.assertEquals(result, expected) 

    def test_t9DestinationDecision(self):
        result, expected = self.create_destinationTest(self.t9)
        self.assertEquals(result, expected) 

    def test_t10DestinationDecision(self):
        result, expected = self.create_destinationTest(self.t10)
        self.assertEquals(result, expected) 
    
    def test_t1Sorting(self):
        result, expected = self.create_sortTest(self.t1)
        self.assertEquals(result, expected)

    def test_t2Sorting(self):
        result, expected = self.create_sortTest(self.t2)
        self.assertEquals(result, expected)

    def test_t3Sorting(self):
        result, expected = self.create_sortTest(self.t3)
        self.assertEquals(result, expected)

    def test_t4Sorting(self):
        result, expected = self.create_sortTest(self.t4)
        self.assertEquals(result, expected)

    def test_t5Sorting(self):
        result, expected = self.create_sortTest(self.t5)
        
        self.assertEquals(result, expected)

    def test_t6Sorting(self):
        result, expected = self.create_sortTest(self.t6)
        self.assertEquals(result, expected)

    def test_t7Sorting(self):
        result, expected = self.create_sortTest(self.t7)
        self.assertEquals(result, expected)

    def test_t8Sorting(self):
        result, expected = self.create_sortTest(self.t8)
        self.assertEquals(result, expected)
    
    def test_t8Sorting(self):
        result, expected = self.create_sortTest(self.t8)
        self.assertEquals(result, expected)
    
    def test_t8Sorting(self):
        result, expected = self.create_sortTest(self.t8)
        self.assertEquals(result, expected)
    
    def test_t9Sorting(self):
        result, expected = self.create_sortTest(self.t9)
        self.assertEquals(result, expected)
    
    def test_t10Sorting(self):
        result, expected = self.create_sortTest(self.t10)
        self.assertEquals(result, expected)

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(manage, 'Manage Pkg Tests', ManageTestCase)