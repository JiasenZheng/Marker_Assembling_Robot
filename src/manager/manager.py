"""
This class is used to manage markers and caps
"""
from copy import deepcopy

class manage:
    def __init__(self):
        self.ans = []

    def fbsearch(self, arr, data):
        """
        Given a target data value, this function will look 
        for data that is similar in a seperate array.
        """
        threshold = 3
        if len(arr) < 2:
            ans = deepcopy(arr[0])
            # del arr[0]
            return ans
        front = 0
        back = len(arr)-1
        while front <= back:
            if self.colorMatch(arr[front], data, threshold) or self.colorMatch(arr[back], data, threshold):
                if arr[front] == data:
                    ans = front
                    # del arr[front] Not certain whether this value needs to be deleted. Should be used for comparison later.
                else:
                    ans = back
                    # del arr[back]
                return ans
            front += 1
            back -= 1
        return None

    def colorMatch(self, a, b, threshold):
        """
        Given two one d array of 3 elements this function will compare hsv values
        And return whether the two pieces of data are similar enough to match. 
        """
        return (abs(a-b) < threshold)

    def matching(self, sub1, sub2, event=None):
        """
        Func:matching: Returns an array of items that "match" eachother
        Param:sub1: First list of values.
        Param:sub2: Second list of vlaues.
        """
        for x, item in enumerate(sub1):
            y = self.fbsearch(sub2, item)
            if y != None:
                self.ans.append([x, y])
        return self.ans

    def genList(self, lst):
        """
        Func:genMatch: Creates generated list object.
        General Use: In order to iterate through object say: 
            obj = match.genMatch(lst)
            iter = next(obj)
        Param:lst: List of matches
        """
        for index, item in enumerate(lst):
            yield index, item

    def allNil(self, data, nil):
        """
        Func:allNil: Returns true if all data being compared is equal to some nil value.
        Param:data:Data being observed.
        Param:nil:Nil value to compare.
        """
        for t in data:
            if t != nil:
                return False
        return True
    
    def nilCheck(loc):
        """Checks if a cell is empty"""
        return
    