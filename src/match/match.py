"""
This class performs match functions
"""
from copy import deepcopy

class match:
    def __init__(self):
        return
    
    def fbsearch(self, arr, data):
        if len(arr) < 2:
            ans = deepcopy(arr[0])
            del arr[0]
            return ans
        front = 0
        back = len(arr)-1
        while front <= back:
            if arr[front] == data or arr[back] == data:
                if arr[front] == data:
                    ans = deepcopy(arr[front])
                    # del arr[front] Not certain whether this value needs to be deleted. Should be used for comparison later.
                else:
                    ans = deepcopy(arr[back])
                    # del arr[back]
                return ans
            front += 1
            back -= 1
        return None

    def matching(self, sub1, sub2):
        """
        Func:matching: Returns an array of items that "match" eachother
        Param:sub1: First list of values.
        Param:sub2: Second list of vlaues.
        """
        ans = []
        for x in sub1:
            y = self.fbsearch(sub2, x)
            if y != None:
                ans.append([x, y])
        return ans

    def genMatch(lst):
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
    
    