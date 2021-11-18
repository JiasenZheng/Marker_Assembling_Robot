"""
This class performs match functions
"""
import rospy
import numpy as np

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
                    del arr[front]
                else:
                    ans = deepcopy(arr[back])
                    del arr[back]
                return ans
            front += 1
            back -= 1
        return None

    def matching(self, sub1, sub2):
        ans = []
        for x in sub1:
            y = self.fbsearch(sub2, x)
            if y != None:
                ans.append([x, y])
        return ans, sub1, sub2
