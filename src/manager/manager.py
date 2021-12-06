"""
This class is used to manage markers and caps
"""
import numpy as np
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

    def matchedSearch(self, pairs, candidate):
            """
            This search looks for whether two elements are already paired.
            """
            for item in pairs:
                if item[1] == candidate[1] or item[0] == candidate[0]:
                    return True
            return False

    def thoroughMatching(self, arr1, arr2):
        ans=[]
        for index, item in enumerate(arr1):
            if item == 0: continue
            pm = self.fullSearch(arr2, item)
            for item2 in pm:
                if arr2[item2] == 0: continue
                candidate = [index, item2]
                if len(ans) == 0: ans.append(candidate)
                if not self.matchedSearch(ans, candidate): ans.append(candidate)
            # print(ans)
        return ans

    def fullSearch(self, arr, data):
        """Returns a list of multiple values that can match with data"""
        if data == 0:
            threshold = 1
        else:
            threshold = 6
        ans = []
        for index, item in enumerate(arr):
            if self.colorMatch(item, data, threshold):
                ans.append(index)
        return ans

    def colorMatch(self, a, b, threshold):
        """
        Given two one d array of 3 elements this function will compare hsv values
        And return whether the two pieces of data are similar enough to match. 
        """
        return (abs(a-b) <= threshold)


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

    def swap(self, A, p, q):
        A[p], A[q] = A[q], A[p]
        return A

    def partition(self, A, p, r):
        x = A[r][0]
        i = p-1
        for j in range(p, r):
            if A[j][0] <= x:
                i += 1
                self.swap(A, i, j)
        self.swap(A, i+1, r)
        return i+1
    
    def QuickSort(self, A, p, r):
        if p < r:
            q = self.partition(A, p, r)
            self.QuickSort(A, p, q-1)
            self.QuickSort(A, q+1, r)

    def bind(self, A, B):
        def compress(B):
            return A[B[0]]
        def tie(A, B):
            # print(B)
            return [A, B]
        newA = list(map(compress, B))
        return list(map(tie, newA, B))

    def unbind(self, C):
        def untie(C):
            return C[1]
        return list(map(untie, C))

    def sort(self, A, B):
        if type(A) != type(list()):
            A = A.tolist()
        if type(B) != type(list()):
            B = B.tolist()
        C = self.bind(A, B)
        # print(C)
        self.QuickSort(C, 0, len(C)-1)
        # print(C)
        return self.unbind(C)

m = manage()

a = [78,27,122, 77, 11, 121, 25, 11, 100]

b = [[0,2],[1,8],[2,5],[3,3],[4,0],[5,7],[8,1]]

# print(a[b[0]])

c = m.sort(a, b)

print(c)

for i in c:
    print(a[i[0]])