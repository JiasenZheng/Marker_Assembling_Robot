"""
This class is used to manage markers and caps
"""
import numpy as np
from copy import deepcopy

class manage:
    def __init__(self):
        self.ans = []

    def matchedSearch(self, pairs, candidate):
            """
            Function:matchedSearch: Takes a list of pairs and a candidate and returns true if either element of
            the candidate exists in the list of pairs.
            Param:pairs: List of pairs
            Param:candidates: Candidate to be added to list of pairs.
            """
            for item in pairs:
                if item[1] == candidate[1] or item[0] == candidate[0]:
                    return True
            return False

    def thoroughMatching(self, arr1, arr2):
        """
        Function:thoroughMatching: Matches items of two arrays together.
        Param:arr1: Array of items to be matched to arr2
        Param:arr2: Array of items to be matched to arr1
        Return:ans: List matching pairs indices from arr1 and arr2
        """
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
        """
        Function:fullSearch: Returns a list of multiple values that can match with data
        Param:arr: An array of values to search through.
        Param:data: The item we are searching for in the array.
        Return:ans: List of index values that links to data that is similar to item being searched for.
        """
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
        Given two h values and a threshold, return true if the difference of the two h values is
        less than or equal to the threshold.
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
        """
        Helper function that swaps two elements of an array.
        """
        A[p], A[q] = A[q], A[p]
        return A

    def partition(self, A, p, r):
        """
        Helper function that partitions for QuickSort.
        """
        x = A[r][0]
        i = p-1
        for j in range(p, r):
            if A[j][0] <= x:
                i += 1
                self.swap(A, i, j)
        self.swap(A, i+1, r)
        return i+1
    
    def QuickSort(self, A, p, r):
        """Standard QuickSort Algorithm"""
        if p < r:
            q = self.partition(A, p, r)
            self.QuickSort(A, p, q-1)
            self.QuickSort(A, q+1, r)

    def bind(self, A, B):
        """
        Function:Bind: Takes a list of items with its match list and creates a new list where
        the items of the list include an a list containing the item in the first index
        and the second element is its corresponding matched pair.
        Param:A: List of items
        Param:B: List of matching pairs
        Returns: List of elements that follow the format [A, B]
        """
        def compress(B):
            return A[B[0]]
        def tie(A, B):
            # print(B)
            return [A, B]
        newA = list(map(compress, B))
        return list(map(tie, newA, B))

    def unbind(self, C):
        """
        Function:unbind: Unbinds a binded list.
        Param:C: A binded list.
        Returns: List from the B part of the Binded list
        """
        def untie(C):
            return C[1]
        return list(map(untie, C))

    def sort(self, A, B):
        """
        function:sort: Binds two lists Sorts the list according to the values of the A list
        And returns a sorted version of the B List.
        Param:A: List of values to sort by.
        Param:B: List of values to be sorted.
        Returns: A sorted version of list B.
        """
        # if type(A) != type(list()):
        #     A = A.tolist()
        # if type(B) != type(list()):
        #     B = B.tolist()
        C = self.bind(A, B)
        # print(C)
        self.QuickSort(C, 0, len(C)-1)
        # print(C)
        return self.unbind(C)

# m = manage()

# a = [78,27,122, 77, 11, 121, 25, 11, 100]

# b = [[0,2],[1,8],[2,5],[3,3],[4,0],[5,7],[8,1]]

# # print(a[b[0]])

# c = m.sort(a, b)

# print(c)

# for i in c:
#     print(a[i[0]])