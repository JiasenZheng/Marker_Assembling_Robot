import unittest
import vision

import cv2 as cv
import numpy as np
import os
from os.path import dirname, abspath, join

from vision.vision1 import detect, detect_contour2

## Definining Lower and Upper Hue Bounds for color check; only checking "H" of HSV value
BABYBLUE = np.hstack((np.array([95]), np.array([105])))
PURPLE = np.hstack((np.array([111]), np.array([121])))
PINK = np.hstack((np.array([171]), np.array([181])))
YELLOW = np.hstack((np.array([22]), np.array([32])))
ORANGE = np.hstack((np.array([5]), np.array([15])))
GREEN = np.hstack((np.array([74]), np.array([84])))
EMPTY = np.hstack((np.array([0]), np.array([0])))

class VisionTestCase(unittest.TestCase):
    def __init__(self, *args):
        super(VisionTestCase, self).__init__(*args)
        self.img_dir = os.path.join(dirname(dirname(abspath(__file__))), 'src/vision/pictures/')
        
        ## Tests
        self.A1 = { "img": "assembled.png", 
                    "truth": (BABYBLUE, PURPLE, PINK, YELLOW, GREEN, PURPLE, YELLOW, PINK, ORANGE)}
        self.A2 = { "img": "assembled2.png", 
                    "truth": (BABYBLUE, GREEN, PINK, YELLOW, ORANGE, YELLOW, PURPLE, PINK, PURPLE)}
        self.A3 = { "img": "assembled3.png", 
                    "truth": (BABYBLUE, GREEN, PINK, YELLOW, ORANGE, PINK, YELLOW, PURPLE, PURPLE)}
        self.A4 = { "img": "assembled4.png", 
                    "truth": (BABYBLUE, GREEN, PINK, YELLOW, GREEN, PINK, YELLOW, PURPLE, PURPLE)}
        self.A5 = { "img": "assembled5.png", 
                    "truth": (BABYBLUE, YELLOW, PINK, YELLOW, EMPTY, GREEN, PURPLE, EMPTY, ORANGE)}


        self.M1 = { "img": "markers2.png",
                    "truth":  (BABYBLUE, ORANGE, GREEN, BABYBLUE,GREEN, PURPLE, PINK, YELLOW, YELLOW)}
        
        self.M2 = { "img": "markers3.png",
                "truth":  (BABYBLUE, ORANGE, PURPLE, PURPLE, ORANGE, YELLOW, YELLOW, GREEN, GREEN)}

        self.M3 = { "img": "markers4.png",
                "truth":  (BABYBLUE, ORANGE, YELLOW, PURPLE, ORANGE, GREEN, PURPLE, YELLOW, GREEN)}

        self.M4 = {"img": "markers5.png",
                    "truth": (BABYBLUE, ORANGE, YELLOW, PINK, ORANGE, PURPLE, GREEN, YELLOW, PURPLE)}
   


        self.C1 = {"img": "caps1.png",
                    "truth": (BABYBLUE, GREEN, BABYBLUE, ORANGE, YELLOW, PINK, YELLOW, GREEN, PURPLE)}

        self.C2 = {"img": "caps2.png",
                    "truth":  (YELLOW, PURPLE, PURPLE, PINK, GREEN, GREEN, PINK, BABYBLUE, ORANGE)}

        self.C3 = {"img": "caps3.png",
                    "truth": (YELLOW, PURPLE, PINK, PURPLE, PINK, GREEN, GREEN, BABYBLUE, ORANGE)}

        self.C4 = {"img": "caps4.png",
                    "truth": (YELLOW, PURPLE, PINK, PURPLE, ORANGE, YELLOW, GREEN, BABYBLUE, BABYBLUE)}
                                                                                                
    


    def test_detectContours_A1(self):
        a1 = create(self.img_dir, self.A1)
        self.assertTrue(a1)

    def test_detectContours_A2(self):
        a2 = create(self.img_dir, self.A2)
        self.assertTrue(a2)    

    def test_detectContours_A3(self):
        a3 = create(self.img_dir,self.A3)
        self.assertTrue(a3)

    def test_detectContours_A4(self):
        a4 = create(self.img_dir,self.A4)
        self.assertTrue(a4)

    def test_detectContours_A5(self):
        a5 = create(self.img_dir,self.A5)
        self.assertTrue(a5)

    def test_detectContours_M1(self):
        m1 = create(self.img_dir,self.M1)
        self.assertTrue(m1)

    def test_detectContours_M2(self):
        m2 = create(self.img_dir,self.M2)
        self.assertTrue(m2)

    def test_detectContours_M3(self):
        m3 = create(self.img_dir,self.M3)
        self.assertTrue(m3)

    def test_detectContours_M4(self):
        m4 = create(self.img_dir,self.M4)
        self.assertTrue(m4)

    def test_detectContours_C1(self):
        c1 = create(self.img_dir,self.C1)
        self.assertTrue(c1)

    def test_detectContours_C2(self):
        c2 = create(self.img_dir,self.C2)
        self.assertTrue(c2)

    def test_detectContours_C3(self):
        c3 = create(self.img_dir,self.C3)
        self.assertTrue(c3)

    def test_detectContours_C4(self):
        c4 = create(self.img_dir,self.C4)
        self.assertTrue(c4)  


def create(dir_path, test_dict):

    filepath = os.path.join(dir_path, test_dict["img"])
    img = cv.imread(filepath)
    _, grid = detect_contour2(img, (3,3), [990,600],[0,0])
    truth_arr = np.vstack(test_dict["truth"])
    colorsCheck = inRange(np.array(grid), truth_arr[:, 1], truth_arr[:, 0])
    print("grid: ", grid)
    print("truth_arr: ", truth_arr)
    print("colorsCheck", colorsCheck)
    return np.all(colorsCheck)

def inRange(src, upper, lower):
    return np.logical_and(np.greater_equal(src, lower), np.less_equal(src, upper))

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(vision, "Vision Pkg Test", VisionTestCase)