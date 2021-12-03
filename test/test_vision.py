import unittest
import vision

import cv2 as cv
import numpy as np
import os
from os.path import dirname, abspath, join

from vision.vision1 import detect, detect_contour2

## Definining Lower and Upper HSV Bounds for color check
BABYBLUE = np.hstack((np.array([]), np.array([])))
PURPLE = np.hstack((np.array([]), np.array([])))
PINK = np.hstack((np.array([]), np.array([])))
YELLOW = np.hstack((np.array([]), np.array([])))
ORANGE = np.hstack((np.array([]), np.array([])))
GREEN = np.hstack((np.array([]), np.array([])))
EMPTY = np.hstack((np.array([]), np.array([])))

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
                    "truth": (BABYBLUE, YELLOW, PINK, YELLOW, EMPTY, GREEN, PURPLE, YELLOW, ORANGE)}


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

        self.C4 = {"img": "cap4.png",
                    "truth": (YELLOW, PURPLE, PINK, PURPLE, ORANGE, YELLOW, GREEN, BABYBLUE, BABYBLUE)}
                                                                                                
    
    def create_test(self, test_dict):

        filepath = os.path.join(self.img_dir, test_dict["img"])
        img = cv.imread(filepath)
        _, grid = detect_contour2(img, (3,3), [640,480],[0,0])
        truth_arr = np.vstack(test_dict["truth"])
        print("grid: ", grid)
        print("truth_arr: ", grid)
        colorsCheck = cv.inRange(grid.T, truth_arr[:, :3], truth_arr[:, 3:])
        print("colorsCheck", colorsCheck)
        return np.all(colorsCheck)


    def test_detectContours_A1(self):
        a1 = self.create_test(self.A1)
        self.assertTrue(a1)

    def test_detectContours_A2(self):
        a2 = self.create_test(self.A2)
        self.assertTrue(a2)    

    def test_detectContours_A3(self):
        a3 = self.create_test(self.A3)
        self.assertTrue(a3)

    def test_detectContours_A4(self):
        a4 = self.create_test(self.A4)
        self.assertTrue(a4)

    def test_detectContours_A5(self):
        a5 = self.create_test(self.A5)
        self.assertTrue(a5)



    def test_detectContours_M1(self):
        m1 = self.create_test(self.M1)
        self.assertTrue(m1)

    def test_detectContours_M2(self):
        m2 = self.create_test(self.M2)
        self.assertTrue(m2)

    def test_detectContours_M3(self):
        m3 = self.create_test(self.M3)
        self.assertTrue(m3)

    def test_detectContours_M4(self):
        m4 = self.create_test(self.M4)
        self.assertTrue(m4)



    def test_detectContours_C1(self):
        c1 = self.create_test(self.C1)
        self.assertTrue(c1)

    def test_detectContours_C2(self):
        c2 = self.create_test(self.C2)
        self.assertTrue(c2)

    def test_detectContours_C3(self):
        c3 = self.create_test(self.C3)
        self.assertTrue(c3)

    def test_detectContours_C4(self):
        c4 = self.create_test(self.C4)
        self.assertTrue(c4)     

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(vision, "Vision Pkg Test", VisionTestCase)

        