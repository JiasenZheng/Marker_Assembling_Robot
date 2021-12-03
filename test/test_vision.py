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
                                       
    def create_test(self, test_dict):

        filepath = os.path.join(self.img_dir, test_dict["img"])
        img = cv.imread(filepath)
        _, grid = detect_contour2(img)
        truth_arr = np.vstack(test_dict["truth"])
        print("grid: ", grid)
        print("truth_arr: ", grid)
        colorsCheck = cv.inRange(grid.T, truth_arr[:, :3], truth_arr[:, 3:])
        print("colorsCheck", colorsCheck)
        return np.all(colorsCheck)


    def test_detectContours_A1(self):
        a1 = self.create_test(self.A1)
        self.assertTrue(a1)

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(vision, "Vision Pkg Test", VisionTestCase)

        