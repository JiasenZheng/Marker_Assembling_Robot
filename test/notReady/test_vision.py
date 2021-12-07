

import unittest
import vision
import cv2 as cv
import numpy as np
import os
from os.path import dirname, abspath, join

from vision.vision1 import detect, detect_contour2

## Definining Lower and Upper Hue Bounds for color check; only checking "H" of HSV value
BABYBLUE = np.hstack((np.array([94]), np.array([106])))
PURPLE = np.hstack((np.array([110]), np.array([122])))
PINK = np.hstack((np.array([170]), np.array([182])))
YELLOW = np.hstack((np.array([21]), np.array([33])))
ORANGE = np.hstack((np.array([4]), np.array([16])))
GREEN = np.hstack((np.array([73]), np.array([85])))
EMPTY = np.hstack((np.array([0]), np.array([0])))

class VisionTestCase(unittest.TestCase):
    def __init__(self, *args):
        super(VisionTestCase, self).__init__(*args)
        self.img_dir = os.path.join(dirname(dirname(abspath(__file__))), 'src/vision/pictures/RepositionedPhotos/')
        
        ## Tests
        self.C1 = { "img": "caps1.png", 
                    "truth": (ORANGE, BABYBLUE, GREEN, EMPTY, ORANGE, ORANGE, EMPTY, ORANGE, GREEN)}
        self.C2 = { "img": "caps2.png", 
                    "truth": (GREEN, EMPTY, EMPTY, GREEN, YELLOW, ORANGE, ORANGE, ORANGE, BABYBLUE)}
        self.C3 = { "img": "caps3.png", 
                    "truth": (GREEN, BABYBLUE, ORANGE, GREEN, ORANGE, ORANGE, YELLOW, BABYBLUE, GREEN)}


        self.M1 = { "img": "marker1.png",
                    "truth":  (YELLOW, ORANGE, BABYBLUE, GREEN, GREEN, BABYBLUE, ORANGE, EMPTY, GREEN)}       
        self.M2 = { "img": "marker2.png",
                    "truth":  (YELLOW, ORANGE, BABYBLUE, GREEN, GREEN, BABYBLUE, EMPTY, EMPTY, EMPTY)}
        self.M3 = { "img": "marker3.png",
                    "truth":  (EMPTY, BABYBLUE, EMPTY, ORANGE, ORANGE, ORANGE, EMPTY, EMPTY, EMPTY)}


        self.A1 = {"img": "assembled1.png",
                    "truth": (BABYBLUE, GREEN, EMPTY, ORANGE, EMPTY, EMPTY, EMPTY, EMPTY, ORANGE)}
        self.A2 = {"img": "assembled2.png",
                    "truth":  (BABYBLUE, GREEN, YELLOW, ORANGE, BABYBLUE, GREEN, GREEN, EMPTY, ORANGE)}
                                                                                                
    
    def test_detectContours_A1(self):
        a1 = create(self.img_dir, self.A1,[550,390], [400, 150], np.array([6,30,60]), np.array([180,255,255]))
        self.assertTrue(a1)

    def test_detectContours_A2(self):
        a2 = create(self.img_dir, self.A2, [550,390], [400, 150], np.array([6,30,60]), np.array([180,255,255]))
        self.assertTrue(a2)    


    def test_detectContours_M1(self):
        m1 = create(self.img_dir,self.M1,[750,510], [200,100], np.array([6,30,60]), np.array([180,255,255]))
        self.assertTrue(m1)

    def test_detectContours_M2(self):
        m2 = create(self.img_dir,self.M2,[750,510], [200,100], np.array([6,30,60]), np.array([180,255,255]))
        self.assertTrue(m2)

    def test_detectContours_M3(self):
        m3 = create(self.img_dir,self.M3,[750,510], [200,100], np.array([6,30,60]), np.array([180,255,255]))
        self.assertTrue(m3)

    def test_detectContours_M4(self):
        m4 = create(self.img_dir, self.M4, [750,510], [200,100], np.array([6,30,60]), np.array([180,255,255]))
        self.assertTrue(m4)

    def test_detectContours_C1(self):
        c1 = create(self.img_dir, self.C1, [910,750], [180,0], np.array([0,90,0]),np.array([180,255,255]))
        self.assertTrue(c1)

    def test_detectContours_C2(self):
        c2 = create(self.img_dir, self.C2, [910,750], [180,0], np.array([0,90,0]),np.array([180,255,255]))
        self.assertTrue(c2)

    def test_detectContours_C3(self):
        c3 = create(self.img_dir, self.C3, [910,750], [180,0], np.array([0,90,0]),np.array([180,255,255]))
        self.assertTrue(c3)




def create(dir_path, test_dict, bb_start, bb_size, lower_threshold, upper_threshold):

    filepath = os.path.join(dir_path, test_dict["img"])
    # print(filepath)
    img = cv.imread(filepath)
    # print(img.shape)
    cv.imshow("testing", img)
    _, grid = detect_contour2(img, (3,3), bb_size, bb_start, lower_threshold, upper_threshold)
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