import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":
    file_name = "image.png"
    img = cv.imread(file_name)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    res = cv.bitwise_and(hsv,hsv,mask= mask)



    plt.imshow(cv.cvtColor(img, cv.COLOR_BGR2RGB))
    plt.show()