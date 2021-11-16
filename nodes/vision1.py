import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":
    file_name = "image.png"
    img0 = cv.imread(file_name)
    img = cv.GaussianBlur(img0, (5, 5), 2)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    lower_thresh = np.array([0,100,0])
    upper_thresh = np.array([255,255,255])
    mask = cv.inRange(hsv, lower_thresh, upper_thresh)
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

    cs = []
    for contour in contours:
        area = cv.contourArea(contour)
        if area >500:
            M = cv.moments(contour)
            cv.drawContours(img0,contour,-1,(0,255,0),3)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            c = (cx,cy)
            cv.circle(img0, c, 3 , (0,0,0))
            cv.putText(img0, "({},{})".format(c[0],c[1]),(c[0]-40,c[1]-20),cv.FONT_HERSHEY_PLAIN, 1, (0,0,0),2)

            cs.append(c)
    


    # plt.imshow(mask)
    figure = plt.figure()
    subplot1 = figure.add_subplot(1,2,1)
    subplot2 = figure.add_subplot(1,2,2)

    subplot1.imshow(cv.cvtColor(mask, cv.COLOR_BGR2RGB))
    subplot2.imshow(cv.cvtColor(img0, cv.COLOR_BGR2RGB))
    plt.show()