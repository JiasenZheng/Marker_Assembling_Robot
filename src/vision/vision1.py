import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

def detect_contour(img0,grid_size = (1,3),pixel_size = [475,125], starting_pixel = [125,200]):
    grid = np.zeros((grid_size[0],grid_size[1],3), np.uint8)
    img = cv.GaussianBlur(img0, (5, 5), 2)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    lower_thresh = np.array([0,100,0])
    upper_thresh = np.array([255,255,255])
    mask = cv.inRange(hsv, lower_thresh, upper_thresh)
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    x_interval = pixel_size[0]/grid_size[1]
    y_interval = pixel_size[1]/grid_size[0]
    cs = []
    for contour in contours:
        area = cv.contourArea(contour)
        if area > 500:
            M = cv.moments(contour)
            cv.drawContours(img0,contour,-1,(0,255,0),3)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            c = [cx,cy]
            c = np.subtract(c,starting_pixel)
            gx = int(c[0]//x_interval)
            gy = int(c[1]//y_interval)
            grid[gy,gx] = hsv[cy,cx]
            cs.append(c)
    cv.rectangle(img0,starting_pixel,np.add(starting_pixel,pixel_size),(0,0,255),3)

    return img0, grid

def detect_contour2(img0,grid_size = (1,3),pixel_size = [475,125], starting_pixel = [125,200]):
    list_H = []
    img0 = img0[starting_pixel[1]:starting_pixel[1]+pixel_size[1],starting_pixel[0]:starting_pixel[0]+pixel_size[0]]
    grid_len = grid_size[0]*grid_size[1]
    grid = np.zeros((grid_size[0],grid_size[1],3), np.uint8)
    img = cv.GaussianBlur(img0, (5, 5), 2)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    lower_thresh = np.array([0,100,0])
    upper_thresh = np.array([255,255,255])
    mask = cv.inRange(hsv, lower_thresh, upper_thresh)
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    x_interval = pixel_size[0]/grid_size[1]
    y_interval = pixel_size[1]/grid_size[0]
    cs = []
    for contour in contours:
        area = cv.contourArea(contour)
        if area > 600:
            M = cv.moments(contour)
            cv.drawContours(img0,contour,-1,(0,255,0),3)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            c = [cx,cy]
            # c = np.subtract(c,starting_pixel)
            gx = int(c[0]//x_interval)
            gy = int(c[1]//y_interval)
            grid[gy,gx] = hsv[cy,cx]
            cs.append(c)
    grid = grid.reshape(1,grid_len,3)[0]
    for hsv in grid:
        list_H.append(hsv[0])
    return img0, list_H

def detect(img0):
    img = cv.GaussianBlur(img0, (5, 5), 2)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    lower_thresh = np.array([0,100,0])
    upper_thresh = np.array([255,255,255])
    mask = cv.inRange(hsv, lower_thresh, upper_thresh)
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    if len(contours)!=0:
        for contour in contours:
            area = cv.contourArea(contour)
            if area > 500:
                cv.drawContours(img0,contour,-1,(0,255,0),3)

    return img0

## Utility helper function
def plot_image(filename):
    img = cv.imread(filename)
    plt.imshow(cv.cvtColor(img, cv.COLOR_BGR2RGB))
    plt.show()

def find_matching(markers,caps,ignores,error = 3):
    for a, marker in enumerate(markers):
        if (marker == 0) or (a in ignores): 
            continue
        for b, cap in enumerate(caps):
            if abs(marker - cap) <= error:
                return [a,b]
        ignores.append(a)
        return [None, None]
        
        


if __name__ == "__main__":
    # image = cv.imread('image.png')
    # image, list_h = detect_contour2(image)
    markers = [0, 77, 109, 10,0]
    caps = [0,109, 0, 10,0]
    ignores = []
    [a,b] = find_matching(markers, caps, ignores)
    print(a,b)
    # print(list_h)
    # plt.imshow(image)
    # plt.show()
