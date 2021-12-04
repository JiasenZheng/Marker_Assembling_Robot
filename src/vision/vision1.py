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
    # cv.rectangle(img0,starting_pixel,np.add(starting_pixel,pixel_size),(0,0,255),3)
    grid = grid.reshape(1,grid_len,3)[0]

    return img0, grid

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

if __name__ == "__main__":

    image = cv.imread('image.png')
    img0, grid = detect_contour2(image)
    print("image.png: ", grid)

    ## assembled
    image1 = cv.imread('pictures/assembled.png')
    img1, grid1 = detect_contour2(image1,(3,3), [450, 300], [110, 64])
    print("assembled: ", grid1)

    # ## assembled
    # image1 = cv.imread('pictures/assembled2.png')
    # img1, grid1 = detect_contour2(image1,(3,3), [450, 300], [110, 64])
    # print("assembled: ", grid1)

    # ## assembled
    # image1 = cv.imread('pictures/assembled3.png')
    # img1, grid1 = detect_contour2(image1,(3,3), [450, 300], [110, 64])
    # print("assembled: ", grid1)

    # ## assembled
    # image1 = cv.imread('pictures/assembled4.png')
    # img1, grid1 = detect_contour2(image1,(3,3), [450, 300], [110, 64])
    # print("assembled: ", grid1)

    # ## assembled
    # image1 = cv.imread('pictures/assembled5.png')
    # img1, grid1 = detect_contour2(image1,(3,3), [450, 300], [110, 64])
    # print("assembled: ", grid1)
    

    ## caps1
    image2 = cv.imread('pictures/caps1.png')
    img2, grid2 = detect_contour2(image2,(3,3), [640, 480], [0, 0])
    print("caps1: ", grid2)

    # ## caps1
    # image2 = cv.imread('pictures/caps2.png')
    # img2, grid2 = detect_contour2(image2,(3,3), [640, 480], [0, 0])
    # print("caps1: ", grid2)

    # ## caps1
    # image2 = cv.imread('pictures/caps3.png')
    # img2, grid2 = detect_contour2(image2,(3,3), [640, 480], [0, 0])
    # print("caps1: ", grid2)

    # ## caps1
    # image2 = cv.imread('pictures/caps4.png')
    # img2, grid2 = detect_contour2(image2,(3,3), [640, 480], [0, 0])
    # print("caps1: ", grid2)



    ## markers2
    image3 = cv.imread('pictures/markers2.png')
    img3, grid3 = detect_contour2(image3,(3,3), [580, 450], [0, 0])
    print("markers2: ", grid3)

    image3 = cv.imread('pictures/markers3.png')
    img3, grid3 = detect_contour2(image3,(3,3), [580, 450], [0, 0])
    print("markers2: ", grid3)

    image3 = cv.imread('pictures/markers4.png')
    img3, grid3 = detect_contour2(image3,(3,3), [580, 450], [0, 0])
    print("markers2: ", grid3)

    image3 = cv.imread('pictures/markers5.png')
    img3, grid3 = detect_contour2(image3,(3,3), [580, 450], [0, 0])
    print("markers2: ", grid3)

    ## 