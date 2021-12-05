import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

def detect_contour(img0,grid_size = (1,3),pixel_size = [475,125], starting_pixel = [125,200]):
    grid = np.zeros((grid_size[0],grid_size[1],3), np.uint8)
    img = cv.GaussianBlur(img0, (5, 5), 2)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    lower_thresh = np.array([0,100,20])
    upper_thresh = np.array([180,255,255])
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
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    lower_thresh = np.array([6,100,20])
    upper_thresh = np.array([180,255,255])
    mask = cv.inRange(img_hsv, lower_thresh, upper_thresh)
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
            # c = np.subtract(c,starting_pixel)
            gx = int(c[0]//x_interval)
            gy = int(c[1]//y_interval)
            if gx == 1 and gy == 1:
                cx+=10
                cy+=10
            grid[gy,gx] = img_hsv[cy,cx]
            cs.append(c)
    grid = grid.reshape(1,grid_len,3)[0]
    for hsv in grid:
        list_H.append(hsv[0])
    list_H.reverse()
    return img0, list_H

def detect(img0):
    img = cv.GaussianBlur(img0, (5, 5), 2)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    lower_thresh = np.array([0,100,20])
    upper_thresh = np.array([180,255,255])
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

# def find_matching(markers,caps,ignores,error = 3):
#     for a, marker in enumerate(markers):
#         if (marker == 0) or (a in ignores): 
#             continue
#         for b, cap in enumerate(caps):
#             if abs(marker - cap) <= error:
#                 return [a,b],ignores
#         ignores.append(a)
#         return [None, None],ignores
        
        

    ## assembled
    image1 = cv.imread('pictures/assembled.png')
    img1, grid1 = detect_contour2(image1,(3,3), [450, 300], [110, 64])
    print("assembled: ", grid1)
    
    ## caps1
    image2 = cv.imread('pictures/caps1.png')
    img2, grid2 = detect_contour2(image2,(3,3), [640, 480], [0, 0])
    print("caps1: ", grid2)

    ## markers2
    image3 = cv.imread('pictures/markers2.png')
    img3, grid3 = detect_contour2(image3,(3,3), [580, 450], [0, 0])
    print("markers2: ", grid3)


if __name__ == "__main__":
<<<<<<< HEAD
    image = cv.imread('/home/jason/ros/fpws/src/final-project-group-4-inc/src/vision/pictures/assembled3.png')
    image, list_H = detect_contour2(image,grid_size=(3,3),pixel_size = (650,520), starting_pixel = (330,80))

    print(list_H)
    plt.imshow(cv.cvtColor(image, cv.COLOR_BGR2RGB))
    plt.show()
=======
    # image = cv.imread('/home/jason/ros/fpws/src/final-project-group-4-inc/src/vision/pictures/markers1.png')
    # image, list_H = detect_contour2(image,grid_size=(3,3),pixel_size=(830,580),starting_pixel=(150,50))
    # markers = [0, 77, 109, 10,0]
    # caps = [0,109, 0, 10,0]
    # ignores = []
    # [a,b] = find_matching(markers, caps, ignores)
    # print(a,b)
    # print(list_H)
    # plt.imshow(cv.cvtColor(image, cv.COLOR_BGR2RGB))
    # plt.show()
>>>>>>> fa36e3243050e5dc7e2f67042cde6a0ff8c2d823


    # # Example runs of detect_contour2
    # ## assembled
    # image1 = cv.imread('pictures/assembled.png')
    # img1, grid1 = detect_contour2(image1,(3,3), [650, 520], [330, 80])
    # print("assembled: ", grid1)

    ## assembled1
    image1 = cv.imread('pictures/assembled1.png')
    img1, grid1 = detect_contour2(image1,(3,3), [650, 520], [330, 80])
    print("assembled: ", grid1)

    ## assembled2
    image1 = cv.imread('pictures/assembled2.png')
    img1, grid1 = detect_contour2(image1,(3,3), [650, 520], [330, 80])
    print("assembled: ", grid1)

    ## assembled3
    image1 = cv.imread('pictures/assembled3.png')
    img1, grid1 = detect_contour2(image1,(3,3), [650, 520], [330, 80])
    print("assembled: ", grid1)

    ## assembled4
    image1 = cv.imread('pictures/assembled4.png')
    img1, grid1 = detect_contour2(image1,(3,3), [650, 520], [330, 80])
    print("assembled: ", grid1)
    
    ## caps1
    image2 = cv.imread('pictures/caps1.png')
    img2, grid2 = detect_contour2(image2,(3,3), [910,650], [180,0])
    print("caps1: ", grid2)

    ## caps2
    image2 = cv.imread('pictures/caps2.png')
    img2, grid2 = detect_contour2(image2,(3,3), [910,650], [180,0])
    print("caps1: ", grid2)

    ## caps3
    image2 = cv.imread('pictures/caps3.png')
    img2, grid2 = detect_contour2(image2,(3,3), [910,650], [180,0])
    print("caps1: ", grid2)

    ## caps4
    image2 = cv.imread('pictures/caps4.png')
    img2, grid2 = detect_contour2(image2,(3,3), [910,650], [180,0])
    print("caps1: ", grid2)

    ## markers1
    image3 = cv.imread('pictures/markers1.png')
    img3, grid3 = detect_contour2(image3,(3,3), [830,580], [150,50])
    print("markers2: ", grid3)

    ## markers2
    image3 = cv.imread('pictures/markers2.png')
    img3, grid3 = detect_contour2(image3,(3,3), [830,580], [150,50])
    print("markers2: ", grid3)

    ## markers3
    image3 = cv.imread('pictures/markers3.png')
    img3, grid3 = detect_contour2(image3,(3,3), [830,580], [150,50])
    print("markers2: ", grid3)

    ## markers4
    image3 = cv.imread('pictures/markers4.png')
    img3, grid3 = detect_contour2(image3,(3,3), [830,580], [150,50])
    print("markers2: ", grid3)
