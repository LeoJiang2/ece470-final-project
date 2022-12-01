import cv2
import numpy as np

theta = 0
beta = 768.5
tx = 0.288
ty = 0.1055

#params for magic numbers
Or = 240
Oc = 320

# Function that converts image coord to world coord
# Note: input x corresponds to columns in the image, input y is rows in the image
def IMG2W(x,y):
    Xc = ( y - Or )/beta
    Yc = ( x - Oc )/beta

    Xc_prime = Xc * np.cos(theta) - Yc * np.sin(theta)
    Yc_prime = Xc * np.sin(theta) + Yc * np.cos(theta)

    Xw = tx + Xc_prime
    Yw = ty + Yc_prime

    return [Xw, Yw]

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = False
    params.minArea = 100

    # Filter by Circularity
    params.filterByCircularity = False
    #params.minCircularity = 0.5

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================
    if color == "white":
        lower = (0,0,175)     
        upper = (5,5,255)  

    elif color == "purple":
        lower = (125,50,50)   
        upper = (145, 255, 240) 

    elif color == "red":
        lower = (0, 100, 20)
        upper = (10, 255, 255)

    elif color == "green":
        lower = (120, 50, 50)
        upper = (180, 255, 255)

    elif color == "yellow":
        lower = (22, 50, 50)
        upper = (45, 255, 240)

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, outImage=np.array([]), color=(10, 255, 255))

    xw_yw = []

    if(num_blobs == 0):
        #print("No block found!")
        f = 0 # do nothing
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))
            # print((blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)
    #print("pp poopoo check")
    # print(keypoints)
    return xw_yw