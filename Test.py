import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob

cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
cv2.namedWindow("Binary", cv2.WINDOW_NORMAL)
cv2.namedWindow("Binary2", cv2.WINDOW_NORMAL)

frames_folder = 'videos/Lab/LapC1/frames/'
# loop over the frames with glob
for frame in glob.glob(frames_folder + '*.png'):
    image = cv2.imread(frame)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
    thresh2 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

    # show the frame and the binary image
    cv2.imshow("Frame", image)
    cv2.imshow("Binary", thresh)
    cv2.imshow("Binary2", thresh2)
    key = cv2.waitKey(0) & 0xFF
    print(frame)
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break