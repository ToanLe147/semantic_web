#!/usr/bin/env python
import cv2
import numpy as np
import os


def nothing(x):
    pass


# def down_scale_image(image_input, times):
#     layer = image_input.copy()
#     gaussian_pyramid = [layer]
#     for i in range(times + 1):
#         layer = cv2.pyrDown(layer)
#         gaussian_pyramid.append(layer)
#     return gaussian_pyramid[times]


cv2.namedWindow("Trackbars")
cv2.createTrackbar("L-H", "Trackbars", 0, 179, nothing)
# cv2.createTrackbar("L-S", "Trackbars", 0, 255, nothing)
# cv2.createTrackbar("L-V", "Trackbars", 0, 255, nothing)
# cv2.createTrackbar("U-H", "Trackbars", 0, 179, nothing)
# cv2.createTrackbar("U-S", "Trackbars", 0, 255, nothing)
# cv2.createTrackbar("U-V", "Trackbars", 0, 255, nothing)

test_path = os.path.realpath("src/test/hsv_test.png")
# print(test_path)

while True:

    # frame = down_scale_image(cv2.imread(test_path), 3)
    frame = cv2.imread(test_path)
    # print(frame)

    lh = cv2.getTrackbarPos("L-H", "Trackbars")  # 50
    # ls = cv2.getTrackbarPos("L-S", "Trackbars")  # 110
    # lv = cv2.getTrackbarPos("L-V", "Trackbars")  # 74
    # us = cv2.getTrackbarPos("U-S", "Trackbars")  # 179
    # uh = cv2.getTrackbarPos("U-H", "Trackbars")  # 255
    # uv = cv2.getTrackbarPos("U-V", "Trackbars")  # 255

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # define range of blue color in HSV
    lower_blue = np.array([110, 50, 50])
    upper_blue = np.array([130, 255, 255])
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # imgray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # ret, mask = cv2.threshold(imgray, lh, 255, 0)

    _, contour, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for i in contour:
        approx = cv2.approxPolyDP(i, 0.01*cv2.arcLength(i, True), True)
        print(len(approx))
        if len(approx) == 3:
            cv2.drawContours(frame, [i], 0, (0, 255, 0), 3)

    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)

    key = cv2.waitKey(1)

    if key == 27:
        break

cv2.destroyAllWindows()
