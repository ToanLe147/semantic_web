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
cv2.createTrackbar("L-S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L-V", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("U-H", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("U-S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("U-V", "Trackbars", 0, 255, nothing)

test_path = os.path.realpath("src/test/hsv_test.png")
print(test_path)

while True:

    # frame = down_scale_image(cv2.imread(test_path), 3)
    frame = cv2.imread(test_path)
    print(frame)

    lh = cv2.getTrackbarPos("L-H", "Trackbars")  # 50
    ls = cv2.getTrackbarPos("L-S", "Trackbars")  # 110
    lv = cv2.getTrackbarPos("L-V", "Trackbars")  # 74
    us = cv2.getTrackbarPos("U-S", "Trackbars")  # 179
    uh = cv2.getTrackbarPos("U-H", "Trackbars")  # 255
    uv = cv2.getTrackbarPos("U-V", "Trackbars")  # 255

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    l_b = np.array([lh, ls, lv])
    u_b = np.array([uh, us, uv])

    mask = cv2.inRange(hsv, l_b, u_b)

    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)

    key = cv2.waitKey(1)

    if key == 27:
        break

cv2.destroyAllWindows()
