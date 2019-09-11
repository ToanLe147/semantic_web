#!/usr/bin/env python
import pyrealsense2 as rs
import numpy as np
import cv2
# import rospy

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Publishing Video
# pub = rospy.Publisher("video", )


def nothing(x):
    pass


cv2.namedWindow("Trackbars")
cv2.createTrackbar("L-H", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("L-S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L-V", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("U-H", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("U-S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("U-V", "Trackbars", 0, 255, nothing)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # colorizer = rs.colorizer()
        # depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))

        # Show images
        frame = depth_colormap
        frame = cv2.flip(frame, 1)

        lh = cv2.getTrackbarPos("L-H", "Trackbars")
        ls = cv2.getTrackbarPos("L-S", "Trackbars")
        lv = cv2.getTrackbarPos("L-V", "Trackbars")
        uh = cv2.getTrackbarPos("U-H", "Trackbars")
        us = cv2.getTrackbarPos("U-S", "Trackbars")
        uv = cv2.getTrackbarPos("U-V", "Trackbars")

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        l_b = np.array([lh, ls, lv])
        u_b = np.array([uh, us, uv])

        mask = cv2.inRange(hsv, l_b, u_b)

        cv2.imshow("frame", frame)
        cv2.imshow("mask", mask)
        cv2.imshow("depth", depth_image)

        key = cv2.waitKey(1)

        if key == 27:
            break

    cv2.destroyAllWindows()

finally:

    # Stop streaming
    pipeline.stop()
