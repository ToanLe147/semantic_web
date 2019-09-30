#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Camera:
    def __init__(self):
        self.image_input = rospy.Subscriber("/camera/rgb/image_color", Image,
        self.callback)
        self.image_output = rospy.Publisher("/detected_objects", Image,
        queue_size=10)
        self.bridge = CvBridge()

    def callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.detect()
        except CvBridgeError as e:
            print(e)

    def detect(self):
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        lb = np.array([50, 110, 74])
        ub = np.array([179, 255, 255])

        mask = cv2.inRange(hsv, lb, ub)

        # Contour detetcion
        _, contour, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for i in contour:
            area = cv2.contourArea(i)
            approx = cv2.approxPolyDP(i, 0.03 * cv2.arcLength(i, True), True)

            if area > 400:
                cv2.drawContours(self.img, [approx], 0, (0, 0, 0), 3)
                if len(approx) == 3:
                    print("Triangle area {}".format(area))
                if len(approx) == 4:
                    print("Rectangle area {}".format(area))
                if len(approx) == 5:
                    print("Pentagon area {}".format(area))

        self.visual(mask)

    def visual(self, mask):
        cv2.imshow("Frame", self.img)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1)
        if key == 27:
            cv2.destroyAllWindows()

    def observe(self):
        print()


def main():
    kinect = Camera()
    rospy.init_node("detection_hsv", anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    main()
