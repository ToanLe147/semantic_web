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
        self.bridge = CvBridge()
        self.scene = {}
        self.previous_scene = {}
        self.detected = {}

    def callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.detect()
            self.scan()
            # self.visual()
            print(self.scene.keys())
        except CvBridgeError as e:
            print(e)

    def detect(self):
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        lb = np.array([50, 110, 74])
        ub = np.array([179, 255, 255])

        mask = cv2.inRange(hsv, lb, ub)
        # self.mask = mask

        # Contour detetcion
        _, contour, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for i in contour:
            area = cv2.contourArea(i)
            approx = cv2.approxPolyDP(i, 0.01 * cv2.arcLength(i, True), True)
            # print(len(approx))
            # print("===")
            if area > 400:
                cv2.drawContours(self.img, [approx], 0, (0, 0, 0), 2)
                if len(approx) == 3:
                    value = self.update_detected_shape(approx, 3)
                    self.update_name("Triangle", value)
                if len(approx) == 4:
                    value = self.update_detected_shape(approx, 4)
                    self.update_name("Rectangle", value)
                if len(approx) == 5:
                    value = self.update_detected_shape(approx, 5)
                    self.update_name("Pentagon", value)

    def visual(self):
        cv2.imshow("Frame", self.img)
        # cv2.imshow("Mask", self.mask)

        key = cv2.waitKey(1)
        if key == 27:
            cv2.destroyAllWindows()

    def scan(self):
        if not self.scene:
            order = len(self.scene) + 1
            self.scene["Scene_{}".format(order)] = self.detected
            self.previous_scene.update(self.detected)
            print("Update Initial Scene")
        elif len(self.previous_scene) < len(self.detected):
            print("===============")
            order = len(self.scene) + 1  # Update order of scene
            new_detected = self.detected
            map(new_detected.pop, self.previous_scene)  # Seperate new object
            self.previous_scene.update(new_detected)  # Update previous scene for next scan
            # Update scene to query later
            self.scene["Scene_{}".format(order)] = new_detected
            print("Update New Object Scene")
        else:
            print("*****")
            return

    def update_name(self, name, value):
        index = 0
        name_list = list(self.detected.keys())
        # Check the name and value of detected shape in current list
        if not self.detected:
            self.detected[name] = value
        else:
            while name in name_list:
                if value == self.detected[name]:
                    # This shape is already added
                    return
                else:
                    index = index + 1
                    orgi = name
                    if "_" in name:
                        orgi, _ = name.split("_")
                    name = orgi + "_" + str(index)

            # Add new detected shape
            self.detected[name] = value

    @staticmethod
    def update_detected_shape(approx_coordinates, number_of_corner):
        result = []
        for i in range(number_of_corner):
            point = list(approx_coordinates[i][0])
            result.append(point)
        return result


def main():
    kinect = Camera()
    rospy.init_node("detection_hsv", anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    main()
