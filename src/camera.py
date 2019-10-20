#!/usr/bin/env python
import cv2
import numpy as np
from collections import OrderedDict
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from camera_3D import Segmentor
import tf
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from uploader import Ontology


Scene_3D = Segmentor()
KnowledgeBase = Ontology()


class Camera:
    def __init__(self):
        self.image_input = rospy.Subscriber("/camera/rgb/image_color",
                                            Image,
                                            self.callback)
        self.image_input = rospy.Subscriber("detect_image", String, self.trigger)
        self.bridge = CvBridge()
        self.scene = []
        self.tf_handler = tf.TransformListener()
        self.previous_scene = OrderedDict()
        self.detected = OrderedDict()
        self.update_trigger = 0
        self.image_trigger = 0  # Added for testing

    def trigger(self, msg):
        self.image_trigger = int(msg.data)
        return self.image_trigger

    def callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.image_trigger == 1:
                self.img = img
                self.detect()
                # print(self.detected)
                self.scan()
                self.visual()
                # for i in list(self.scene.keys()):
                #     print("{}: {}".format(i, self.scene[i].keys()))
                # print("==================")
            else:
                # print("Scanning")
                cv2.destroyAllWindows()
            # print(self.image_trigger)
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
        if not contour:
            self.detected = OrderedDict()
        for i in contour:
            area = cv2.contourArea(i)
            approx = cv2.approxPolyDP(i, 0.01 * cv2.arcLength(i, True), True)
            # print(len(approx))
            # print("===")
            if area > 400:
                cv2.drawContours(self.img, [approx], 0, (0, 0, 0), 2)
                if len(approx) == 3:
                    value = self.update_detected_shape(approx, 3)
                    centroid = self.get_center_point(value)
                    self.update_name("Triangle", value, centroid)
                if len(approx) == 4:
                    value = self.update_detected_shape(approx, 4)
                    centroid = self.get_center_point(value)
                    self.update_name("Rectangle", value, centroid)
                if len(approx) == 5:
                    value = self.update_detected_shape(approx, 5)
                    centroid = self.get_center_point(value)
                    self.update_name("Pentagon", value, centroid)

    def visual(self):  # for testing
        cv2.imshow("Frame", self.img)

        # cv2.imshow("Mask", self.mask)

        key = cv2.waitKey(1)
        if key == 27:
            cv2.destroyAllWindows()

    def scan(self):
        if not self.scene and self.detected:
            self.previous_scene.update(self.detected)
            self.scene = self.detected.items()
            KnowledgeBase.update_property("Kinect", "Current_state", self.scene)
            print("Update Initial Scene")
        elif len(self.previous_scene) < len(self.detected):
            # print("===============")
            # Update previous scene for next scan
            self.previous_scene = self.detected
            # Update scene for query
            self.scene = self.detected.items()
            KnowledgeBase.update_property("Kinect", "Current_state", self.scene)
            print("Update New Object Scene")
        elif not self.detected:
            self.previous_scene = self.detected
            self.scene = self.detected.items()
            KnowledgeBase.update_property("Kinect", "Current_state", self.scene)
            print("No Object Scene")
        else:
            # print("*****")
            self.update_trigger = 0

    def update_name(self, name, value, centroid):
        index = -1  # add index number Zero to sync with Gazebo experiment
        name_list = list(self.detected.keys())
        # Check the name and value of detected shape in current list
        if not self.detected:
            self.detected[name] = {"Boundary": value, "Centroid": centroid}
        else:
            while name in name_list:
                if value == self.detected[name]["Boundary"]:
                    # This shape is already added
                    return
                else:
                    # Update name of new detected
                    index = index + 1
                    orgi = name
                    if "_" in name:
                        orgi, _ = name.split("_")
                    name = orgi + "_" + str(index)

            # Add new detected shape
            self.detected[name] = {"Boundary": value, "Centroid": centroid}

    @staticmethod
    def update_detected_shape(approx_coordinates, number_of_corner):
        '''
        This function transform data from opencv image coordinates to simple
        list, which is easy to use for later callback function
        '''
        result = []
        for i in range(number_of_corner):
            point = list(approx_coordinates[i][0])
            result.append(point)
        return result

    def get_center_point(self, list_of_points):
        # Get size of image frame
        h, w, _ = self.img.shape
        # Get pointcloud data from pixel coordinates of image frame
        points_list = Scene_3D.cloud_list
        x = [p[0] for p in list_of_points]
        y = [p[1] for p in list_of_points]
        centroid = [sum(x) / len(list_of_points), sum(y) / len(list_of_points)]
        pcl_index = (centroid[1] * w) + centroid[0]
        pcl_point = list(points_list[pcl_index])
        # Convert pointcloud data from camera_link perspective to robot perspective
        center_point = self.transform_point(pcl_point, "camera_link", "base_link")
        return center_point

    def transform_point(self, point, src_frame, target_frame):
        # Initial PointStamped point
        pt = PointStamped()
        pt.header.stamp = rospy.Time.now()
        pt.header.frame_id = src_frame
        pt.point.x = point[0]
        pt.point.y = point[1]
        pt.point.z = point[2]
        # Transform
        self.tf_handler.waitForTransform(src_frame, target_frame, rospy.Time.now(), rospy.Duration(0.5))
        rs_point = self.tf_handler.transformPoint(target_frame, pt)
        # Convert PointStamped result to list
        result = [rs_point.point.x, rs_point.point.y, rs_point.point.z]
        return result


def main():
    rospy.init_node("shape_detection", anonymous=True)
    kinect = Camera()

    rospy.spin()


if __name__ == '__main__':
    main()
