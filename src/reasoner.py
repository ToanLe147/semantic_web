#!/usr/bin/env python
import roslibpy
from uploader import Ontology


class Reasoner:
    def __init__(self):
        # Connect with ROS
        self.client = roslibpy.Ros(host='localhost', port=9090)
        self.client.run()

        # ROS topic to control the system
        self.camera_scan = roslibpy.Topic(self.client, 'detect_image', 'std_msgs/String')
        self.robot_move = roslibpy.Topic(self.client, 'target_pose', 'geometry_msgs/Pose')
        self.gripper_grasp = roslibpy.Topic(self.client, 'gripper_grasping', 'std_msgs/String')
        self.camera_scan.advertise()
        self.robot_move.advertise()
        self.gripper_grasp.advertise()

        # Uploading data
        self.uploader = Ontology()

    def move(self):
        print
