#!/usr/bin/env python
import roslibpy
from uploader import Ontology
from collections import OrderedDict

KnowledgeBase = Ontology()


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

    def check_position(self, list1, list2):
        '''
        This function checks if centroid of detected shape is correct
        '''
        pose_dif = [abs(list1[0] - list2[0]),
                    abs(list1[1] - list2[1]),
                    abs(list1[2] - list2[2])]
        result = all(i < 0.02 for i in pose_dif)
        return result

    def generate_task(self):
        self.task = OrderedDict()
        instance = "DemonstrationLearning_Task"
        scene = OrderedDict(eval(KnowledgeBase.get_property(instance,
                                                            "Current_state")))
        target = OrderedDict(eval(KnowledgeBase.get_property(instance, "Data")))

        # Compare two scene (Targer vs Current Scene)
        if len(scene.keys()) != 0:
            for shape in scene.keys():
                if shape in target.keys():
                    # Check position of shape
                    if self.check_position(scene[shape]["Centroid"],
                                           target[shape]["Centroid"]):
                        self.task[shape] = {"task": "Available {}".format(shape),
                                            "color": "success"}
                        target.pop(shape)
                    else:
                        self.task[shape] = {"task": "Modify {}".format(shape),
                                            "color": "warning"}
                        target.pop(shape)
                else:
                    self.task[shape] = {"task": "Remove {}".format(shape),
                                        "color": "danger"}

        for shape in target.keys():
            self.task[shape] = {"task": "Add {}".format(shape),
                                "color": "primary"}

        # Update Ontology
        KnowledgeBase.update_property(instance, "Status", str(self.task))
        return self.task

    def perform_task(self):
        print

    def robot_move(self, position):
        print

    def gripper_grasp(self, command):
        # "command" parameter is a string of object name to pick object or Zero
        # to place object
        print

    def camera_scan(self):
        print

    def add(self):
        print

    def remove(self):
        print

    def modify(self):
        print
