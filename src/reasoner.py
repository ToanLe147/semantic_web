#!/usr/bin/env python
import roslibpy
from uploader import Ontology
from collections import OrderedDict
import rospkg
from tf.transformations import quaternion_from_euler
import os
import time

KnowledgeBase = Ontology()
rospack = rospkg.RosPack()


class GazeboSyncing:
    def __init__(self):
        print("***")

    def check_name(self, shape):
        # Sync with Gazebo environement
        gazebo_list = eval(KnowledgeBase.get_property("Kinect", "Data"))
        Gazebo_index = -1
        while shape in gazebo_list:
            # Update name of new gazebo object
            Gazebo_index = Gazebo_index + 1
            Gazebo_orgi = shape
            if "#" in shape:
                Gazebo_orgi, _ = shape.split("#")
            shape = Gazebo_orgi + "#" + str(Gazebo_index)
        return shape


class Reasoner:
    def __init__(self, client):
        # Connect with ROS
        self.client = client

        # ROS topic to control the system
        self.camera_scan = roslibpy.Topic(self.client, 'detect_image',
                                                       'std_msgs/String')
        self.robot_move = roslibpy.Topic(self.client, 'target_pose',
                                                      'geometry_msgs/Pose')
        self.gripper_grasp = roslibpy.Topic(self.client, 'gripper_grasping',
                                                         'std_msgs/String')
        self.update_planning = roslibpy.Topic(self.client, 'add_box_ur5',
                                                           'std_msgs/String')
        self.camera_scan.advertise()
        self.robot_move.advertise()
        self.gripper_grasp.advertise()
        self.update_planning.advertise()

        # ROS services to control Gazebo environemnt
        self.dir_path = rospack.get_path('semantic_web') + "/simulation/assembly_samples"
        self.shape = ["Triangle", "Rectangle", "Pentagon"]
        self.spawn_srv = roslibpy.Service(self.client,
                                          '/gazebo/spawn_sdf_model',
                                          'gazebo_msgs/SpawnModel')
        self.delete_srv = roslibpy.Service(self.client,
                                           '/gazebo/delete_model',
                                           'gazebo_msgs/DeleteModel')

        # Temporary data
        self.task = OrderedDict()
        self.gazebo_object_names = eval(KnowledgeBase.get_property("Kinect", "Data"))

        # Control Gazebo spawn/delete models
        self.gazebo = GazeboSyncing()

    def check_name(self, name):
        if "#" in name:
            name, _ = name.split("#")
        if name in self.shape:
            # print(name)
            return name
        elif "_" in name:
            shape, index = name.split("_")
            # print("shape_t", shape)
            return self.check_name(shape)
        else:
            print("There is no shape")
            return False

    def creat_gazebo_model(self, name):
        req = {"initial_pose": {"position": {}, "orientation": {}}}
        rr = rp = ry = 0
        shape = self.check_name(name)
        # print(shape)
        if shape:
            self.path = os.path.join(self.dir_path, shape + "/model.sdf")
            path = open(self.path, "r")
            file = path.read()
            # print(file)

            if shape == "Pentagon":
                px = 0.9
                py = 1.0
                pz = 1.15

            if shape == "Triangle":
                px = 0.9
                py = 0.4
                pz = 1.15

            if shape == "Rectangle":
                px = 0.9
                py = 0.7
                pz = 1.15

            req["model_name"] = name
            req["model_xml"] = file
            req["initial_pose"]["position"]["x"] = px
            req["initial_pose"]["position"]["y"] = py
            req["initial_pose"]["position"]["z"] = pz

            q = quaternion_from_euler(rr, rp, ry)
            req["initial_pose"]["orientation"]["x"] = q[0]
            req["initial_pose"]["orientation"]["y"] = q[1]
            req["initial_pose"]["orientation"]["z"] = q[2]
            req["initial_pose"]["orientation"]["w"] = q[3]

            request = roslibpy.ServiceRequest(req)
            result = self.spawn_srv.call(request)
            self.gazebo_object_names.append(name)
            KnowledgeBase.update_property("Kinect", "Data", str(self.gazebo_object_names))
            return result

    def picking_base(self, shape):
        pickup_place = [0, 0, 0]
        if "_" in shape:
            shape, _ = shape.split("_")
        if shape == "Triangle":
            pickup_place = [0.6, -0.6, 0.18]
        if shape == "Rectangle":
            pickup_place = [0.3, -0.6, 0.18]
        if shape == "Pentagon":
            pickup_place = [0.0, -0.6, 0.18]
        return pickup_place

    def check_position(self, list1, list2, threshold):
        '''
        This function checks if centroid of detected shape is correct
        '''
        pose_dif = [abs(list1[0] - list2[0]),
                    abs(list1[1] - list2[1]),
                    abs(list1[2] - list2[2])]
        result = all(i < threshold for i in pose_dif)
        return result

    def generate_task(self):
        self.task = OrderedDict()
        task = "DemonstrationLearning_Task"
        scene = OrderedDict(eval(KnowledgeBase.get_property("Kinect",
                                                            "Current_state")))
        target = OrderedDict(eval(KnowledgeBase.get_property(task, "Data")))

        # Compare two scene (Target vs Current Scene)
        if len(scene.keys()) != 0:
            for shape in scene.keys():
                if shape in target.keys():
                    # Check position of shape
                    if self.check_position(scene[shape]["Centroid"],
                                           target[shape]["Centroid"], 0.035):
                        self.task[shape] = {"task": "Available {}".format(shape),
                                            "color": "success",
                                            "centroid": target[shape]["Centroid"]}
                        target.pop(shape)
                    else:
                        self.task[shape] = {"task": "Modify {}".format(shape),
                                            "color": "warning",
                                            "modified_pose": scene[shape]["Centroid"],
                                            "centroid": target[shape]["Centroid"]}
                        target.pop(shape)
                else:
                    self.task[shape] = {"task": "Remove {}".format(shape),
                                        "color": "danger",
                                        "modified_pose": scene[shape]["Centroid"]}

        for shape in target.keys():
            shapeG = self.gazebo.check_name(shape)
            # Update ADD Task
            self.task[shapeG] = {"task": "Add {}".format(shapeG),
                                 "color": "primary",
                                 "modified_pose": self.picking_base(shape),
                                 "centroid": target[shape]["Centroid"]}

        # Update Ontology
        KnowledgeBase.update_property(task, "Status", str(self.task))
        return self.task

    def perform_task(self):
        for shape in self.task:
            if self.task[shape]["color"] == "primary":
                self.add(shape)
        time.sleep(1)
        self.Robot_move("home")

    def Robot_move(self, position, *modified_z):
        if position == "home":
            position = [0, 0, 0]
        if position == "go back":
            position = [-1, -1, -1]
        msg_move = {"position": {"x": position[0],
                                 "y": position[1],
                                 "z": position[2]}}
        if len(modified_z) != 0:
            msg_move["position"]["z"] = position[2] + 0.07
        self.robot_move.publish(msg_move)

    def Gripper_grasp(self, command):
        # "command" parameter is a string of object name to pick object or Zero
        # to place object
        msg_grasp = {"data": command}
        self.gripper_grasp.publish(msg_grasp)
        print

    def Camera_scan(self, command):
        msg_scan = {"data": command}
        self.camera_scan.publish(msg_scan)
        print

    def add(self, shape):
        result = self.creat_gazebo_model(shape)
        time.sleep(1)
        KnowledgeBase.update_property("UR5", "Status")
        if result["success"]:
            print("Add {} in Gazebo: {}".format(shape, result["success"]))
            self.Robot_move(self.task[shape]["modified_pose"])
            robotStatus = KnowledgeBase.get_property("UR5", "Status")
            while robotStatus != "Reached":
                time.sleep(1)
                robotStatus = KnowledgeBase.get_property("UR5", "Status")
            print("robot ", robotStatus)
            self.update_planning.publish({"data": "1"})
            time.sleep(2)
            if robotStatus == "Reached":
                KnowledgeBase.update_property("UR5", "Status")
                self.Gripper_grasp(shape)
                gripperStatus = eval(KnowledgeBase.get_property("Vacuum_gripper", "Status"))
                while not gripperStatus:
                    time.sleep(1)
                    gripperStatus = eval(KnowledgeBase.get_property("Vacuum_gripper", "Status"))
                print("gripper ", gripperStatus)
                time.sleep(2)
                if gripperStatus:
                    self.Robot_move(self.task[shape]["centroid"], 1)
                    robotStatus = KnowledgeBase.get_property("UR5", "Status")
                    while robotStatus != "Reached":
                        time.sleep(1)
                        robotStatus = KnowledgeBase.get_property("UR5", "Status")
                    print("robot moved ", robotStatus)
                    time.sleep(2)
                    if robotStatus == "Reached":
                        self.Gripper_grasp("0")
                        gripperStatus = eval(KnowledgeBase.get_property("Vacuum_gripper", "Status"))
                        while not gripperStatus:
                            time.sleep(1)
                            gripperStatus = eval(KnowledgeBase.get_property("Vacuum_gripper", "Status"))
                        print("gripper holded", gripperStatus)
                        time.sleep(2)
                        if not gripperStatus:
                            print("Finish ADD {}".format(shape))
                            self.update_planning.publish({"data": "0"})

    def remove(self):
        print

    def modify(self):
        print
