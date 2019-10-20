#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel
from tf.transformations import quaternion_from_euler
import os
# import rospkg

# rospack = rospkg.RosPack()


class assembly_objects:
    def __init__(self):
        self.spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        # self.dir_path = rospack.get_path('semantic_web') + "/simulation/assembly_samples"
        self.dir_path = "/home/nico/catkin_ws/src/semantic_web/simulation/assembly_samples"
        self.shape = ["Triangle", "Rectangle", "Pentagon"]
        # print()
        self.spawn_srv.wait_for_service()
        self.delete_srv.wait_for_service()

    def check_name(self, name):
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

    def human_task(self, name, px, py, pz):
        req = SpawnModelRequest()
        rr = rp = ry = 0
        shape = self.check_name(name)
        # print(shape)
        if shape:
            self.path = os.path.join(self.dir_path, shape + "/model.sdf")
            path = open(self.path, "r")
            file = path.read()
            # print(file)

            req.model_name = name
            req.model_xml = file
            req.initial_pose.position.x = px
            req.initial_pose.position.y = py
            req.initial_pose.position.z = pz

            q = quaternion_from_euler(rr, rp, ry)
            req.initial_pose.orientation.x = q[0]
            req.initial_pose.orientation.y = q[1]
            req.initial_pose.orientation.z = q[2]
            req.initial_pose.orientation.w = q[3]

            self.spawn_srv.call(req)

    def creat_gazebo_model(self, name):
        req = SpawnModelRequest()
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

            req.model_name = name
            req.model_xml = file
            req.initial_pose.position.x = px
            req.initial_pose.position.y = py
            req.initial_pose.position.z = pz

            q = quaternion_from_euler(rr, rp, ry)
            req.initial_pose.orientation.x = q[0]
            req.initial_pose.orientation.y = q[1]
            req.initial_pose.orientation.z = q[2]
            req.initial_pose.orientation.w = q[3]

            result = self.spawn_srv.call(req)
            return result


if __name__ == '__main__':

    rospy.init_node('spawn_models')
    objects = assembly_objects()

    while not rospy.is_shutdown():
        user = str(raw_input("add shape: "))
        objects.creat_gazebo_model(user)

    rospy.spin()
