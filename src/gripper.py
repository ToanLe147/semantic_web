#!/usr/bin/env python
import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from std_msgs.msg import String
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose


class Gripper:
    def __init__(self):
        self.command = rospy.Subscriber('gripper_grasping', String, self.callback)
        self.gazebo_conditions = rospy.Subscriber("/gazebo/link_states", LinkStates, self.gazebo_callback)
        self.status = False
        self.common_link = "link"
        self.gripper = "ur5"
        self.gripper_link = "wrist_3_link"
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self.holding_object = ""
        self.attach_srv.wait_for_service()
        self.detach_srv.wait_for_service()
        self.gazebo_objects = {}

    def gazebo_callback(self, msg):
        gazebo_link_name = msg.name
        gazebo_link_pose = msg.pose
        self.gazebo_objects = dict(zip(gazebo_link_name, gazebo_link_pose))

    def grasping_condition(self, object):
        grasping = False
        gripper_link_name = self.gripper + "::" + self.gripper_link
        object_link_name = object + "::" + self.common_link
        gripper_link_pose = self.gazebo_objects[gripper_link_name]
        object_link_pose = self.gazebo_objects[object_link_name]
        if abs(gripper_link_pose.position.z - object_link_pose.position.z) < 0.15:
            if abs(gripper_link_pose.position.y - object_link_pose.position.y) <= 0.02:
                if abs(gripper_link_pose.position.x - object_link_pose.position.x) <= 0.02:
                    grasping = True
        print("===============")
        print(gripper_link_pose)
        print("~~~~~", grasping)
        print(object_link_pose)
        print("****************")
        return grasping

    def pick(self, object):
        if self.grasping_condition(object):
            req = AttachRequest()
            self.status = True
            req.model_name_1 = self.gripper
            req.link_name_1 = self.gripper_link
            req.model_name_2 = object
            req.link_name_2 = self.common_link
            self.attach_srv.call(req)
            self.holding_object = object
        return self.status

    def place(self):
        req = AttachRequest()
        self.status = False
        req.model_name_1 = self.gripper
        req.link_name_1 = self.gripper_link
        req.model_name_2 = self.holding_object
        req.link_name_2 = self.common_link
        self.detach_srv.call(req)
        self.holding_object = ""
        return self.status

    def callback(self, msg):
        command = msg.data
        if command != "0":
            object = command
            status = self.pick(object)
        else:
            status = self.place()
        return status


def main():
    rospy.init_node("hack_gripper", anonymous=True)
    gripper_ur5 = Gripper()

    while not rospy.is_shutdown():
        command = int(raw_input("Command: "))
        if command == 1:
            object = str(raw_input("object: "))
            status = gripper_ur5.pick(object)
            print(status)

        if command == 0:
            status = gripper_ur5.place()
            print(status)

    rospy.spin()


if __name__ == '__main__':
    main()
