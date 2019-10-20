#!/usr/bin/env python
import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState, GetLinkState
from std_srvs.srv import Empty
from uploader import Ontology

KnowledgeBase = Ontology()


class Gripper:
    def __init__(self):
        # Gazebo Interface
        self.command = rospy.Subscriber('gripper_grasping', String, self.callback)
        self.gazebo_conditions = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.gazebo_conditions.wait_for_service()
        self.attach_srv.wait_for_service()
        self.detach_srv.wait_for_service()
        self.set_state.wait_for_service()
        # self.get_state.wait_for_service()
        self.pause.wait_for_service()
        self.unpause.wait_for_service()
        self.gazebo_objects = {}

        # Gripper Initialization
        self.status = False
        self.common_link = "link"
        self.gripper = "ur5"
        self.gripper_link = "wrist_3_link"
        self.holding_object = ""
        self.update_gripper_info(False)

    def update_gripper_info(self, *grasping):
        if len(grasping) != 0:
            KnowledgeBase.update_property("Vacuum_gripper", "Current_state", grasping[0])
        KnowledgeBase.update_property("Vacuum_gripper", "Status", self.status)
        if self.status:
            KnowledgeBase.update_property("Vacuum_gripper", "Data", self.holding_object)
        else:
            KnowledgeBase.update_property("Vacuum_gripper", "Data")

    def gazebo_callback(self, object):
        gripper_link_name = self.gripper + "::" + self.gripper_link
        object_link_name = object + "::" + self.common_link
        gripper_pose = self.gazebo_conditions.call(gripper_link_name, "world")
        object_pose = self.gazebo_conditions.call(object_link_name, "world")
        gazebo_link_name = [gripper_link_name, object_link_name]
        gazebo_link_pose = [gripper_pose, object_pose]
        self.gazebo_objects = dict(zip(gazebo_link_name, gazebo_link_pose))

    def grasping_condition(self, object):
        grasping = False
        gripper_link_name = self.gripper + "::" + self.gripper_link
        object_link_name = object + "::" + self.common_link
        gripper_link_pose = self.gazebo_objects[gripper_link_name].link_state.pose
        try:
            object_link_pose = self.gazebo_objects[object_link_name].link_state.pose
        except KeyError:
            print("Object name error")
            return grasping
        if abs(gripper_link_pose.position.z - object_link_pose.position.z) < 0.2:
            if abs(gripper_link_pose.position.y - object_link_pose.position.y) <= 0.1:
                if abs(gripper_link_pose.position.x - object_link_pose.position.x) <= 0.1:
                    grasping = True
        # print("===============")
        # print(gripper_link_pose)
        # print("~~~~~", grasping)
        # print(object_link_pose)
        # print("****************")
        self.update_gripper_info(grasping)
        return grasping

    def move_object(self, object):
        # Prepare msg for set state ROS service
        object_msg = ModelState()
        object_msg.model_name = object

        # Implement new pose for object
        gripper_link_name = self.gripper + "::" + self.gripper_link
        object_link_name = object + "::" + self.common_link
        gripper = self.gazebo_objects[gripper_link_name].link_state
        object_state = self.gazebo_objects[object_link_name].link_state
        object_msg.pose = object_state.pose
        object_msg.twist = object_state.twist
        object_msg.reference_frame = "world"
        # Vacuum Gripper drags the object close to suction cup
        object_msg.pose.position.z = gripper.pose.position.z - 0.1

        # print(gripper.pose.position.z)
        # print("------------SEPERATE-------------")
        # print(object_msg)
        self.set_state.call(object_msg)
        # print("set state")

    def pick(self, object):
        self.gazebo_callback(object)
        if self.grasping_condition(object):
            # Pause physic environment
            self.pause.call()

            req = AttachRequest()
            self.status = True
            req.model_name_1 = self.gripper
            req.link_name_1 = self.gripper_link
            req.model_name_2 = object
            req.link_name_2 = self.common_link
            self.move_object(object)
            self.attach_srv.call(req)
            self.holding_object = object
            self.update_gripper_info(False)

            # Release physic environment
            self.unpause.call()
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
        self.update_gripper_info(False)
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
