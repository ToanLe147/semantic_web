#!/usr/bin/env python
import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from std_msgs.msg import String
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import SetModelState, GetModelState
from std_srvs.srv import Empty
from uploader import Ontology

KnowledgeBase = Ontology()


class Gripper:
    def __init__(self):
        # Gazebo Interface
        self.command = rospy.Subscriber('gripper_grasping', String, self.callback)
        self.gazebo_conditions = rospy.Subscriber("/gazebo/link_states",
                                                  LinkStates,
                                                  self.gazebo_callback)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.attach_srv.wait_for_service()
        self.detach_srv.wait_for_service()
        self.set_state.wait_for_service()
        self.get_state.wait_for_service()
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

    def gazebo_callback(self, msg):
        gazebo_link_name = msg.name
        gazebo_link_pose = msg.pose
        self.gazebo_objects = dict(zip(gazebo_link_name, gazebo_link_pose))

    def grasping_condition(self, object):
        grasping = False
        gripper_link_name = self.gripper + "::" + self.gripper_link
        object_link_name = object + "::" + self.common_link
        gripper_link_pose = self.gazebo_objects[gripper_link_name]
        try:
            object_link_pose = self.gazebo_objects[object_link_name]
        except KeyError:
            print("Object name error")
            return grasping
        if abs(gripper_link_pose.position.z - object_link_pose.position.z) < 0.2:
            if abs(gripper_link_pose.position.y - object_link_pose.position.y) <= 0.1:
                if abs(gripper_link_pose.position.x - object_link_pose.position.x) <= 0.1:
                    grasping = True
        print("===============")
        print(gripper_link_pose)
        print("~~~~~", grasping)
        print(object_link_pose)
        print("****************")
        self.update_gripper_info(grasping)
        return grasping

    def move_object(self, object):
        self.pause.call()
        gripper = self.get_state.call(self.gripper, self.gripper_link)
        object = self.get_state.call(object, self.common_link)
        object.pose.position.z = gripper.pose.position.z - 0.03
        self.set_state.call(object)
        self.unpause.call()
        print("set state")

    def pick(self, object):
        if self.grasping_condition(object):
            req = AttachRequest()
            self.status = True
            req.model_name_1 = self.gripper
            req.link_name_1 = self.gripper_link
            req.model_name_2 = object
            req.link_name_2 = self.common_link
            self.attach_srv.call(req)
            self.move_object(object)
            self.holding_object = object
            self.update_gripper_info(False)
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
