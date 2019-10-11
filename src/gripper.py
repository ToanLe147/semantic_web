#!/usr/bin/env python
import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest


class gripper:
    def __init__(self):
        self.status = False
        self.common_link = "link"
        self.gripper = "wrist_3_link"
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
        self.holding_object = ""

    def pick(self, object):
        req = AttachRequest()
        self.status = True
        req.model_name_1 = self.gripper
        req.link_name_1 = self.common_link
        req.model_name_2 = object
        req.link_name_2 = self.common_link
        self.attach_srv.call(req)
        self.holding_object = object
        return self.status

    def place(self):
        req = AttachRequest()
        self.status = False
        req.model_name_1 = self.gripper
        req.link_name_1 = self.common_link
        req.model_name_2 = self.holding_object
        req.link_name_2 = self.common_link
        self.detach_srv.call(req)
        self.holding_object = ""
        return self.status


def main():
    rospy.init_node("hack_gripper", anonymous=True)
    gripper_ur5 = gripper()
    gripper_ur5.attach_srv.wait_for_service()
    gripper_ur5.detach_srv.wait_for_service()

    while not rospy.is_shutdown():
        command = int(raw_input("Command: "))
        if command == 1:
            object = input("object: ")
            status = gripper_ur5.pick(object)
            rospy.loginfo("Gripper status (True: close, False: open): ", status)

        if command == 0:
            status = gripper_ur5.place()
            rospy.loginfo("Gripper status (True: close, False: open): ", status)

    rospy.spin()


if __name__ == '__main__':
    main()
