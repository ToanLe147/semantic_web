#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty


class gripper:
    def __init__(self, name, quality):
        self.name = name  # /ur5/vacuum_gripper
        self.number_of_gripper = quality  # 9
        self.status_sub = rospy.Subscriber("", self.gripper_status)
        self.trigger_sub = rospy.Subscriber("", self.trigger)
        self.status = "off"
        self.trigger_status = 0

    def gripper_status(self, msg):
        if msg.data:
            self.status = msg.data

    def trigger(self, msg):
        self.trigger_status = msg.data
        if self.trigger_status:
            self.combination(True)
        else:
            self.combination(False)

    def combination(self, on):
        for i in range(self.number_of_gripper):
            if on:
                self.gripper_on(str(i))
            else:
                self.gripper_off(str(i))

    def gripper_on(self, index):
        gripper_sv = self.name + index + '/on'
        # Wait till the srv is available
        rospy.wait_for_service(gripper_sv)
        try:
            # Create a handle for the calling the srv
            turn_on = rospy.ServiceProxy(gripper_sv, Empty)
            # Use this handle just like a normal function and call it
            resp = turn_on()
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def gripper_off(self, index):
        gripper_sv = self.name + index + '/off'
        # Wait till the srv is available
        rospy.wait_for_service(gripper_sv)
        try:
            # Create a handle for the calling the srv
            turn_off = rospy.ServiceProxy(gripper_sv, Empty)
            # Use this handle just like a normal function and call it
            resp = turn_off()
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
