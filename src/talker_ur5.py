#!/usr/bin/env python

# This script is pretend to be the hololen, which publishes position for the robot to go and doing "pick and place" task
# Student: Toan Le - 267945
# Course: Robot Project Work 2018-2019


import rospy

from geometry_msgs.msg import Pose


def talker():
    pub = rospy.Publisher('target_pose', Pose, queue_size=10)
    rospy.init_node('psuedo_ur5', anonymous=True)
    rate = rospy.Rate(1)  # 0.05hz

    desired_pose1 = Pose()

    # desired_pose2 = Pose()
    # desired_pose2.position.x = 0.5
    # desired_pose2.position.y = 0.1
    # desired_pose2.position.z = 0.3
    #
    #
    # pose = [desired_pose1, desired_pose2]
    # index = 0

    while not rospy.is_shutdown():

        print('=======================')
        x = float(input('Input x: '))
        y = float(input('Input y: '))
        z = float(input('Input z: '))
        #x1 = float(input('Input x_angle: '))
        #y1 = float(input('Input y_angle: '))
        #z1 = float(input('Input z_angle: '))
        #w = float(input('Input w_angle: '))

        desired_pose1.position.x = x
        desired_pose1.position.y = y
        desired_pose1.position.z = z
        #desired_pose1.orientation.x = x1
        #desired_pose1.orientation.y = y1
        #desired_pose1.orientation.z = z1
        #desired_pose1.orientation.w = w
        print(desired_pose1)

        pub.publish(desired_pose1)
        #
        # index += 1
        # if index == 2:
        #     index = 0

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
