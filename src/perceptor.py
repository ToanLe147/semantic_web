#!/usr/bin/env python
import rospy
from camera import Camera
from uploader import Ontology
from robot import Robot
from gripper import Gripper


rospy.init_node("Perceptor", anonymous=True)
# Environment
kinect = Camera()
perception = Ontology()
ur5 = Robot()
gripper = Gripper()

if __name__ == '__main__':
    try:

        # Update perception data
        while not rospy.is_shutdown():
            if kinect.update_trigger == 1:
                perception.update_property("DemonstrationLearning_Task", "Current_state", kinect.scene)

        # Keep rospy running
        rospy.spin()

    except rospy.ROSInterruptException:
        print('==== STOPPING ROS ====')
        pass
