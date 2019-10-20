#!/usr/bin/env python
import rospy
from camera import Camera
from uploader import Ontology
from robot import Robot
from gripper import Gripper


rospy.init_node("Perceptor", anonymous=True)

# Refresh Ontology when system is restarted
KnowledgeBase = Ontology()
KnowledgeBase.update_property("UR5", "Current_state")
KnowledgeBase.update_property("UR5", "Data")
KnowledgeBase.update_property("UR5", "Initial_state")
KnowledgeBase.update_property("UR5", "Status")
KnowledgeBase.update_property("Vacuum_gripper", "Current_state")
KnowledgeBase.update_property("Vacuum_gripper", "Status")
KnowledgeBase.update_property("Kinect", "Current_state")
KnowledgeBase.update_property("DemonstrationLearning_Task", "Status")

# Environment
kinect = Camera()
ur5 = Robot()
gripper = Gripper()

if __name__ == '__main__':
    try:

        # Update perception data
        # while not rospy.is_shutdown():
        # if kinect.update_trigger == 1:
            # perception.update_property("DemonstrationLearning_Task", "Current_state", kinect.scene)

        # Keep rospy running
        rospy.spin()

    except rospy.ROSInterruptException:
        print('==== STOPPING ROS ====')
        pass
