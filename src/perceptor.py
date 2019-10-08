#!/usr/bin/env python
import rospy
from shape_detection import Camera
from uploader import ontology


rospy.init_node("Perceptor")

# Environment
kinect = Camera()
perception = ontology()

# Update perception data
while not rospy.is_shutdown():
    if kinect.update_trigger == 1:
        perception.update_property("DemonstrationLearning_Task", "Data", kinect.scene)
        on = 0

# Keep rospy running
rospy.spin()
