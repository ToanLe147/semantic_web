#!/usr/bin/env python
import rospy
from camera import Camera
from uploader import Ontology
from robot import Robot
from gripper import Gripper


if __name__ == '__main__':
    try:
        rospy.init_node("Perceptor", anonymous=True)
        # Environment
        kinect = Camera()
        perception = Ontology()
        ur5 = Robot()
        gripper = Gripper()

        # Adding obstacles to Planning Scene
        raw_input("add Table")
        ur5.add_obstacle("table", [0.8, 1.4, 1.02], [0, 0.5, -0.54, 0, 0, 0.7071068, 0.7071068])
        raw_input("add picking base")
        ur5.add_obstacle("picking_base", [0.3, 1, 1.12], [0.3, -0.6, -0.49, 0, 0, 0.7071068, 0.7071068])
        raw_input("add box below")
        ur5.add_obstacle("robot_base", [0.5, 1, 1], [0, -0.175, -0.52, 0, 0, 0.7071068, 0.7071068])

        # Update perception data
        while not rospy.is_shutdown():
            if kinect.update_trigger == 1:
                perception.update_property("DemonstrationLearning_Task", "Current_state", kinect.scene)
            

        # Keep rospy running
        rospy.spin()

    except rospy.ROSInterruptException:
        print('==== STOPPING ROS ====')
        pass
