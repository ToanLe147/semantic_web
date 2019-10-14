#!/usr/bin/env python

import sys
import copy
import rospy
import numpy as np
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import PlanningScene


class Robot:
    def __init__(self):
        # Initial setup for UR5
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.Subscriber('target_pose', Pose, self.move)
        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()
        # Adding obstacles to Planning Scene
        # self.add_obstacle("table", [0.8, 1.4, 1.02], [0, 0.5, -0.549, 0, 0, 0.7071068, 0.7071068])
        # self.update_planning_scene("table", True, 4)
        # self.add_obstacle("picking_base", [0.3, 1, 1.12], [0.3, -0.6, -0.49, 0, 0, 0.7071068, 0.7071068])
        # self.update_planning_scene("picking_base", True, 4)
        # self.add_obstacle("robot_base", [0.5, 1, 1], [0, -0.175, -0.55, 0, 0, 0.7071068, 0.7071068])
        # self.update_planning_scene("robot_base", True, 4)

        self.ur5 = moveit_commander.MoveGroupCommander('manipulator')
        self.ur5.set_pose_reference_frame("base_link")
        self.end_effector = self.ur5.get_end_effector_link()

        # Allow replanning to increase the odds of a solution
        self.ur5.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.ur5.set_goal_position_tolerance(0.01)
        self.ur5.set_goal_orientation_tolerance(0.1)

        # Guild UR5 go to ready position
        self.ur5.set_named_target('home')
        self.ur5.go()

    def update_planning_scene(self, name, box_is_known, timeout=4):
        box_name = name
        scene = self.scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while(seconds - start < timeout) and not (rospy.is_shutdown):
            is_known = box_name in scene.get_known_object_names()
            if box_is_known == is_known:
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def add_obstacle(self, name, size, pose):  # Add boxes
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x = pose[0]
        box_pose.pose.position.y = pose[1]
        box_pose.pose.position.z = pose[2]
        box_pose.pose.orientation.x = pose[3]
        box_pose.pose.orientation.y = pose[4]
        box_pose.pose.orientation.z = pose[5]
        box_pose.pose.orientation.w = pose[6]
        box_name = name
        self.scene.add_box(box_name, box_pose, size=tuple(size))

    def visual(self, *desired_pose):
        print('========= TARGET POINT ==========')
        print(desired_pose)
        print(self.scene.get_known_object_names())

        print('======= ROBOT STATE =======')
        print(self.robot.get_current_state().joint_state.position)
        print('================')

        print('==== EE POSE ====')
        print(self.ur5.get_current_pose().pose)
        print('=============')

    def move(self, msg):
        desired_pose = msg.position
        waypoint = []
        start_point = self.ur5.get_current_pose(self.end_effector).pose

        poseTarget = copy.deepcopy(start_point)

        # 1st orient gripper and move forward/backward
        poseTarget.position.x = desired_pose.x

        # 2nd move sides
        poseTarget.position.y = desired_pose.y

        # 3rd move up/down
        poseTarget.position.z = desired_pose.z

        # orientation
        # poseTarget.orientation = desired_orientation

        waypoint.append(copy.deepcopy(poseTarget))

        # Set the internal state to the current state
        self.ur5.set_start_state_to_current_state()

        if np.sqrt((poseTarget.position.x - start_point.position.x)**2 +
                   (poseTarget.position.y - start_point.position.y)**2 +
                   (poseTarget.position.z - start_point.position.z)**2) < 0.1:
            rospy.loginfo("Warnig: target position overlaps with the initial position!")

        # Compute Cartesian path. The return value is a tuple: a fraction of how much of the path was followed, the actual
        # RobotTrajectory.
        (plan, fraction) = ur5.compute_cartesian_path(
            waypoint,  # waypoints to follow
            0.01,  # eef_step, which is set to 0.01m ~ 1 cm
            0.0,  # jump_threshold, which is set to 0 to disable it. The jump_threshold specifies the maximum distance in
            # configuration space between consecutive points in the resulting path
        )

        ur5.execute(plan, wait=True)
        # self.ur5.set_pose_target(poseTarget)
        # self.ur5.go(wait=True)
        # self.ur5.stop()
        # # It is always good to clear your targets after planning with poses.
        # # Note: there is no equivalent function for clear_joint_value_targets()
        # self.ur5.clear_pose_targets()

        # Visualization
        self.visual(desired_pose)


if __name__ == '__main__':
    try:
        rospy.init_node('ur5_cartesian_pose', anonymous=True)
        ur5 = Robot()
        # Adding obstacles to Planning Scene
        raw_input("add Table")
        ur5.add_obstacle("table", [0.8, 1.4, 1.02], [0, 0.5, -0.549, 0, 0, 0.7071068, 0.7071068])
        raw_input("add picking base")
        ur5.add_obstacle("picking_base", [0.3, 1, 1.12], [0.3, -0.6, -0.49, 0, 0, 0.7071068, 0.7071068])
        raw_input("add box below")
        ur5.add_obstacle("robot_base", [0.5, 1, 1], [0, -0.175, -0.55, 0, 0, 0.7071068, 0.7071068])
        rospy.spin()

    except rospy.ROSInterruptException:
        # When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()
        print('==== STOPPING ====')
        pass
