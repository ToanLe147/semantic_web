#!/usr/bin/env python

import sys
import copy
import rospy
import numpy as np
import moveit_commander
import geometry_msgs.msg


class Robot:
    def __init__(self):
        # Initial setup for UR5
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.Subscriber('target_pose', geometry_msgs.msg.Pose, self.move)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
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

    def visual(self, *desired_pose):
        print('========= TARGET POINT ==========')
        print(desired_pose)

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
        # (plan, fraction) = ur5.compute_cartesian_path(
        #     waypoint,  # waypoints to follow
        #     0.01,  # eef_step, which is set to 0.01m ~ 1 cm
        #     0.0,  # jump_threshold, which is set to 0 to disable it. The jump_threshold specifies the maximum distance in
        #     # configuration space between consecutive points in the resulting path
        # )

        # ur5.execute(plan, wait=True)
        self.ur5.set_pose_target(poseTarget)
        self.ur5.go(wait=True)
        self.ur5.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.ur5.clear_pose_targets()

        # Visualization
        self.visual(desired_pose)


if __name__ == '__main__':
    try:
        rospy.init_node('ur5_cartesian_pose', anonymous=True)
        ur5 = Robot()
        rospy.spin()

    except rospy.ROSInterruptException:
        # When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()
        print('==== STOPPING ====')
        pass
