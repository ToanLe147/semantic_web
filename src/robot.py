#!/usr/bin/env python

# This script is subscriber of /hololens topic, which is published from HoloLens user.
# The code is written based on Moveit's move_group_Python_interface.py
# Student: Toan Le - 267945
# Course: Robot Project Work 2018-2019


import sys
import copy
import rospy
import numpy as np
import moveit_commander
import geometry_msgs.msg


# Handle ur5 movements
def move_ur5(msg, args):

    robot = args[0]
    ur5 = args[1]

    # Set the reference frame for pose targets
    reference_frame = "base_link"

    # Set the ur5_arm reference frame accordingly
    ur5.set_pose_reference_frame(reference_frame)

    end_effector = ur5.get_end_effector_link()

    # Allow replanning to increase the odds of a solution
    ur5.allow_replanning(True)

    # Allow some leeway in position (meters) and orientation (radians)
    ur5.set_goal_position_tolerance(0.01)
    ur5.set_goal_orientation_tolerance(0.1)

    print('========= PLANNING ==========')

    desired_pose = msg.position
    #desired_orientation = msg.orientation
    print(desired_pose)
    #print(desired_orientation)
    print('========================================================')

    # ==================================== Cartesian Move Done
    waypoint = []
    # Not adding START POINT into WAYPOINT solved problem named
    # "Trajectory message contains waypoints that are not strictly increasing in time."
    start_point = ur5.get_current_pose(end_effector).pose

    poseTarget = copy.deepcopy(start_point)

    # 1st orient gripper and move forward/backward
    poseTarget.position.x = desired_pose.x

    # 2nd move sides
    poseTarget.position.y = desired_pose.y

    # 3rd move up/down
    poseTarget.position.z = desired_pose.z

    # orientation
    #poseTarget.orientation = desired_orientation

    waypoint.append(copy.deepcopy(poseTarget))

    # Set the internal state to the current state
    ur5.set_start_state_to_current_state()

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
    ur5.set_pose_target(poseTarget)
    plan = ur5.go(wait=True)
    ur5.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    ur5.clear_pose_targets()

    print('==== ROBOT STATE AFTER MOVING ====')
    print(robot.get_current_state().joint_state.position)
    print('=============')

    print('==== EE POSE ====')
    print(ur5.get_current_pose().pose)
    print('=============')


def subscriber():
    print()
    print('========================================================')

    # Initializing moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_cartesian_pose', anonymous=True)

    # Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()  # WHAT CAN WE DO WITH THIS???

    # Instantiate UR5 as group of joints. This interface can be used to plan and execute motion of UR5
    ur5 = moveit_commander.MoveGroupCommander('manipulator')
    ur5.set_named_target('home')
    plan1 = ur5.go()

    # Print some features of UR5
    print('======= Reference frame:', ur5.get_planning_frame())
    print('======= End Effector:', ur5.get_end_effector_link())

    print('======= ROBOT STATE =======')
    print(robot.get_current_state().joint_state.position)
    print('================')

    print('==== EE POSE ====')
    print(ur5.get_current_pose().pose)
    print('=============')

    rospy.Subscriber('target_pose', geometry_msgs.msg.Pose, move_ur5, (robot, ur5), queue_size = 1)

    rospy.spin()


if __name__ == '__main__':
    try:
        subscriber()

    except rospy.ROSInterruptException:
        # When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()
        print('==== STOPPING ====')
        pass