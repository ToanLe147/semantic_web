#!/usr/bin/env python

import sys
import copy
import rospy
import numpy as np
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from uploader import Ontology
from tf.transformations import quaternion_from_euler
import time

KnowledgeBase = Ontology()


class Robot:
    def __init__(self):
        # Initial setup for UR5
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.Subscriber('target_pose', Pose, self.move)
        rospy.Subscriber('add_box_ur5', String, self.add_object)
        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        self.ur5 = moveit_commander.MoveGroupCommander('manipulator')
        self.ur5.set_pose_reference_frame("base_link")
        self.end_effector = self.ur5.get_end_effector_link()

        # Allow replanning to increase the odds of a solution
        self.ur5.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.ur5.set_goal_position_tolerance(0.01)
        self.ur5.set_goal_orientation_tolerance(0.01)

        # Guild UR5 go to ready position
        self.ur5.set_named_target('home')
        self.ur5.go(wait=True)
        self.backup_pose()

    def target_reaching(self, target):
        status = False
        _pose = self.ur5.get_current_pose().pose
        msg = [target.position.x, target.position.y, target.position.z]
        KnowledgeBase.update_property("UR5", "Data", msg)
        if abs(_pose.position.x - target.position.x) <= 0.01:
            if abs(_pose.position.y - target.position.y) <= 0.01:
                if abs(_pose.position.z - target.position.z) <= 0.01:
                    status = True
        if status:
            KnowledgeBase.update_property("UR5", "Status", "Reached")
        else:
            KnowledgeBase.update_property("UR5", "Status")

    def add_object(self, msg):
        time.sleep(1)
        box_pose = PoseStamped()
        box_pose.header.frame_id = "wrist_3_link"
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.1
        box_pose.pose.position.z = 0.0
        q = quaternion_from_euler(1.570796327, 0, 0)
        box_pose.pose.orientation.x = q[0]
        box_pose.pose.orientation.y = q[1]
        box_pose.pose.orientation.z = q[2]
        box_pose.pose.orientation.w = q[3]
        box_name = "box"
        if msg.data == "1":
            self.scene.add_box(box_name, box_pose, size=(0.16, 0.16, 0.02))
            time.sleep(1)
            # Attach
            grasping_group = 'manipulator'
            touch_links = self.robot.get_link_names(group=grasping_group)
            self.scene.attach_box("wrist_3_link", box_name, touch_links=touch_links)
        if msg.data == "0":
            self.scene.remove_attached_object("wrist_3_link", name=box_name)
            time.sleep(1)
            self.scene.remove_world_object(box_name)

    def backup_pose(self):
        # Save previous pose
        _pose = self.ur5.get_current_pose().pose
        self.previous_pose = [_pose.position.x, _pose.position.y, _pose.position.z]
        KnowledgeBase.update_property("UR5", "Initial_state", self.previous_pose)

    def update_knowledgeBase(self, status):
        # Update
        if status:
            current_pose = self.ur5.get_current_pose().pose
            pose = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
            KnowledgeBase.update_property("UR5", "Current_state", pose)

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
        # Go up
        self.ur5.set_named_target('up')
        self.ur5.go(wait=True)

        # Handle recieved message
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

        # Update target reaching status
        self.target_reaching(poseTarget)

        # return to previous pose
        if desired_pose.x == desired_pose.y == desired_pose.z == -1:
            _previous_pose = self.previous_pose
            poseTarget.position.x = round(_previous_pose[0], 2)
            poseTarget.position.y = round(_previous_pose[1], 2)
            poseTarget.position.z = round(_previous_pose[2], 2)
            self.ur5.set_pose_target(poseTarget)

            # backup previous_pose
            self.backup_pose()

            moved_status = self.ur5.go(wait=True)
            self.ur5.stop()
            self.ur5.clear_pose_targets()

            # Update robot state
            self.update_knowledgeBase(moved_status)

            # Visualization
            # self.visual(desired_pose)

            # Update target reaching status
            self.target_reaching(poseTarget)
            return

        waypoint.append(copy.deepcopy(poseTarget))

        # backup previous_pose
        self.backup_pose()

        # Set the internal state to the current state
        self.ur5.set_start_state_to_current_state()

        if np.sqrt((poseTarget.position.x - start_point.position.x)**2 +
                   (poseTarget.position.y - start_point.position.y)**2 +
                   (poseTarget.position.z - start_point.position.z)**2) < 0.1:
            rospy.loginfo("Warnig: target position overlaps with the initial position!")

        self.ur5.set_pose_target(poseTarget)
        if desired_pose.x == desired_pose.y == desired_pose.z == 0:
            self.ur5.set_named_target('home')

        moved_status = self.ur5.go(wait=True)
        # print(moved_status)
        self.ur5.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.ur5.clear_pose_targets()

        # Update robot state
        self.update_knowledgeBase(moved_status)

        # Visualization
        # self.visual(desired_pose)

        # Update target reaching status
        self.target_reaching(poseTarget)


if __name__ == '__main__':
    try:
        rospy.init_node('ur5_gazebo_robot', anonymous=True)
        ur5 = Robot()
        rospy.spin()

    except rospy.ROSInterruptException:
        # When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()
        print('==== STOPPING ====')
        pass
