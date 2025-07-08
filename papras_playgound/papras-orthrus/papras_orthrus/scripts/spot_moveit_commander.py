#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import sys

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose

import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String

import moveit_commander
import moveit_msgs.msg


class SpotMoveitCommander(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # arm move groups
        left_arm_group_name = "arm1"
        self.left_arm_move_group = moveit_commander.MoveGroupCommander(left_arm_group_name)
        right_arm_group_name = "arm2"
        self.right_arm_move_group = moveit_commander.MoveGroupCommander(right_arm_group_name)
        both_arm_group_name = "arm1_2"
        self.both_arm_move_group = moveit_commander.MoveGroupCommander(both_arm_group_name)

        # gripper move groups
        left_gripper_group_name = "soft_gripper1"
        self.left_gripper_move_group = moveit_commander.MoveGroupCommander(left_gripper_group_name)
        right_gripper_group_name = "soft_gripper2"
        self.right_gripper_move_group = moveit_commander.MoveGroupCommander(right_gripper_group_name)
        both_gripper_group_name = "soft_gripper1_2"
        self.both_gripper_move_group = moveit_commander.MoveGroupCommander(both_gripper_group_name)
    
    def set_movegroup_config(self, move_group, 
                                    acc_scale=0.1, 
                                    vel_scale=0.1, 
                                    plan_time=0.3, 
                                    plan_attempt=30):
        move_group.set_max_acceleration_scaling_factor(acc_scale)
        move_group.set_max_velocity_scaling_factor(vel_scale)
        move_group.set_planning_time(plan_time)
        move_group.set_num_planning_attempts(plan_attempt)

    def move_to_named_pose(self, move_group, named_target):
        move_group.set_start_state_to_current_state()
        move_group.set_named_target(named_target)
        move_group.plan()
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()