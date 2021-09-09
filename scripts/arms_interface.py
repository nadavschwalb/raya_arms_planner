#!/usr/bin/python
import sys
import os
from moveit_commander import exception
from moveit_commander.exception import MoveItCommanderException
import rospy
import roslib
from copy import deepcopy
roslib.load_manifest("raya_arms_planner")
import actionlib
from moveit_commander import move_group
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_commander.robot import RobotCommander
from sensor_msgs.msg import JointState
from rosgraph.names import SEP
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from raya_arms_planner.RayaArmsPlanner import JointGoalPlanner, PoseGoalPlanner

if __name__ == "__main__":
    rospy.init_node("arm_interface_action_server")
    joint_action_server = JointGoalPlanner()
    pose_action_server = PoseGoalPlanner()
    rospy.loginfo("Arms Interface Action Server Initialized")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("keyboard interupt detected, exiting...")
        exit()
    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('raya_arms_interface', anonymous=True)
    # robot = RobotCommander()
    # scene = PlanningSceneInterface()
    # left_move_group = MoveGroupCommander("left_arm")
    # right_move_group = MoveGroupCommander("right_arm")
    # goal_pose = Pose()
    # goal_pose.position.x = 0.28
    # goal_pose.position.y = 0.25
    # goal_pose.position.z = 1.48
    # q = quaternion_from_euler(0,0,0)
    # goal_pose.orientation.x = q[0]
    # goal_pose.orientation.y = q[1]
    # goal_pose.orientation.z = q[2]
    # goal_pose.orientation.w = q[3]
    # print(goal_pose)
    # left_move_group.set_pose_target(goal_pose)
    # plan = left_move_group.plan()
    # if(plan):
    #     rospy.loginfo("got plan")
    # else:
    #     rospy.loginfo("failed to get plan")
    # left_move_group.execute(plan,wait=True)
    # left_move_group.stop()
