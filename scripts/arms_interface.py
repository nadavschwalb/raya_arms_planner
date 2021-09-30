#!/usr/bin/python
import sys
import os
from math import radians
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
from moveit_msgs.msg import RobotTrajectory, ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint , JointTrajectory
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from raya_arms_planner.RayaArmsPlanner import JointGoalPlanner, PoseGoalPlanner, GripperController

if __name__ == "__main__":
    rospy.init_node("arm_interface_action_server")
    joint_action_server = JointGoalPlanner()
    pose_action_server = PoseGoalPlanner()
    gripper_control_server = GripperController()
    rospy.loginfo("Arms Interface Action Server Initialized")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("keyboard interupt detected, exiting...")
        exit()
    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('raya_arms_interface', anonymous=True)
    # trajectory_goal_pub = rospy.Publisher("/execute_trajectory/goal",ExecuteTrajectoryActionGoal,queue_size=10)
    # robot = RobotCommander()
    # scene = PlanningSceneInterface()
    # left_move_group = MoveGroupCommander("left_arm")
    # right_move_group = MoveGroupCommander("right_arm")
    # left_hand_group = MoveGroupCommander("left_hand")
    # right_hand_group = MoveGroupCommander("right_hand")

    # trajectory_goal = ExecuteTrajectoryActionGoal()

    # trajectory = RobotTrajectory()
    # trajectory.joint_trajectory.header.frame_id = "base_link"
    # trajectory.joint_trajectory.header.stamp = rospy.Time().now()
    # trajectory.joint_trajectory.joint_names = left_hand_group.get_active_joints()
    # trajectory.joint_trajectory.points = []
    # start_time = rospy.Time().now()
    # for i in range(0,15):
    # # for i in range(14,-1,-1):
    #     rospy.loginfo(i)
    #     point = JointTrajectoryPoint()
    #     point.positions = [radians(2*i)]
    #     point.time_from_start = rospy.Time().now() - start_time
    #     trajectory.joint_trajectory.points.append(point)

    # trajectory_goal.goal.trajectory = deepcopy(trajectory)
    # rospy.loginfo(trajectory_goal)
    # trajectory_goal_pub.publish(trajectory_goal)
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
