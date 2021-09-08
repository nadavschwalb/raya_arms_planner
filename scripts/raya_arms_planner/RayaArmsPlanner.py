#!/usr/bin/python
import sys
import os
import rospy
import roslib
import moveit_commander
import moveit_msgs.msg
from copy import deepcopy
roslib.load_manifest("raya_arms_planner")
import actionlib
from moveit_commander import move_group
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_commander.robot import RobotCommander
from moveit_commander.exception import MoveItCommanderException
from sensor_msgs.msg import JointState
from rosgraph.names import SEP
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler

from raya_msgs.msg import ArmJointPlannerAction, ArmJointPlannerActionGoal, ArmJointPlannerActionFeedback, ArmJointPlannerActionResult,\
                          ArmPosePlannerAction, ArmPosePlannerActionGoal, ArmPosePlannerActionFeedback, ArmPosePlannerActionResult

class JointGoalPlanner:
    def __init__(self):
        self.action_server = actionlib.SimpleActionServer("arms_joint_planner",ArmJointPlannerAction,execute_cb=self.plan_joint_action,auto_start=False)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.left_move_group = MoveGroupCommander("left_arm")
        self.left_arm_plan = move_group.RobotTrajectory()
        self.right_move_group = MoveGroupCommander("right_arm")
        self.right_arm_plan = move_group.RobotTrajectory()
        self.action_server.start()
        self._result = ArmJointPlannerActionResult()
        self._goal = ArmJointPlannerActionGoal()
        # rospy.Subscriber("move_group/fake_controller_joint_states",JointState,callback=self.update_progress)
        # self.left_arm_progress = 0
        # self.right_arm_progress = 0
        rospy.loginfo("raya Joint Goal Planner initialized")
    
    def plan_joint_action(self,joint_goal):
        rospy.loginfo("recieved joint goal\n{}".format(joint_goal))
    
        if("left" in joint_goal.arm):
            try:
                self.left_arm_plan = self.left_move_group.plan(joint_goal.position)
            except MoveItCommanderException:
                rospy.loginfo("failed to plan for joint goal, is the goal in bounds?")
                self._result.result.goal_reached = False
                self._result.result.arm = "left"
                self.action_server.set_aborted(result=self._result.result,text="failed to get plan for left arm")
                return
            
            # self.left_arm_progress = 0
            self.left_move_group.execute(self.left_arm_plan)
            rospy.loginfo("goal reached")
            self._result.result.goal_reached = True
            self._result.result.arm = "left"
            self.action_server.set_succeeded(result=self._result.result,text="left arm goal reached")

        elif("right" in joint_goal.arm):
            try:
                self.right_arm_plan = self.right_move_group.plan(joint_goal.position)

            except MoveItCommanderException:
                rospy.loginfo("failed to plan for joint goal, is the goal in bounds?")
                self._result.result.goal_reached = False
                self._result.result.arm = "right"
                self.action_server.set_aborted(result=self._result.result,text="failed to get plan for right arm")
                return

            # self.right_arm_progress = 0
            self.right_move_group.execute(self.right_arm_plan,wait=True)
            rospy.loginfo("goal reached")
            self._result.result.goal_reached = True
            self._result.result.arm = "right"
            self.action_server.set_succeeded(result=self._result.result,text="right arm goal reached")
        else:
            rospy.loginfo("not a move group")
    
    # def update_progress(self,joint_state_msg):      
    #     if("left" in joint_state_msg.name[0]):
    #         self.left_arm_progress+=1
    #         feedback = ArmJointPlannerActionFeedback()
    #         feedback.header.seq = self.left_arm_progress
    #         feedback.header.stamp = rospy.Time().now()
    #         trajectory_len = len(self.left_arm_plan.joint_trajectory.points)
    #         if(trajectory_len <= 0):
    #             return
    #         feedback.feedback.percentage_complete = float(self.left_arm_progress/trajectory_len)
    #         rospy.loginfo("progress: {} total: {}".format(self.left_arm_progress,trajectory_len))
    #         self.action_server.publish_feedback(feedback.feedback)
    #     elif("right" in joint_state_msg.name[0]):
    #         self.right_arm_progress += 1
    #         feedback = ArmJointPlannerActionFeedback()
    #         feedback.header.seq = self.right_arm_progress
    #         feedback.header.stamp = rospy.Time().now()
    #         trajectory_len = len(self.right_arm_plan.joint_trajectory.points)
    #         if(trajectory_len <= 0):
    #             return
    #         feedback.feedback.percentage_complete = float(self.right_arm_progress/trajectory_len)
    #         rospy.loginfo("progress: {} total: {}".format(self.right_arm_progress,trajectory_len))
    #         self.action_server.publish_feedback(feedback.feedback)
    #     else:
    #         return


class PoseGoalPlanner:
    def __init__(self):
        self.action_server = actionlib.SimpleActionServer("arms_pose_planner",ArmPosePlannerAction,execute_cb=self.plan_pose_goal,auto_start=False)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.left_move_group = MoveGroupCommander("left_arm")
        self.left_arm_plan = move_group.RobotTrajectory()
        self.right_move_group = MoveGroupCommander("right_arm")
        self.right_arm_plan = move_group.RobotTrajectory()
        self.action_server.start()
        self._result = ArmJointPlannerActionResult()
        self._goal = ArmJointPlannerActionGoal()
    
    def plan_pose_goal(self,pose_goal):
        rospy.loginfo("recieved arm pose goal\n{}".format(pose_goal))

        if("left" in pose_goal.arm):
            try:
                self.left_move_group.set_pose_target(pose_goal.goal_pose)
                self.left_arm_plan = self.left_move_group.plan()
            except MoveItCommanderException:
                rospy.loginfo("failed to plan a trajectory for goal pose, is the pose in bounds?")
                self._result.result.goal_reached = False
                self._result.result.arm = "left"
                self.action_server.set_aborted(result=self._result.result,text="failed to get plan for left arm")
                return
            self.left_move_group.execute(self.left_arm_plan,wait=True)
            self.left_move_group.stop()
            self.left_move_group.clear_pose_targets()
            rospy.loginfo("goal reached")
            self._result.result.goal_reached = True
            self._result.result.arm = "left"
            self.action_server.set_succeeded(result=self._result.result,text="left arm goal reached")

        elif("right" in pose_goal.arm):
            try:
                self.right_move_group.set_pose_target(pose_goal.goal_pose)
                self.right_arm_plan = self.right_move_group.plan()
            except MoveItCommanderException:
                rospy.loginfo("failed to plan a trajectory for goal pose, is the pose in bounds?")
                self._result.result.goal_reached = False
                self._result.result.arm = "right"
                self.action_server.set_aborted(result=self._result.result,text="failed to get plan for right arm")
                return
            self.right_move_group.execute(self.right_arm_plan,wait=True)
            self.right_move_group.stop()
            self.right_move_group.clear_pose_targets()
            rospy.loginfo("goal reached")
            self._result.result.goal_reached = True
            self._result.result.arm = "right"
            self.action_server.set_succeeded(result=self._result.result,text="right arm goal reached")
        else:
            rospy.loginfo("not a move group")

            

            