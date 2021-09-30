#!/usr/bin/python
from math import radians
import sys
import os
from moveit_commander import robot
import rospy
import roslib
import moveit_commander
import moveit_msgs.msg
from copy import deepcopy
from rospy.service import ServiceException

from urdf_parser_py.urdf import Robot
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
from moveit_msgs.msg import RobotTrajectory, ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint , JointTrajectory
from raya_msgs.msg import JointProperties
from raya_msgs.msg import ArmJointPlannerAction, ArmJointPlannerActionGoal, ArmJointPlannerActionFeedback, ArmJointPlannerActionResult,\
                          ArmPosePlannerAction, ArmPosePlannerActionGoal, ArmPosePlannerActionFeedback, ArmPosePlannerActionResult
from raya_msgs.srv import RobotModelInfo, RobotModelInfoRequest, RobotModelInfoResponse
from raya_msgs.srv import GripperCommand, GripperCommandRequest, GripperCommandResponse
from kdl_parser_py import urdf


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
                if len(self.left_arm_plan.joint_trajectory.joint_names) == 0:
                    raise MoveItCommanderException
            except MoveItCommanderException:
                rospy.loginfo("failed to plan a trajectory for goal pose, is the pose in bounds?")
                self._result.result.goal_reached = False
                self._result.result.arm = "left"
                self.action_server.set_aborted(result=self._result.result,text="failed to get plan for left arm")
                return
            except Exception as e:
                rospy.loginfo(e)
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
                if len(self.right_arm_plan.joint_trajectory.joint_names) == 0:
                    raise MoveItCommanderException
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

class RobotModelInfoService:
    def __init__(self):
        self.service = rospy.Service("robot_model_info",\
            RobotModelInfo,self.handle_info_request)
        self.robot_commander = RobotCommander()
        rospy.loginfo("robot model info service initialized")
    def handle_info_request(self,request):
        rospy.loginfo("retreiving {} info".format(request.move_group))
        robot_model = Robot().from_parameter_server()
        joint_dict = robot_model.joint_map
        response = RobotModelInfoResponse()
        try:
            active_joint_names = self.robot_commander.get_active_joint_names(request.move_group)
        except:
            rospy.logerr("failed to retrieve joint properties, not a vaible move group")
            raise ServiceException("not a viable move group")
        response.move_group = request.move_group
        for joint_name in active_joint_names:
            joint_properties = JointProperties()
            joint_properties.name = joint_name
            joint_properties.type = joint_dict[joint_name].type
            joint_properties.lower_limit = joint_dict[joint_name].limit.lower
            joint_properties.upper_limit = joint_dict[joint_name].limit.upper
            response.joints.append(joint_properties)
        return response


class GripperController:
    def __init__(self,gripper_travel=15):
        self.service = rospy.Service("control_gripper",\
            GripperCommand, self.handle_gripper_command)
        self.trajectory_pub = rospy.Publisher("execute_trajectory/goal",\
            ExecuteTrajectoryActionGoal,queue_size=10)
        rospy.loginfo("gripper controller started")
        self.left_hand_state = "open"
        self.right_hand_state = "open"      
        self.left_hand_group = MoveGroupCommander("left_hand")
        self.right_hand_group = MoveGroupCommander("right_hand")
        self.GRIPPER_TRAVEL = gripper_travel
        
    def handle_gripper_command(self,request):

        # request = GripperCommandRequest() #/TODO don't forget to comment out
        trajectory_goal = ExecuteTrajectoryActionGoal()
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.header.frame_id = "base_link"
        trajectory.joint_trajectory.header.stamp = rospy.Time().now()
        trajectory.joint_trajectory.points = []
        response = GripperCommandResponse()
        if "left" in request.gripper:
            if request.state in self.left_hand_state:
                rospy.loginfo(self.left_hand_state)
                raise ServiceException("gripper already in desired state")
            trajectory.joint_trajectory.joint_names = self.left_hand_group.get_active_joints()
            trajectory.joint_trajectory.points = self.calculate_trajectory(request.state)
            self.left_hand_state = request.state
            response.final_state = self.left_hand_state
        elif "right" in request.gripper:
            if request.state in self.left_hand_state:
                raise ServiceException("gripper already in desired state")
            trajectory.joint_trajectory.joint_names = self.right_hand_group.get_active_joints()
            trajectory.joint_trajectory.points = self.calculate_trajectory(request.state)
            self.left_hand_state = request.state
            response.final_state = self.left_hand_state
        else:
            raise ServiceException("incorrect gripper name, choose \"left\" or \"right\"")

        trajectory_goal.goal.trajectory = trajectory
        rospy.loginfo(trajectory_goal)
        self.trajectory_pub.publish(trajectory_goal)
        
        return response
            
    def calculate_trajectory(self,state):
        points = []
        duration = rospy.Duration(0,2e7)
        if "close" in state:
            for i in range(0,self.GRIPPER_TRAVEL):
                point = JointTrajectoryPoint()
                point.positions = [radians(i)]
                point.time_from_start += duration
                points.append(point)
            return points
        elif "open" in state:
            for i in range(self.GRIPPER_TRAVEL-1,-1,-1):
                point = JointTrajectoryPoint()
                point.positions = [radians(i)]
                point.time_from_start += duration
                points.append(point)
            return points
        else:
            raise ServiceException("requested state is incorrect, choose \"open\" or \"closed\"")
            

            