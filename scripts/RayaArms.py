#!/usr/bin/python
import sys
import os
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
from raya_arms_planner.msg import MoveArmPoseAction, MoveArmPoseActionGoal, MoveArmPoseActionResult

class RayaArms():
    def __init__(self):
        self.action_server = actionlib.SimpleActionServer("move_arm_pose"\
            ,MoveArmPoseAction,self.move_arm_to_pose)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.left_move_group = MoveGroupCommander("left_arm")
        self.right_move_group = MoveGroupCommander("right_arm")
        self.action_server.start()
        self._result = MoveArmPoseActionResult()
        rospy.loginfo("raya arms planner initialized")

    def move_arm_to_pose(self,goal):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "base_link"
        goal_pose.header.seq = rospy.Time().now()
        goal_pose.pose = deepcopy(goal.goal_pose)
        rospy.loginfo("got goal pose\n\n{}".format(goal_pose))
        self.left_move_group.set_pose_target(goal_pose)
        plan = self.left_move_group.plan()
        if not plan:
            raise ArmsError("plan failed, pose not in range of arms")
            self.action_server.set_aborted(text="aborting move arm to pose action")
        self.left_move_group.execute(plan,wait=True)
        self.left_move_group.stop()
        self.left_move_group.clear_pose_targets()
        self._result.result.goal_reached = True
        rospy.loginfo(self._result)
        self.action_server.set_succeeded()
        rospy.loginfo("arm movement executed succesfuly")
        


class ArmsError(RuntimeError):
    def __init__(self,message):
        self.message = message

    def __str__(self):
        return self.message

if __name__ == "__main__":
    rospy.init_node("raya_arms_server")
    server = RayaArms()
    rospy.spin()