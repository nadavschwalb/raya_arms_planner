#!/usr/bin/python
from os import name
from re import I, U
from kdl_parser_py import urdf
import rospy
from urdf_parser_py.urdf import Joint, JointLimit, Robot
from raya_arms_planner.RayaArmsPlanner import RobotModelInfoService
if __name__ == "__main__":
    rospy.init_node("robot_model_info")
    server = RobotModelInfoService()
    rospy.spin()
    
    
    
    