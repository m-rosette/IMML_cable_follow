#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from trial_control.msg import Movement
from trial_control.msg import MoveAction, MoveGoal, MoveFeedback, MoveResult
import yaml
from time import sleep
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from tf.transformations import *
from copy import deepcopy


# General idea - this is high level trial control

# Gripper_Cmd - publish gripper position/current values here
# UR_something - publish cartesian movement changes here
# __action_server - use this to start data collection
# camera_service - takes photo, saves it, and returns cable pose
# roslaunch ur5_moveit_config moveit_rviz.launch

class CableTrace:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        # Create the MoveItInterface necessary objects
        arm_group_name = "manipulator"
        self.robot = moveit_commander.RobotCommander("robot_description")
        self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
        self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
        self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                        DisplayTrajectory,
                                                        queue_size=20)
        
        self.arm_group.allow_replanning(1)  

        # Create action server
        self.move_server = SimpleActionServer("move_server", MoveAction, execute_cb=self.mod_position_callback, auto_start=False)
        self.move_server.start()
        rospy.loginfo("Move server up!")
    
        
    def mod_position_callback(self, goal):
      """ Modifies the end effector position in 2D

      Returns true when complete, false if failed

      z+ is straight out from end effector
      y- is directly out from plug
      """

      x = goal.x

      self.arm_group.set_pose_reference_frame("tool0")
      final_pose = PoseStamped()

      q_change = quaternion_from_euler(-.5, 0, 0) 
      final_pose.pose.orientation.x = q_change[0]
      final_pose.pose.orientation.y = q_change[1]
      final_pose.pose.orientation.z = q_change[2]
      final_pose.pose.orientation.w = q_change[3]
      # final_pose = tf2_geometry_msgs.do_transform_pose(final_pose, self.transform1)
      plan, fraction = self.arm_group.compute_cartesian_path([final_pose.pose], 0.01,2)   
      success = self.arm_group.execute(plan)
      # Plan
      # self.arm_group.plan(final_pose)
      # self.arm_group.go()




    def main(self):
      
      self.mod_position()


      rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cable_trace', anonymous=True)
    cable_trace = CableTrace()
    cable_trace.main()
