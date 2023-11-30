#!/usr/bin/env python2

import rospy
import moveit_commander
import sys
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
import yaml
from time import sleep


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
    
        
    def mod_position(self, x = 0, y = 0, theta = 0):
      """ Modifies the end effector position in 2D

      Returns true when complete, false if failed

      z+ is straight out from end effector
      y- is directly out from plug
      """
      # tf_buffer = tf2_ros.Buffer()
      # tf_listener = tf2_ros.TransformListener(tf_buffer)
      # sleep(5.0)

      # frames_dict = yaml.safe_load(tf_buffer.all_frames_as_yaml())
      # frames_list = list(frames_dict.keys())
      # print(frames_list)
      print("Normal: ", self.arm_group.get_current_pose())
      self.arm_group.set_pose_reference_frame("tool0")
      new_pose = Pose()
      new_pose.position.x -= .05
   

      # new_pose = Pose()
      # new_pose = current_pose.pose
      # new_pose.position.x -= .05
      # print(new_pose)

      plan, fraction = self.arm_group.compute_cartesian_path([new_pose], 0.01,2)    
      # #s = self.arm_group.get_current_state()
      # plan = self.arm_group.retime_trajectory(s, plan, algorithm="iterative_spline_parameterization")
      success = self.arm_group.execute(plan)
      # print(success)

 
        

    def main(self):
      
      self.mod_position()


      rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cable_trace', anonymous=True)
    cable_trace = CableTrace()
    cable_trace.main()
