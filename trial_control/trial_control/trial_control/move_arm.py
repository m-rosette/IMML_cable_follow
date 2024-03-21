#!/usr/bin/env python2

import rospy
import moveit_commander
import sys
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose, PoseStamped
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

        tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(tfBuffer)

        self.transform1 = tfBuffer.lookup_transform("tool0", 'ee_gripper', rospy.Time.now(), rospy.Duration(1.0))
        # print(self.transform1)
        # self.transform = tfBuffer.lookup_transform("base_link", 'tool0', rospy.Time.now(), rospy.Duration(1.0))
        # print(self.transform)
    
        
    def mod_position(self, x = 0, y = 0, theta = 0):
      """ Modifies the end effector position in 2D

      Returns true when complete, false if failed

      z+ is straight out from end effector
      y- is directly out from plug
      """
      # # print(self.arm_group.get_pose_reference_frame())
      # # print(self.arm_group.get_current_pose())
      # # # print(self.arm_group.set_end_effector_link("ee_gripper"))
      # # self.arm_group.set_pose_reference_frame("ee_gripper")
      # # print(self.arm_group.get_pose_reference_frame())
      # # print(self.arm_group.get_current_pose())

      # # First step -> create pose and modify to be in tool0 frame
      # #self.arm_group.set_pose_reference_frame("tool0")
      # new_pose = self.arm_group.get_current_pose()
      
      # print("ahh")
      # print(new_pose)
      # hi = raw_input()
      # # We want to transform this pose to the ee_gripper 
      # pose_transformed = tf2_geometry_msgs.do_transform_pose(new_pose, self.transform)

      # print("This is the transformed into ee_gripper")
      # print(pose_transformed)
      # hi = raw_input()
      # # Now we want to modify the pose
      # # pose_transformed.pose.position.x -= .05

      # # About y if plug is upward
      # q_change = quaternion_from_euler(.5, 0, 0) 
      # current_q = deepcopy([pose_transformed.pose.orientation.x, pose_transformed.pose.orientation.y, pose_transformed.pose.orientation.z, pose_transformed.pose.orientation.w])
      # q_new = quaternion_multiply(q_change, current_q)
      # pose_transformed.pose.orientation.x = q_new[0]
      # pose_transformed.pose.orientation.y = q_new[1]
      # pose_transformed.pose.orientation.z = q_new[2]
      # pose_transformed.pose.orientation.w = q_new[3]







      # # Now we transform it back into tool0 frame
      # print("Pose in ee_gripper frame")
      # print(pose_transformed)

      # final_pose = tf2_geometry_msgs.do_transform_pose(pose_transformed, self.transform1)
      # print("Final pose")
      # print(final_pose)

      # print("here")

      # hi = raw_input()
      # # self.arm_group.plan(final_pose.pose)

      self.arm_group.set_pose_reference_frame("ee_gripper")
      final_pose = PoseStamped()
      # final_pose.pose.position.z -= .05
      # next_pose = tf2_geometry_msgs.do_transform_pose(final_pose, self.transform1)
      # last_pose = tf2_geometry_msgs.do_transform_pose(next_pose, self.transform)
      # print(last_pose)
      # q_change = quaternion_from_euler(-.5, 0, 0) 
      # final_pose.pose.orientation.x = q_change[0]
      # final_pose.pose.orientation.y = q_change[1]
      # final_pose.pose.orientation.z = q_change[2]
      # final_pose.pose.orientation.w = q_change[3]
      # final_pose = tf2_geometry_msgs.do_transform_pose(final_pose, self.transform1)
      plan, fraction = self.arm_group.compute_cartesian_path([final_pose.pose], 0.01,2)   
      success = self.arm_group.execute(plan)
      # Plan
      # self.arm_group.plan(final_pose)
      # self.arm_group.go()


      # # In pose/out pose frame
      # in_frame = "ee_gripper"
      # out_frame = "tool0"
      # # tf_buffer = tf2_ros.Buffer()
      # # tf_listener = tf2_ros.TransformListener(tf_buffer)
      # # sleep(5.0)

      # # frames_dict = yaml.safe_load(tf_buffer.all_frames_as_yaml())
      # # frames_list = list(frames_dict.keys())
      # # print(frames_list)
      
      # new_pose = Pose()
      # new_pose.position.z += .05

      # # trans_pose = self.transform_pose(new_pose, in_frame, out_frame)
 
      # self.arm_group.set_pose_reference_frame("ee_gripper")
      # self.arm_group.plan(trans_pose)
      # # new_pose = Pose()
      # # new_pose = current_pose.pose
      # # new_pose.position.x -= .05
      # # print(new_pose)

      # # plan, fraction = self.arm_group.compute_cartesian_path([new_pose], 0.01,2)    
      # # #s = self.arm_group.get_current_state()
      # # plan = self.arm_group.retime_trajectory(s, plan, algorithm="iterative_spline_parameterization")
      # #success = self.arm_group.execute(plan)
      # print(success)



    def main(self):
      
      self.mod_position()


      rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cable_trace', anonymous=True)
    cable_trace = CableTrace()
    cable_trace.main()
