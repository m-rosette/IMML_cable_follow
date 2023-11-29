#!/usr/bin/env python2

import rospy
import moveit_commander
import sys


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
        arm_group_name = "arm"
        self.robot = moveit_commander.RobotCommander("robot_description")
        self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
        self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
        self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
        
        self.arm_group.allow_replanning(1)  
        

        

    def main(self):
      
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cable_trace', anonymous=True)
    cable_trace = CableTrace()
    cable_trace.main()










#!/usr/bin/env python
#modified from example code in kortex_examples
import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import *
import math
import shape_msgs.msg
from sensor_msgs.msg import JointState
import actionlib
from infrastructure_msgs.msg import StageAction, StageGoal, StageFeedback, StageResult
from copy import deepcopy
import roslaunch

class GraspDatasetTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(GraspDatasetTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')
    self.pose_list = []
    self.pose_num = 0 # used to determine if to move to top or side starting pos

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
      
      self.arm_group.allow_replanning(1)  
      
      

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())

      # Start action server callback
      self.start_arm = actionlib.SimpleActionServer("start_arm_sequence", StageAction, self.start_arm_sequence_callback, False)
      self.start_arm.start()


      
      #rospy.Timer(rospy.Duration(.05), self.gripper_status_publish)
      
      #self.remove_box("base")
      self.add_box(pos=[.75,0,-.05], dim=[2,2,.1], name="based")
      rospy.sleep(1)
      #self.remove_box("cube")
      self.add_box(pos=[.583,0,.05], dim=[.053,.053,.12], name="cube")
      #self.remove_box("cam_stand")
      self.add_box(pos=[.62,0,.9], dim=[.1, .1, .5], name="cam_stand")
      #self.remove_box("backdrop")
      self.add_box(pos=[1.08,0,.5], dim=[.1, 2, 1], name="backdrop")
      #self.remove_box("top")
      self.add_box(pos=[.08,0,1.1], dim=[1, 1, .1], name="top")
      self.add_box(pos=[.75,0,-.05], dim=[2,2,.1], name="based2")
      self.add_box(pos=[0,.41,0], dim=[2,.1,2], name="drape")
      self.add_box(pos=[.583,-.34,0.5], dim=[1.5,.04,1], name="right_blocker")

      self.is_init_success = True

      self.generation_start_pose = self.get_cartesian_pose()
      #self.generate_poses_top_XTRANS() # done!
      #self.generate_poses_side_XTRANS() # done!
      #self.generate_poses_top_ZROT() # done! 
      #self.generate_poses_top_YROT() # done!
      #self.generate_poses_top_YTRANS() # done!
      # self.generate_poses_top_XROT() # done!
      # self.generate_poses_side_ZROT() # done!

      self.generate_poses_side_XROT() # 20
      #self.generate_poses_side_YTRANS() # 10
      self.generate_start_poses()
      rospy.loginfo("Arm poses generated.")



      

    
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True
    rospy.spin()

  def plan_cartestian(self, pose):
    waypoints = []
    waypoints.append(deepcopy(pose))
    plan, fraction = self.arm_group.compute_cartesian_path(waypoints, 0.01,1)    
    success = self.arm_group.execute(plan)
    return success
 

  def set_cube_in_goal(self):
    actual_pose = self.get_cartesian_pose()
    actual_pose.position.x = .58
    actual_pose.position.y = .25
    actual_pose.position.z = .13
    waypoints = []
    waypoints.append(deepcopy(actual_pose))
    plan, fraction = self.arm_group.compute_cartesian_path(waypoints, 0.01,2)    
    #s = self.arm_group.get_current_state()
    #plan = self.arm_group.retime_trajectory(s, plan, algorithm="iterative_spline_parameterization")
    success = self.arm_group.execute(plan)
    
    #success = self.reach_cartesian_pose(pose=actual_pose, tolerance=0.02, constraints=None)
    #self.arm_group.clear_path_constraints()
    return success
   


  def remove_box(self, name):
    self.scene.remove_world_object(name)
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < 5) and not rospy.is_shutdown():
      # Test if the box is in the scene.
      is_known = name in self.scene.get_known_object_names()
      if is_known == False:
        return True
      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False

  def add_box(self, dim, pos, name):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = pos[0] # slightly above the end effector
    box_pose.pose.position.y = pos[1] # slightly above the end effector    
    box_pose.pose.position.z = pos[2] # slightly above the end effector
    box_name = name
    self.scene.add_box(box_name, box_pose, size=(dim[0], dim[1], dim[2]))

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < 5) and not rospy.is_shutdown():
      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in self.scene.get_known_object_names()
      # Test if we are in the expected state
      if is_known == True:
        return True
      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    # If we exited the while loop without returning then we timed out
    return False

  def reach_named_position(self, target):
    arm_group = self.arm_group
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    planned_path1 = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(planned_path1, wait=True)

  def reach_joint_angles(self, tolerance, joint_angles):  
    arm_group = self.arm_group
    success = True
    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)
    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)
    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      joint_positions[0] = joint_angles[0]
      joint_positions[1] = joint_angles[1]
      joint_positions[2] = joint_angles[2]
      joint_positions[3] = joint_angles[3]
      joint_positions[4] = joint_angles[4]
      joint_positions[5] = joint_angles[5]
      joint_positions[6] = joint_angles[6]
    arm_group.set_joint_value_target(joint_positions)
    # Plan and execute in one command
    success &= arm_group.go(wait=True)
    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group
    print(self.arm_group.get_planning_frame)
    # Get the current pose and display it
    pose = arm_group.get_current_pose("tool_frame")
    #rospy.loginfo("Actual cartesian pose is : ")
    #rospy.loginfo(pose.pose)
    return pose.pose

  def reach_cartesian_pose_path(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)
    rospy.loginfo("Planning and going to the Cartesian Pose")
    # 

    for i in range(2):
      plan, fraction = arm_group.compute_cartesian_path([pose],0.01,2)      
      worked = arm_group.execute(plan, wait=True)
      if fraction < 1.0:
        continue
      else:
        break
      if fraction < 1.0 and i == 1:
        rospy.logerr("Path doesn't complete.")
    return worked 

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose, end_effector_link="tool_frame") #arm_group.compute_cartesian_path([pose],0.01,2)#
    print("fractionm")
    #print(fraction)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    plan = arm_group.plan()
    worked = arm_group.execute(plan, wait=True)
    return worked 

  def reach_gripper_position(self, relative_position):
    #gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 

def main():
  example = GraspDatasetTrajectories()
  # example.execute_poses()
  #sexample.test_pose_list()

if __name__ == '__main__':
  main()