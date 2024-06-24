#!/usr/bin/env python3


import rospy
import time
import threading
from std_msgs.msg import String, Int16
from actionlib import SimpleActionClient, SimpleActionServer
from papillarray_ros_v2.msg import SensorState
from papillarray_ros_v2.srv import *
from geometry_msgs.msg import Pose, PoseStamped
from trial_control.msg import PullAction, PullGoal, PullFeedback, PullResult, MoveAction, MoveGoal, MoveFeedback, MoveResult, RecordGoal, RecordAction, PullGoal, PullAction
from trial_control.srv import PictureTrigger, PictureTriggerResponse, TactileGlobal, CablePullTrigger
from geometry_msgs.msg import PoseStamped
import os
import numpy as np
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from tf.transformations import *

import matplotlib.pyplot as plt


class CablePull:
    def __init__(self):
        # Initialize gripper variables
        self.grip_current = 10.0
        self.grip_force_increment = 2
        self.grip_max = 40
        self.cable_state_tactile = True
        self.disconnection_factor = 0.7 # used 0.6 for IMECE
        self.tactile_0_global_y_prev = 0
        self.tactile_1_global_y_prev = 0
        self.tactile_0_slipstate = []
        self.tactile_1_slipstate = []   
        self.tactile_0_global_xyz = []
        self.tactile_1_global_xyz = []
        self.global_x_max = 4.0
        self.global_y_max = 2.5
        self.global_z_max = 9.0
        self.global_z_exceeded = False
        self.global_y_exceeded = False   
        self.ur5_y_pos = 0

        # Initialize sliding window slope variables
        self.preset_slope = 200.0    # Set your desired preset slope
        self.window_size = 10
        self.numerator_diff = []
        self.denominator_diff = []

        # Setup the recording action client
        self.record_ac = SimpleActionClient("record_server", RecordAction)
        rospy.loginfo("Waiting for recording server to come up.")
        self.record_ac.wait_for_server()
        rospy.loginfo("Recording server up!")

        # Gripper Publisher
        self.gripper_pos_pub = rospy.Publisher('Gripper_Cmd', String, queue_size=5)

        # Create action client to communicate with CableTrace
        self.move_client = SimpleActionClient("move_server", MoveAction)
        rospy.loginfo("Waiting for move_server...")
        self.move_client.wait_for_server()
        rospy.loginfo("move_server found.")

        # Subscribe to UR5
        # rospy.Subscriber('tool0', PoseStamped, self.ur5_callback)
        # Create a TransformListener object
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create service server
        rospy.wait_for_service('tactile_global')
        rospy.loginfo("Global tactile service up, ready!")
        self.tactile_global = rospy.ServiceProxy('tactile_global', TactileGlobal)

        # Create action server
        self.pull_server = SimpleActionServer("pull_server", PullAction, execute_cb=self.pull_test_callback, auto_start=False)
        self.pull_server.start()

        rospy.loginfo("Everything up!")

    def preempt_movement(self):
        # Send preempt request to CableTrace
        self.move_client.cancel_all_goals()
        rospy.loginfo("Canceling ur5 movement")

    def get_tool0_pose(self):
        try:
            # Lookup the transform from the base frame to the tool0 frame
            trans = self.tf_buffer.lookup_transform("base_link", "tool0", rospy.Time(0), rospy.Duration(1.0))

            # Create a PoseStamped message to store the pose of tool0
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "base_link"

            # Set the position
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            self.ur5_y_pos = trans.transform.translation.y
            # print(trans.transform.translation.y)
            pose.pose.position.z = trans.transform.translation.z

            # Set the orientation
            pose.pose.orientation.x = trans.transform.rotation.x
            pose.pose.orientation.y = trans.transform.rotation.y
            pose.pose.orientation.z = trans.transform.rotation.z
            pose.pose.orientation.w = trans.transform.rotation.w

            return pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed.")
            return None

    def bias_request_srv_client(self):
        rospy.wait_for_service('/hub_0/send_bias_request')
        srv = rospy.ServiceProxy('/hub_0/send_bias_request', BiasRequest)
        success = srv()
        return success

    # def ur5_callback(self):
    #     # rospy.loginfo(f"Received end effector position: {data.pose.position}")
    #     # rospy.loginfo(f"Received end effector orientation: {data.pose.orientation}")
    #     self.ur5_y_pos = data.pose.position.x

    def slip_check(self, x_data, y_data):
        self.numerator_diff = np.diff(y_data[-self.window_size-1:])
        self.denominator_diff = np.diff(x_data[-self.window_size-1:])
        
        # Calculate slope if there is no division by zero
        if len(self.numerator_diff) >= self.window_size:
            avg_num_diff = np.mean(self.numerator_diff)
            avg_denom_diff = np.mean(self.denominator_diff)
            if avg_denom_diff != 0:
                return avg_num_diff / avg_denom_diff
        return None

    def pull_test_callback(self, goal):
        # Initialize lists to store data for slope calculation
        ur5_y_pos_ls = []
        global_y_tactile_0 = []
        global_y_tactile_1 = []

        temp_tactile0_list = []
        temp_tactile1_list = []
        discon0 = []
        discon1 = []
        slipslope0 = []
        slipslope1 = []

        #open gripper
        self.gripper_pos_pub.publish(f"position_3300")

        self.bias_request_srv_client()
        rospy.loginfo("Biasing tactile sensors")

        # Send the filename to the record action (starts recording)
        goal = RecordGoal(file_name='unseated_failed_1')
        self.record_ac.send_goal(goal)
        # time.sleep(2.5)

        self.gripper_pos_pub.publish(f"current_10")
        rospy.loginfo("Closing the gripper")
        time.sleep(0.5)
        
        iteration = 0
        success = True  # Assume success initially

        while self.cable_state_tactile:
            if self.pull_server.is_preempt_requested():
                rospy.loginfo('Pull action preempted')
                self.pull_server.set_preempted()
                success = False
                break

            self.global_force = self.tactile_global()
            self.tactile_0_global_xyz = self.global_force.tactile0_global
            self.tactile_1_global_xyz = self.global_force.tactile1_global
            time.sleep(0.1)

            if self.tactile_0_global_xyz[2] > self.global_z_max or self.tactile_1_global_xyz[2] > self.global_z_max:
                self.global_z_exceeded = True
                rospy.logwarn("Global tactile Z force threshold exceeded. Terminating cable pull")
                # self.preempt_movement()
                success = False 
                break
            
            if np.abs(self.tactile_0_global_xyz[1]) > self.global_y_max or np.abs(self.tactile_1_global_xyz[1]) > self.global_y_max:
                self.global_y_exceeded = True
                rospy.logwarn("Global tactile Y force threshold exceeded. Terminating cable pull")
                # self.preempt_movement()
                success = False
                break

            # Get current data for slope calculation and add to lists
            self.get_tool0_pose()
            ur5_y_pos_ls.append(self.ur5_y_pos)
            global_y_tactile_0.append(self.tactile_0_global_xyz[1])
            global_y_tactile_1.append(self.tactile_1_global_xyz[1])

            # Get slip slope
            current_slope_0 = self.slip_check(ur5_y_pos_ls, global_y_tactile_0)
            current_slope_1 = self.slip_check(ur5_y_pos_ls, global_y_tactile_1)

            # Slip detection    
            if current_slope_0 is not None and current_slope_1 is not None and iteration < 40: # TEMPORARY ITERATION 50 CHECK ================================================

                slipslope0.append(np.abs(current_slope_0))
                slipslope1.append(np.abs(current_slope_1))

                # If current slope is greater than the slip_slop -> increase grip
                if np.abs(current_slope_0) > self.preset_slope or np.abs(current_slope_1) > self.preset_slope:
                    self.grip_current += self.grip_force_increment
                    rospy.loginfo(f"Slip detected [New Current Goal: {self.grip_current}]")
                    self.gripper_pos_pub.publish(f"current_{str(int(self.grip_current))}")

            # Safety grip force check
            if self.grip_current > self.grip_max:
                rospy.logwarn('max grip reached')
                plt.plot(discon0)
                # plt.plot(temp_tactile1_list)
                # plt.plot(slipslope1)
                plt.plot(discon1)
                # plt.show()
                plt.savefig('/root/catkin_ws/src/trial_control/src/images/plot.png')  # Save the plot to a file
                print(iteration)
                # self.preempt_movement()
                success = False
                break
            
            # Check if there is cable disconnection
            # Get the average over the last three global y-force readings
            recent_tactile_0_avg = np.average(global_y_tactile_0[-3:])
            recent_tactile_1_avg = np.average(global_y_tactile_1[-3:])

            # Saving data for plot validation
            temp_tactile0_list.append(np.abs(self.tactile_0_global_xyz[1]))
            temp_tactile1_list.append(np.abs(self.tactile_1_global_xyz[1]))
            discon0.append(self.disconnection_factor * np.abs(recent_tactile_0_avg))
            discon1.append(self.disconnection_factor * np.abs(recent_tactile_1_avg))

            # Checking for cable disconnection
            diff0 = np.abs(self.tactile_0_global_xyz[1]) - self.disconnection_factor * np.abs(recent_tactile_0_avg)
            diff1 = np.abs(self.tactile_1_global_xyz[1]) - self.disconnection_factor * np.abs(recent_tactile_1_avg)
            if iteration > 7:
                if diff0 < 0 or diff1 < 0:
                    self.cable_state_tactile = False
                    rospy.logwarn("Cable disconnected")
                    # self.preempt_movement()
                    success = False

            # if iteration > 5:
            #     if (np.abs(self.tactile_0_global_xyz[1]) - self.disconnection_factor * np.abs(recent_tactile_0_avg)) < 0:
            #     # if np.abs(self.tactile_0_global_xyz[1]) < self.disconnection_factor * np.abs(recent_tactile_0_avg): # or np.abs(self.tactile_1_global_xyz[1]) < self.disconnection_factor * np.abs(recent_tactile_1_avg):
            #         self.cable_state_tactile = False
            #         rospy.logwarn("Cable disconnected")
            
            # Update the previous global readings
            self.tactile_0_global_y_prev = self.tactile_0_global_xyz[1]
            self.tactile_1_global_y_prev = self.tactile_1_global_xyz[1]

            if self.global_y_exceeded or self.global_z_exceeded:
                # Stop moving the arm
                self.result = PullResult()
                self.result.end_condition = "global force exceeded"
                self.pull_server.set_succeeded(self.result)
                # self.preempt_movement()
                success = False 

                # Open the gripper
                self.gripper_pos_pub.publish("position_3300")
            
            # if self.global_y_exceeded == False and self.global_z_exceeded == False:
            #     rospy.loginfo("Cable unseated")
            #     time.sleep(1)

            #     # Stop moving the arm
            #     self.result = PullResult()
            #     self.result.end_condition = "cable unseated"
            #     self.pull_server.set_succeeded(self.result)
            #     self.preempt_movement()

            #     # Open the gripper
            #     self.gripper_pos_pub.publish("position_3300")
            
            iteration += 1

        self.preempt_movement()
        self.gripper_pos_pub.publish("position_3300")

        result = PullResult()
        result.end_condition = "success" if success else "failure"
        if success:
            self.pull_server.set_succeeded(result)
        else:
            self.pull_server.set_aborted(result)
        

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cable_pull', anonymous=True)
    record = CablePull()
    record.main()