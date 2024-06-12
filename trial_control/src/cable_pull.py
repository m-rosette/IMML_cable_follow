#!/usr/bin/env python3


import rospy
import time
import threading
from std_msgs.msg import String, Int16
from actionlib import SimpleActionClient
from papillarray_ros_v2.msg import SensorState
from trial_control.msg import PullAction, PullGoal, PullFeedback, PullResult
from trial_control.srv import PictureTrigger, PictureTriggerResponse, TactileGlobal, CablePullTrigger
import os
import numpy as np


class CablePull:
    def __init__(self):
        # Initialize gripper variables
        self.grip_current = 18.0
        self.grip_force_increment = 2
        self.grip_max = 80
        self.cable_state_tactile = True
        self.disconnection_factor = 0.6
        self.tactile_0_global_y_prev = 0
        self.tactile_1_global_y_prev = 0
        self.tactile_0_slipstate = []
        self.tactile_1_slipstate = []   
        self.tactile_0_global_xyz = []
        self.tactile_1_global_xyz = []
        self.global_x_max = 4.0
        self.global_y_max = 5.0
        self.global_z_max = 9.0
        self.global_z_exceeded = False
        self.global_y_exceeded = False    

        # Initialize sliding window slope variables
        self.preset_slope = 0.005    # Set your desired preset slope
        self.window_size = 10
        self.numerator_diff = []
        self.denominator_diff = []

        # Gripper Publisher
        self.gripper_pos_pub = rospy.Publisher('Gripper_Cmd', String, queue_size=5)

        # Subscribe to tactile sensor feedback
        self.tactile_0_sub = rospy.Subscriber('hub_0/sensor_0', SensorState, self.tactile_0_callback)
        self.tactile_1_sub = rospy.Subscriber('hub_0/sensor_1', SensorState, self.tactile_1_callback)

        # Create service server
        rospy.wait_for_service('tactile_global_check')
        rospy.loginfo("Global tactile service up, ready!")
        self.tactile_global = rospy.ServiceProxy('tactile_global_check', TactileGlobal)

        # Create action server
        self.pull_server = SimpleActionServer("pull_server", PullAction, execute_cb=self.pull_test_callback, auto_start=False)
        self.pull_server.start()

        rospy.loginfo("Everything up!")

    def check_threshold_force(self):
        if self.tactile_0_global_xyz[2] > global_z_threshold or self.tactile_1_global_xyz[2] > global_z_threshold:
            self.global_z_exceeded = True
            rospy.logwarn("Global tactile Z force threshold exceeded. Terminating cable pull")
            break
        
        if np.abs(self.tactile_0_global_xyz[1]) > global_y_threshold or np.abs(self.tactile_1_global_xyz[1]) > global_y_threshold:
            self.global_y_exceeded = True
            rospy.logwarn("Global tactile Y force threshold exceeded. Terminating cable pull")
            break

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

    def pull_test_callback(self):
        # Initialize lists to store data for slope calculation
        stepper_pos = []
        global_y_tactile_0 = []
        global_y_tactile_1 = []

        while self.cable_state_tactile:
            self.global_force = self.tactile_global()
            self.tactile_0_global_xyz = self.global_force.tactile0_global
            self.tactile_1_global_xyz = self.global_force.tactile1_global
            time.sleep(0.1)

            self.check_threshold_force()

            # Get current data for slope calculation and add to lists
            stepper_pos.append(self.cable_pull_jig_state.stepper_position)
            global_y_tactile_0.append(self.tactile_0_global_xyz[1])
            global_y_tactile_1.append(self.tactile_1_global_xyz[1])

            # Get slip slope
            current_slope_0 = self.slip_check(stepper_pos, global_y_tactile_0)
            current_slope_1 = self.slip_check(stepper_pos, global_y_tactile_1)

            # Slip detection
            if current_slope_0 is not None and current_slope_1 is not None:
                # If current slope is greater than the slip_slop -> increase grip
                if np.abs(current_slope_0) > self.preset_slope or np.abs(current_slope_1) > self.preset_slope:
                    self.grip_current += self.grip_force_increment
                    rospy.loginfo(f"Slip detected [New Current Goal: {self.grip_current}]")

                    self.gripper_pos_pub.publish(f"current_{str(self.grip_current)}")

            # Safety grip force check
            if self.grip_current > self.grip_max:
                rospy.logwarn('max grip reached')
                break
            
            # Check if there is cable disconnection
            # Get the average over the last three global y-force readings
            recent_tactile_0_avg = np.average(global_y_tactile_0[-3:])
            recent_tactile_1_avg = np.average(global_y_tactile_1[-3:])
            if np.abs(self.tactile_0_global_xyz[1]) < self.disconnection_factor * np.abs(recent_tactile_0_avg) or np.abs(self.tactile_1_global_xyz[1]) < self.disconnection_factor * np.abs(recent_tactile_1_avg):
                self.cable_state_tactile = False
                rospy.logwarn("Cable disconnected")
            
            # Update the previous global readings
            self.tactile_0_global_y_prev = self.tactile_0_global_xyz[1]
            self.tactile_1_global_y_prev = self.tactile_1_global_xyz[1]

        if self.global_y_exceeded or self.global_z_exceeded:
            # Stop moving the arm
            self.result = PullResult()
            self.result.end_condition = "global force exceeded"
            self.pull_server.set_succeeded(self.result)

            # Open the gripper
            self.gripper_pos_pub.publish("position_10000")
        
        if self.global_y_exceeded == False and self.global_z_exceeded == False:
            rospy.loginfo("Cable unseated")
            time.sleep(1)

            # Stop moving the arm
            self.result = PullResult()
            self.result.end_condition = "cable unseated"
            self.pull_server.set_succeeded(self.result)

            # Open the gripper
            self.gripper_pos_pub.publish("position_10000")
                

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cable_pull', anonymous=True)
    record = CablePull()
    record.main()