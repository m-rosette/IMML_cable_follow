#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

import os
import csv
from collections import OrderedDict
import threading

import time

from std_msgs.msg import Int16, Int64, Float64
from sensor_interfaces.msg import SensorState
from trial_control_msgs.action import RecordData


class Record(Node):
    def __init__(self):
        super().__init__('record')
        self.storage_directory = '/home/marcus/IMML/ros2_ws/src/IMML_cable_follow/trial_control/trial_control/resource/'

        self.mutex = threading.Lock()

        self.sample_rate = 0.05 # seconds

        # Initialize dictionaries to store data from subscribers
        self.initialize_tactile_dict()

        # Subscribe to gripper position and current and cable seatment status
        self.gripper_pos_sub = self.create_subscription(Int64, 'present_position', self.gripper_pos_callback, 10)
        self.gripper_current_sub = self.create_subscription(Float64, 'motor_current', self.gripper_current_callback, 10)
        self.cable_status_sub = self.create_subscription(Int16, 'cable_status', self.cable_status_callback, 10)

        # Subscribe to tactile sensor feedback
        self.tactile_0_sub = self.create_subscription(SensorState, 'hub_0/sensor_0', self.tactile_0_callback, 10)
        self.tactile_1_sub = self.create_subscription(SensorState, 'hub_0/sensor_1', self.tactile_1_callback, 10)

        # Create action server
        self.record_server = ActionServer(self, RecordData, "record_server", self.action_callback)
        self.get_logger().info("Everything up!")

    def action_callback(self, goal_handle):
        self.filename = goal_handle.request.filename
        file_path = os.path.join(self.storage_directory, f"{self.filename}.csv")        

        # Handle if no filename is given
        if self.filename == '':
            self.get_logger().error("Error: No filename provided. Terminating recording...")
            goal_handle.abort()
            return RecordData.Result(result=False)

        # Check if the file already exists
        if os.path.exists(file_path):
            self.get_logger().error(f"Error: File '{self.filename}' already exists in the directory. Terminating recording...")
            goal_handle.abort()
            return RecordData.Result(result=False)
        
        self.get_logger().info(f"Recording starting. Saving to {self.filename}")    

        with open(self.storage_directory + str(self.filename) + '.csv', 'w') as csvfile:
            # Write the header
            w = csv.DictWriter(csvfile, self.combine_dicts().keys())
            w.writeheader()

            while rclpy.ok() and goal_handle.is_active:
                self.current_time['timestamp'] = time.time() 

                # Combine all of the data into one dictionary
                combined_dict = self.combine_dicts()
                w.writerow(combined_dict)
                rclpy.spin_once(self)
                time.sleep(self.sample_rate)

            # Handle goal completion or preemption
            if goal_handle.is_active:
                goal_handle.succeed()
                self.get_logger().info("Recording stopped.")
            else:
                self.get_logger().info("Recording preempted.")

        # Set goal outcome
        goal_handle.succeed()
        self.get_logger().info("File successfully created")

        # Return result
        return RecordData.Result(result="Saved file")

    def combine_dicts(self):
        """
        Combines data from tactile_0 and tactile_1 dictionaries.
        """
        combined_dict = OrderedDict()
        self.mutex.acquire()
        combined_dict.update(self.current_time)
        combined_dict.update(self.cable_status)
        combined_dict.update(self.gripper_position)
        combined_dict.update(self.gripper_current)
        combined_dict.update(self.tactile_0)
        combined_dict.update(self.tactile_1)
        self.mutex.release()
        return combined_dict

    def tactile_0_callback(self, tac_msg):
        # Saves the subscribed tactile 0 data to variable
        self.mutex.acquire()
        for i in range(9):
            self.tactile_0[f'0_dX_{i}'] = tac_msg.pillars[i].dx
            self.tactile_0[f'0_dY_{i}'] = tac_msg.pillars[i].dy
            self.tactile_0[f'0_dZ_{i}'] = tac_msg.pillars[i].dz
            self.tactile_0[f'0_fX_{i}'] = tac_msg.pillars[i].fx
            self.tactile_0[f'0_fY_{i}'] = tac_msg.pillars[i].fy
            self.tactile_0[f'0_fZ_{i}'] = tac_msg.pillars[i].fz
            self.tactile_0[f'0_incontact_{i}'] = tac_msg.pillars[i].in_contact
            self.tactile_0[f'0_slipstate_{i}'] = tac_msg.pillars[i].slip_state

        self.tactile_0['0_gfx'] = tac_msg.gfx
        self.tactile_0['0_gfy'] = tac_msg.gfy
        self.tactile_0['0_gfz'] = tac_msg.gfz
        self.tactile_0['0_gtx'] = tac_msg.gtx
        self.tactile_0['0_gty'] = tac_msg.gty
        self.tactile_0['0_gtz'] = tac_msg.gtz
        self.tactile_0['0_friction_est'] = tac_msg.friction_est
        self.tactile_0['0_target_grip_force'] = tac_msg.target_grip_force
        self.tactile_0['0_is_sd_active'] = tac_msg.is_sd_active
        self.tactile_0['0_is_ref_loaded'] = tac_msg.is_ref_loaded
        self.tactile_0['0_is_contact'] = tac_msg.is_contact
        self.mutex.release()

    def tactile_1_callback(self, tac_msg):
        # Saves the subscribed tactile 1 data to variable
        self.mutex.acquire()
        for i in range(9):
            self.tactile_1[f'1_dX_{i}'] = tac_msg.pillars[i].dx
            self.tactile_1[f'1_dY_{i}'] = tac_msg.pillars[i].dy
            self.tactile_1[f'1_dZ_{i}'] = tac_msg.pillars[i].dz
            self.tactile_1[f'1_fX_{i}'] = tac_msg.pillars[i].fx
            self.tactile_1[f'1_fY_{i}'] = tac_msg.pillars[i].fy
            self.tactile_1[f'1_fZ_{i}'] = tac_msg.pillars[i].fz
            self.tactile_1[f'1_incontact_{i}'] = tac_msg.pillars[i].in_contact
            self.tactile_1[f'1_slipstate_{i}'] = tac_msg.pillars[i].slip_state

        self.tactile_1['1_gfx'] = tac_msg.gfx
        self.tactile_1['1_gfy'] = tac_msg.gfy
        self.tactile_1['1_gfz'] = tac_msg.gfz
        self.tactile_1['1_gtx'] = tac_msg.gtx
        self.tactile_1['1_gty'] = tac_msg.gty
        self.tactile_1['1_gtz'] = tac_msg.gtz
        self.tactile_1['1_friction_est'] = tac_msg.friction_est
        self.tactile_1['1_target_grip_force'] = tac_msg.target_grip_force
        self.tactile_1['1_is_sd_active'] = tac_msg.is_sd_active
        self.tactile_1['1_is_ref_loaded'] = tac_msg.is_ref_loaded
        self.tactile_1['1_is_contact'] = tac_msg.is_contact
        self.mutex.release()
    
    def gripper_pos_callback(self, pos_msg):
        # Saves the subscribed gripper position data to variable
        self.gripper_position['gripper_pos'] = pos_msg.data

    def gripper_current_callback(self, current_msg):
        # Saves the subscribed gripper current data to variable
        self.gripper_current['gripper_current'] = current_msg.data

    def cable_status_callback(self, cable_msg):
        # Saves the subscribed cable status data to variable
        self.cable_status['cable_status'] = cable_msg.data

    def initialize_tactile_dict(self):
        """
        Initializes all of the keys in each ordered dictionary. This ensures the header and order is correct even if recording starts before data is published.
        """
        self.current_time = OrderedDict({'timestamp': None} )
        self.cable_status = OrderedDict({'cable_status': None})
        self.gripper_position = OrderedDict({'gripper_pos': None})
        self.gripper_current = OrderedDict({'gripper_current': None})
        self.tactile_0 = OrderedDict()
        self.tactile_1 = OrderedDict()
        
        for i in range(9):
            self.tactile_0[f'0_dX_{i}'] = None
            self.tactile_0[f'0_dY_{i}'] = None
            self.tactile_0[f'0_dZ_{i}'] = None
            self.tactile_0[f'0_fX_{i}'] = None
            self.tactile_0[f'0_fY_{i}'] = None
            self.tactile_0[f'0_fZ_{i}'] = None
            self.tactile_0[f'0_incontact_{i}'] = None
            self.tactile_0[f'0_slipstate_{i}'] = None

            self.tactile_1[f'1_dX_{i}'] = None
            self.tactile_1[f'1_dY_{i}'] = None
            self.tactile_1[f'1_dZ_{i}'] = None
            self.tactile_1[f'1_fX_{i}'] = None
            self.tactile_1[f'1_fY_{i}'] = None
            self.tactile_1[f'1_fZ_{i}'] = None
            self.tactile_1[f'1_incontact_{i}'] = None
            self.tactile_1[f'1_slipstate_{i}'] = None

        self.tactile_0['0_gfx'] = None
        self.tactile_0['0_gfy'] = None
        self.tactile_0['0_gfz'] = None
        self.tactile_0['0_gtx'] = None
        self.tactile_0['0_gty'] = None
        self.tactile_0['0_gtz'] = None
        self.tactile_0['0_friction_est'] = None
        self.tactile_0['0_target_grip_force'] = None
        self.tactile_0['0_is_sd_active'] = None
        self.tactile_0['0_is_ref_loaded'] = None
        self.tactile_0['0_is_contact'] = None

        self.tactile_1['1_gfx'] = None
        self.tactile_1['1_gfy'] = None
        self.tactile_1['1_gfz'] = None
        self.tactile_1['1_gtx'] = None
        self.tactile_1['1_gty'] = None
        self.tactile_1['1_gtz'] = None
        self.tactile_1['1_friction_est'] = None
        self.tactile_1['1_target_grip_force'] = None
        self.tactile_1['1_is_sd_active'] = None
        self.tactile_1['1_is_ref_loaded'] = None
        self.tactile_1['1_is_contact'] = None


def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    record = Record()

    # Give control over to ROS2
    rclpy.spin(record)

    # Shutdown cleanly
    rclpy.shutdown()


if __name__ == '__main__':
    main()
