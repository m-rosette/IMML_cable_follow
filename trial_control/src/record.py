#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int64
import os
import numpy as np
from papillarray_ros_v2.msg import SensorState
from time import sleep
import csv
from trial_control.msg import RecordAction, RecordGoal, RecordFeedback, RecordResult
from actionlib import SimpleActionServer
from copy import deepcopy as copy
import threading
import time
import shutil
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from tf.transformations import *


# Serves as an action client that starts data recording (saves data just to csv)

class Record:
    def __init__(self):
        self.storage_directory = '/data/grip_data/'
        # The file number we are saving 
        # self.file_num = self.get_start_file_index()

        self.mutex = threading.Lock()
        self.r = rospy.Rate(20)

        # Initialize dictionaries to store data from subscribers
        self.initialize_tactile_dict()

        # Subscribe to gripper position feedback 
        self.gripper_pos_sub = rospy.Subscriber('Gripper_Pos', Int64, self.pos_callback)

        # Subscribe to tactile sensor feedback
        self.tactile_0_sub = rospy.Subscriber('hub_0/sensor_0', SensorState, self.tactile_0_callback)
        self.tactile_1_sub = rospy.Subscriber('hub_0/sensor_1', SensorState, self.tactile_1_callback)

        # Create a TransformListener object
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Initialize a timer to call the polling function at a specified interval (e.g., 10 Hz)
        self.timer = rospy.Timer(rospy.Duration(1.0 / 20), self.poll_transform)

        # Create action server
        self.record_server = SimpleActionServer("record_server", RecordAction, execute_cb=self.action_callback, auto_start=False)
        self.record_server.start()
        self.result = RecordResult()
        rospy.loginfo("Everything up!")      
   
    def action_callback(self, goal):
        self.file_name = goal.file_name
        # self.file_name = "0_0"
        rospy.loginfo("Recording starting. Saving to %s.csv", self.file_name)

        # Combine all of the data into one dictionary
        self.mutex.acquire()
        combined_dict = copy(self.current_time)
        combined_dict.update(self.position)
        combined_dict.update(self.ur5_y_pos)
        combined_dict.update(copy(self.tactile_0))#.update(copy(self.tactile_1))
        combined_dict.update(copy(self.tactile_1))
        self.mutex.release()
        t1 = time.time()
        # print("tactile: ", self.tactile_0)
        with open(self.storage_directory + str(self.file_name) + '.csv', 'w') as csvfile:
            # Write the header
            w = csv.DictWriter(csvfile, combined_dict)
            w.writeheader()

            while not self.record_server.is_preempt_requested() and not rospy.is_shutdown():
                self.current_time['timestamp'] = time.time() 

                # Combine all of the data into one dictionary
                t2 = time.time()
                delta = t2-t1
                if delta > 7.5:
                    break
                self.mutex.acquire()
                combined_dict = copy(self.current_time)
                combined_dict.update(self.position)
                combined_dict.update(self.ur5_y_pos)
                combined_dict.update(copy(self.tactile_0))#.update(copy(self.tactile_1))
                combined_dict.update(copy(self.tactile_1))
                self.mutex.release()
                w.writerow(combined_dict)
                self.r.sleep()
        shutil.copy(self.storage_directory + str(self.file_name) + '.csv', self.storage_directory + 'temp_0.csv')
        # Read in csv and convert to float32 array
        my_data = np.genfromtxt(self.storage_directory + str(self.file_name) + '.csv', delimiter=',')
        # Get just the last 30 rows
        my_data = my_data[-30:].astype(dtype=np.float32)
        # Flatten
        flattened_data = my_data.flatten()
        print(flattened_data)

        # Convert to result msg type
        self.result.data = flattened_data
        print(self.result.data.shape)
    

        # Return the data
        self.record_server.set_succeeded(self.result)
        rospy.loginfo("Recording stopped.")

    def pos_callback(self, pos_in):
        # Saves the subscribed gripper position data to variable
        self.position['gripper_pos'] = pos_in.data

    def poll_transform(self, event):
        # Lookup the transform from the base frame to the tool0 frame
        try:
            trans = self.tf_buffer.lookup_transform("base_link", "tool0", rospy.Time(0), rospy.Duration(1.0))
            self.ur5_y_pos['ur5_y_pos'] = trans.transform.translation.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Transform lookup failed")

    def tactile_0_callback(self, tac_in):
        # Saves the subscribed tactile 0 data to variable
        #self.tactile_0 = tac_in
        self.mutex.acquire()
        self.tactile_0 = {} 
        for i in range(9):
            
            self.tactile_0['0_dX_'+str(i)] = tac_in.pillars[i].dX
            self.tactile_0['0_dY_'+str(i)] = tac_in.pillars[i].dY
            self.tactile_0['0_dZ_'+str(i)] = tac_in.pillars[i].dZ
            self.tactile_0['0_fX_'+str(i)] = tac_in.pillars[i].fX
            self.tactile_0['0_fY_'+str(i)] = tac_in.pillars[i].fY
            self.tactile_0['0_fZ_'+str(i)] = tac_in.pillars[i].fZ
            self.tactile_0['0_incontact_'+str(i)] = tac_in.pillars[i].in_contact
            self.tactile_0['0_slipstate_'+str(i)] = tac_in.pillars[i].slip_state

        self.tactile_0['0_gfx'] = tac_in.gfX
        self.tactile_0['0_gfy'] = tac_in.gfY
        self.tactile_0['0_gfz'] = tac_in.gfZ
        self.tactile_0['0_gtx'] = tac_in.gtX
        self.tactile_0['0_gty'] = tac_in.gtY
        self.tactile_0['0_gtz'] = tac_in.gtZ
        self.tactile_0['0_friction_est'] = tac_in.friction_est
        self.tactile_0['0_target_grip_force'] = tac_in.target_grip_force
        self.tactile_0['0_is_sd_active'] = tac_in.is_sd_active
        self.tactile_0['0_is_ref_loaded'] = tac_in.is_ref_loaded
        self.tactile_0['0_is_contact'] = tac_in.is_contact  
        self.mutex.release()          
            
    def tactile_1_callback(self, tac_in):
        # Saves the subscribed tactile 1 data to variable
        self.mutex.acquire()
        self.tactile_1 = {}
        for i in range(9):
            
            self.tactile_1['1_dX_'+str(i)] = tac_in.pillars[i].dX
            self.tactile_1['1_dY_'+str(i)] = tac_in.pillars[i].dY
            self.tactile_1['1_dZ_'+str(i)] = tac_in.pillars[i].dZ
            self.tactile_1['1_fX_'+str(i)] = tac_in.pillars[i].fX
            self.tactile_1['1_fY_'+str(i)] = tac_in.pillars[i].fY
            self.tactile_1['1_fZ_'+str(i)] = tac_in.pillars[i].fZ
            self.tactile_1['1_incontact_'+str(i)] = tac_in.pillars[i].in_contact
            self.tactile_1['1_slipstate_'+str(i)] = tac_in.pillars[i].slip_state

        self.tactile_1['1_gfx'] = tac_in.gfX
        self.tactile_1['1_gfy'] = tac_in.gfY
        self.tactile_1['1_gfz'] = tac_in.gfZ
        self.tactile_1['1_gtx'] = tac_in.gtX
        self.tactile_1['1_gty'] = tac_in.gtY
        self.tactile_1['1_gtz'] = tac_in.gtZ
        self.tactile_1['1_friction_est'] = tac_in.friction_est
        self.tactile_1['1_target_grip_force'] = tac_in.target_grip_force
        self.tactile_1['1_is_sd_active'] = tac_in.is_sd_active
        self.tactile_1['1_is_ref_loaded'] = tac_in.is_ref_loaded
        self.tactile_1['1_is_contact'] = tac_in.is_contact   
        self.mutex.release()   

    def initialize_tactile_dict(self):
        """
        Initializes all of the keys in each ordered dictionary. This ensures the header and order is correct even if recording starts before data is published.
        """
        # Time
        self.current_time = {'timestamp': None}
        # Position
        self.position = {'gripper_pos': None}
        # UR5 Position
        self.ur5_y_pos = {'ur5_y_pos': 0}
        # Tactile sensor 
        self.tactile_0 = {}
        self.tactile_1 = {}

        for i in range(9):
            self.tactile_0['0_dX_'+str(i)] = None
            self.tactile_0['0_dY_'+str(i)] = None
            self.tactile_0['0_dZ_'+str(i)] = None
            self.tactile_0['0_fX_'+str(i)] = None
            self.tactile_0['0_fY_'+str(i)] = None
            self.tactile_0['0_fZ_'+str(i)] = None
            self.tactile_0['0_incontact_'+str(i)] = None
            self.tactile_0['0_slipstate_'+str(i)] = None

            self.tactile_1['1_dX_'+str(i)] = None
            self.tactile_1['1_dY_'+str(i)] = None
            self.tactile_1['1_dZ_'+str(i)] = None
            self.tactile_1['1_fX_'+str(i)] = None
            self.tactile_1['1_fY_'+str(i)] = None
            self.tactile_1['1_fZ_'+str(i)] = None
            self.tactile_1['1_incontact_'+str(i)] = None
            self.tactile_1['1_slipstate_'+str(i)] = None

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


    def get_start_file_index(self):
        # Returns the starting file number (old number)
        current_files = os.listdir(self.storage_directory)
        try:
            numbers = np.array([i.split('.csv', 1)[0] for i in current_files], dtype=int)
            return np.max(numbers) 
        except:
            return 0

    def main(self):
        # for i in range(30):
        #sleep(2)
        #self.action_callback('hey')
            # sleep(1)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('record', anonymous=True)
    record = Record()
    record.main()
