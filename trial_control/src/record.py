#!/usr/bin/env python2

import rospy
from std_msgs.msg import String, Int64
import os
import numpy as np
from papillarray_ros_v2.msg import SensorState
from time import sleep
import csv
from collections import OrderedDict
from trial_control.msg import RecordAction, RecordGoal, RecordFeedback, RecordResult
from actionlib import SimpleActionServer
from copy import deepcopy as copy

# Serves as an action client that starts data recording (saves data just to csv)

class Record:
    def __init__(self):
        self.actively_reading = False
        self.storage_directory = '/root/data'
        # The file number we are saving 
        self.file_num = self.get_start_file_index()

        # Initialize dictionaries to store data from subscribers
        self.initialize_tactile_dict()

        # Subscribe to gripper position feedback 
        self.gripper_pos_sub = rospy.Subscriber('Gripper_Pos', Int64, self.pos_callback)

        # Subscribe to tactile sensor feedback
        self.tactile_0_sub = rospy.Subscriber('hub_0/sensor_0', SensorState, self.tactile_0_callback)
        self.tactile_1_sub = rospy.Subscriber('hub_0/sensor_1', SensorState, self.tactile_1_callback)

        # Create action server
        self.record_server = SimpleActionServer("record_server", RecordAction, execute_cb=self.action_callback, auto_start=False)
        self.record_server.start()
        rospy.loginfo("Everything up!")

        self.storage_directory = '/root/data'


        
   
    def action_callback(self, goal):
        self.file_num += 1
        rospy.loginfo("Recording starting. Saving to %d.csv", self.file_num)

        # Combine all of the data into one dictionary
        self.actively_reading = True
        rospy.sleep(.01)
        combined_dict = OrderedDict(copy(self.position).items() + copy(self.tactile_0).items() + copy(self.tactile_1).items())
        self.actively_reading = False

        # print("tactile: ", self.tactile_0)
        with open('/root/data/' + str(self.file_num) + '.csv', 'w') as csvfile:
            # Write the header
            w = csv.DictWriter(csvfile, combined_dict)
            w.writeheader()

            while not self.record_server.is_preempt_requested() and not rospy.is_shutdown():
                # Combine all of the data into one dictionary
                self.actively_reading = True
                rospy.sleep(.01)
                combined_dict = OrderedDict(copy(self.position).items() + copy(self.tactile_0).items() + copy(self.tactile_1).items())
                self.actively_reading = False
                w.writerow(combined_dict)
                rospy.sleep(.1)

            rospy.loginfo("Recording stopped.")

    def pos_callback(self, pos_in):
        # Saves the subscribed gripper position data to variable
        self.position['gripper_pos'] = pos_in.data

    def tactile_0_callback(self, tac_in):
        # Saves the subscribed tactile 0 data to variable
        #self.tactile_0 = tac_in
        while self.actively_reading:
            rospy.sleep(.01)
        self.tactile_0 = OrderedDict() 
        for i in range(8):
            
            self.tactile_0['0_dX_'+str(i)] = tac_in.pillars[i].dX
            self.tactile_0['0_dY_'+str(i)] = tac_in.pillars[i].dY
            self.tactile_0['0_dZ_'+str(i)] = tac_in.pillars[i].dZ
            self.tactile_0['0_fX_'+str(i)] = tac_in.pillars[i].fX
            self.tactile_0['0_fY_'+str(i)] = tac_in.pillars[i].fY
            self.tactile_0['0_fZ_'+str(i)] = tac_in.pillars[i].fZ
            self.tactile_0['0_incontact_'+str(i)] = tac_in.pillars[i].in_contact
            self.tactile_0['0_slipstate_'+str(i)] = tac_in.pillars[i].slip_state

        self.tactile_0['0_friction_est'] = tac_in.friction_est
        self.tactile_0['0_target_grip_force'] = tac_in.target_grip_force
        self.tactile_0['0_is_sd_active'] = tac_in.is_sd_active
        self.tactile_0['0_is_ref_loaded'] = tac_in.is_ref_loaded
        self.tactile_0['0_is_contact'] = tac_in.is_contact            
            
    def tactile_1_callback(self, tac_in):
        # Saves the subscribed tactile 1 data to variable
        while self.actively_reading:
            rospy.sleep(.01)
        self.tactile_1 = OrderedDict() 
        for i in range(8):
            
            self.tactile_1['1_dX_'+str(i)] = tac_in.pillars[i].dX
            self.tactile_1['1_dY_'+str(i)] = tac_in.pillars[i].dY
            self.tactile_1['1_dZ_'+str(i)] = tac_in.pillars[i].dZ
            self.tactile_1['1_fX_'+str(i)] = tac_in.pillars[i].fX
            self.tactile_1['1_fY_'+str(i)] = tac_in.pillars[i].fY
            self.tactile_1['1_fZ_'+str(i)] = tac_in.pillars[i].fZ
            self.tactile_1['1_incontact_'+str(i)] = tac_in.pillars[i].in_contact
            self.tactile_1['1_slipstate_'+str(i)] = tac_in.pillars[i].slip_state

        self.tactile_1['1_friction_est'] = tac_in.friction_est
        self.tactile_1['1_target_grip_force'] = tac_in.target_grip_force
        self.tactile_1['1_is_sd_active'] = tac_in.is_sd_active
        self.tactile_1['1_is_ref_loaded'] = tac_in.is_ref_loaded
        self.tactile_1['1_is_contact'] = tac_in.is_contact      

    def initialize_tactile_dict(self):
        """
        Initializes all of the keys in each ordered dictionary. This ensures the header and order is correct even if recording starts before data is published.
        """
        # Position 
        self.position = OrderedDict({'gripper_pos': None})
        # Tactile sensor 
        self.tactile_0 = OrderedDict()
        self.tactile_1 = OrderedDict()

        for i in range(8):
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

        self.tactile_0['0_friction_est'] = None
        self.tactile_0['0_target_grip_force'] = None
        self.tactile_0['0_is_sd_active'] = None
        self.tactile_0['0_is_ref_loaded'] = None
        self.tactile_0['0_is_contact'] = None

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
