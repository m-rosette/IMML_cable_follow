#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
import os
import numpy as np

# Serves as an action client that starts data recording 


class Record:
    def __init__(self):
        # The file number we are saving 
        self.file_num = self.get_start_file_index()
        # Position from the callback
        self.position = 0

        # Subscribe to gripper position feedback 
        # TODO: need to update to correct topic
        self.gripper_pos_sub = rospy.Subscriber('Gripper_Cmd', String, self.pos_callback)

        self.storage_directory = '/root/data'
        pass


   
    def action_callback(self, goal):


        os.listdir(self.storage_directory)
    #     with open('root/data/eggs.csv', newline='') as csvfile:

    # spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')

    # for row in spamreader:

    #     print(', '.join(row))

    def pos_callback(self, input):
        # TODO: probably need to split the string
        self.position = int(input)

    def tactile_sensor_callback(self, input):
        pass


    def get_start_file_index(self):
        # Returns the starting file number (old number + 1)
        current_files = os.listdir(self.storage_directory)
        numbers = np.array([i.split('.csv', 1)[0] for i in current_files], dtype=int)
        return np.max(numbers) + 1

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    record = Record()
    #rospy.init_node('record', anonymous=True)
    record.main()
