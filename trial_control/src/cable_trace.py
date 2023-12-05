#!/usr/bin/env python3

import rospy
from trial_control.msg import MoveAction, MoveGoal, Movement, RecordGoal, RecordAction
from actionlib import SimpleActionClient
from std_msgs.msg import String, Int16
from lbc_project.srv import MoveData, MoveDataResponse
from trial_control.srv import PictureTrigger, PictureTriggerResponse
import os
"""
1) Freedrive the robot to the the starting location on the cable

Repeat:
A) Close the gripper, recording data
B) Open the gripper
C) Pass the data to the LSTM service
D) Get back the delta x, y, theta 
E) Call the move_arm action server

"""




class CableTrace:
    def __init__(self):
        self.storage_directory = '/root/data'
        self.file_num = 0
        # Set up the movement action client
        self.move_ac = SimpleActionClient("move_server", MoveAction)
        rospy.loginfo("Waiting for movement server to come up.")
        self.move_ac.wait_for_server()
        rospy.loginfo("Movement server up!")

        # Setup the recording action client
        self.record_ac = SimpleActionClient("record_server", RecordAction)
        rospy.loginfo("Waiting for recording server to come up.")
        self.record_ac.wait_for_server()
        rospy.loginfo("Recording server up!")


        rospy.wait_for_service('picture_trigger')
        rospy.loginfo("Picture service up, ready!")
        self.pic = rospy.ServiceProxy('picture_trigger', PictureTrigger)

        # Set up the gripper position/current publisher
        self.gripper_pos_pub = rospy.Publisher('Gripper_Cmd', String, queue_size=5)

        rospy.wait_for_service('gripper_move_data')
        rospy.loginfo("LSTM service up, ready!")
        self.lstm = rospy.ServiceProxy('gripper_move_data', MoveData)

    def main(self):
        # Open gripper
        self.gripper_pos_pub.publish("position_10000")
        # Drive robot to start
        _ = input("Freedrive robot to start, then start program, then hit enter.")
        self.file_num = self.get_start_file_index()

        while not rospy.is_shutdown():
            print("FILE NUM: ",self.file_num)
            goal = RecordGoal(file_name=str(self.file_num))
            self.record_ac.send_goal(goal)

            # Close gripper
            self.gripper_pos_pub.publish("current_3")
            rospy.sleep(2.5) # Sleep a tiny bit then stop recording
            
            self.record_ac.wait_for_result()
            result = self.record_ac.get_result()

            # Take a picture
            picture_return = self.pic(trial_num = str(self.file_num))


            # print(result)

            # Open the gripper
            self.gripper_pos_pub.publish("position_10000")

            # TODO: Call LSTM service
            move_return = self.lstm(gripper_data = result.data, move_right=True)

            print(move_return)
            #input("Enter to move")

            # Move robot with movement from LSTM
            movement = Movement(dx = move_return.x, dy = move_return.y, dtheta = -move_return.angle)
            # movement = Movement(dx = 0, dy = 0, dtheta = 45)
            goal = MoveGoal(delta_move=movement)
            self.move_ac.send_goal(goal)
            self.move_ac.wait_for_result()
            #input("Enter to continue to next step")
            self.file_num += 1

    def get_start_file_index(self):
        # Returns the starting file number (old number)
        current_files = os.listdir(self.storage_directory)
        try:
            numbers = np.array([i.split('.')[0] for i in current_files], dtype=int)
            print(numbers)
            return np.max(numbers) + 1
        except Exception as e:
            return 0
      


if __name__ == '__main__':
    rospy.init_node('cable_trace', anonymous=True)
    cable_trace = CableTrace()
    cable_trace.main()
