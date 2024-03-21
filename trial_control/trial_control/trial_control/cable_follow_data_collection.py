#!/usr/bin/env python2

import rospy
from std_msgs.msg import String, Int16
from actionlib import SimpleActionClient
from trial_control.msg import RecordAction, RecordGoal
from trial_control.srv import PictureTrigger, PictureTriggerResponse
import os
import numpy as np

# General idea - this is high level trial control

# Gripper_Cmd - publish gripper position/current values here
# UR_something - publish cartesian movement changes here
# __action_server - use this to start data collection
# camera_service - takes photo, saves it, and returns cable pose

class CableFollow:
    def __init__(self):

        self.storage_directory = '/root/data'
        self.file_num = self.get_start_file_index()
        print(self.file_num)
    
        # Setup the gripper position publisher
        # Possible messages are "current_000" or "position_000" where 000 is the value
        self.gripper_pos_pub = rospy.Publisher('Gripper_Cmd', String, queue_size=5)



        # Setup the recording action client
        self.record_ac = SimpleActionClient("record_server", RecordAction)
        rospy.logwarn("Waiting for recording server to come up.")
        self.record_ac.wait_for_server()
        rospy.loginfo("Recording server up!")
        rospy.logwarn("Waiting for picture service to come up.")
        rospy.wait_for_service('picture_trigger')
        rospy.loginfo("Picture service up, ready!")
        self.pic = rospy.ServiceProxy('picture_trigger', PictureTrigger)
        

        

    def main(self):
        # Get the image number to save
        self.gripper_pos_pub.publish("position_10000")

        # Hit enter to continue
        user_con = raw_input("Enter to take initial picture.")
        print(self.file_num)
        picture_return = self.pic(trial_num = str(self.file_num))

        user_con = raw_input("Enter to start.")
        counter = 0
        while not rospy.is_shutdown():
            if counter > 9: 
                break
        # Take picture
            
            
            # if rospy.is_shutdown():
            #     break
            name = str(self.file_num)+"_"+str(counter)
            goal = RecordGoal(file_name=name)
            self.record_ac.send_goal(goal)
            # Close the gripper

            self.gripper_pos_pub.publish("current_3")
            
            # Sleep a tiny bit then stop recording
            rospy.sleep(2.5)
            self.record_ac.cancel_all_goals()
            pic_name = str(self.file_num)+"_"+str(counter)
            picture_return = self.pic(trial_num = pic_name)

            # Open the gripper
            self.gripper_pos_pub.publish("position_10000")
            counter += 1
            rospy.sleep(1.75)




        # for i in range(100):
        #     print("Waiting a second")

        #     rospy.sleep(1)
            

        #     if rospy.is_shutdown():
        #         break


            
        #     # Sample publish gripper position
        #     # cmd_msg = "position_101027"
        #     # self.gripper_pos_pub.publish(cmd_msg)

        #     # rospy.sleep(1)

        #     # cmd_msg = "position_100775"
        #     # self.gripper_pos_pub.publish(cmd_msg)
        #     # rospy.sleep(1)


        rospy.spin()

    def get_start_file_index(self):
        # Returns the starting file number (old number)
        current_files = os.listdir(self.storage_directory)
        try:
            numbers = np.array([i.split('_')[0] for i in current_files], dtype=int)
            print(numbers)
            return np.max(numbers) + 1
        except Exception as e:
            return 0

if __name__ == '__main__':
    rospy.init_node('cable_follow', anonymous=True)
    cable_follow = CableFollow()
    cable_follow.main()
