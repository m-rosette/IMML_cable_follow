#!/usr/bin/env python2

import rospy


# General idea - this is high level trial control
"""
1) Freedrive the robot to the the starting location on the cable

Repeat:
A) Close the gripper, recording data
B) Open the gripper
C) Pass the data to the LSTM service
D) Get back the delta x, y, theta 
E) Call the move_arm action server

"""




# Gripper_Cmd - publish gripper position/current values here
# UR_something - publish cartesian movement changes here
# __action_server - use this to start data collection
# camera_service - takes photo, saves it, and returns cable pose

class CableTrace:
    def __init__(self):

        

        

    def main(self):
      
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cable_trace', anonymous=True)
    cable_trace = CableTrace()
    cable_trace.main()
