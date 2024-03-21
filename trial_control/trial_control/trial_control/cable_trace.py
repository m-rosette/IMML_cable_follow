#!/usr/bin/env python3

import rclpy
from trial_control_msgs.action import MoveAction, MoveGoal, Movement
from rclpy.action import SimpleActionClient
from std_msgs.msg import String, Int16

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

        self.move_ac = SimpleActionClient("move_server", MoveAction)
        rospy.loginfo("Waiting for movement server to come up.")
        self.move_ac.wait_for_server()
        rospy.loginfo("Movement server up!")

        self.gripper_pos_pub = rospy.Publisher('Gripper_Cmd', String, queue_size=5)

        

    def main(self):
        # Open gripper
        self.gripper_pos_pub.publish("position_10000")
        # Drive robot to start
        _ = input("Freedrive robot to start, then start program, then hit enter.")

        while not rospy.is_shutdown():
            # TODO: Start recording here

            # Close gripper
            self.gripper_pos_pub.publish("current_3")
            rospy.sleep(2.5) # Sleep a tiny bit then stop recording
            
            # TODO: Stop recording here

            # Open the gripper
            self.gripper_pos_pub.publish("position_10000")

            # TODO: Call LSTM service

            # Move robot with movement from LSTM
            movement = Movement(dx = -.01, dy = -.01, dtheta = -.1)
            goal = MoveGoal(delta_move=movement)
            self.move_ac.send_goal(goal)
      


if __name__ == '__main__':
    rospy.init_node('cable_trace', anonymous=True)
    cable_trace = CableTrace()
    cable_trace.main()