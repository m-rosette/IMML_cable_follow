#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
from actionlib import SimpleActionClient
from trial_control.msg import RecordAction, RecordGoal

# General idea - this is high level trial control

# Gripper_Cmp - publish gripper position/current values here
# UR_something - publish cartesian movement changes here
# __action_server - use this to start data collection
# camera_service - takes photo, saves it, and returns cable pose

class CableFollow:
    def __init__(self):

        # Setup the gripper position publisher
        # Possible messages are "current_000" or "position_000" where 000 is the value
        self.gripper_pos_pub = rospy.Publisher('Gripper_Cmd', String, queue_size=5)

        # Setup the recording action client
        self.record_ac = SimpleActionClient("record_server", RecordAction)
        rospy.logwarn("Waiting for recording server to come up.")
        self.record_ac.wait_for_server()
        rospy.loginfo("Recording server up, ready!")

        

    def main(self):
        for i in range(100):
            print("Waiting a second")

            rospy.sleep(1)
            goal = RecordGoal(start=True)
            self.record_ac.send_goal(goal)
            rospy.sleep(2)
            self.record_ac.cancel_all_goals()

            if rospy.is_shutdown():
                break


            
            # Sample publish gripper position
            # cmd_msg = "position_101027"
            # self.gripper_pos_pub.publish(cmd_msg)

            # rospy.sleep(1)

            # cmd_msg = "position_100775"
            # self.gripper_pos_pub.publish(cmd_msg)
            # rospy.sleep(1)


        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('cable_follow', anonymous=True)
    cable_follow = CableFollow()
    cable_follow.main()
