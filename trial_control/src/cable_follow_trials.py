#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

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

        pass

    def main(self):
        for i in range(10):
            
            # Sample publish gripper position
            # cmd_msg = "position_101027"
            # self.gripper_pos_pub.publish(cmd_msg)

            rospy.sleep(1)

            cmd_msg = "position_100775"
            self.gripper_pos_pub.publish(cmd_msg)
            rospy.sleep(1)


        rospy.spin()

if __name__ == '__main__':
    cable_follow = CableFollow()
    rospy.init_node('cable_follow', anonymous=True)
    cable_follow.main()
