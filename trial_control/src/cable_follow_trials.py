#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

class CableFollow:
    def __init__(self):

        # Setup the gripper position publisher
        # Possible messages are "current_000" or "position_000" where 000 is the value
        self.gripper_pos_pub = rospy.Publisher('Gripper_Cmd', String, queue_size=5)

        pass

    def main(self):
        for i in range(10):
            
            cmd_msg = "position_101027"
            self.gripper_pos_pub.publish(cmd_msg)

            rospy.sleep(1)

            cmd_msg = "position_100775"
            self.gripper_pos_pub.publish(cmd_msg)
            rospy.sleep(1)


        rospy.spin()

if __name__ == '__main__':
    cable_follow = CableFollow()
    rospy.init_node('cable_follow', anonymous=True)
    cable_follow.main()
