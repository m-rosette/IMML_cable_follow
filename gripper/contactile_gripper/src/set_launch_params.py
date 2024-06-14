#!/usr/bin/env python3

import rospy
import serial
import serial.tools.list_ports

def main():
    rospy.init_node('set_com_port_param', anonymous=True)

    # Get the port of the contactille sensor
    com_port = find_contactile_port()
    # Set the parameter
    rospy.set_param('papillarray_ros_v2_node/com_port', com_port)
    rospy.loginfo('Successfully set COM parameter to: %s', com_port)
    
def find_contactile_port():
    """Returns the comport path for the contactile sensor hub."""
    comport_info = get_com_info()
    for port in comport_info:
        # if port['serial_number'] == '10129740': # Original contactile sensors
        if port['serial_number'] == '13442510': # Newer contactile sensors
            return port['device']
    print("\nContactile sensor hub port info not found.\n")
    raise LookupError

def get_com_info():
    comport_info = []
    comports = serial.tools.list_ports.comports()
    for comport in comports:
        comport_info.append(comport.__dict__)
    return comport_info



if __name__ == '__main__':
    main()



# <node pkg="contactile_gripper" name="control" type="control_node.py" launch-prefix="gnome-terminal --" output="screen"/>

# <node pkg="contactile_gripper" name="UI" type="ui_node.py" launch-prefix="gnome-terminal --" output="screen"/>