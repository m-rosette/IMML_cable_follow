# ROB 537 Cable Following
## Kyle, Marcus, Jostan, Keegan

### How to use
First, connect the gripper via USB. If using a Docker container, be sure to connect the USB before starting the container (or restart the container).

Launch the gripper bringup with:
```console
roslaunch contactille_gripper gripper_bringup.launch
```
This will identify ports for the Contactille sensors and Dynamixel controlller, and start feedback and control nodes.




Credit to Kyle Mathenia for the gripper ROS code. Check out his original code [here](https://github.com/kylemathenia/IMML_Contactile_Gripper).


Need to add pip install ordereddict

roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.254.174.50

roslaunch ur_calibration calibration_correction.launch robot_ip:=169.254.174.50 target_filename:="${HOME}/my_robot_calibration.yaml"


pip install scikit-image