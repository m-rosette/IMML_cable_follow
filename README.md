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
