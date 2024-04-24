# Dinova
ROS software stack for Dinova (dingo + kinova). 

## Running a real robot
Turn off default ros packages by Clearpath
``` bash
sudo systemctl stop ros.service
```
To launch the control interface for Dinova: 
``` bash
    roslaunch dinova_bringup dinova.launch
```
To launch the control interface for Dingo: 
``` bash
    roslaunch dinova_bringup dingo.launch
```
To launch the control interface for Kinova: 
``` bash
    roslaunch dinova_bringup kinova.launch
```

## Description
1. `dinova_bringup` -> Contains launch files for controlling Dinova, Kinova and Dingo. 
2. `dinova_control` -> Contains ROS driver for Kinova and the node that collects states from all robots. For more information [here](/dinova_control/README.md).
3. `dinova_description` -> URDF files for Dinova, Kinova and Dingo. For more information [here](/dinova_description/README.md).
4. `dinova_gazebo` -> Gazebo package for simulating Dinova, Kinova and Dingo. For more information [here](/dinova_gazebo/README.md).


# TODO:
1. Vicon node is missing
2. Gripper_driver
