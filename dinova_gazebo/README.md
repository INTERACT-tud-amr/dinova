# dinova_gazebo

<table>
  <tr>
    <td><b>dingo-o</b></th>
    <td><b>kinova</b></th>
    <td><b>dinova</b></th>
  </tr> 
  <tr>
    <td> <img src="./assets/images/dingo-o-lidar.png"  alt="1" height = 360px ></td>
    <td> <img src="./assets/images/kinova.png"  alt="1" height = 360px ></td>
    <td> <img src="./assets/images/dinova.png"  alt="1" height = 360px ></td>
  </tr> 
</table>

## Requirements
``` bash
sudo apt install ros-${ROS_DISTRO}-velocity-controllers
sudo apt install ros-${ROS_DISTRO}-velodyne-gazebo-plugins
sudo apt install ros-${ROS_DISTRO}-velodyne-description
```

## Running a simulation in Gazebo:
To launch the simulation with `dinova`:
``` bash
   roslaunch dinova_gazebo gazebo_dinova.launch
```
To launch the simulation with `dingo` only:
``` bash
   roslaunch dinova_gazebo gazebo_dingo_omni.launch
```
To launch the simulation with `kinova` only:
``` bash
   roslaunch dinova_gazebo gazebo_kinova.launch
```
To launch the simulation with the lidar and dingo:
``` bash
   roslaunch dingo_kinova_description gazebo_dingo_omni.launch lidar:=true
```

## Controlling robots
The default mode is `position`. It can be changed to `velocity` by setting the
mode argument as:

``` bash
   roslaunch dingo_kinova_description gazebo_dingo_kinova.launch mode:=velocity
```
### Position mode
There are two topics for controlling the robot in position mode:
1. `/joints_position_controller/command` -  Position interface for each joint of Kinova: [q1, q2, q3, q4, q5, q6] in rad
2. `/omnidrive_position_controller/command` -  Position interface for omnidrive: [linear_x, linear_y, angular_z] in [m, m rad
### Velocity mode
There are two topics for controlling the robot:
1. `/joints_velocity_controller/command` - Velocity interface for each joint of Kinova: [q1, q2, q3, q4, q5, q6] in rad/s
2. `/omnidrive_velocity_controller/command` - Velocity interface for omnidrive: [linear_x, linear_y, angular_z] in [m/s, m/s, rad/s]








