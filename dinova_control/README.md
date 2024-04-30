# dinova_control

## Requirements
1. Install Kinova python-API:
https://github.com/Kinovarobotics/kortex/blob/master/api_python/examples/readme.md


## Control modes for kinova
The default control mode is `High Level Velocity`. If you want to change the control modes, you need to call rosservices:
1. `/kinova/change_to_HLC_position` - The mode is switched to HLC position control and the `/kinova/command` topic expects desired joint positions
2. `/kinova/change_to_HLC_velocity` - The mode is switched to HLC position control and the `/kinova/command` topic expects desired joint velocities
3. `/kinova/change_to_LLC_position` - The mode is switched to LLC position control and the `/kinova/command` topic expects desired joint positions `NOT RECOMMENDED YET`
4. `/kinova/change_to_LLC_velocity` - The mode is switched to LLC position control and the `/kinova/command` topic expects desired joint velocities. `NOT RECOMMENDED YET`

## Important topics
Topics:
1. `/kinova/command` - Topic for controlling kinova's joints
2. `/cmd_vel` - Topic for controlling omnibase's velocities
3. `/kinova/error_ack` - Topic for acknowledging faults
4. `/kinova/joint_states` - Topic for subscribing the actual state of the robot in [rad, rad/s, Nm]
5. `/dinova/joint_states` - Topic for subscribing the actual state of dinova [wheels, q]. This is mainly use for RViz, tf, ...
6. `/dinova/omni_states` - Topic for subscribing the actual state of dinova [x, y, theta, q]. The omnibase pose is received from odometry.
7. `/dinova/omni_states_vicon` - Topic for subscribing the actual state of dinova [x, y, theta, q]. The omnibase pose is received from vicon.


## Useful services:
1. `rosservice call /kinova/go_zero_position` - Move an arm to zero joint position
2. `rosservice call /kinova/go_home_position` - Move an arm to home joint position
3. `rosservice call /kinova/gripper/open` - Open gripper
4. `rosservice call /kinova/gripper/close` - Close gripper






