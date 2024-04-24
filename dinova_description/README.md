# dinova_description

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



## Visualizing a robot model in RViz and `joint_state_gui_publisher`:
To launch the visualization with `dinova`:
``` bash
   roslaunch dinova_gazebo gazebo_dinova.launch
```
To launch the visualization with `dingo` only:
``` bash
   roslaunch dinova_gazebo gazebo_dingo_omni.launch
```
To launch the visualization with `kinova` only:
``` bash
   roslaunch dinova_gazebo gazebo_kinova.launch
```
To launch the visualization with the lidar and dingo:
``` bash
   roslaunch dingo_kinova_description gazebo_dingo_omni.launch lidar:=true
```

## Converting xacro file to urdf file









