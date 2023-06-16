# ROS UWB V2V Ranging System Simulation
This package is to simulate the self-propelled rail vehicle V2V ranging functionalities based on Decawave DWM1001-DEV in ROS environment. For each vehicle (designated as `robot{}`), generated V2V ranging map is published as JSON into `/robot{}/robot_dist` topic. 

![](https://raw.githubusercontent.com/hegxiten/uwb_v2v_ros_sim/master/docs/1.png)
![](https://raw.githubusercontent.com/hegxiten/uwb_v2v_ros_sim/master/docs/2.png)

## Positions of the UWB Anchors(Slaves) and Tags(Masters)

The position of the individual UWB nodes are defined as the "Informative Position" (details can be found at the submitted manuscript **Development and Testing of A UWB-based Vehicle-toVehicle (V2V) Ranging System for Self-Propelled Rail
Vehicles** submitted to *IEEE Transactions on Vehicular Technology*)

To define the informative positions of individual UWB nodes, change the `joint_origin` argument for each `xacro:uwb_link` in `rail_vehicle_description/model.urdf.xacro`.

## Platform and Environment
`lsb_release -a` outputs:
```
Distributor ID: Ubuntu
Description:    Ubuntu 20.04.6 LTS
Release:        20.04
Codename:       focal
```
`rosversion -d` outputs:
```
noetic
```
`gazebo --version` outputs:
```
Gazebo multi-robot simulator, version 11.13.0
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org
```


## Installation

`git clone https://github.com/hegxiten/uwb_v2v_ros_sim.git` to your catkin workspace `src/` directory and execute `catkin_make` in the workspace root directory.

## Dependencies ROS packages

`rospack depends1 uwb_v2v_ros_sim` outputs:
```
roscpp
rospy
std_msgs
message_runtime
gazebo_ros
gazebo_plugins
gazebo_ros_control
turtlebot3_description
turtlebot3_gazebo
```

## Launching the simulation

There are two ways to launch the simulation: 1. from the rosnode command line, 2. from the launch file.

1. Launching from the rosnode command line:
   
   `rosrun uwb_v2v_ros_sim main.py`

   The simulation arguments (such as approach speeds) can be editted inside the `main.py` file. Launching from the rosnode command line is useful for repeated testings configurable under the `main.py` file.

2. Launching from the launch file:
   
    `roslaunch uwb_v2v_ros_sim multi_robot.launch 'robot_num:=2' 'robot_spacing:=20' 'movement_direction:=-1' 'speed:=0' 'kill_on_end:=0' 'max_range:=500'` 
   
   The simulation arguments (such as approach speeds) must be passed directly into the commands like above. Launching from the launch file is useful for one-time testing.

## Published Topic

`rostopic echo /robot{idx}/robot_dist` can give you direct outputs for each vehicle (robot) identified by their index `idx`. 

## Message Type

Currently, there is no custom message type defined. The message type is `std_msgs/String` and the message content is a JSON string. The JSON string can be parsed into a JSON object in Python.

## Probability Model and Stale Threshold 

The pack loss probability model is defined as a function of the distance between the two vehicles that can be modified in `src/utils/utils.py`

The stale threshold (in seconds) for networking can be modified in `src/constants.py` 

## Outputs:

When launched, you are expecting the 
![](https://raw.githubusercontent.com/hegxiten/uwb_v2v_ros_sim/master/docs/3.png)
