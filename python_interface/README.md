## TBD

[//]: # ()
[//]: # (## Quadruped_ROS_Simulation_V2)

[//]: # (Software package for all Unitree robots at USC-DRCL &#40;A1, AlienGo&#41;)

[//]: # (    Note: Slight modifications are needed to accomandate Go1 and B1 robots)

[//]: # (## Packages:)

[//]: # (* Quadruped robot control: `quadruped_control`)

[//]: # (* Robot description: `a1_description`, `aliengo_description`, `laikago_description`, `go1_description`, `b1_description`, `z1_description`)

[//]: # (* Robot and joints controller: `unitree_controller`)

[//]: # (* ROS messages: `unitree_legged_msgs`)

[//]: # (* Gazebo envrionment: `unitree_gazebo`)

[//]: # (* Unitree stock simulation controller `unitree_legged_control`)

[//]: # (* Real robot control related: `unitree_legged_real`)

[//]: # ()
[//]: # (## Code structure for quadruped_control package)

[//]: # (- [Common] includes leg controller, state Estimation, kinematic parameters and utility functions)

[//]: # (- [Interface] IO interfaces for ROS/hardware)

[//]: # (    * ROS: keyboard commands )

[//]: # (    * Hardware: joystick &#40;under development&#41;)

[//]: # (- [Messages] messages for communication &#40;corresponds with messages in unitree_legged_sdk&#41;)

[//]: # (- [FSM] finite state machine and FSM states to switch between controllers &#40;passive, PDstand, QPstand, MPC locomotion&#41;)

[//]: # ()
[//]: # (## Dependencies)

[//]: # (* [Boost]&#40;http://www.boost.org&#41; &#40;version 1.5.4 or higher&#41;)

[//]: # (* [CMake]&#40;http://www.cmake.org&#41; &#40;version 2.8.3 or higher&#41;)

[//]: # (* [LCM]&#40;https://lcm-proj.github.io&#41; &#40;version 1.4.0 or higher&#41;)

[//]: # (* [ROS]&#40;http://wiki.ros.org/&#41; &#40;melodic or neotic&#41;)

[//]: # (* [Gazebo]&#40;https://gazebosim.org/home&#41;)

[//]: # (* [Eigen3]&#40;https://eigen.tuxfamily.org/index.php?title=Main_Page&#41; &#40;>3.3&#41;)

[//]: # (* [unitree_legged_sdk] check the readme inside `unitree_ros` folder on how to config the `unitree_legged_sdk`)

[//]: # (* [ROS_Packages] For differnet ros distro, simply change the package distro name.)

[//]: # ()
[//]: # (For ROS Melodic)

[//]: # (```)

[//]: # (sudo apt-get install ros-melodic-controller-manager ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-velocity-controllers ros-melodic-position-controllers ros-melodic-robot-controllers ros-melodic-robot-state-publisher ros-melodic-gazebo-ros ros-melodic-gazebo-ros-control)

[//]: # (```)

[//]: # (For ROS Noetic)

[//]: # (```)

[//]: # (sudo apt-get install ros-noetic-controller-manager ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-velocity-controllers ros-noetic-position-controllers ros-noetic-robot-controllers ros-noetic-robot-state-publisher ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control)

[//]: # (```)

[//]: # ()
[//]: # (## Installation)

[//]: # ( Compile the package at your ROS workspce with `catkin_make -DCMAKE_BUILD_TYPE=Release`)

[//]: # (If any error about `unitree_legged_real` occurs, check your installation of the `unitree_legged_sdk`)

[//]: # ()
[//]: # (## How to Run)

[//]: # (* launch the A1 robot with `roslaunch unitree_gazebo normal.launch rname:=a1` )

[//]: # (You can change the argument to launch different robots within this package)

[//]: # ()
[//]: # (* run the quadruped control node in a NEW terminal `rosrun quadruped_control quad_control`)

[//]: # ()
[//]: # (## User Commands & Interface)

[//]: # (* check the `Interface/KeyBoard.cpp` to see and modify definition of keyboard commands)

[//]: # ()
[//]: # (## Todo)

[//]: # (* Further development and extension for different usages)

[//]: # ()
[//]: # (## Commonly asked Questions)

[//]: # (* Using virtual machine is not recommended)

[//]: # (* Try using WSL is a waste of time)

[//]: # (* if the MPC solve time exceeds 5ms, try find a better PC to run the package &#40;given that Ubuntu system is intalled&#41;)

[//]: # (* if the simulation doesn't run well and the time factor is quite low, change the real_time_update_rate smaller based on your computer performance &#40;e.g 100&#41; in `/unitree_ros/unitree_gazebo/launch/world/* .world` file)
