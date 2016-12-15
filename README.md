# dynamics_ws

WPI RBE-500 Dynamics_Project_2016 :

Contributors

Ankur Agrawal

Janani Mohan

Praneeta Mallela

Sathya Narayanan

Project Goal : Accurate BGA chip placement on a PCB using ABB IRB 120 Industrial robot

This repository contains the necessary packages for simulating ABB IRB 120 for a pick and place application of BGA on a PCB. The simulation is carried out in Gazebo using ROS. A Vacuum gripper is attached to the end effector for pick and place operation. The uses of different packages are as follows: 

irb120 : Trajectory Planning

irb120_perception : Image Processing for determination of the position and orientation of the BGA. 

irb120_tf_calc : Gives the Position of Camera_frame with respect to the World.

Future Goal : To work on different sizes of BGAs and different orientations of BGAs.

Dependent Packages to be installed:

ros-indigo : http://www.ros.org/

gazebo2/ gazebo7 : http://gazebosim.org/

OpenCV : http://opencv.org/

Eigen : http://eigen.tuxfamily.org/index.php?title=Main_Page

ros-controllers : ``` sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control ```
 
ros-gazebo-pkgs for vaccum gripper plugins : https://github.com/ros-simulation/gazebo_ros_pkgs

Clone this repository using: 

``` 
git clone https://github.com/sathya1995/dynamics_ws.git
```

Build the Package using:

```
catkin_make
```

Source the package to ROS Path: 
```
source devel/setup.bash
```

Important Note:

```
Kindly copy the folder named irb120_model in irb120_gazebo/worlds and paste it in the location ~/.gazebo/models
```

The following steps are to be done in the order specified below: 

Launch gazebo: 
```
roslaunch irb120-gazebo irb120_gazebo.launch
```

Run the Node for trajectory planning: 
``` 
rosrun irb120 irb120_node 
```

Run the tf node: 
``` 
rosrun irb120_tf_calc irb120_tf_calc_node 
```

Run the Node for image processing: 
```
rosrun irb120_perception irb120_perception_node 
```

The Robot should be moving to desired place for picking the BGA and then placing the BGA on the PCB at the desired orientation. 
