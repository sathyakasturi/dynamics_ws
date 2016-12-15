# dynamics_ws

WPI RBE-500 Dynamics_Project_2016 :

Contributors

Ankur Agrawal

Janani Mohan

Praneeta Mallela

Sathya Narayanan

Project Goal : Accurate BGA chip placement on a PCB using ABB IRB 120 Industrial robot

Simulation: The entire simulation is in Gazebo. The ROS packages irb120 and irb120_perception executes the trajectory planning and the image processing of the entire robot.A vacuum suction pump is attached onto the end effector and it is activated during pick operation and deactivated during place operation. Currently the detection of BGA chips at zero degree orientation is picked up and placed at the same orientation on the PCB. Future Goal : To work on different sizes of BGAs and different orientations of BGAs.

Dependent Packages to be installed:

ros-indigo : http://www.ros.org/

gazebo2/ gazebo7 : http://gazebosim.org/

OpenCV : http://opencv.org/

Eigen : http://eigen.tuxfamily.org/index.php?title=Main_Page

ros-controllers : sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control
 
ros-gazebo-pkgs for vaccum gripper plugins : https://github.com/ros-simulation/gazebo_ros_pkgs

Setting Up Workspace: 
'''
    mkdir -p ~/dynamics_ws/src && cd ~/dynamics_ws/src
    catkin_init_workspace
    cd ~/dynamics_ws
    catkin_make
'''

Usage:

source devel/setup.bash
Launch gazebo: 
''' roslaunch irb120-gazebo irb120_gazebo.launch '''

Run the Node for trajectory planning: 
''' rosrun irb120 irb120_node '''

Run the tf node: 
''' rosrun irb120_tf_calc irb120_tf_calc_node '''

Run the Node for image processing: 
''' rosrun irb120_perception irb120_perception_node '''


Important Note:
'''
Kindly copy the folder named irb120_model in irb120_gazebo/worlds and paste it in the location ~/.gazebo/models
'''
