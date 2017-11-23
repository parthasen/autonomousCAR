# ROS
## Installation

http://wiki.ros.org/kinetic/Installation/Ubuntu

1. Setup your computer to accept software from packages.ros.org
    `(SDC3) octo@octo:~$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
2. Set up your keys
    `(SDC3) octo@octo:~$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116`
3.  make sure your Debian package index is up-to-date: 
    
        (SDC3) octo@octo:~$ sudo apt-get update
        (SDC3) octo@octo:~$ sudo apt-get install ros-kinetic-desktop-full   
4. Initialize rosdep

        sudo rosdep init
        rosdep update
5. It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched: 

        echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
        source ~/.bashrc

## Environment
1. Setup`(SDC3) octo@octo:~$ source /opt/ros/kinetic/setup.bash`
2. Inspecting the Environment `(SDC3) octo@octo:~$ env'



    
    
## Introduction
###  ROS master - first terminal
    octo@octo:~/catkin_ws$ roscore
### second terminal
    rosrun turtlesim turtlesim_node
### third terminal
    rosrun turtlesim turtle_teleop_key
### Fourth terminal
 
 octo@octo:~$ rosnode list
    /rosout
    /teleop_turtle
    /turtlesim
 
 octo@octo:~$ rostopic list
    /rosout
    /rosout_agg
    /turtle1/cmd_vel
    /turtle1/color_sensor
    /turtle1/pose
 
 octo@octo:~$ rostopic info /turtle1/cmd_vel 
    Type: geometry_msgs/Twist

        Publishers: 
         * /teleop_turtle (http://octo:41470/)

        Subscribers: 
         * /turtlesim (http://octo:44078/)
 
    
        octo@octo:~$ rosmsg list
        octo@octo:~$ rosmsg show geometry_msgs/Twist
        octo@octo:~$ rosmsg info
        rosed geometry_msgs Twist.msg
        octo@octo:~$ rosed geometry_msgs Twist.msg
        
        ## Echo message
        octo@octo:~$ rostopic echo /turtle1/cmd_vel

## Catkin

##### Creating Space
    octo@octo:~$ mkdir -p ~/catkin_ws/src
    octo@octo:~$ cd ~/catkin_ws/src
    
initialize the catkin workspace`octo@octo:~/catkin_ws/src$ catkin_init_workspace`

Let’s list the contents of the current directory to see what changed `ws/src$ ls -l`

Return to the top level directory `octo@octo:~/catkin_ws/src$ cd ~/catkin_ws`

build the workspace (Note: you must issue this command from within the top level directory (i.e., within catkin_ws NOT catkin_ws/src)
        
        octo@octo:~/catkin_ws$ catkin_make
        
http://wiki.ros.org/catkin/conceptual_overview        

    octo@octo:~/catkin_ws$ ls

##### Cloning packages

    octo@octo:~/catkin_ws$ cd ~/catkin_ws/src
    octo@octo:~/catkin_ws/src$ git clone https://github.com/udacity/simple_arm_01.git simple_arm
Building the simple_arm package:

    octo@octo:~/catkin_ws/src$ cd ~/catkin_ws
    octo@octo:~/catkin_ws$ catkin_make
    
Error: Could not find a package configuration file provided by  "controller_manager"   
    
    octo@octo:~/catkin_ws$ sudo apt-get install ros-indigo-controller-manager
    octo@octo:~/catkin_ws$ catkin_make

##### ROS LAUNCH

    octo@octo:~/catkin_ws$ source devel/setup.bash
    octo@octo:~/catkin_ws$ roslaunch simple_arm robot_spawn.launch
[WARN] [WallTime: 1511177315.031336] [0.000000] Controller Spawner couldn't find the expected controller_manager ROS interface.

    octo@octo:~/catkin_ws$ rosdep check simple_arm
    octo@octo:~/catkin_ws$ rosdep install -i simple_arm
https://github.com/udacity/simple_arm    
    
    

    



