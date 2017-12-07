# ROS
## Installation

http://wiki.ros.org/kinetic/Installation/Ubuntu

### I am using ENV. python 2.7.11

#### 1. Setup your sources.list
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#### 2. Set up keys
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
#### 3. Installation
    sudo apt-get update
    sudo apt-get install ros-kinetic-desktop-full   
#### 4. Initialize rosdep
    sudo rosdep init
    rosdep update
#### 5. Environment setup
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
   **change**
    `source /opt/ros/kinetic/setup.bash` and `env`

## Catkin
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
#### initialize the catkin workspace
    catkin_init_workspace
#### list the contents of the current directory 
    ls -l
#### Return to the top level directory
    cd ~/catkin_ws

#### build the workspace (Note: within catkin_ws NOT catkin_ws/src)
    catkin_make
        
http://wiki.ros.org/catkin/conceptual_overview        

## Introduction--turtlesim
###  ROS master - first terminal
    roscore
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
## simple_arm
### Cloning
    cd ~/catkin_ws/src
    git clone https://github.com/udacity/simple_arm_01.git simple_arm
#### Building
    cd ~/catkin_ws
    catkin_make
##### Error: Could not find a package configuration file provided by  "controller_manager"   
    octo@octo:~/catkin_ws$ sudo apt-get install ros-indigo-controller-manager
    octo@octo:~/catkin_ws$ catkin_make
#### ROS LAUNCH
    octo@octo:~/catkin_ws$ source devel/setup.bash
    octo@octo:~/catkin_ws$ roslaunch simple_arm robot_spawn.launch
[WARN] [WallTime: 1511177315.031336] [0.000000] Controller Spawner couldn't find the expected controller_manager ROS interface.
    octo@octo:~/catkin_ws$ rosdep check simple_arm
    octo@octo:~/catkin_ws$ rosdep install -i simple_arm
https://github.com/udacity/simple_arm    
    
##### Dependencies
ROS packages have two different types of dependencies: build dependencies, and run dependencies. This error message was due to a missing runtime dependency. The rosdep tool will check for a package's missing dependencies, download them, and install them.
To check for missing dependencies in the simple_arm package:`$ rosdep check simple_arm`
   
-----
- catkin_ws is workspace

-- simple_arm is package

--- The first node `simple_mover`. simple_mover does nothing more than publish joint angle commands to `simple_arm`.
Second node `arm_mover`. `arm_mover` provides a service called safe_move, which allows the arm to be moved to any position within its workspace which has been deemed to be “safe”. The safe zone is bounded by minimum and maximum joint angles, and is configurable via the ROS’ parameter server. The last node is the `look_away node`. This node subscribes to a topic where camera data is being published. When the camera detects an image with uniform color, meaning it’s looking at the sky, the node will call the safe_move service to move the arm to a new position.

    



