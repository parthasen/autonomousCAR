# ROS
### roscore

    octo@octo:~/catkin_ws$ roscore

PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.21

NODES

auto-starting new master
process[master]: started with pid [5107]
ROS_MASTER_URI=http://octo:11311/

setting /run_id to 8bfc93e0-b1b8-11e7-be21-3417ebb661ee
process[rosout-1]: started with pid [5120]
started core service [/rosout]

### another terminal
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

### Simulation
octo@octo:~$ rosrun turtlesim turtle_teleop_key
