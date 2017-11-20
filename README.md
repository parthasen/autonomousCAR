# ROS
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




