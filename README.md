# robot_control

This package can be used on ROS to control turtlebot robots :

- You can control the Turtlebot Burger with 1 bump sensor, 1 servomotor SM-10 for the gripper and 1 servomotor SM-10 attached to rotate the Raspberry Pi camera.
- You can control the Turtlebot Waffle with a WidowX arm attached on it. You can also control each joint of this arm. And finally, you will be able to detect an object and take it with the arm thanks to OpenCV and the Raspberry Pi camera.

-------------------------------------------------------------------------------------------------------------------------

## Download :  
  
* __Download on the PC :__  
cd ~/catkin_ws/src  
git clone https://github.com/clbess/robot_control  
sudo chmod 777 -R robot_control/script
cd ~/catkin_ws  
catkin_make  
  
* __Download on the robot :__  
cd ~/catkin_ws/src
git clone https://github.com/clbess/robot_control  
(Only for Turtlebot Waffle with a Widowx arm :  
git clone https://github.com/clbess/widowx_arm  
git clone https://github.com/vanadiumlabs/arbotix_ros)  
cd ~/catkin_ws  
catkin_make  
  
## How to control the robots :   
  
* __On the PC terminal :__  
roscore  
  
* __On the robot terminal :__  
roslaunch robot_control bringup_burger.launch (or bringup_waffle.launch)  
   
* __On the PC terminal :__   
roslaunch robot_control teleop_burger.launch (or teleop_waffle.launch)  
  
## If you just want to search for an object with the Turtlebot waffle :   
  
* __On the PC terminal :__  
roscore  
  
* __On the robot terminal :__  
roslaunch robot_control bringup_waffle.launch  
  
* __On the PC terminal :__   
roslaunch robot_control detection.launch  
