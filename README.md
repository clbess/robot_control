# robot_control

This package can be used on ROS to control turtlebot robots :

- You can control the Turtlebot Burger with 2 bump sensors, 1 IR Sensor and a Servo motor SM-10 attached to rotate the Raspberry Pi camera.
- You can control the Turtlebot Waffle with a WidowX arm attached on it. You can also control each joint of this arm. And finally, you will be able to detect an object and take it with the arm thanks to OpenCV and the Raspberry Pi camera.

-------------------------------------------------------------------------------------------------------------------------

ROBOT_CONTROL :

Download on the PC : 
git clone https://github.com/clbess/robot_control
cd ~/catkin_ws
catkin_make

Download on the robot :
git clone https://github.com/clbess/robot_control
git clone https://github.com/clbess/widowx_arm
git clone https://github.com/vanadiumlabs/arbotix_ros
cd ~/catkin_ws
catkin_make

To control the robot:

On the PC terminal : 
roscore

On the robot terminal :
roslaunch robot_control bringup_burger.launch (or bringup_waffle.launch)

On the PC terminal : 
roslaunch robot_control teleop_burger.launch (or teleop_waffle.launch)

If you just want to search for an object with the Turtlebot waffle : 

On the PC terminal : 
roscore

On the robot terminal :
roslaunch robot_control bringup_waffle.launch

On the PC terminal : 
roslaunch robot_control detection.launch
