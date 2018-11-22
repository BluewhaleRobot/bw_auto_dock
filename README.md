# bw_auto_dock
ros  automatic charging package

## 1.Hardware Requirements
Need BlueWhale automatic charging kit,you can buy it from www.bwbot.org  Current only support diffDriver mobile base.

## 2.Subscribed Topics      
bw_auto_dock/EnableCharge (std_msgs/Bool)
    Turn automatic charging on or off.   

bw_auto_dock/dockposition_save (std_msgs/Bool)
    Save dock position to local file.

odom (nav_msgs/Odometry)
    Used to get robot current position

## 3.Publish Topics
barDetectFlag  (std_msgs/Bool)
    Turn off infrared obstacle avoidance system  

cmd_vel    (geometry_msgs/Twist)
    A stream of velocity commands meant for execution by mobile base

bw_auto_dock/Chargecurrent (std_msgs/Float32)
    Charging current

bw_auto_dock/Chargepower (std_msgs/Float32)
    Charging voltage

bw_auto_dock/Batterypower (std_msgs/Float32)
    battery voltage

bw_auto_dock/Chargestatus (std_msgs::Int32)
    Charging status

bw_auto_dock/Crashdetector (std_msgs::Int32)
    Collision detection status

bw_auto_dock/Dockpostion (std_msgs::Int32)
    Dock position

bw_auto_dock/IRsensor1 (std_msgs::Int32)
    left sensor1 infrared receiver data

bw_auto_dock/IRsensor2 (std_msgs::Int32)
    left sensor2 infrared receiver data

bw_auto_dock/IRsensor3 (std_msgs::Int32)
    right sensor2 infrared receiver data

bw_auto_dock/IRsensor4 (std_msgs::Int32)
    right sensor1 infrared receiver data

## 4.Parameters

~back_distance (double, default: 0.4)
  The distance from the charging location to the base_link location,base_link is generally located in the middle of the two drive wheels.

~max_linearspeed (double, default: 0.2)
  Maximum line speed during automatic charging alignment.

~max_rotspeed (double, default: 1.4)
  Maximum line speed during automatic charging alignment.

~back_dock_kp (double, default: 0.2)
  PID kp parameter during automatic charging alignment.

~back_dock_ki (double, default: 0.04)
  PID ki parameter during automatic charging alignment.

~back_dock_kd (double, default: 0.0)
  PID kd parameter during automatic charging alignment.

~port (string, default: /dev/ttyUSB004)
  Automatic charging device serial port number.

~odom_frame_id (string, default: odom)
  The frame name of odom topic

~station_filename (string, default: dock_station.txt)
  File name of the charging station location,the full path is "～/slamdb/"

~grid_length (double, default: 4.0)

## 5.Usage:
### Download to xiaoqiang ros workspace
```
cd ~/Documents/ros/src
git clone https://github.com/BlueWhaleRobot/bw_auto_dock.git
cd ..
catkin_make
```
### Quickstart
```
roslaunch bw_auto_dock xiaoqiang_local.launch
```
### Tutorial
http://community.bwbot.org/topic/501/bw_auto_dock自动充电功能包的使用和实现原理

## Made with :heart: by BlueWhale Tech corp.
