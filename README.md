# ROS_Quadruped

Repository to control a Quadruped Robot via ROS Noetic on a Raspberry Pi.
![](media_files/20221017_184835.gif)
|:--:|
|<b>Crawl Pattern</b>|

## 1. Main Components
- Raspberry Pi 3B+ (https://www.raspberrypi.com/products/raspberry-pi-3-model-b-plus/)
- RDS3225 Servos (or similar) 
- Adafruits PCA9685 Servo Driver (https://www.adafruit.com/product/815)
- Slamtec RPLidar A1 (https://www.slamtec.com/en/Lidar/A1)
- Batterypack 7.2V (Powersupply Servos)
- Waveshare UPS HAT (Powersupply RPi, https://www.waveshare.com/wiki/UPS_HAT)

## 2. Ros Packages

### 2.1 quad_pkg
Imported by hw_motion_main.py to calculate angles for each Leg.
#### 2.1.1 config.py
Contains shared objects which are used in multiple files and a setup function for I2C- and PCA Objects (I2C: Communication protocol between RPi & Servo driver, PCA: Servo driver).
(To Do: define geometry constats which are currently defined in hw_ctrl/hw_motion_main.py)
#### 2.1.2 motion.py
Contains class "Leg" and "Quadruped" which are used to compute the angles of each joint of every leg. 
\t##### Leg methods:
- Vectorgeometry to calculate angles of joints.
\t##### Quadruped methods:
- Combine and coordinate Legs of Robot
- Movement patterns

### 2.2 rplidar_ros
The Lidar is used to detect obstacles around the Robot. The information gained from the Lidar is published by rplidar_ros/node.cpp to /scan. Slamtec provides a ROS package to drive their RPLidar A1. (https://github.com/Slamtec/rplidar_ros)
![](media_files/20221017_184533.gif)
|:--:|
|<b>Obstacle avoidance with Lidar (sped up)</b>|

### 2.3 motion_planning
motion_planning_main.cpp subscribes to /scan from Lidar, processes the received information to avoid obstacles and publishes to /direction. Default direction is forward.

### 2.4 hw_ctrl
hw_motion_main.py subscribes to /direction and uses quad_pkg to control the physical Legs and move the Robot towards the desired direction. 
(To Do: move geometry constants to quad_pkg/config.py)



 
