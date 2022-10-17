# ROS_Quadruped

Repository to control a Quadruped Robot with ROS Noetic build on a Raspberry Pi 3B+.
![](media_files/20221017_184835.gif)
|:--:|
|<b>Crawl Pattern</b>|

### Main Components
- Raspberry Pi 3B+ (https://www.raspberrypi.com/products/raspberry-pi-3-model-b-plus/)
- RDS3225 Servos (or similar) 
- Adafruits PCA9685 Servo Driver (https://www.adafruit.com/product/815)
- Slamtec RPLidar A1 (https://www.slamtec.com/en/Lidar/A1)
- Batterypack 7.2V (Powersupply Servos)
- Waveshare UPS HAT (Powersupply RPi, https://www.waveshare.com/wiki/UPS_HAT)

### Ros Packages
##### rplidar_ros
The Lidar is used to detect obstacles around the Robot. The information gained from the Lidar is published by rplidar_ros to /scan and processed by motion_planning to avoid obstacles. Slamtec provides a ROS package to drive their RPLidar A1. (https://github.com/Slamtec/rplidar_ros)
![](media_files/20221017_184533.gif)
|:--:|
|<b>Obstacle avoidance with Lidar (sped up)</b>|

##### 
