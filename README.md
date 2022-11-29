# ROS_Quadruped

Repository to control a Quadruped Robot via ROS Noetic on a Raspberry Pi.
![](media_files/20221017_184835.gif)
|:--:|
|<b>Crawl Pattern</b>|


>**Note:**
>This Repository is a work in progress and serves as a place to take notes, it is not intended to be cloned. Readability and structure of the code need to be improved. However if you are interested in the project and have questions, don't hesitate to contact me.


## 1. Main Hardware Components
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
##### Leg methods:
- Vectorgeometry to calculate angles of joints.
##### Quadruped methods:
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

## 3. Startup Procedure 
on remote machine:
1. source all Terminals:  `source devel/setup.bash`
2. start master:          `roscore`
3. launch lidar:          `roslaunch rplidar_ros rplidar.launch`
4. launch webcam          `roslaunch video_stream_opencv webcam.launch`
5. run motion_planning:   `rosrun motion_planning motion_planning_main`
6. run hw_ctrl:           `rosrun hw_ctrl hw_motion_main.py`

### 3.1 View Data in Rviz
(0. export ROS_MASTER_URI & ROS_IP on both, local and remote machine, 
if not already defined in ~.bashrc)
on local machine:
1. test connection:     `rostopic list`
2. start rviz:          `rviz rviz`
3. Add Image:           "Add" > "Image"
   3.1 subscribe to topic:  "Image" > "Image Topic" > "/webcam/image_raw"
4. Add LaserScan:       "Add" > "LaserScan"
   4.1 subscribe to topic:  "LaserScan" > "Topic" > "/scan"
   4.2 change fixed frame:  "Global Options" > "Fixed Frame" > "/laser"

## 4. Mechanics and Geometry
![](media_files/Top_view.jpeg)
|:--:|
|<b>Top View</b>|

![](media_files/Perspective_view.jpeg)
|:--:|
|<b>Perspective View</b>|

## 5. Wiring

Connect Servos to Servo Driver:
```     
        config.kit.servo[2].angle = limitAngle(self.legA.alphaCorr, minAngle, maxAngle)
        config.kit.servo[3].angle = limitAngle(self.legA.betaCorr, minAngle, maxAngle)
        config.kit.servo[4].angle = limitAngle(self.legA.gammaCorr, minAngle, maxAngle)

        config.kit.servo[5].angle = limitAngle(self.legB.alphaCorr, minAngle, maxAngle)
        config.kit.servo[6].angle = limitAngle(self.legB.betaCorr, minAngle, maxAngle)
        config.kit.servo[7].angle = limitAngle(self.legB.gammaCorr, minAngle, maxAngle)

        config.kit.servo[8].angle = limitAngle(self.legC.alphaCorr, minAngle, maxAngle)
        config.kit.servo[9].angle = limitAngle(self.legC.betaCorr, minAngle, maxAngle)
        config.kit.servo[10].angle = limitAngle(self.legC.gammaCorr, minAngle, maxAngle)

        config.kit.servo[11].angle = limitAngle(self.legD.alphaCorr, minAngle, maxAngle)
        config.kit.servo[12].angle = limitAngle(self.legD.betaCorr, minAngle, maxAngle)
        config.kit.servo[13].angle = limitAngle(self.legD.gammaCorr, minAngle, maxAngle)
```
