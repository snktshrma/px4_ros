# px4_ros_slam package for non-gps navigation

## Description : 
This is a ROS package for non-gps navigation for px4 containing all the required files and listed dependencies. This also contains a file from [thien94/vision_to_mavros](https://github.com/thien94/vision_to_mavros) to set origin. 

This package contains all the modified files for the following packages:
````
mavros
cartographer_ros
robot_pose_publisher
navigation
````



## Requirements :
Ubuntu version 20.04

ROS Noetic

## Usage 1 :

On the px4 latest stable, run:

````
source ~/ardupilot/Tools/environment_install/install-ROS-ubuntu.sh
````
This will install the whole px4_ros_slam environment and install all the requirements.

## Usage 2 :

This package contains a main launch file launching all the required launch files in a go.

Simply,

````
cd <ros_ws>/src/
git clone https://github.com/snktshrma/px4_ros_slam
git clone https://github.com/GT-RAIL/robot_pose_publisher.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
cd px4_ros_slam
catkin build
````

## Run :

### On 1st Terminal
````
roslaunch px4_ros_slam gzbo.launch #for quadrotor
````
or
````
roslaunch px4_ros_slam gzbo_rover.launch #for rover
````

### On 2nd Terminal (under ~/ardupilot/ArduCopter or ~/ardupilot/Rover)
````
../Tools/autotest/sim_vehicle.py -f gazebo-iris
````
or
````
sim_vehicle.py -v APMrover2 -f gazebo-rover -m --mav10 -I1
````

### On 3rd Terminal
````
roslaunch px4_ros_slam main.launch
````
