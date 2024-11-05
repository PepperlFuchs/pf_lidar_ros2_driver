## ROS2 driver for R2000 and R2300 laser scanners

![Build Status](https://github.com/PepperlFuchs/pf_lidar_ros2_driver/actions/workflows/main.yml/badge.svg?branch=main)

**Prerequisites:**  
OMDxxx-R2000 Hardware & Firmware >= 1.50 (No support of OBDxxx-R2000 devices)  
OMDxxx-R2300 Hardware >= 0.95, Firmware >= 0.97

**Required platform:**  
Ubuntu-20.04/ROS-Foxy OR Ubuntu-20.04/ROS-Galactic OR Ubuntu-22.04/ROS-Humble OR Ubuntu-24.04/ROS-Jazzy

Note: The ROS1 driver is available here: https://github.com/PepperlFuchs/pf_lidar_ros_driver
  
**Clone the repository:**  
Clone the repository in the `src` folder of your ROS workspace
```
git clone https://github.com/PepperlFuchs/pf_lidar_ros2_driver.git
```
  
**Install the missing dependencies:**  
```
export ROS_DISTRO=foxy OR export ROS_DISTRO=galactic OR export ROS_DISTRO=humble OR export ROS_DISTRO=jazzy
cd <path/to/workspace>
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
```
  
**Build the workspace:**  
```
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source <path/to/workspace>/install/setup.bash
```
  
**Usage:**  
Now you are ready to use the driver. Make the necessary power and ethernet connections. Make sure your computer's IP address is on the same subnet mask as that of the device. Change the `scanner_ip` value in the respective yaml config file that can be found in: `src/pf_lidar_ros_driver/src/pf_driver/config/`. You can now launch one of the drivers in the following manner:  
R2000:
```
ros2 launch pf_driver r2000.launch.py
```
R2300 4-layer:
```
ros2 launch pf_driver r2300.launch.py
```
R2300 1-layer:
```
ros2 launch pf_driver r2300_single_layer.launch.py
```

With R2300, the term scan refers to a contiguous group of measurements spanning one particular horizontal circular
sector. Depending on the orientation of the mirrors on the cube, the scans may be taken in the same or slightly different
layers.  
  
In R2300 4-layer sensors, all four mirrors are inclined slightly differently so that scans are taken at the following vertical
angle (relative to the mounting plane). Note that the layers are numbered in the order they are scanned during one
turn. This is not strictly from bottom to top:

| **Layer index** | **Angle** | **Description** |
|-----------------|-----------|-----------------|
|0 |-4.5°|bottom (connector side)|
|1 |-1.5° | - |
|2 |+4.5° | top |
|3 |+1.5° | - |
