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

**ROS-Topics:**  
The R2000 and R2300 devices each publish two topics. See the topics documentation [topics.md](./docs/topics.md) for more details.

**ROS-Services:**  
The ROS driver offers several ROS services which can be used to communicate with the sensor. Especially
services for sensor parametrization are available. For example to list, get and set parameters. See the 
services documentation [services.md](./docs/services.md) for more details and how to use these services.

**Configure device settings at driver start:**  
It is possible to set device parameters when the driver starts. To do this, the corresponding file in the
directory \src\pf_driver\config\*.yaml (e.g. r2000_params.yaml) must be adapted, which is then used by the
launch file as a parameters file. By adding the following line, the two device parameters 'user_tag' and
'hmi_application_text_1' are set to 'myTag20090505' and 'MyHMIapplText19800128' respectively in a R2000 device.
```
pfsdp_init: ['user_tag=myTag20090505', 'hmi_application_text_1=MyHMIapplText19800128']
```
Other device parameters can be set in this way. An overview of the settable device parameters and their
possible adjustable values can be found in the following documents
[R2000](https://files.pepperl-fuchs.com/webcat/navi/productInfo/doct/doct3469g.pdf) /
[R2300](https://files.pepperl-fuchs.com/webcat/navi/productInfo/doct/doct7001b.pdf)).
