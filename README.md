## mowito_ws

This repository consists of assignments given by the Mowito founders for the ROS2 Internship.

### Objective

Create a ROS2 package with a node for video conversion in ROS2.

### Quick Setup

1. Clone the repository
```sh
git clone https://github.com/mohammadraj101/mowito_ws.git src/
git clone https://github.com/ros-drivers/usb_cam.git src/usb_cam
```
2. Install dependencies
```sh
rosdep install --from-paths src --ignore-src -r -y
```
3. Build the workspace

Make sure you have sourced the ROS2 environment before building:

```sh
source /opt/ros/humble/setup.bash
colcon build 
```
4. Source the workspace

After a successful build, source the workspace:
```sh
source install/setup.bash
```
5. Run the node

Launch the video conversion node:
```sh
ros2 launch img_conv launch_service.yaml
```
6. Test the node

To test the service call:

For changing to greyscale image
```sh
ros2 service call /img_convert img_conv/srv/ImgConvert "{mode: true}"
```
For changing to rgb image
```sh
ros2 service call /img_convert img_conv/srv/ImgConvert "{mode: false}"
```
#### Package Structure
```sh
mowito_ws/src
├── img_conv
│   ├── CMakeLists.txt
│   ├── include
│   │   └── img_conv
│   ├── launch
│   │   └── launch_service.yaml
│   ├── package.xml
│   ├── src
│   │   ├── img_client.cpp
│   │   ├── img_service.cpp
│   │   ├── img_subscriber.cpp
│   │   └── publisher.cpp
│   └── srv
│       └── ImgConvert.srv
└── README.md
```
#### Service Definition

img_conv/srv/ImgConvert.srv
```sh
bool mode
---
string result
```
#### Dependencies

Ensure the following dependencies are installed:
```sh
rclcpp

sensor_msgs

cv_bridge

OpenCV
```
Install using:
```sh
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
```
Contributors

RAJ MOHAMMAD



