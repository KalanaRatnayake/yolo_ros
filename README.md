# yolo_ros

A ros package that can load given YOLO models.

## Setup

Clone this repository with

```bash
git clone https://github.com/AIResearchLab/yolo_ros.git
```

Run following to install dependencies

```bash
cd yolo_ros/yolo_ros/YOLO
pip3 install -r requirements.txt
```

## Build the package

At the workspace root run,

```bash
colcon build
```

## Start the system

To start the system run,

```bash
source ./install/setup.bash
ros2 launch yolo_ros yolo.launch.py
```