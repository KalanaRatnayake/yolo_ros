# yolo_ros

## Setup

Clone this repository with and install dependencies.

```bash
git clone https://github.com/AIResearchLab/yolo_ros.git
cd yolo_ros
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
