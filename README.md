# yolo_ros

## Setup

Clone this repository with and install dependencies.

```bash
git clone https://github.com/AIResearchLab/yolo_ros.git
git clone https://github.com/AIResearchLab/yolo_ros_msgs.git
cd yolo_ros
pip3 install -r requirements.txt
```

## Build the package

At the workspace root run,

```bash
colcon build
```

## Start the system

### With ros2 launch
To use the launch file, run,

```bash
source ./install/setup.bash
ros2 launch yolo_ros yolo.launch.py
```

### With ros2 run
To start the system run,

```bash
source ./install/setup.bash
ros2 run yolo_ros yolo_ros          
```
or
```bash
source ./install/setup.bash
ros2 run yolo_ros yolo_ros --ros-args -p "yolo_model:=yolov9t.pt" \
                                      -p "input_topic:=/camera/color/image_raw" \
                                      -p "publish_annotated_image:=False" \
                                      -p "output_annotated_topic:=/yolo_ros/annotated_image" \
                                      -p "output_detailed_topic:=/yolo_ros/detection_result" \
                                      -p "confidence_threshold:=0.25" \
                                      -p "device:='0'"                                 
```