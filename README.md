# yolo_ros

## Docker Usage

### Docker Compose based use

Add the following snippet under `services` to any compose.yaml file to add this container.

```bash
  yolo:
    image: ghcr.io/kalanaratnayake/yolo:humble
    environment:
      - YOLO_MODEL=yolov9t.pt
      - INPUT_TOPIC=/camera/color/image_raw
      - PUBLISH_ANNOTATED_IMAGE=False
      - OUTPUT_ANNOTATED_TOPIC=/yolo_ros/annotated_image
      - OUTPUT_DETAILED_TOPIC=/yolo_ros/detection_result
      - CONFIDENCE_THRESHOLD=0.25
      - DEVICE='0' 
    restart: unless-stopped
    privileged: true
    network_mode: host
    volumes:
      - /yolo:/yolo
```

### Setup for pulling container from ghcr.io and running

Clone this reposiotory

```bash
git clone https://github.com/KalanaRatnayake/yolo_ros.git
```

Pull the Docker image and start compose (No need to run `docker compose build`)
```bash
cd yolo_ros/docker
docker compose pull
docker compose up
```

### Setup for building the container on device running

Clone this reposiotory

```bash
git clone https://github.com/KalanaRatnayake/yolo_ros.git
```

Build the Docker image and start compose
```bash
cd yolo_ros/docker
docker compose -f compose-build.yaml build
docker compose -f compose-build.yaml up
```

## Setup

Clone this repository with and install dependencies.

```bash
git clone https://github.com/KalanaRatnayake/yolo_ros.git
git clone https://github.com/KalanaRatnayake/yolo_ros_msgs.git
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