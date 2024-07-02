# yolo_ros

## Docker Usage

### Docker Compose based use

Add the following snippet under `services` to any compose.yaml file to add this container.

```bash
  yolo:
    image: ghcr.io/kalanaratnayake/yolo-ros:humble
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
<br>

Replace `image` parameter with following values for respective systems.

| System       | ROS Version | Value |
| :---         | :---        | :---  |
| AMD64        | Humble      | ghcr.io/kalanaratnayake/yolo-ros:humble |
| Jetson Nano  | Humble      | ghcr.io/kalanaratnayake/yolo-ros:humble-j-nano |

<br>

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

<br>

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

<br>
<br>

## Native Usage

Clone this repository with and install dependencies.

```bash
git clone https://github.com/KalanaRatnayake/yolo_ros.git
git clone https://github.com/KalanaRatnayake/yolo_ros_msgs.git
cd yolo_ros
pip3 install -r requirements.txt
```

<br>

### Build the package

If required, edit the parameters at `config/yolo_ros_params.yaml' and then at the workspace root run,
```bash
colcon build
```
<br>

### Start the system

To use the launch file, run,

```bash
source ./install/setup.bash
ros2 launch yolo_ros yolo.launch.py
```