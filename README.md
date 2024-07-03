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
      - USE_FUSE=False
      - USE_ONNX=False
      - USE_TENSORRT=False
    restart: unless-stopped
    privileged: true
    network_mode: host
    volumes:
      - /yolo:/yolo
```

Replace `image` parameter with following values for respective systems.

| System       | ROS Version | Value |
| :---         | :---        | :---  |
| AMD64        | Humble      | ghcr.io/kalanaratnayake/yolo-ros:humble |
| Jetson Nano  | Humble      | ghcr.io/kalanaratnayake/yolo-ros:humble-j-nano |


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

### Setup for building the container on device and running

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

### Build the package

If required, edit the parameters at `config/yolo_ros_params.yaml' and then at the workspace root run,
```bash
colcon build
```
### Start the system

To use the launch file, run,

```bash
source ./install/setup.bash
ros2 launch yolo_ros yolo.launch.py
```

<br>
<br>

## Parameter decription

| ROS Parameter           | Docker ENV parameter    | Default Value               | Description |
| :---                    | :---                    | :---:                       | :---        |
| yolo_model              | YOLO_MODEL              | `yolov9t.pt`                | Model to be used. see [1] for default models and [2] for custom models |
| input_topic             | INPUT_TOPIC             | `/camera/color/image_raw`   | Topic to subscribe for RGB image. Accepts `sensor_msgs/Image` |
| publish_annotated_image | PUBLISH_ANNOTATED_IMAGE | `False`                     | Whether to publish annotated image, increases callback execution time when set to `True` |
| output_annotated_topic  | OUTPUT_ANNOTATED_TOPIC  | `/yolo_ros/annotated_image` | Topic for publishing annotated images uses `sensor_msgs/Image` |
| output_detailed_topic   | OUTPUT_DETAILED_TOPIC   | `/yolo_ros/detection_result`| Topic for publishing detailed results uses `yolo_ros_msgs/YoloResult` |
| confidence_threshold    | CONFIDENCE_THRESHOLD    | `0.25`                      | Confidence threshold for predictions |
| device                  | DEVICE                  | `'0'`                       | `cpu` for CPU, `0` for gpu, [`0`, `1`, ...] if there are multiple GPUs |
| use_tensorrt            | USE_TENSORRT            | `False`                     | Whether to use tensorrt based optimizations |
| use_onnx                | USE_ONNX                | `False`                     | Whether to use onnx based optimizations |
| use_fuse                | USE_FUSE                | `False`                     | Whether to use model.fuse() for optimizations |


[1] If the model is available at [ultralytics models](https://docs.ultralytics.com/models/), It will be downloaded from the cloud at the startup. We are using docker volumes to maintain downloaded weights so that weights are not downloaded at each startup.

[2] Give the custom model weight file's name as `YOLO_MODEL` parameter. Update the docker volume tag to direct to the folder where the weight file exist. As an example if the weight file is in `/home/user/Downloads/model/yolov9t.pt` then update the volumes tag as follows
```bash
    volumes:
      - /home/user/Downloads/model:/yolo
```