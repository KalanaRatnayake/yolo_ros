# yolo_ros

## Docker Usage by adding to compose.yml file

To use GPU with docker while on AMD64 systems, install [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) with given instructions.

### Supported platforms

Replace `image` parameter in the compose.yml with following values for respective systems.

| System       | ROS Version | Value | Size |
| :---         | :---        | :---  | :---:|
| AMD64        | Humble      | ghcr.io/kalanaratnayake/yolo-ros:humble | 5.64 GB |
| Jetson Nano  | Humble      | ghcr.io/kalanaratnayake/yolo-ros:humble-j-nano | 3.29GB |

### Default models with Docker Compose

Add the following snippet under `services` to any compose.yaml file to add this container and use an existing model.

```bash
services:
  yolo:
    image: ghcr.io/kalanaratnayake/yolo-ros:humble
    environment:
      - YOLO_MODEL=yolov9t.pt
      - INPUT_TOPIC=/camera/color/image_raw
      - PUBLISH_ANNOTATED_IMAGE=True
      - OUTPUT_ANNOTATED_TOPIC=/yolo_ros/annotated_image
      - OUTPUT_DETAILED_TOPIC=/yolo_ros/detection_result
      - CONFIDENCE_THRESHOLD=0.25
      - DEVICE='0'
    restart: unless-stopped
    privileged: true
    network_mode: host
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]   
    volumes:
      - yolo:/yolo

volumes:
  yolo:
```

### Custom models with Docker Compose

Add the following snippet under `services` to any compose.yaml file to add this container and use an existing model.

```bash
services:
  yolo:
    image: ghcr.io/kalanaratnayake/yolo-ros:humble
    environment:
      - YOLO_MODEL=weights/yolov9t.pt
      - INPUT_TOPIC=/camera/color/image_raw
      - PUBLISH_ANNOTATED_IMAGE=True
      - OUTPUT_ANNOTATED_TOPIC=/yolo_ros/annotated_image
      - OUTPUT_DETAILED_TOPIC=/yolo_ros/detection_result
      - CONFIDENCE_THRESHOLD=0.25
      - DEVICE='0'
    restart: unless-stopped
    privileged: true
    network_mode: host
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]   
    volumes:
      - type: bind
        source: /home/kalana/Downloads/weights
        target: /yolo/weights

volumes:
  yolo:
```

## Docker Usage with this repository

Clone this reposiotory

```bash
mkdir -p yolo_ws/src && cd yolo_ws/src
git clone https://github.com/KalanaRatnayake/yolo_ros.git && cd ..
```

<details> 
<summary> <h3> on AMD64 </h3> </summary>

Pull the Docker image and start compose (No need to run `docker compose build`)
```bash
cd src/yolo_ros/docker
docker compose -f compose.amd64.yaml pull
docker compose -f compose.amd64.yaml up
```
</details>

<details> 
<summary> <h3> on ARM64 (non-jetson like Raspberry Pi) </h3> </summary>

Pull the Docker image and start compose (No need to run `docker compose build`)
```bash
cd src/yolo_ros/docker
docker compose -f compose.ard64.yaml pull
docker compose -f compose.ard64.yaml up
```
</details>

<details> 
<summary> <h3> on JetsonNano </h3> </summary>

Pull the Docker image and start compose (No need to run `docker compose build`)
```bash
cd src/yolo_ros/docker
docker compose -f compose.jnano.yaml pull
docker compose -f compose.jnano.yaml up
```
</details>

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
| device                  | DEVICE                  | `'0'`                       | `cpu` for CPU, `0` for gpu, `0,1,2,3` if there are multiple GPUs |
| use_tensorrt            | USE_TENSORRT            | `False`                     | Whether to use tensorrt based optimizations |
| use_onnx                | USE_ONNX                | `False`                     | Whether to use onnx based optimizations |
| use_fuse                | USE_FUSE                | `False`                     | Whether to use model.fuse() for optimizations |

[1] If the model is available at [ultralytics models](https://docs.ultralytics.com/models/), It will be downloaded from the cloud at the startup. We are using docker volumes to maintain downloaded weights so that weights are not downloaded at each startup. Use the snipped in [Default models with Docker Compose](https://github.com/KalanaRatnayake/yolo_ros#default-models-with-docker-compose)

[2] Give the custom model weight file's name as `YOLO_MODEL` parameter. Update the docker volume source tag to direct to the folder and use docker bind-mounts instead of docker volumes where the weight file exist in the host machine. As an example if the weight file is in `/home/kalana/Downloads/weight/yolov9s.pt` then use the snipped in [Custom models with Docker Compose](https://github.com/KalanaRatnayake/yolo_ros#custom-models-with-docker-compose)