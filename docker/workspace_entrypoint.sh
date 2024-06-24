#!/bin/bash
set -e

# start udev
/lib/systemd/systemd-udevd --daemon
    
# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$WORKSPACE_ROOT/install/setup.bash"

ros2 run yolo_ros yolo_ros --ros-args -p "yolo_model:=$YOLO_MODEL" \
                                      -p "input_topic:=$INPUT_TOPIC" \
                                      -p "publish_annotated_image:=$PUBLISH_ANNOTATED_IMAGE" \
                                      -p "output_annotated_topic:=$OUTPUT_ANNOTATED_TOPIC" \
                                      -p "output_detailed_topic:=$OUTPUT_DETAILED_TOPIC" \
                                      -p "confidence_threshold:=$CONFIDENCE_THRESHOLD" \
                                      -p "device:=$DEVICE"

exec "$@"
