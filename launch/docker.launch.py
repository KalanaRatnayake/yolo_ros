import os

import ament_index_python.packages
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import yaml


def generate_launch_description():

    yolo_model_param                = DeclareLaunchArgument('yolo_model',               default_value="yolov9s.pt")
    input_rgb_topic_param           = DeclareLaunchArgument('input_rgb_topic',          default_value="/camera/color/image_raw")
    input_depth_topic_param         = DeclareLaunchArgument('input_depth_topic',        default_value="/camera/depth/points")
    subscribe_depth_param           = DeclareLaunchArgument('subscribe_depth',          default_value=True)
    publish_annotated_image_param   = DeclareLaunchArgument('publish_annotated_image',  default_value=True)
    annotated_topic_param           = DeclareLaunchArgument('annotated_topic',          default_value="/yolo_ros/annotated_image")
    detailed_topic_param            = DeclareLaunchArgument('detailed_topic',           default_value="/yolo_ros/detection_result")
    threshold_param                 = DeclareLaunchArgument('threshold',                default_value=0.25)
    device_param                    = DeclareLaunchArgument('device',                   default_value="0")


    yolo_ros_node = launch_ros.actions.Node(package='yolo_ros',
                                              executable='yolo_ros',
                                              output='both'
                                              )

    ld = LaunchDescription()

    ld.add_action(yolo_model_param)
    ld.add_action(input_rgb_topic_param)
    ld.add_action(input_depth_topic_param)
    ld.add_action(subscribe_depth_param)
    ld.add_action(publish_annotated_image_param)
    ld.add_action(annotated_topic_param)
    ld.add_action(detailed_topic_param)
    ld.add_action(threshold_param)
    ld.add_action(device_param)

    ld.add_action(yolo_ros_node)

    return ld