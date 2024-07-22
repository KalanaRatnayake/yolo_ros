import os

import ament_index_python.packages
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import yaml


def generate_launch_description():

    yolo_model_param                = DeclareLaunchArgument(name='yolo_model',               default_value="yolov9s.pt")
    input_rgb_topic_param           = DeclareLaunchArgument(name='input_rgb_topic',          default_value="/camera/color/image_raw")
    input_depth_topic_param         = DeclareLaunchArgument(name='input_depth_topic',        default_value="/camera/depth/points")
    subscribe_depth_param           = DeclareLaunchArgument(name='subscribe_depth',          default_value="True")
    publish_annotated_image_param   = DeclareLaunchArgument(name='publish_annotated_image',  default_value="True")
    annotated_topic_param           = DeclareLaunchArgument(name='annotated_topic',          default_value="/yolo_ros/annotated_image")
    detailed_topic_param            = DeclareLaunchArgument(name='detailed_topic',           default_value="/yolo_ros/detection_result")
    threshold_param                 = DeclareLaunchArgument(name='threshold',                default_value="0.25")
    device_param                    = DeclareLaunchArgument(name='device',                   default_value="'0'")

    yolo_ros_node = launch_ros.actions.Node(package='yolo_ros',
                                              executable='yolo_ros',
                                              output='both',
                                              parameters=[{
                                                  'yolo_model': LaunchConfiguration('yolo_model'),
                                                  'input_rgb_topic': LaunchConfiguration('input_rgb_topic'),
                                                  'input_depth_topic': LaunchConfiguration('input_depth_topic'),
                                                  'subscribe_depth': LaunchConfiguration('subscribe_depth'),
                                                  'publish_annotated_image': LaunchConfiguration('publish_annotated_image'),
                                                  'annotated_topic': LaunchConfiguration('annotated_topic'),
                                                  'detailed_topic': LaunchConfiguration('detailed_topic'),
                                                  'threshold': LaunchConfiguration('threshold'),
                                                  'device': LaunchConfiguration('device'),
                                              }]
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