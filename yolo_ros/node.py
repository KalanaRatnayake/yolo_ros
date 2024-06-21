import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

from ultralytics import YOLO

class YoloROS(Node):

    def __init__(self):
        super().__init__('yolo_ros')

        self.bridge = CvBridge()

        self.model  = YOLO("yolov9t.pt")

        self.subscriber_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription       = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, qos_profile=self.subscriber_qos_profile)
        self.publisher_image    = self.create_publisher(Image, '/yolo_ros/annotated_image', 10)
        
        self.subscription  # prevent unused variable warning


    def image_callback(self, received_msg):

        self.input_image = self.bridge.imgmsg_to_cv2(received_msg, desired_encoding="bgr8")

        self.result = self.model(self.input_image, verbose=False,)

        self.output_image = self.result[0].plot(
            conf=0.25,
            line_width=1,
            font_size=1,
            font="Arial.ttf",
            labels=True,
            boxes=True,
        )
        
        result_msg = self.bridge.cv2_to_imgmsg(self.output_image, encoding="bgr8")

        self.publisher_image.publish(result_msg)
        self.get_logger().debug('Publishing: "%s"' % result_msg.header.frame_id)


def main(args=None):
    rclpy.init(args=args)

    node = YoloROS()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()