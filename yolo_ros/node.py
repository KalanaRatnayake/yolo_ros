import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

class YoloROS(Node):

    def __init__(self):
        super().__init__('yolo_ros')

        self.subscription       = self.create_subscription(Image, 'image_in', self.image_callback, 10)
        self.publisher_image    = self.create_publisher(Image, 'image_out', 10)
        
        self.subscription  # prevent unused variable warning


    def image_callback(self, received_msg):
        self.get_logger().debug('Received image')


        result_msg = Image()
        result_msg.data

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