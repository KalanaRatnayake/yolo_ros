import rclpy
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from yolo_ros_msgs.msg import YoloResult

from cv_bridge import CvBridge

from ultralytics import YOLO

class YoloROS(Node):

    def __init__(self):
        super().__init__('yolo_ros')

        self.declare_parameter("yolo_model",                "yolov8n.pt")
        self.declare_parameter("input_topic",               "/camera/color/image_raw")
        self.declare_parameter("publish_annotated_image",   False)
        self.declare_parameter("output_annotated_topic",    "/yolo_ros/annotated_image")
        self.declare_parameter("output_detailed_topic",     "/yolo_ros/detection_result")
        self.declare_parameter("confidence_threshold",      0.25)
        self.declare_parameter("device",                    "cpu")
        self.declare_parameter("use_fuse",                  False)
        self.declare_parameter("use_onnx",                  False)
        self.declare_parameter("use_tensorrt",              False)

        self.yolo_model                 = self.get_parameter("yolo_model").get_parameter_value().string_value
        self.input_topic                = self.get_parameter("input_topic").get_parameter_value().string_value
        self.publish_annotated_image    = self.get_parameter("publish_annotated_image").get_parameter_value().bool_value
        self.output_annotated_topic     = self.get_parameter("output_annotated_topic").get_parameter_value().string_value
        self.output_detailed_topic      = self.get_parameter("output_detailed_topic").get_parameter_value().string_value
        self.confidence_threshold       = self.get_parameter("confidence_threshold").get_parameter_value().double_value
        self.device                     = self.get_parameter("device").get_parameter_value().string_value
        self.use_onnx                   = self.get_parameter("use_onnx").get_parameter_value().bool_value
        self.use_tensorrt               = self.get_parameter("use_tensorrt").get_parameter_value().bool_value
        self.use_fuse                   = self.get_parameter("use_fuse").get_parameter_value().bool_value

        self.bridge = CvBridge()

        if self.use_tensorrt:
            self.init_model     = YOLO(self.yolo_model)                         # Load pytorch model
            self.exported_model = self.init_model.export(format="engine")       # Export the model to TensorRT format. returns '<model_name>.engine'
            self.model          = YOLO(self.exported_model)                     # Load the exported TensorRT model
            
        elif self.use_onnx:
            self.init_model     = YOLO(self.yolo_model)                         # Load pytorch model
            self.exported_model = self.init_model.export(format="onnx")         # Export the model to ONXX format. returns '<model_name>.onnx'
            self.model          = YOLO(self.exported_model, task="detect")        # Load the exported TensorRT model

        elif self.use_fuse:
            self.model          = YOLO(self.yolo_model)
            self.model.fuse()

        else:
            self.model          = YOLO(self.yolo_model)

        self.subscriber_qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                                 history=QoSHistoryPolicy.KEEP_LAST,
                                                 depth=1
                                                 )

        self.subscription       = self.create_subscription(Image, self.input_topic, self.image_callback, qos_profile=self.subscriber_qos_profile)
        
        self.publisher_results  = self.create_publisher(YoloResult, self.output_detailed_topic, 10)

        if self.publish_annotated_image:
            self.publisher_image    = self.create_publisher(Image, self.output_annotated_topic, 10)

        self.subscription  # prevent unused variable warning
        self.counter = 0
        self.time = 0


    def image_callback(self, received_msg):
        start = time.time_ns()

        self.input_image = self.bridge.imgmsg_to_cv2(received_msg, desired_encoding="bgr8")

        self.result = self.model.predict(
            source = self.input_image,
            conf=self.confidence_threshold,
            device=self.device,
            verbose=False
        )

        if self.result is not None:
            detection_msg = YoloResult()

            detection_msg.header       = received_msg.header
            detection_msg.source_image = received_msg

            for bbox, cls, conf in zip(self.result[0].boxes.xywh, self.result[0].boxes.cls, self.result[0].boxes.conf):

                detection_msg.bbx_center_x.append(float(bbox[0]))
                detection_msg.bbx_center_y.append(float(bbox[1]))
                detection_msg.bbx_size_w.append(float(bbox[2]))
                detection_msg.bbx_size_h.append(float(bbox[3]))

                class_name_msg      = String()
                class_name_msg.data = self.result[0].names.get(int(cls))
                
                detection_msg.class_name.append(class_name_msg)
                detection_msg.confidence.append(float(conf))

            self.publisher_results.publish(detection_msg)

            if self.publish_annotated_image:
                self.output_image = self.result[0].plot(
                                                        conf=True,
                                                        line_width=1,
                                                        font_size=1,
                                                        font="Arial.ttf",
                                                        labels=True,
                                                        boxes=True,
                                                    )
                    
                result_msg = self.bridge.cv2_to_imgmsg(self.output_image, encoding="bgr8")
                
                self.publisher_image.publish(result_msg)

        self.counter += 1
        self.time += time.time_ns() - start

        if (self.counter == 100):
            self.get_logger().info('Callback execution time for 100 loops: %d ms' % ((self.time/100)/1000000))
            self.time = 0
            self.counter = 0


def main(args=None):
    rclpy.init(args=args)

    node = YoloROS()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()