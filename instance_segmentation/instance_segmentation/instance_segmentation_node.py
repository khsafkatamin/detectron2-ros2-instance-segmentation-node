import cv2
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog

class Detectron2Node(Node):

    def __init__(self):
        super().__init__('instance_segmentation_node')
        self.bridge = CvBridge()

        # Declare parameters with default values
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('model_path', '/docker-ros/additional-files/model/model_final.pth')
        self.declare_parameter('config_path', '/docker-ros/additional-files/configs/Cityscapes/mask_rcnn_R_50_FPN.yaml')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        config_path = self.get_parameter('config_path').get_parameter_value().string_value

        # Setup the model and configuration
        self.cfg = self.setup_cfg(config_path, model_path)
        self.predictor = DefaultPredictor(self.cfg)

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.output_publisher = self.create_publisher(Image, '/detectron2/instance_segmentation', 10)

    def setup_cfg(self, config_file, model_weights, confidence_threshold=0.8):
        """
        Setup the configuration for inference.
        """
        cfg = get_cfg()
        cfg.merge_from_file(config_file)
        cfg.MODEL.WEIGHTS = model_weights
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = confidence_threshold
        cfg.MODEL.ROI_HEADS.NUM_CLASSES = 8  # Change based on your dataset
        cfg.freeze()
        return cfg

    def image_callback(self, msg):
        """
        Callback to process incoming camera images and run inference.
        """
        # Convert ROS image message to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting ROS image message: {e}")
            return

        # Run inference
        outputs = self.predictor(cv_image)

        # Visualize the results
        v = Visualizer(cv_image[:, :, ::-1], MetadataCatalog.get(self.cfg.DATASETS.TEST[0]), scale=1.2)
        v = v.draw_instance_predictions(outputs["instances"].to("cpu"))
        result_image = v.get_image()[:, :, ::-1]

        # Convert result image back to ROS message
        result_ros_image = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')

        # Publish the result
        self.output_publisher.publish(result_ros_image)
        self.get_logger().info("Published detection output.")

def main(args=None):
    rclpy.init(args=args)
    node = Detectron2Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
