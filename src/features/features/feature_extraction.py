import rclpy as ros
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class FeatureExtractionNode(Node):

    def __init__(self):
        super().__init__('feature_extraction_node')

        self.sift = cv2.xfeatures2d.SIFT_create()
        self.bridge = CvBridge()

        self.image = None
        self.image_sub = self.create_subscription(Image, "camera", self.receive_image, 100)
        self.features_vis_pub = self.create_publisher(Image, "features_vis", 100)

        self.timer = self.create_timer(0.1, self.publish_features)
    
    def receive_image(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)

    def publish_features(self):
        if self.image is not None:
            keypoints, descriptors = self.sift.detectAndCompute(self.image, None)
            sifted_image = cv2.drawKeypoints(self.image, keypoints, self.image, color=(0, 255, 200))
            msg = self.bridge.cv2_to_imgmsg(sifted_image, encoding="bgr8")
            self.features_vis_pub.publish(msg)


def main(args=None):
    ros.init(args=args)
    node = FeatureExtractionNode()
    ros.spin(node)

if __name__ == '__main__':
    main()
