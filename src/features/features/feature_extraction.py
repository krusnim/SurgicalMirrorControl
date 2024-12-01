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

        # Corners
        self.corners_vis_pub = self.create_publisher(Image, "mirror_corners_vis", 100)

        self.timer = self.create_timer(0.1, self.publish_features_and_corners)
    
    def receive_image(self, msg):
        #self.image = self.bridge.imgmsg_to_cv2(msg)

        # Just adding error handling
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")

    # Detects corner of mirror in given image
    def detect_mirror_corners(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        corners_image = image.copy()
        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) == 4:  # Quadrilateral detected
                for point in approx:
                    cv2.circle(corners_image, tuple(point[0]), 5, (0, 255, 0), -1)  # Draw corner points

        return corners_image

    def publish_features_and_corners(self):
        # if self.image is not None:
        #     keypoints, descriptors = self.sift.detectAndCompute(self.image, None)
        #     sifted_image = cv2.drawKeypoints(self.image, keypoints, self.image, color=(0, 255, 200))
        #     msg = self.bridge.cv2_to_imgmsg(sifted_image, encoding="bgr8")
        #     self.features_vis_pub.publish(msg)

        if self.image is not None:
            # SIFT feature extraction
            keypoints, descriptors = self.sift.detectAndCompute(self.image, None)
            sifted_image = cv2.drawKeypoints(self.image, keypoints, self.image, color=(0, 255, 200))

            # Publish SIFT features visualization
            try:
                sift_msg = self.bridge.cv2_to_imgmsg(sifted_image, encoding="bgr8")
                self.features_vis_pub.publish(sift_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"Error publishing SIFT features: {e}")

            # Mirror corner detection
            corners_image = self.detect_mirror_corners(self.image)

            # Publish mirror corners visualization
            try:
                corners_msg = self.bridge.cv2_to_imgmsg(corners_image, encoding="bgr8")
                self.corners_vis_pub.publish(corners_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"Error publishing mirror corners: {e}")


def main(args=None):
    ros.init(args=args)
    node = FeatureExtractionNode()
    ros.spin(node)

if __name__ == '__main__':
    main()
