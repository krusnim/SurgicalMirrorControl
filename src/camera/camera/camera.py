import rclpy as ros
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from  isr_proj2_customs.msg import NXTCommand

class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.cam = cv2.VideoCapture(4)
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "camera", 10000)

        self.timer = self.create_timer(0.1, self.publish_image)

        # self.frame_width = int(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        # self.frame_height = int(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

    
    def publish_image(self):
        try:
            success, frame = self.cam.read()
            if not success:
                self.get_logger().warn("Error reading camera input")
                return
            
            # Throughput is very lacking for image messages due to
            #   ... unclear, but probably inefficiency with translating back
            #   and forth from Python objects
            frame = cv2.resize(frame, (400, 400))
            
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(msg)
        except CvBridgeError as e:
            self.get_logger().warn("CV bridge error")


        
        

def main(args=None):
    ros.init(args=args)
    node = CameraNode()
    ros.spin(node)

if __name__ == '__main__':
    main()
