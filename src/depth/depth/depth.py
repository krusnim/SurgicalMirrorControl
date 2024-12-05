from transformers import pipeline
from PIL import Image
import matplotlib.pyplot as plt
import requests

import rclpy as ros
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge, CvBridgeError
import numpy as np



class DepthNode(Node):

    def __init__(self):
        super().__init__('depth_node')
        self.bridge = CvBridge()

        self.image = None
        self.depth_pub = self.create_publisher(Image, "depth", 100)
        self.image_sub = self.create_subscription(Image, "mirror_image", self.image_callback, 100)
        self.depth_srv = self.create_service(Trigger, "depthing", self.do_depth_service)

        # self.frame_width = int(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        # self.frame_height = int(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.image = None

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)

    def do_depth_service(self, request, response):
        if self.image is not None:
            cv2.imwrite("tmp_mirror.png", self.image)

            pipe = pipeline(task="depth-estimation", model="depth-anything/Depth-Anything-V2-Small-hf")
            depth_est = np.array(pipe('./tmp_mirror.png')["depth"])

            msg = self.bridge.cv2_to_imgmsg(depth_est, encoding="mono8")

            self.get_logger().info(f"Depth estimation done; got image of shape {str(depth_est.shape)}")
            self.depth_pub.publish(msg)
            response.success = True
        else:
            self.get_logger().warn("No image available to depth")
            response.success = False
        
        return response
        




def main(args=None):
    ros.init(args=args)
    node = DepthNode()
    ros.spin(node)

if __name__ == '__main__':
    main()
