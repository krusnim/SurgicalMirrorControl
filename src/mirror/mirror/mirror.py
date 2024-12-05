import rclpy as ros
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from  isr_proj2_customs.msg import NXTCommand

class MirrorNode(Node):

    def __init__(self):
        super().__init__('mirror_node')
        self.bridge = CvBridge()

        self.blob_pub = self.create_publisher(Image, "mirror_marker_blobs", 100)
        self.contour_pub = self.create_publisher(Image, "mirror_marker_contours", 100)
        self.poly_pub = self.create_publisher(Image, "mirror_rect", 100)
        self.image_pub = self.create_publisher(Image, "mirror_image", 100)
        self.timer = self.create_timer(0.1, self.process_image)

        self.image_sub = self.create_subscription(Image, "camera", self.camera_callback, 100)

        # self.frame_width = int(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        # self.frame_height = int(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.image = None

    def camera_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)
    
    def process_image(self):
        if self.image is None:
            self.get_logger().warn("No camera image to process")
            return
        
        im = self.image
        im_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

        # 1: Mask out the magic color
        # lower = np.array([40, 0, 150])
        # upper = np.array([80, 255, 255])
        lower = np.array([40, 50, 150])
        upper = np.array([80, 255, 255])
        mask = 255 - cv2.inRange(im_hsv, lower, upper)

        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        self.blob_pub.publish(mask_msg)

        # 2: Contour detection on the mask

        contours, _ = cv2.findContours(255 - mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours]
        contours.sort(key=len, reverse=True)

        if len(contours) < 4:
            self.get_logger().warn("Cannot find contours")
            return

        im_with_contours = cv2.drawContours(im.copy(), contours, -1, (0, 255, 255), 3)
        contours = contours[0:4]
        contour_msg = self.bridge.cv2_to_imgmsg(im_with_contours, encoding="bgr8")
        self.contour_pub.publish(contour_msg)

        # 3: Mirror corner detection based on the contours
        corner_centers = np.array([np.mean(np.array(contours[i]), axis=0) for i in range(4)])
        corner_centers = np.int32(np.reshape(corner_centers, (-1, 2)))
        mirror_center = np.mean(corner_centers, axis=0)

        inner_corners = []
        for c in contours[0:4]:
            dists = np.sqrt(np.sum((c - mirror_center)**2, axis=2))
            i = np.argmin(dists)
            inner_corners += [c[i, :, :]]

        # corners = np.array(inner_corners)[:, 0, :]

        # Rearrange the corners so they're always clockwise
        def ang(c):
            return np.arctan2(c[:, 1]-mirror_center[1], c[:, 0]-mirror_center[0])

        inner_corners.sort(key=ang)
        corners = inner_corners
        corners = np.array(inner_corners)[:, 0, :]
        int_corners = np.int32(corners)
        float_corners = np.float32(corners)

        # self.get_logger().warn(str(corners))


        im_with_poly = cv2.polylines(im.copy(), [int_corners], True, (0, 255, 255), 3)
        poly_msg = self.bridge.cv2_to_imgmsg(im_with_poly, encoding="bgr8")
        self.poly_pub.publish(poly_msg)

        # 4: Perspective transform
        targ = np.float32([[300,0],[0,0],[0,300],[300,300]])
        M = cv2.getPerspectiveTransform(float_corners, targ)
        im_from_mirror = cv2.warpPerspective(im.copy(),M,(300, 300))[20:280, 20:280]
        im_from_mirror = self.rotate_image(im_from_mirror, 270)

        mirror_msg = self.bridge.cv2_to_imgmsg(im_from_mirror, encoding="bgr8")
        self.image_pub.publish(mirror_msg)


        
    def rotate_image(self, image, angle):
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        return result
        
        

def main(args=None):
    ros.init(args=args)
    node = MirrorNode()
    ros.spin(node)

if __name__ == '__main__':
    main()
