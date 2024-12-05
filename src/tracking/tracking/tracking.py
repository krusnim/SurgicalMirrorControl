import rclpy as ros
from rclpy.node import Node
from math import sin, cos
import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Point, Vector3
from  isr_proj2_customs.msg import NXTJointState, MirrorPose

class TrackingNode(Node):

    def __init__(self):
        super().__init__('mirror_tracking_node')

        self.bridge = CvBridge()

        objp = np.zeros((6*8,3), np.float32)
        objp[:,:2] = np.mgrid[0:6,0:8].T.reshape(-1,2)
        objpoints = []
        imgpoints = []
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        cal_img = cv2.imread("/home/mel/isr_proj2/src/camera/calibration.jpeg")
        # cal_img = cv2.resize(cal_img, (400, 400))
        gray = cv2.cvtColor(cal_img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (6,8), None)
        if not ret:
            raise RuntimeError("Could not find chessboard in calibration image")
        

        corners2 = cv2.cornerSubPix(gray, corners, (6,6), (-1,-1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners2)
        err, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        self.camera_matrix = mtx
        self.get_logger().info("Camera calibrated.")

        self.tool_pos = None
        self.image = None

        self.tool_sub = self.create_subscription(MirrorPose, "mirror_pose", self.tool_sub_callback, 10)
        self.camera_sub = self.create_subscription(Image, "camera", self.camera_sub_callback, 10)
        self.center_pub = self.create_publisher(Image, "mirror_center_imgs", 10)

    def camera_sub_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)
    
    def tool_sub_callback(self, msg):
        if self.image is None:
            self.get_logger().warn("No camera image available")
            return
        

        pnt = np.array([msg.center_x, msg.center_z, msg.center_y])

        # was having camera calibration issues so had to
        #   hand calibrate (do not recommend)
        wx = 45 / 41 * pnt[2]
        wy = 33 / 41 * pnt[2]

        center = np.array([
            pnt[0]/wx * 640 + 320,
            -pnt[1]/wy * 480 + 240,
        ], dtype=np.int32)
        
        self.get_logger().info(str(pnt))
        self.get_logger().info(str(center))
        # center = cv2.projectPoints(pnt, np.eye(3), np.zeros(3), self.camera_matrix, np.zeros(5))[0]
        # center = np.reshape(center, (2,))
        # center = (int((center[0])), int((center[1])))
        img = cv2.circle(self.image, center, radius=5, color=(200, 255, 100), thickness=-1)
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.center_pub.publish(img_msg)


def clamp(n, smallest, largest): 
    return max(smallest, min(n, largest)) 

def main(args=None):
    ros.init(args=args)
    node = TrackingNode()
    ros.spin(node)


if __name__ == '__main__':
    main()
