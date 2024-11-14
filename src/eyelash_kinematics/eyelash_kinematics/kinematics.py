import rclpy as ros
from rclpy.node import Node
from math import sin, cos
import numpy as np

from geometry_msgs.msg import Point, Vector3
from  isr_proj2_customs.msg import NXTJointState, MirrorPose

class EyelashKinematicsNode(Node):

    def __init__(self):
        super().__init__('eyelash_kinematics_node')
        self.j = None
        self.pose_pub = self.create_publisher(MirrorPose, "eyelash_mirror_pose", 10)
        self.joint_sub = self.create_subscription(NXTJointState, "nxt_motor_states", self.joint_sub_callback, 10)
        self.pub_timer = self.create_timer(0.1, self.pub_callback)

    def joint_sub_callback(self, msg):
        self.j = np.array([msg.joint1, msg.joint2])

    def pub_callback(self):
        ps = self.kinematics(self.j)
        if ps is None:
            return
        
        msg = MirrorPose()
        # Annoyingly ros geometry_messages don't natively use numpy
        msg.center_x = ps[0][0]
        msg.center_y = ps[0][1]
        msg.center_z = ps[0][2]
        msg.upper_right_x = ps[1][0]
        msg.upper_right_y = ps[1][1]
        msg.upper_right_z = ps[1][2]
        msg.lower_right_x = ps[2][0]
        msg.lower_right_y = ps[2][1]
        msg.lower_right_z = ps[2][2]
        msg.lower_left_x  = ps[3][0]
        msg.lower_left_y  = ps[3][1]
        msg.lower_left_z  = ps[3][2]
        msg.upper_left_x  = ps[4][0]
        msg.upper_left_y  = ps[4][1]
        msg.upper_left_z  = ps[4][2]

        self.pose_pub.publish(msg)
        

    # Gives mirror position (in camera/world frame) from joint position 
    def kinematics(self, j):
        if self.j is None:
            self.get_logger().warn("No joint information for kinematics")
            return None
        
        K1 = j[0]
        K2 = j[1]
        T_1_C = -np.array([-0.1778, 0.0381, -0.03048])
        R_1_C = np.eye(3)
        T_2_1 = -np.array([0, 0.032, 0.0742])
        R_2_1 = np.array(
            [[cos(K1), -sin(K1), 0],
            [sin(K1), cos(K1), 0], 
            [0, 0, 1]]
        )
        T_3_2 = -np.array([0.205, 0.012, 0.016])
        R_3_2 = np.array([
            [1, 0, 0],
            [0, cos(K2/24), -sin(K2/24)],
            [0, sin(K2/24), cos(K2/24)]]
        )

        # We want to track the corners of the mirror as well as the center
        cr = np.zeros(3)
        ul = np.array([-1.75, 0,  1.75])
        ur = np.array([ 1.75, 0,  1.75])
        ll = np.array([-1.75, 0, -1.75])
        lr = np.array([ 1.75, 0, -1.75])

        ps = []
        for x in [cr, ur, lr, ll, ul]:
            j2 = R_3_2 @ (x  - T_3_2)
            j1 = R_2_1 @ (j2 - T_2_1)
            c  = R_1_C @ (j1 - T_1_C)
            ps += [c]

        return ps


def main(args=None):
    ros.init(args=args)
    node = EyelashKinematicsNode()
    ros.spin(node)


if __name__ == '__main__':
    main()
