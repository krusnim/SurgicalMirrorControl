import rclpy as ros
from rclpy.node import Node
from math import sin, cos, pi
import numpy as np

from  isr_proj2_customs.msg import AdeeptJointState, MirrorPose

class AdeeptKinematicsNode(Node):

    def __init__(self):
        super().__init__('adeept_kinematics_node')
        self.j = None
        self.pose_pub = self.create_publisher(MirrorPose, "mirror_pose", 10)
        self.joint_sub = self.create_subscription(AdeeptJointState, "adeept_joint_states", self.joint_sub_callback, 10)
        self.pub_timer = self.create_timer(0.1, self.pub_callback)

    def joint_sub_callback(self, msg):
        self.j = np.array([msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5])

    def pub_callback(self):
        ps = self.mirror_kinematics(self.j)
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
    def mirror_kinematics(self, j):
        if self.j is None:
            self.get_logger().warn("No joint information for kinematics")
            return None
        
        j1 = self.j[0] * pi/180
        j2 = self.j[1] * pi/180
        j3 = self.j[2] * pi/180
        j4 = self.j[3] * pi/180
        j5 = self.j[4] * pi/180

        # Some of the joints are a little crooked; here's an offset
        o3 = 8 * pi/180
        o4 = 4 * pi/180

        T_1_C = np.array([-160, 196, -148.84])
        T_2_1 = np.array([0, 0, 90.58])
        T_3_2 = np.array([0, -64.91, 0])
        T_4_3 = np.array([0, 3.80, 59.36])
        T_5_4 = np.array([-10.45, -8.02, 20.22])
        T_E_5 = np.array([-12.06, -106, 0])


        rx = lambda j: np.array([[1, 0, 0], [0, cos(j), -sin(j)], [0, sin(j), cos(j)]])
        ry = lambda j: np.array([[cos(j), 0, sin(j)], [0, 1, 0], [-sin(j), 0, cos(j)]])
        rz = lambda j: np.array([[cos(j), -sin(j), 0], [sin(j), cos(j), 0], [0, 0, 1]])

        R_1_C = np.eye(3)
        R_2_1 = rz(j1)
        R_3_2 = rx(-j2)
        R_4_3 = rx(j3+o3)
        R_5_4 = rz(j4+o4)
        R_E_5 = rx(-j5)

        x_frameE = np.zeros(3)
        x_frame5 = R_E_5 @ (x_frameE + T_E_5)
        x_frame4 = R_5_4 @ (x_frame5 + T_5_4)
        x_frame3 = R_4_3 @ (x_frame4 + T_4_3)
        x_frame2 = R_3_2 @ (x_frame3 + T_3_2)
        x_frame1 = R_2_1 @ (x_frame2 + T_2_1)
        x_framec = R_1_C @ (x_frame1 + T_1_C)

        # We want to track the corners of the mirror as well as the center
        cr = np.zeros(3)
        ul = np.array([-1.25, 0,  1.25])
        ur = np.array([ 1.25, 0,  1.25])
        ll = np.array([-1.25, 0, -1.25])
        lr = np.array([ 1.25, 0, -1.25])

        ps = []
        for x in [cr, ur, lr, ll, ul]:
            x_frameE = x
            x_frame5 = R_E_5 @ (x_frameE + T_E_5)
            x_frame4 = R_5_4 @ (x_frame5 + T_5_4)
            x_frame3 = R_4_3 @ (x_frame4 + T_4_3)
            x_frame2 = R_3_2 @ (x_frame3 + T_3_2)
            x_frame1 = R_2_1 @ (x_frame2 + T_2_1)
            x_framec = R_1_C @ (x_frame1 + T_1_C)

            # In practice the arm hangs a couple centimeters down and out due to gravity
            #   (because crappy servos, parts, etc). Very annoying.
            #   We can't really model this in full so we just take a guess
            x_framec[2] -= 60
            x_framec[0] += 30

            ps += [x_framec]

        return ps


def main(args=None):
    ros.init(args=args)
    node = AdeeptKinematicsNode()
    ros.spin(node)


if __name__ == '__main__':
    main()
