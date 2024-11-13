import rclpy as ros
from rclpy.node import Node
from math import sin, cos
import numpy as np

from geometry_msgs.msg import Point, Vector3
from  isr_proj2_customs.msg import NXTJointState

class EyelashKinematicsNode(Node):

    def __init__(self):
        super().__init__('eyelash_kinematics_node')
        self.j = None
        self.pos_pub = self.create_publisher(Point, "eyelash_pos", 10)
        self.norm_pub = self.create_publisher(Vector3, "eyelash_norm", 10)
        self.joint_sub = self.create_subscription(NXTJointState, "nxt_motor_states", self.joint_sub_callback, 10)
        self.pub_timer = self.create_timer(0.1, self.pub_callback)

    def joint_sub_callback(self, msg):
        self.j = np.array([msg.joint1, msg.joint2])

    def pub_callback(self):
        pos, norm = self.kinematics(self.j)
        if pos is not None:
            pos_msg = Point()
            pos_msg.x = pos[0]
            pos_msg.y = pos[1]
            pos_msg.z = pos[2]
            self.pos_pub.publish(pos_msg)

        if norm is not None:
            norm_msg = Vector3()
            norm_msg.x = norm[0]
            norm_msg.y = norm[1]
            norm_msg.z = norm[2]
            self.norm_pub.publish(norm_msg)

    # Gives tool position (in camera/world frame) from joint position 
    def kinematics(self, j):
        if self.j is None:
            self.get_logger().warn("No joint information for kinematics")
            return None, None
        
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
        j2 = R_3_2 @ (   - T_3_2)
        j1 = R_2_1 @ (j2 - T_2_1)
        c  = R_1_C @ (j1 - T_1_C)
        norm = np.array([0, -1, 0]) # Normal of mirror is negative y axis in tool frame
        j2_n = R_3_2 @ (norm - T_3_2)
        j1_n = R_2_1 @ (j2_n - T_2_1)
        c_n  = R_1_C @ (j1_n - T_1_C)
        return c, (c_n-c)


def main(args=None):
    ros.init(args=args)
    node = EyelashKinematicsNode()
    ros.spin(node)


if __name__ == '__main__':
    main()
