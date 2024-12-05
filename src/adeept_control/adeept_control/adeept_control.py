import rclpy as ros
from rclpy.node import Node
import serial

from  isr_proj2_customs.msg import AdeeptJointCommand, AdeeptJointState

class AdeeptControlNode(Node):

    def __init__(self):
        super().__init__('adeept_control_node')

        self.joint_states = [0, 0, 0, 0, 0]
        self.ser = serial.Serial('/dev/ttyACM0')

        self.motor_subscriber = self.create_subscription(
            AdeeptJointCommand,
            'adeept_joint_commands',
            self.joint_command_sub_callback,
            100)
        
        self.joint_publisher = self.create_publisher(AdeeptJointState, "adeept_joint_states", 10)
        
        self.pub_timer = self.create_timer(0.1, self.joint_state_pub_callback)

    def joint_command_sub_callback(self, msg):
        self.ser.write(b'<%d, %d>' % (msg.joint_id, msg.pos))

        # These hobby servos don't have any feedback (sad)
        #   We'll just assume they make it to where they're supposed to go
        #   Hypothetically we could do something REALLY fancy with CV
        #   to avoid this problem but... no
        self.joint_states[msg.joint_id-1] = msg.pos

    def joint_state_pub_callback(self):
        msg = AdeeptJointState()
        msg.joint1 = float(self.joint_states[0])
        msg.joint2 = float(self.joint_states[1])
        msg.joint3 = float(self.joint_states[2])
        msg.joint4 = float(self.joint_states[3])
        msg.joint5 = float(self.joint_states[4])
        self.joint_publisher.publish(msg)


    

def main(args=None):
    ros.init(args=args)
    node = AdeeptControlNode()
    ros.spin(node)


if __name__ == '__main__':
    main()
