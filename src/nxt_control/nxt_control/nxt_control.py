import nxt
import nxt.motcont
import nxt.locator
import rclpy as ros
from usb.core import USBError
from rclpy.node import Node
import math

from  isr_proj2_customs.msg import NXTCommand, NXTJointState

class NXTNode(Node):

    def __init__(self):
        super().__init__('nxt_control_node')
        
        self.brick = nxt.locator.find()
        self.motcont = nxt.motcont.MotCont(self.brick)
        self.motcont.start()

        self.ports = {
            "joint1": nxt.motor.Port(0),
            "joint2": nxt.motor.Port(1),
            "joint3": nxt.motor.Port(2),
        }

        self.motors = {
            "joint1": nxt.motor.Motor(self.brick, self.ports["joint1"]),
            "joint2": nxt.motor.Motor(self.brick, self.ports["joint2"]),
            "joint3": nxt.motor.Motor(self.brick, self.ports["joint3"]),
        }


        self.motor_subscriber = self.create_subscription(
            NXTCommand,
            'nxt_motor_commands',
            self.motor_callback,
            100)
        
        self.joint_publisher = self.create_publisher(NXTJointState, "nxt_motor_states", 10)
        
        self.pub_timer = self.create_timer(0.1, self.joint_pub_callback)

    def motor_callback(self, msg):
        try:
            current_pos = self.motors[msg.joint].get_tacho().block_tacho_count / 360 * 2*math.pi
            delta = int((msg.position - current_pos) / (2*math.pi) * 360)
            power = int(msg.power*100 if delta > 0 else -msg.power*100)
            delta = abs(delta)
            
            self.motcont.cmd(
                self.ports[msg.joint],
                power,
                delta,
                brake=True
            )
        except KeyError as e:
            self.get_logger().warn(f"Invalid joint \"{msg.joint}\"")
        except USBError as e:
            self.get_logger().warn(f"Could not communicate with the NXT brick")

    def joint_pub_callback(self):
        msg = NXTJointState()
        msg.joint1 = self.motors["joint1"].get_tacho().block_tacho_count / 360 * 2*math.pi
        msg.joint2 = self.motors["joint2"].get_tacho().block_tacho_count / 360 * 2*math.pi
        msg.joint3 = self.motors["joint3"].get_tacho().block_tacho_count / 360 * 2*math.pi
        self.joint_publisher.publish(msg)


    

def main(args=None):
    ros.init(args=args)
    node = NXTNode()
    ros.spin(node)


if __name__ == '__main__':
    main()
