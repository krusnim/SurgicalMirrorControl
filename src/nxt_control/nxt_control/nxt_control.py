import nxt
import nxt.motcont
import nxt.locator
import rclpy as ros
from usb.core import USBError
from rclpy.node import Node

from  isr_proj2_customs.msg import NXTCommand

class NXTNode(Node):

    def __init__(self):
        super().__init__('nxt_control_node')
        

        self.ports = {
            "joint1": nxt.motor.Port(0),
            "joint2": nxt.motor.Port(1),
            "joint3": nxt.motor.Port(2),
        }

        self.brick = nxt.locator.find()
        self.motcont = nxt.motcont.MotCont(self.brick)
        self.motcont.start()

        self.motor_subscriber = self.create_subscription(
            NXTCommand,
            'nxt_motor',
            self.motor_callback,
            100)

    def motor_callback(self, msg):
        try:
            self.motcont.cmd(
                self.ports[msg.joint],
                msg.power,
                msg.position,
                brake=True
            )
        except KeyError as e:
            self.get_logger().warn(f"Invalid joint \"{msg.joint}\"")
        except USBError as e:
            self.get_logger().warn(f"Could not communicate with the NXT brick")
    

def main(args=None):
    ros.init(args=args)
    node = NXTNode()
    ros.spin(node)


if __name__ == '__main__':
    main()
