#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class FourWheelPublisher(Node):
    def __init__(self):
        super().__init__('four_wheel_publisher')
        # Publishers for each wheel
        self.pub_fl = self.create_publisher(Float32, '/wheel_fl_vel', 10)
        self.pub_fr = self.create_publisher(Float32, '/wheel_fr_vel', 10)
        self.pub_rl = self.create_publisher(Float32, '/wheel_rl_vel', 10)
        self.pub_rr = self.create_publisher(Float32, '/wheel_rr_vel', 10)
        self.get_logger().info("Use keys: w/forward, s/back, a/left, d/right, x/stop")

    def send_vel(self, fl, fr, rl, rr):
        self.pub_fl.publish(Float32(data=fl))
        self.pub_fr.publish(Float32(data=fr))
        self.pub_rl.publish(Float32(data=rl))
        self.pub_rr.publish(Float32(data=rr))

    def set_command(self, key):
        if key == 'w':     # forward
            self.send_vel(1.0, 1.0, 1.0, 1.0)
        elif key == 's':   # backward
            self.send_vel(-1.0, -1.0, -1.0, -1.0)
        elif key == 'a':   # turn left
            self.send_vel(-1.0, 1.0, -1.0, 1.0)
        elif key == 'd':   # turn right
            self.send_vel(1.0, -1.0, 1.0, -1.0)
        elif key == 'x':   # stop
            self.send_vel(0.0, 0.0, 0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = FourWheelPublisher()

    import sys, termios, tty
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while rclpy.ok():
            key = sys.stdin.read(1)
            node.set_command(key)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

