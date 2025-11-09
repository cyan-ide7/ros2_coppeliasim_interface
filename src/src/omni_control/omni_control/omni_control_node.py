#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class OmniPublisher(Node):
    def __init__(self):
        super().__init__('omni_publisher')
        self.publisher = self.create_publisher(Twist, '/omni_cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.get_logger().info("Omniwheel controller ready. Use keys: w/s/a/d/q/e/x")

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.wz
        self.publisher.publish(msg)

    def set_command(self, key):
        if key == 'w': self.vx, self.vy, self.wz = 0.5, 0.0, 0.0
        elif key == 's': self.vx, self.vy, self.wz = -0.5, 0.0, 0.0
        elif key == 'a': self.vx, self.vy, self.wz = 0.0, 0.5, 0.0
        elif key == 'd': self.vx, self.vy, self.wz = 0.0, -0.5, 0.0
        elif key == 'q': self.vx, self.vy, self.wz = 0.0, 0.0, 0.5
        elif key == 'e': self.vx, self.vy, self.wz = 0.0, 0.0, -0.5
        elif key == 'x': self.vx, self.vy, self.wz = 0.0, 0.0, 0.0
        self.get_logger().info(f"vx={self.vx:.2f} vy={self.vy:.2f} wz={self.wz:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = OmniPublisher()

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
