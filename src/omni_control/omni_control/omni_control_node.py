#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class WASDControl(Node):
    def __init__(self):
        super().__init__("wasd_control")
        self.pubs = [
            self.create_publisher(Float32, '/wheel_fl_vel', 10),
            self.create_publisher(Float32, '/wheel_fr_vel', 10),
            self.create_publisher(Float32, '/wheel_rl_vel', 10),
            self.create_publisher(Float32, '/wheel_rr_vel', 10),
        ]
        self.get_logger().info("WASD control ready. w=forward, s=back, a=left, d=right, x=stop")

    def publish(self, speeds):
        for i in range(4):
            msg = Float32()
            msg.data = speeds[i]
            self.pubs[i].publish(msg)

def main(args=None):
    import sys, termios, tty
    rclpy.init(args=args)
    node = WASDControl()
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while rclpy.ok():
            key = sys.stdin.read(1)
            if key == 'w':  # forward
                speeds = [1.0, 1.0, 1.0, 1.0]
            elif key == 's':  # backward
                speeds = [-1.0, -1.0, -1.0, -1.0]
            elif key == 'a':  # left turn
                speeds = [-1.0, 1.0, -1.0, 1.0]
            elif key == 'd':  # right turn
                speeds = [1.0, -1.0, 1.0, -1.0]
            elif key == 'x':  # stop
                speeds = [0.0, 0.0, 0.0, 0.0]
            else:
                continue
            node.publish(speeds)
        rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
