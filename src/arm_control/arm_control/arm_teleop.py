#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        self.joint_pubs = [
            self.create_publisher(Float32, f'/arm_joint_{i}_pos', 10) for i in range(1, 6)
        ]
        self.joint_positions = [0.0] * 5
        self.gripper_pub = self.create_publisher(Int32, '/gripper_cmd', 10)
        self.get_logger().info("1-5/Shift+1-5: joints, c/o: gripper, all via keyboard.")

    def set_joint(self, idx, delta):
        self.joint_positions[idx] += delta
        self.joint_pubs[idx].publish(Float32(data=self.joint_positions[idx]))

    def set_gripper(self, close):
        self.gripper_pub.publish(Int32(data=int(close)))
        self.get_logger().info("Gripper " + ("close" if close else "open"))

def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleop()

    import sys, termios, tty
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while rclpy.ok():
            key = sys.stdin.read(1)
            if key in '12345':
                idx = int(key) - 1
                node.set_joint(idx, 0.1)
            elif key in '!@#$%':
                idx = '!@#$%'.index(key)
                node.set_joint(idx, -0.1)
            elif key == 'c':
                node.set_gripper(1)
            elif key == 'o':
                node.set_gripper(0)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
