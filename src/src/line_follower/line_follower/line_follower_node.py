#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.ir_sub = self.create_subscription(Float32MultiArray, '/line_bot/ir_data', self.ir_callback, 10)
        self.prox_sub = self.create_subscription(Bool, '/line_bot/prox', self.prox_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/line_bot/cmd_vel', 10)

        self.Kp = 0.8
        self.Ki = 0.0
        self.Kd = 0.05
        self.integral = 0.0
        self.prev_error = 0.0
        self.max_linear = 0.25
        self.base_linear = 0.15

    def prox_callback(self, msg: Bool):
        if msg.data:
            cmd = Twist()
            self.cmd_pub.publish(cmd)

    def ir_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 5:
            return
        weights = [-2.0, -1.0, 0.0, 1.0, 2.0]
        weighted_sum = 0.0
        sum_val = 0.0
        for i, v in enumerate(msg.data):
            val = 1.0 - float(v)
            weighted_sum += weights[i] * val
            sum_val += val
        if sum_val < 1e-6:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 * (1 if self.prev_error >= 0 else -1)
            self.cmd_pub.publish(cmd)
            return

        error = weighted_sum / (sum_val + 1e-6)
        dt = 0.05
        self.integral += error * dt
        derivative = (error - self.prev_error) / (dt if dt > 0 else 1e-6)
        self.prev_error = error

        correction = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        cmd = Twist()
        cmd.linear.x = max(0.0, min(self.max_linear, self.base_linear))
        cmd.angular.z = -correction
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
