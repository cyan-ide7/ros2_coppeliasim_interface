#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class ArenaManagerNode(Node):
    def __init__(self):
        super().__init__('arena_manager_node')
        self.gate1_sub = self.create_subscription(Bool, '/arena/gate1', self.gate1_cb, 10)
        self.gate2_sub = self.create_subscription(Bool, '/arena/gate2', self.gate2_cb, 10)
        self.gate3_sub = self.create_subscription(Bool, '/arena/gate3', self.gate3_cb, 10)

        self.pump_pub = self.create_publisher(Bool, '/arena/pump_cmd', 10)
        self.lcd_pub = self.create_publisher(String, '/arena/lcd_text', 10)

        self.pump_active = False
        self.pump_duration = 10.0
        self.pump_start_time = None

        self.create_timer(0.05, self.timer_cb)

    def gate1_cb(self, msg: Bool):
        if msg.data and not self.pump_active:
            self.start_pump()

    def gate2_cb(self, msg: Bool):
        if msg.data:
            pass

    def gate3_cb(self, msg: Bool):
        if msg.data:
            pass

    def start_pump(self):
        self.pump_active = True
        self.pump_start_time = self.get_clock().now()
        b = Bool()
        b.data = True
        self.pump_pub.publish(b)

    def stop_pump(self):
        self.pump_active = False
        b = Bool()
        b.data = False
        self.pump_pub.publish(b)
        txt = String()
        txt.data = 'Check weight' 
        self.lcd_pub.publish(txt)

    def timer_cb(self):
        if self.pump_active and self.pump_start_time is not None:
            elapsed = (self.get_clock().now() - self.pump_start_time).nanoseconds / 1e9
            if elapsed >= self.pump_duration:
                self.stop_pump()

def main(args=None):
    rclpy.init(args=args)
    node = ArenaManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
