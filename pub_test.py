#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import random

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        self.messages = [
            "ข้อความทดสอบ",
            "สวัสดี จาก ROS2!",
            "ระบบกำลังทำงาน",
            "WebSocket เรียลไทม์",
            "ROS2 กับ Flask",
        ]

    def timer_callback(self):
        msg = String()
        
        # เลือกข้อความแบบสุ่มและใส่เลขนับ
        random_msg = random.choice(self.messages)
        msg.data = f'{random_msg} #{self.count}'
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'กำลังส่ง: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    test_publisher = TestPublisher()
    
    try:
        rclpy.spin(test_publisher)
    except KeyboardInterrupt:
        pass
    
    test_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()