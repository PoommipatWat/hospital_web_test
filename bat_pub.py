import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import random
import time

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')

        self.i = 0.0
        # สร้าง publisher สำหรับ topic 'battery_state'
        self.publisher_ = self.create_publisher(BatteryState, 'battery/state', 10)
        # ตั้ง timer ให้ publish ข้อมูลทุก 1 วินาที
        timer_period = 0.1  # วินาที
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Battery Publisher Node has been started')

    def timer_callback(self):
        # สร้าง message object
        msg = BatteryState()
        
        # จำลองค่าระดับแบตเตอรี่ (0-100%)
        msg.percentage = self.i
        self.i += 1
        if self.i > 100:
            self.i = 0.0
        msg.voltage = random.uniform(11.0, 12.6)  # จำลองโวลต์สำหรับแบตเตอรี่ LiPo
        msg.current = random.uniform(0.0, 1.0)  # จำลองกระแสไฟฟ้า (Ampere)
        msg.charge = random.uniform(0.0, 1.0)  # จำลองปริมาณการชาร์จ (Ampere-hour)
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        
        # ตั้งค่า timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Publish message
        self.publisher_.publish(msg)
        
        # Log ข้อมูลที่ publish
        self.get_logger().info(f'Battery Level: {msg.percentage:.1f}% | Voltage: {msg.voltage:.2f}V')

def main(args=None):
    rclpy.init(args=args)
    battery_publisher = BatteryPublisher()
    
    try:
        rclpy.spin(battery_publisher)
    except KeyboardInterrupt:
        pass
    
    # Cleanup
    battery_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()