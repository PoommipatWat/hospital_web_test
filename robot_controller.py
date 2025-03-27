import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from sensor_msgs.msg import BatteryState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import time
import json
import numpy as np
import base64
from io import BytesIO
from PIL import Image
import math
import threading
import tf_transformations

class RobotController(Node):
    def __init__(self, socketio_instance):
        super().__init__('robot_controller')

        # QoS Profile for map and battery subscription
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # QoS Profile for TF and battery
        common_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.socketio = socketio_instance
        
        # Publishers
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.twist_msg = Twist()

        self.last_time = time.time()
        
        # Subscribers
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos)
            
        self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            common_qos)
        
        self.create_subscription(
            BatteryState,
            '/battery/state', 
            self.battery_callback,
            common_qos)
            
        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            
        # Store the latest data with thread-safe mechanism
        self.latest_map = None
        self.map_metadata = None
        self.robot_pose = None
        self.battery_state = None
        
        # Thread-safe lock
        self._data_lock = threading.Lock()
        
        # Optimization: Reduce update frequency
        self.visualization_timer = self.create_timer(0.2, self.update_visualization)  # 5Hz
        
        # Counter for reducing WebSocket emissions
        self._update_counter = 0
        self._last_emitted_pose = None
        
        self.get_logger().info("RobotController initialized with optimized updates")

    def publish_velocity(self, linear_x, angular_z):
        """ส่งคำสั่งความเร็วไปยังหุ่นยนต์"""
        self.twist_msg.linear.x = float(linear_x)
        self.twist_msg.angular.z = float(angular_z)
        self.vel_publisher.publish(self.twist_msg)
    
    def send_nav_goal(self, x, y, theta):
        print(f'send nav goal: {x}, {y}, {theta}')
        """ส่งเป้าหมายการนำทางไปยัง Nav2"""
        goal_msg = NavigateToPose.Goal()
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        
        # Convert theta to quaternion
        goal_pose.pose.orientation.z = math.sin(float(theta) / 2.0)
        goal_pose.pose.orientation.w = math.cos(float(theta) / 2.0)
        
        goal_msg.pose = goal_pose
        
        # Also publish to goal_pose topic for visualization
        self.goal_publisher.publish(goal_pose)
        
        # Send to Nav2
        self.nav_client.wait_for_server()
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info('Goal succeeded!')
            self.socketio.emit('nav_status', {'status': 'success'})
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
            self.socketio.emit('nav_status', {'status': 'failed'})

    def battery_callback(self, msg):
        """บันทึกข้อมูลแบตเตอรี่และส่งค่าผ่าน WebSocket"""
        try:
            # ส่งเฉพาะค่าเปอร์เซ็นต์แบตเตอรี่
            battery_percentage = round(msg.percentage, 1)
            
            # ส่งข้อมูลแบตเตอรี่ผ่าน WebSocket
            self.socketio.emit('ros_battery_message', {
                'data': battery_percentage,
                'voltage' : round(msg.voltage, 2),
                'current': round(msg.current, 2),
            })
        except Exception as e:
            self.get_logger().error(f"Error processing battery message: {str(e)}")

    def tf_callback(self, msg):
        """Process TF messages to extract robot pose"""
        try:
            for transform in msg.transforms:
                # หาตำแหน่งหุ่นยนต์จาก odom -> base_footprint
                if (transform.header.frame_id == 'odom' and 
                    transform.child_frame_id == 'base_footprint'):
                    # เก็บข้อมูลตำแหน่งล่าสุด
                    with self._data_lock:
                        self.robot_pose = {
                            'x': transform.transform.translation.x,
                            'y': transform.transform.translation.y,
                            'orientation': transform.transform.rotation
                        }
        except Exception as e:
            self.get_logger().error(f"Error processing TF: {str(e)}")

    def _is_pose_significantly_changed(self, old_pose, new_pose, threshold=0.02):
        """ตรวจสอบว่าตำแหน่งหุ่นยนต์เปลี่ยนแปลงอย่างมีนัยสำคัญ"""
        if old_pose is None:
            return True
        
        # ตรวจสอบระยะทาง
        distance = math.sqrt(
            (new_pose['x'] - old_pose['x'])**2 + 
            (new_pose['y'] - old_pose['y'])**2
        )
        
        # ตรวจสอบมุม
        old_quat = [
            old_pose['orientation'].x, 
            old_pose['orientation'].y, 
            old_pose['orientation'].z, 
            old_pose['orientation'].w
        ]
        new_quat = [
            new_pose['orientation'].x, 
            new_pose['orientation'].y, 
            new_pose['orientation'].z, 
            new_pose['orientation'].w
        ]
        
        # คำนวณความแตกต่างของมุม
        old_euler = tf_transformations.euler_from_quaternion(old_quat)
        new_euler = tf_transformations.euler_from_quaternion(new_quat)
        angle_diff = abs(old_euler[2] - new_euler[2])
        
        return distance > threshold or angle_diff > 0.1

    def update_visualization(self):
        """Optimized visualization update with reduced data and frequency"""
        try:
            # เพิ่ม counter เพื่อลดความถี่การส่งข้อมูล
            self._update_counter += 1
            
            # ส่งข้อมูลทุก 2-3 รอบ
            if self._update_counter % 2 != 0:
                return
            
            visualization_data = {
                "timestamp": self.get_clock().now().to_msg().sec,
                "robot": None,
                "battery": None
            }

            # Robot Position - Extract from stored pose
            robot_pose = self._safe_get_attribute(self, 'robot_pose')
            if robot_pose is not None:
                # ตรวจสอบการเปลี่ยนแปลงของตำแหน่ง
                if (self._last_emitted_pose is None or 
                    self._is_pose_significantly_changed(self._last_emitted_pose, robot_pose)):
                    # แปลงควอเทอร์เนียนเป็นมุม
                    orientation = robot_pose.get('orientation')
                    if orientation:
                        # ใช้ tf_transformations เพื่อแปลงควอเทอร์เนียนเป็นมุม yaw
                        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion([
                            orientation.x, 
                            orientation.y, 
                            orientation.z, 
                            orientation.w
                        ])
                        
                        # แปลงมุม yaw เป็นองศา
                        yaw_degrees = math.degrees(yaw)
                        
                        visualization_data["robot"] = {
                            "x": round(robot_pose.get('x', 0), 2),
                            "y": round(robot_pose.get('y', 0), 2),
                            "theta": round(yaw_degrees, 1)
                        }
                        
                        # อัปเดตตำแหน่งล่าสุดที่ส่ง
                        self._last_emitted_pose = robot_pose
                else:
                    if time.time() - self.last_time > 5:
                        self.last_time = time.time()
                        orientation = robot_pose.get('orientation')
                        if orientation:
                            # ใช้ tf_transformations เพื่อแปลงควอเทอร์เนียนเป็นมุม yaw
                            (roll, pitch, yaw) = tf_transformations.euler_from_quaternion([
                                orientation.x, 
                                orientation.y, 
                                orientation.z, 
                                orientation.w
                            ])
                            
                            # แปลงมุม yaw เป็นองศา
                            yaw_degrees = math.degrees(yaw)
                            
                            visualization_data["robot"] = {
                                "x": round(robot_pose.get('x', 0), 2),
                                "y": round(robot_pose.get('y', 0), 2),
                                "theta": round(yaw_degrees, 1)
                            }
                            
                            # อัปเดตตำแหน่งล่าสุดที่ส่ง
                            self._last_emitted_pose = robot_pose

            # Send data efficiently
            if visualization_data["robot"]:
                self.socketio.emit('robot_visualization', visualization_data)

        except Exception as e:
            self.get_logger().error(f"Visualization update error: {str(e)}")

    def _safe_get_attribute(self, obj, attr, default=None):
        """Safely get attribute with thread safety"""
        try:
            with self._data_lock:
                return getattr(obj, attr, default)
        except Exception:
            return default

    def map_callback(self, msg):
        """บันทึกข้อมูลแผนที่"""
        try:
            # บันทึกข้อมูล metadata
            with self._data_lock:
                self.latest_map = msg
                self.map_metadata = {
                    "width": msg.info.width,
                    "height": msg.info.height,
                    "resolution": msg.info.resolution,
                    "origin_x": msg.info.origin.position.x,
                    "origin_y": msg.info.origin.position.y
                }
            
            # ทดลองแปลงข้อมูล (เพื่อตรวจสอบข้อผิดพลาด)
            grid_data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            self.get_logger().info("Map data converted successfully")
            
        except Exception as e:
            self.get_logger().error(f"Error processing map data: {str(e)}")

    def handle_request_real_map(self):
        """ส่งข้อมูลแผนที่จริงแบบสี่เหลี่ยมจัตุรัส"""
        try:
            latest_map = self._safe_get_attribute(self, 'latest_map')
            if latest_map is None:
                self.get_logger().info("No map data available")
                return
            
            # แปลงข้อมูลแผนที่เป็นอาร์เรย์
            grid_data = np.array(latest_map.data, dtype=np.int8).reshape(
                latest_map.info.height, latest_map.info.width)
            
            # หาขนาดสูงสุดเพื่อสร้างสี่เหลี่ยมจัตุรัส
            max_size = max(latest_map.info.width, latest_map.info.height)
            
            # สร้างภาพสี RGB ขนาดสี่เหลี่ยมจัตุรัส
            square_img = np.zeros((max_size, max_size, 3), dtype=np.uint8)
            square_img.fill(200)  # เติมพื้นหลังด้วยสีเทา (Unknown)

            # คำนวณตำแหน่งสำหรับวางแผนที่เดิมให้อยู่กึ่งกลาง
            offset_x = (max_size - latest_map.info.width) // 2
            offset_y = (max_size - latest_map.info.height) // 2
            
            # วางข้อมูลแผนที่ลงในภาพสี่เหลี่ยมจัตุรัส
            square_img[offset_y:offset_y + latest_map.info.height, 
                    offset_x:offset_x + latest_map.info.width] = np.flipud(
                        np.where(grid_data[..., None] == -1, [200, 200, 200],  # Unknown: light gray
                                np.where(grid_data[..., None] == 0, [255, 255, 255],  # Free: white
                                        [50, 50, 50])))  # Occupied: dark gray
            
            square_img = np.rot90(square_img, k=-3, axes=(1,0))
            square_img = np.flipud(square_img)
            
            # แปลงเป็นภาพ PIL
            pil_img = Image.fromarray(square_img)
            buffer = BytesIO()
            
            # บันทึกภาพด้วยคุณภาพสูง
            pil_img.save(buffer, format="PNG", optimize=True, quality=100)
            img_str = base64.b64encode(buffer.getvalue()).decode('utf-8')
            
            # ส่งข้อมูลแผนที่
            real_map_data = {
                "metadata": {
                    "width": max_size,  # ปรับเป็นขนาดสี่เหลี่ยมจัตุรัส
                    "height": max_size,  # ปรับเป็นขนาดสี่เหลี่ยมจัตุรัส
                    "resolution": latest_map.info.resolution,
                    "origin_x": latest_map.info.origin.position.x - offset_x * latest_map.info.resolution,  # ปรับ origin
                    "origin_y": latest_map.info.origin.position.y - offset_y * latest_map.info.resolution   # ปรับ origin
                },
                "image": img_str
            }
            
            self.get_logger().info(f"Sending square map: {max_size}x{max_size}")
            self.socketio.emit('ros_map', real_map_data)
        
        except Exception as e:
            self.get_logger().error(f"Error sending square map: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def shutdown(self):
        """ปิด node และ cleanup"""
        self.destroy_node()
        rclpy.shutdown()
        self.get_logger().info("RobotController shutdown")

def spin_robot_controller(controller):
    """ฟังก์ชันสำหรับรัน ROS2 spin ใน thread แยก"""
    rclpy.spin(controller)