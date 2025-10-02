#!/usr/bin/env python3

"""
Fake IMU Publisher for Gazebo Simulation
IMU 센서 데이터가 Gazebo에서 제대로 발행되지 않을 때 사용하는 대체 노드
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
import math

class FakeIMUPublisher(Node):
    def __init__(self):
        super().__init__('fake_imu_publisher')
        
        self.publisher = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(0.01, self.publish_imu)  # 100 Hz
        
        self.get_logger().info('Fake IMU Publisher 시작됨 - /imu 토픽으로 데이터 발행 중')
        
    def publish_imu(self):
        msg = Imu()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Orientation (quaternion) - 정지 상태, z-up
        msg.orientation = Quaternion()
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        
        # Orientation covariance
        msg.orientation_covariance = [
            0.0001, 0.0, 0.0,
            0.0, 0.0001, 0.0,
            0.0, 0.0, 0.0001
        ]
        
        # Angular velocity (rad/s) - 회전하지 않음
        msg.angular_velocity = Vector3()
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        
        # Angular velocity covariance
        msg.angular_velocity_covariance = [
            0.0001, 0.0, 0.0,
            0.0, 0.0001, 0.0,
            0.0, 0.0, 0.0001
        ]
        
        # Linear acceleration (m/s^2) - 중력만 작용
        msg.linear_acceleration = Vector3()
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81  # 중력 가속도
        
        # Linear acceleration covariance
        msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeIMUPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
