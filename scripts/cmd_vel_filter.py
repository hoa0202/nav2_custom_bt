#!/usr/bin/env python3
"""
cmd_vel 필터 노드
- 각속도(wz) boost: 마찰 극복용
- Rate Limiter: 급격한 변화 방지
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class CmdVelFilter(Node):
    def __init__(self):
        super().__init__('cmd_vel_filter')
        
        # 파라미터 선언
        self.declare_parameter('min_angular_vel', 0.55)  # boost 값 (rad/s)
        self.declare_parameter('max_angular_rate', 0.05)  # 스텝당 최대 변화량 (rad/s)
        self.declare_parameter('input_topic', '/cmd_vel_nav')
        self.declare_parameter('output_topic', '/cmd_vel')
        
        self.min_angular_vel = self.get_parameter('min_angular_vel').value
        self.max_angular_rate = self.get_parameter('max_angular_rate').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        # 이전 출력 각속도 저장 (Rate Limiter용)
        self.prev_angular_z = 0.0
        
        self.get_logger().info(f'min_angular_vel (boost): {self.min_angular_vel}')
        self.get_logger().info(f'max_angular_rate: {self.max_angular_rate}')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing to: {output_topic}')
        
        # Subscriber & Publisher
        self.sub = self.create_subscription(
            Twist,
            input_topic,
            self.cmd_vel_callback,
            10
        )
        self.pub = self.create_publisher(Twist, output_topic, 10)
    
    def apply_rate_limit(self, target: float, current: float) -> float:
        """Rate Limiter: 스텝당 최대 변화량 제한"""
        diff = target - current
        if abs(diff) > self.max_angular_rate:
            return current + math.copysign(self.max_angular_rate, diff)
        return target
    
    def cmd_vel_callback(self, msg: Twist):
        out = Twist()
        out.linear = msg.linear
        out.angular = msg.angular
        
        # 1. 각속도 boost (0이 아닐 때)
        target_wz = msg.angular.z
        if target_wz != 0.0:
            target_wz = msg.angular.z + math.copysign(self.min_angular_vel, msg.angular.z)
        
        # 2. Rate Limiter 적용
        out.angular.z = self.apply_rate_limit(target_wz, self.prev_angular_z)
        self.prev_angular_z = out.angular.z
        
        self.get_logger().debug(
            f'wz: {msg.angular.z:.3f} -> boost: {target_wz:.3f} -> rate_limited: {out.angular.z:.3f}'
        )
        
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelFilter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
