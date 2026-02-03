#!/usr/bin/env python3
"""
cmd_vel 필터 노드
각속도(wz)가 min_angular_vel 미만이면 최소값으로 올려서 마찰을 극복
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class CmdVelFilter(Node):
    def __init__(self):
        super().__init__('cmd_vel_filter')
        
        # 파라미터 선언
        self.declare_parameter('min_angular_vel', 0.5)  # 최소 각속도 (rad/s)
        self.declare_parameter('input_topic', '/cmd_vel_nav')  # Nav2 출력
        self.declare_parameter('output_topic', '/cmd_vel')  # 로봇 입력
        
        self.min_angular_vel = self.get_parameter('min_angular_vel').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        self.get_logger().info(f'min_angular_vel: {self.min_angular_vel}')
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
    
    def cmd_vel_callback(self, msg: Twist):
        out = Twist()
        out.linear = msg.linear
        out.angular = msg.angular
        
        # 각속도가 0이 아니고, 절대값이 min_angular_vel 미만이면 최소값으로 올림
        if msg.angular.z != 0.0 and abs(msg.angular.z) < self.min_angular_vel:
            # 부호 유지하면서 최소값 적용
            out.angular.z = math.copysign(self.min_angular_vel, msg.angular.z)
            self.get_logger().debug(
                f'Angular vel adjusted: {msg.angular.z:.3f} -> {out.angular.z:.3f}'
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
