#!/usr/bin/env python3
"""
LaserScan Binning Filter Node
基于laser_filters的LaserScanBinningFilter C++实现
将变化的激光雷达点数固定到指定的bin数量

Author: Based on Anthony Goeckner's C++ implementation
Fixed: 支持非360度扫描范围
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class ScanBinningNode(Node):
    def __init__(self):
        super().__init__('scan_binning_node')

        # 声明参数
        self.declare_parameter('num_bins', 836)
        self.num_bins = self.get_parameter('num_bins').value

        # 订阅和发布
        self.subscription = self.create_subscription(
            LaserScan,
            'scan_raw',
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(LaserScan, 'scan_m10', 10)

        self.get_logger().info(f'Scan Binning Node started with {self.num_bins} bins')
        self.get_logger().info('Subscribing to: scan_raw')
        self.get_logger().info('Publishing to: scan_m10')

    def scan_callback(self, input_scan):
        """
        处理输入scan，应用binning算法
        支持任意角度范围（不仅限于360度）
        """
        # 复制输入数据
        filtered_scan = LaserScan()
        filtered_scan.header = input_scan.header
        filtered_scan.range_min = input_scan.range_min
        filtered_scan.range_max = input_scan.range_max

        # 计算输入scan的角度范围
        input_angle_range = input_scan.angle_max - input_scan.angle_min

        # 设置输出scan的参数
        # 输出的bin数量保持不变，但angle_increment根据实际角度范围计算
        filtered_scan.ranges = [float('nan')] * self.num_bins
        filtered_scan.intensities = [float('nan')] * self.num_bins
        # 关键修复：slam_toolbox期望值公式是 round((max-min)/inc) + 1
        # 所以用 num_bins-1 来计算increment，让slam计算结果等于num_bins
        filtered_scan.angle_increment = input_angle_range / float(self.num_bins - 1)
        filtered_scan.angle_min = input_scan.angle_min
        filtered_scan.angle_max = input_scan.angle_max

        # 时间参数
        filtered_scan.scan_time = input_scan.scan_time
        filtered_scan.time_increment = input_scan.time_increment * len(input_scan.ranges) / self.num_bins

        # 将数据分配到bins中
        for i in range(len(input_scan.ranges)):
            # 计算当前点的角度
            current_angle = input_scan.angle_min + input_scan.angle_increment * i

            # 计算相对于起始角度的偏移
            angle_offset = current_angle - filtered_scan.angle_min

            # 计算bin索引
            bin_index = int(math.floor(angle_offset / filtered_scan.angle_increment))

            # 确保索引在有效范围内
            if 0 <= bin_index < self.num_bins:
                # 复制range和intensity值（只复制有效数据）
                if not math.isinf(input_scan.ranges[i]) and not math.isnan(input_scan.ranges[i]):
                    filtered_scan.ranges[bin_index] = input_scan.ranges[i]
                    if len(input_scan.intensities) > i:
                        filtered_scan.intensities[bin_index] = input_scan.intensities[i]

        # 发布过滤后的scan
        self.publisher.publish(filtered_scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanBinningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
