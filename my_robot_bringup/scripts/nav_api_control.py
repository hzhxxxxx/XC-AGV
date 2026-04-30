#!/usr/bin/env python3
"""
Nav2 API Control Demo - 底盘小车导航控制
"""

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import time


def create_pose_stamped(navigator: BasicNavigator, x, y, yaw):
    """创建位姿消息（yaw为欧拉角弧度）"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    # 欧拉角yaw转四元数
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


def main():
    # --- 初始化
    rclpy.init()
    navigator = BasicNavigator()

    # --- 设置初始位姿 (home_pose: x=2.615, y=-1.323, yaw=1.555)
    # 使用BEST_EFFORT QoS匹配AMCL订阅者
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    initial_pose_pub = navigator.create_publisher(
        PoseWithCovarianceStamped, '/initialpose', qos)

    initial_pose = create_pose_stamped(navigator, 2.615, -1.323, 1.555)
    initial_pose_msg = PoseWithCovarianceStamped()
    initial_pose_msg.header = initial_pose.header
    initial_pose_msg.pose.pose = initial_pose.pose

    time.sleep(0.5)  # 等待发布者建立连接
    # 多次发布确保AMCL收到
    for _ in range(5):
        initial_pose_pub.publish(initial_pose_msg)
        time.sleep(0.2)
    print('已发布初始位姿 (BEST_EFFORT QoS)')

    # --- 等待Nav2激活
    navigator.waitUntilNav2Active()

    # 等待AMCL完成初始定位
    print('等待AMCL初始化定位...')
    time.sleep(2)

    # --- 定义目标点
    # work_pose: x=3.502, y=0.601, yaw≈-0.0673 (从四元数z=-0.0337转换)
    work_pose = create_pose_stamped(navigator, 3.502, 0.601, -0.0673)
    # home_pose
    home_pose = create_pose_stamped(navigator, 2.615, -1.323, 1.555)

    # --- 导航到work_pose
    print('导航到 work_pose...')
    navigator.goToPose(work_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()

    result = navigator.getResult()
    if 'SUCCEEDED' in str(result):
        print('成功到达 work_pose!')
        print('等待10秒...')
        time.sleep(5)
    else:
        print(f'导航失败: {result}')
        rclpy.shutdown()
        return
        
    # --- 导航回home_pose
    print('导航回 home_pose...')
    navigator.goToPose(home_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()

    result = navigator.getResult()
    if 'SUCCEEDED' in str(result):
        print('成功返回 home_pose!')
    else:
        print(f'导航失败: {result}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
