from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_rgbd': False,
                'subscribe_scan': True,
                'subscribe_odom_info': False,
                'approx_sync': True,
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                # 闭环检测参数
                'Rtabmap/DetectionRate': '1',
                'Rtabmap/LoopThr': '0.11',
                # 2D激光scan matching
                'Reg/Strategy': '1',       # 1=ICP
                'Icp/PointToPlane': 'false',
                'Icp/Iterations': '30',
                'Icp/VoxelSize': '0.0',
                'Icp/Epsilon': '0.001',
                'Icp/MaxTranslation': '0.5',
                'Icp/MaxCorrespondenceDistance': '0.1',
                'Icp/PM': 'false',
            }],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/diff_drive_controller/odom'),
            ],
            arguments=['--delete_db_on_start'],
        ),
    ])
