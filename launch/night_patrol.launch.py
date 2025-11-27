from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    # 1) LIMO base
    # 실제 패키지/launch 파일 경로에 맞게 수정
    limo_base_share = get_package_share_directory('limo_base')
    limo_base_launch = os.path.join(limo_base_share, 'launch', 'limo_base.launch.py')
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(limo_base_launch)
        )
    )

    # 2) LiDAR
    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')
    ydlidar_launch = os.path.join(ydlidar_share, 'launch', 'ydlidar_launch.py')
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ydlidar_launch)
        )
    )

    # 3) Depth + RGB 카메라
    orbbec_share = get_package_share_directory('orbbec_camera')
    orbbec_launch = os.path.join(orbbec_share, 'launch', 'astra.launch.py')
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbbec_launch)
        )
    )

    # 4) SLAM (원하면 밤에도 사용)
    slam_share = get_package_share_directory('slam_toolbox')
    slam_launch = os.path.join(slam_share, 'launch', 'online_async_launch.py')
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch)
        )
    )

    # 5) Nav2 bringup
    nav2_share = get_package_share_directory('nav2_bringup')
    nav2_launch = os.path.join(nav2_share, 'launch', 'bringup_launch.py')
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
        )
    )

    # 6) 우리 밤 모드 전용 노드들
    ld.add_action(
        Node(
            package='limo_night_patrol',
            executable='mode_manager',
            name='mode_manager_node',
            output='screen',
            parameters=[{'mode': 'NIGHT'}]
        )
    )

    ld.add_action(
        Node(
            package='limo_night_patrol',
            executable='camera_capture_server',
            name='camera_capture_server_node',
            output='screen',
            parameters=[{'save_dir': '/home/wego/wego_ws/src/LIMO_Smart_Sentinel/captures'}]
        )
    )

    ld.add_action(
        Node(
            package='limo_night_patrol',
            executable='depth_obstacle_detector',
            name='depth_obstacle_detector_node',
            output='screen',
            parameters=[
                {'obstacle_distance_threshold': 0.8},
                {'obstacle_ratio_threshold': 0.02},
            ]
        )
    )

    ld.add_action(
        Node(
            package='limo_night_patrol',
            executable='night_intruder_detector',
            name='night_intruder_detector_node',
            output='screen',
        )
    )

    ld.add_action(
        Node(
            package='limo_night_patrol',
            executable='event_handler_night',
            name='event_handler_night_node',
            output='screen',
        )
    )

    ld.add_action(
        Node(
            package='limo_night_patrol',
            executable='patrol_manager',
            name='patrol_manager_node',
            output='screen',
            parameters=[{'frame_id': 'map'}]
        )
    )

    return ld
