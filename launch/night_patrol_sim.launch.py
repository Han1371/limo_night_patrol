# limo_night_patrol/launch/night_patrol_sim.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # 1) 모드 매니저: NIGHT
    ld.add_action(
        Node(
            package='limo_night_patrol',
            executable='mode_manager',
            name='mode_manager_node',
            output='screen',
            parameters=[{'mode': 'NIGHT'}]
        )
    )

    # 2) 더미 카메라 (RGB)
    ld.add_action(
        Node(
            package='limo_night_patrol',
            executable='dummy_rgb_pub',
            name='dummy_rgb_publisher_node',
            output='screen',
        )
    )

    # 3) 카메라 캡처 서버
    ld.add_action(
        Node(
            package='limo_night_patrol',
            executable='camera_capture_server',
            name='camera_capture_server_node',
            output='screen',
            parameters=[{'save_dir': '/tmp/limo_captures_sim'}]
        )
    )

    # 4) 밤 이벤트 핸들러
    ld.add_action(
        Node(
            package='limo_night_patrol',
            executable='event_handler_night',
            name='event_handler_night_node',
            output='screen',
        )
    )

    # 5) 순찰 매니저
    ld.add_action(
        Node(
            package='limo_night_patrol',
            executable='patrol_manager',
            name='patrol_manager_node',
            output='screen',
            parameters=[{'frame_id': 'map'}]
        )
    )

    # 6) 더미 Nav2 / safety / LIMO
    ld.add_action(
        Node(
            package='limo_night_patrol',
            executable='dummy_navigator',
            name='dummy_navigator_node',
            output='screen',
        )
    )

    ld.add_action(
        Node(
            package='limo_night_patrol',
            executable='dummy_safety_mux',
            name='dummy_safety_mux_node',
            output='screen',
        )
    )

    ld.add_action(
        Node(
            package='limo_night_patrol',
            executable='dummy_limo_base',
            name='dummy_limo_base_node',
            output='screen',
        )
    )

    return ld
