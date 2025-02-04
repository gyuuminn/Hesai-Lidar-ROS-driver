# lidar_point_server/launch/record_and_save.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    # 1) ros2 bag record 실행
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', '/home/eric/0203_bag/810_bag/bag/no/no_sus/3_bag',
            '/lidar_points'
        ],
        output='screen',
        # 이 프로세스가 종료되면 전체 Launch 종료
        on_exit=Shutdown()
    )

    # 2) CSV 저장 노드
    #    (아래 'executable'은 setup.py에서 console_scripts로 등록된 이름이어야 함)
    save_points_node = Node(
        package='lidar_point_server',
        executable='save_points',  # <-- setup.py > entry_points > console_scripts
        name='lidar_point_saver',
        # 이 노드가 종료되어도 전체 Launch 종료
        on_exit=Shutdown()
    )

    return LaunchDescription([
        bag_record,
        save_points_node
    ])
