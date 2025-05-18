import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # パラメータファイルのパス設定
    config_file_path = os.path.join(
        get_package_share_directory('chassis_driver'),
        'config',
        'params.yaml'
    )
    # メイン実行機ノードの作成
    execute_node = Node(
        package = 'chassis_driver',
        executable = 'executor',
        parameters = [config_file_path],
        output='screen'
    )

    # joyノードの追加
    joy_node = Node(
        package = 'joy',
        executable = 'joy_node',
        parameters = [config_file_path],
        output='screen'
    )
    # teleopノードの追加
    teleop_node = Node(
        package = 'teleop_twist_joy',
        executable = 'teleop_node',
        parameters = [config_file_path],
        output='screen'
    )

    # 起動エンティティクラスの作成
    launch_discription = LaunchDescription()

    # 起動の追加
    launch_discription.add_entity(execute_node)
    launch_discription.add_entity(joy_node)
    launch_discription.add_entity(teleop_node)

    return launch_discription
