from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
import launch
import yaml
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import numpy as np

color = 'blue'
namespace = "r1"

def generate_launch_description():
    ### left lidar setup ###
    left_config_file_path = os.path.join(
        get_package_share_directory('cpp1_nhk'),
        'config',
        'params_left.yaml'
    )
    with open(left_config_file_path, 'r') as file:
        left_config_params = yaml.safe_load(file)['urg_node2']['ros__parameters']
    left_config_params["angle_min"] = -np.pi/2
    left_config_params["angle_max"] = np.pi/2
    left_lifecycle_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='urg_node1',
        remappings=[('scan', 'scan1')],
        parameters=[left_config_params],
        namespace=namespace,
        output='screen',
    )
    left_urg_node2_node_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=left_lifecycle_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(left_lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition('true'),
    )
    left_urg_node2_node_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=left_lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(left_lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition('true'),
    )

    ### right lidar setup ###
    right_config_file_path = os.path.join(
        get_package_share_directory('cpp1_nhk'),
        'config',
        'params_right.yaml'
    )
    with open(right_config_file_path, 'r') as file:
        right_config_params = yaml.safe_load(file)['urg_node2']['ros__parameters']
    right_config_params["angle_min"] = -np.pi/2
    right_config_params["angle_max"] = np.pi/2
    right_lifecycle_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='urg_node2',
        remappings=[('scan', 'scan2')],
        parameters=[right_config_params],
        namespace=namespace,
        output='screen',
    )
    right_urg_node2_node_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=right_lifecycle_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(right_lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition('true'),
    )
    right_urg_node2_node_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=right_lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(right_lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition('true'),
    )

    return LaunchDescription([
        # Node( # sample node
        #     package='mysample', # パッケージ名
        #     executable='sample', # 実行ファイル名
        #     name='sample1', # これを設定することで同じノードを別の名前で起動できる。設定しなければ，executableと同じ名前になる。
        #     parameters=[{'color': color}], # colorパラメータにredを設定
        #     remappings=[('/cmd_vel', '/cmd_vel1'),('/topic', 'topic1')] # (元のトピック名，リマッピング後のトピック名)の形式でトピック名を上書き
        #     namespace=namespace, # ノードの名前空間
        #     output='log' # log:ログファイルに出力, screen:コンソールに出力, none
        # ),
        # urg_node1
        left_lifecycle_node,
        left_urg_node2_node_configure_event_handler,
        left_urg_node2_node_activate_event_handler,
        # urg_node2
        right_lifecycle_node,
        right_urg_node2_node_configure_event_handler,
        right_urg_node2_node_activate_event_handler,
        # Node( # v4l2_camera_node 動かなくなってしまったのでusb_camで代替
        #     package='v4l2_camera',
        #     executable='v4l2_camera_node',
        #     name='v4l2_camera_node',
        #     namespace=namespace,
        #     output='log',
        #     parameters=[{'video_device': '/dev/video2'}]
        # ),
        Node(
            package='usb_cam', 
            executable='usb_cam_node_exe', 
            output='screen',
            name='usb_cam',
            namespace=namespace,
            parameters=[
                {'video_device': '/dev/video2'},
                {'image_width': 640},
                {'image_height': 480},
                {'auto_white_balance': True},
                {'autoexposure': True},
                {'autofocus': True},
                ],
        ),
        Node( # f7_node
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_f7',
            namespace=namespace,
            output='screen',
            arguments=['serial', '-b', '115200', '--dev', '/dev/F7-R1'],
        ),
        Node( # esp_node
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_esp',
            namespace=namespace,
            output='screen',
            arguments=['serial', '-b', '115200', '--dev', '/dev/ESP-R1'],
        ),
        Node( # id_color_node
            package='py1_nhk',
            executable='id_color_node',
            name='id_color_node',
            namespace=namespace,
            output='screen',
            parameters=[{'color': color}]
        ),
        Node( # calc_pos_node
            package='cpp1_nhk',
            executable='calc_pos_node',
            name='calc_pos_node',
            namespace=namespace,
            output='screen',
            parameters=[{'color': color}, {'robot_num': 1}],
        ),
        Node( # imu_pub_node
            package='cpp1_nhk',
            executable='imu_pub_node',
            name='imu_pub_node',
            namespace=namespace,
            output='screen',
            parameters=[{'color': color}],
        ),
        Node( # calc_distance_node
            package='cpp1_nhk',
            executable='calc_distance_node',
            name='calc_distance_node',
            namespace=namespace,
            output='screen',
            parameters=[{'color': color}],
        ),
        Node( # dynamixel_node
            package='cpp1_nhk',
            executable='dynamixel_node',
            name='dynamixel_node',
            namespace=namespace,
            output='screen',
            parameters=[{'color': color}, {'device': '/dev/U2D2-R1'}, {'baudrate': 115200}, {'id0': 0}, {'id1': 1}]
        ),
        Node( # auto_launch_node
            package='cpp1_nhk',
            executable='auto_launch_node',
            name='auto_launch_node',
            namespace=namespace,
            output='screen',
            parameters=[{'color': color}],
        ),
        Node( # r1_base_node
            package='cpp1_nhk',
            executable='r1_base_node',
            name='r1_base_node',
            namespace=namespace,
            output='screen',
            parameters=[{'color': color}],
        ),
        Node( # move_certain_node
            package='cpp1_nhk',
            executable='move_certain_node',
            name='move_certain_node',
            namespace=namespace,
            output='screen',
            parameters=[{'color': color}]
        ),
        Node( # take_rice_node
            package='cpp1_nhk',
            executable='take_rice_node',
            name='take_rice_node',
            namespace=namespace,
            output='screen',
            parameters=[{'color': color}],
        ),
        Node( # control_node
            package='cpp1_nhk',
            executable='control_node',
            name='control_node',
            namespace=namespace,
            output='screen',
            parameters=[{'color': color}],
        ),
    ])