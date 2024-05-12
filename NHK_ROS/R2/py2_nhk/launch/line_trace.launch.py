from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

color = 'blue'
namespace = "r2"
# left_lidar_path = get_package_share_directory("py2_nhk") + "/left_lidar_eth.launch.py"
# right_lidar_path = get_package_share_directory("py2_nhk") + "/right_lidar_usb.launch.py"

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='mysample', # パッケージ名
        #     executable='sample', # 実行ファイル名
        #     name='sample1', # これを設定することで同じノードを別の名前で起動できる。設定しなければ，executableと同じ名前になる。
        #     parameters=[{'color': color}], # colorパラメータにredを設定
        #     remappings=[('/cmd_vel', '/cmd_vel1'),('/topic', 'topic1')] # (元のトピック名，リマッピング後のトピック名)の形式でトピック名を上書き
        #     namespace=namespace, # ノードの名前空間
        #     output='log' # log:ログファイルに出力, screen:コンソールに出力, none
        # ),
        # IncludeLaunchDescription(left_lidar_path), # urg_node1
        # IncludeLaunchDescription(right_lidar_path), # urg_node2
        # Node( # v4l2_camera_node
        #     package='v4l2_camera',
        #     executable='v4l2_camera_node',
        #     name='v4l2_camera_node',
        #     namespace=namespace,
        #     output='log',
        #     parameters=[{'video_device': '/dev/video2'}]
        # ),
        Node( # f7_node
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_f7',
            namespace=namespace,
            output='screen',
            arguments=['serial', '-b', '115200', '--dev', '/dev/ttyACM0']
        ),
        Node( # esp_node
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_esp',
            namespace=namespace,
            output='screen',
            arguments=['serial', '-b', '115200', '--dev', '/dev/ttyUSB0']
        ),
        # Node( # detect_ball_node
        #     package='py2_nhk',
        #     executable='detect_ball_node',
        #     name='detect_ball_node',
        #     namespace=namespace,
        #     output='log',
        #     parameters=[{'color': color}]
        # ),
        # Node( # trace_ball_node
        #     package='py2_nhk',
        #     executable='trace_ball_node',
        #     name='trace_ball_node',
        #     namespace=namespace,
        #     output='log',
        #     parameters=[{'color': color}]
        # ),
        # Node( # calc_pos_node
        #     package='cpp1_nhk',
        #     executable='calc_pos_node',
        #     name='calc_pos_node',
        #     namespace=namespace,
        #     output='log',
        #     parameters=[{'color': color}]
        # ),
        Node( # imu_pub_node
            package='cpp1_nhk',
            executable='imu_pub_node',
            name='imu_pub_node',
            namespace=namespace,
            output='log',
            parameters=[{'color': color}],
        ),
        Node( # calc_distance_node
            package='cpp1_nhk',
            executable='calc_distance_node',
            name='calc_distance_node',
            namespace=namespace,
            output='log',
            parameters=[{'color': color}],
        ),
        # Node( # arm_dxl_node
        #     package='cpp2_nhk',
        #     executable='arm_dxl_node',
        #     name='arm_dxl_node',
        #     namespace=namespace,
        #     output='log',
        #     parameters=[{'color': color}, {'device': '/dev/ttyUSB0'}, {'baudrate': 115200}, {'id0': 7}, {'id1': 1}, {'id2': 3}, {'id3': 4}]
        # ),
        # Node( # pan_tilt_node
        #     package='cpp2_nhk',
        #     executable='pan_tilt_node',
        #     name='pan_tilt_node',
        #     namespace=namespace,
        #     output='log',
        #     parameters=[{'color': color}, {'device': '/dev/ttyUSB1'}, {'baudrate': 115200}, {'id0': 1}, {'id1': 5}, {'id2': 6}]
        # ),
        # Node( # arm_control_node
        #     package='cpp2_nhk',
        #     executable='arm_control_node',
        #     name='arm_control_node',
        #     namespace=namespace,
        #     output='log',
        #     parameters=[{'color': color}],
        # ),
        Node( # r2_base_node
            package='cpp2_nhk',
            executable='r2_base_node',
            name='r2_base_node',
            namespace=namespace,
            output='log',
            parameters=[{'color': color}],
        ),
        # Node( # move_target_node
        #     package='cpp2_nhk',
        #     executable='move_target_node',
        #     name='move_target_node',
        #     namespace=namespace,
        #     output='log',
        #     parameters=[{'color': color}]
        # ),
        # Node( # get_ball_node
        #     package='cpp2_nhk',
        #     executable='get_ball_node',
        #     name='get_ball_node',
        #     namespace=namespace,
        #     output='log',
        #     parameters=[{'color': color}],
        # ),
        # Node( # put_ball_node
        #     package='cpp2_nhk',
        #     executable='put_ball_node',
        #     name='put_ball_node',
        #     namespace=namespace,
        #     output='log',
        #     parameters=[{'color': color}],
        # ),
        Node( # line_trace_node
            package='cpp2_nhk',
            executable='line_trace_node',
            name='line_trace_node',
            namespace=namespace,
            output='screen',
            parameters=[{'color': color}],
        ),
        # Node( # select_ball_node
        #     package='cpp2_nhk',
        #     executable='select_ball_node',
        #     name='select_ball_node',
        #     namespace=namespace,
        #     output='log',
        #     parameters=[{'color': color}],
        # ),
        # Node( # bt_node
        #     package='cpp2_nhk',
        #     executable='bt_node',
        #     name='bt_node',
        #     namespace=namespace,
        #     output='log',
        #     parameters=[{'color': color}]
        # ),
    ])