from launch import LaunchDescription
from launch_ros.actions import Node

namespace = "r1"
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
        Node( # joy_linux_node
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            namespace=namespace,
            output='log',
        ),
    ])