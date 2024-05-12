# このファイルについて
このファイルは`line_trace_node.cpp`の内容について説明するとともに，その使用方法，及び今後の改善方針について記すものである。

# line_trace_node.cppについて
### 構造体
`Point2D`：2次元の座標を表す構造体  
- x
- y

`TraceCommand`：ライントレースのコマンドを表す構造体  
- direction：スタート地点から見たエリア3の方向を前とした場合のトレースするラインの方向  0: 直進，1: 右，2: 後退，3: 左
- distance：移動距離[m]
- speed = 2.0：最大速度[m/s]
- use_imu_instead_of_line_sensor = false：ラインセンサーの代わりにIMUを使うかどうか(未実装)

### 定数
`std::vector<TraceCommand> start_command`：スタートからエリア３までのライントレースのコマンド  
`std::vector<TraceCommand> retry_command`：リトライエリアからエリア３までのライントレースのコマンド  
`float DETECT_VEL`：ライン検出モードの時の速度[m/s]  
`float DETECT_RANGE`：ライン検出モードの時にどれだけ探すか[m]。左右or前後にDETECT_RANGE[m]探す  
`float TERMINAL_RANGE`：ライントレース中，目標距離の何メートル手前で動作を変更するか。[m]  
`float TUNE_VEL`：ライントレース中に進行方向に対して垂直方向に動いてラインを探す時の速度[m/s]  
`double MINIMUM_VEL`：最低速度[m/s]  
`float SLOPE_THRESHOLD`：傾斜を検出するための閾値  

### メンバ変数(重要なものだけ)
`std::string color`：パラメータから受け取ったフィールド情報を格納する変数  
`Quaternion line_sensor_data`：サブスクライブしたそのままの値  
`bool is_slope`：傾斜を検出したかどうかのフラグ  
`Point robot_vel_msg`：ロボットの速度を格納するメッセージ。値を入れるとtimer_callback内でpublishされる  

### 関数(重要なものだけ)
`void set_distance(void)`：転がしエンコーダの計測を開始する関数  
`Point2D get_distance(void)`：転がしエンコーダの計測結果を取得する関数  
`void call_line_trace_execute(std::shared_ptr<GoalHandleCLT>& goal_handle)`：CallLineTraceアクションサーバの実行関数。ここに機能を実装している  
`std::vector<TraceCommand> get_commands(int mode)`：ライントレースのコマンドを取得する関数  
`bool search_line(TraceCommand command)`：ラインを見失っている場合，ラインを探す関数  
`void set_speed(TraceCommand command)`：直進時に速度を設定する関数  
`void set_corner_speed(TraceCommand command, TraceCommand next_command)`：曲がる時に速度を設定する関数  
`void tune_vel(TraceCommand command)`：進行方向に対して垂直方向に動いてライン上を維持するための関数  
`bool detect_marker(TraceCommand command)`：進行方向に対して垂直に交わる目印を検出する関数。目印を検出したら進行方向に対して垂直方向の速度を0にする  
`void async_robot_vel3_on(bool send_data, bool& myresult)`：非同期でr2_base_nodeのサービスを呼び出し，このノードからロボットの速度を制御できるようにする関数。send_dataがtureなら制御できるようになる。falseなら制御できなくなる。myresultには非同期でサービスの結果が格納される  
`int main(int argc, char* argv[])`：メイン関数

### 全体設計
ROS関連の処理はすべてコンストラクタに纏めてある。

ライントレースの処理自体は`call_line_trace_execute`関数に実装されている。

今のところライントレースは，`TraceCommand`型のリストを読み込み，現在のコマンドと，次のコマンドを比較しつつ，ひとつづつ処理していくようになっている。

コマンドを処理する部分は大きく分けて以下の三つに分けることができる。
- ロボットがライン上に存在するか判定する。なければ進行方向に対して垂直に移動してラインを探す。
- 次のコマンドが始まる終端付近まで普通にライントレースする
- 終端付近では次のコマンドに応じて処理を変化させる

終端付近の処理は以下の三つに分けることができる。
- 次も同じ方向の場合：減速せずに直進
- 進行方向に対して垂直に曲がる場合：進行方向に対して斜め45度にコースを外れ，次のライン上まで移動する。できれば円弧を描くように滑らかに移動させたいが未実装。
- 次のコマンドが存在しない場合：減速し，目印を見つけ次第停止。

### 実装すべきもの，考慮すべきこと
エリア1とエリア3の地面が黄色い部分ではライントレースをせず，エンコーダの値を用いて移動するようにしなければならない。エンコーダを用いた移動については`MoveCertainNode`に実装してあるので参照するといいかもしれない。

`TraceCommand`に`use_imu_instead_of_line_sensor`を追加したので，これがTrueであれば何らかの特殊な動きをするようにプログラムを変更するといいだろう？

実装すべき処理としては`use_imu_instead_of_line_sensor`がTrueならエンコーダの値を用いて`TraceCommand.direction`の方向へ進み，`is_slope`がTrueになってからFalseに切り替わったタイミングで坂を登り切ったと判断して次のコマンドに移動するようにする(エリア1→エリア2)

エリア2→エリア3の移動に関しては坂が緑色なので，途中まではライントレースが可能であり，上の処理とはまた別の処理を実装すべきだ。
そうなると，`use_imu_instead_of_line_sensor`だけでは処理を指定できないかもしれない。そこらへんは適当に修正すること。

懸念点としては，スタート地点からエンコーダの値のみ（もちろん全体的な話をすればimuの補正はあるが）を用いてエリア２まで移動した場合，ほぼ必ずロボットの位置がライン上からずれるので，坂を上り切ったタイミングでどのようにロボットをライン上に戻していくのかという問題が発生する。

一応現状でもライン上にロボットを戻すような関数は作ってあるが，これはその場左右に動いてラインを探すという，最終手段にも近い挙動を示すのであまりよくない。

もしかすると坂を上り切った時点で自己位置推定を行った方が良いかもしれない……

### 実験を行うときの手順
必要なパッケージなどについては一番上の階層のreadmeに書いてあるが，仮想環境のLinuxではROSとうまく通信できない可能性があるので注意すること。
実験手順についてもそのreadmeに書いてあるが，ここにもう少し詳しく書いておくことにする。

- ライントレース実験手順
1. USBからF7に給電を開始する

2. R2の電源を入れる

3. launchファイルを実行する
```
ros2 launch py2_nhk line_trace.launch.py
```

4. USBからESPに給電を開始する

5. rqtを開く
```
rqt
```

6. rqtからimu_setサービスにTrueを送る(文字では説明が難しい)

7. ターミナルからアクションサーバーを呼び出す
```
ros2 action send_goal /r2/call_line_trace /actuator_custom_msgs/CallLineTrace <yaml形式の値>
```
↑多分こんな感じという説明。実際にやってみないとどういうコマンドかはわからない。

8. ロボットが動き出すはず
