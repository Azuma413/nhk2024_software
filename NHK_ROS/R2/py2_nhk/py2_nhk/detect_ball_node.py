# ライブラリのインポート
# *************************************************************************************************
# ROS関連のライブラリをインポート
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from geometry_msgs.msg import Point
from actuator_custom_msgs.srv import GetSiloState
from ament_index_python.packages import get_package_share_directory

# 画像処理関連のモジュールのインポート
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
from ultralytics import YOLO

# その他のライブラリをインポート
import numpy as np
import torch
from collections import defaultdict

use_gpu = torch.cuda.is_available()
device = torch.device('cuda') if use_gpu else torch.device('cpu')
print("torch: ", device, flush=True)
# *************************************************************************************************
# 定数の定義
# *************************************************************************************************
COMMON_PIXEL = (1280, 720) # (width ,height) color_frameとdepth_frameで共通の画像サイズとする.
SAMPLING_TIME = 30 # Realsenseの画像を取得する周期(ms)
WEIGHTS_PATH = get_package_share_directory("py2_nhk") + "/best_v4.pt" # 重みのパス
CONFIDENCE = 0.7 # YOLOの閾値
PUB_IMG = True # YOLOで処理した後の画像をpublishするかどうか.
DEPTH_LIMIT = 4.0 # サイロ検出における深度の閾値
SEPARATE_VALUE = [0, 256, 512, 768, 1024, 1280] # サイロの領域を分割する値 要調整
# *************************************************************************************************
# クラスの定義
# *************************************************************************************************
class DetectBallNode(Node):

    def __init__(self):
        super().__init__('detect_ball_node')
        # パラメータの宣言と取得
        self.declare_parameter("color", "blue")
        self.color = self.get_parameter('color').get_parameter_value().string_value # blue:右側 red:左側
        # ROSの設定
        self.image_pub = self.create_publisher(Image, 'detect_ball_image', 100)
        self.ball_pos_pub = self.create_publisher(Point, 'detect_ball_pos', 10)
        self.create_service(SetBool, 'start_detect_ball', self.start_detect_ball_callback)
        self.create_service(GetSiloState, 'get_silo_state', self.get_silo_state_callback)
        self.bridge = CvBridge() # OpenCVとROSの画像を変換するためのクラス
        # RealSenseの設定
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, COMMON_PIXEL[0], COMMON_PIXEL[1], rs.format.bgr8, SAMPLING_TIME)
        cfg.enable_stream(rs.stream.depth, COMMON_PIXEL[0], COMMON_PIXEL[1], rs.format.z16, SAMPLING_TIME)
        self.rspipe = rs.pipeline()
        self.rspipe.start(cfg) # RealSenseの画像を取得するためのパイプラインを開始
        # YOLOの設定
        self.model = YOLO(WEIGHTS_PATH)
        self.track_history = defaultdict(lambda: [])
        self.trace_num = None
        # other
        self.start_detect_flag = False
        self.create_timer(0.1, self.timer_callback)
        print("セットアップが完了しました", flush=True)

    def timer_callback(self):
        if self.start_detect_flag:
            ball_pos = Point()
            # RealSenseの画像を取得
            frames = self.rspipe.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            color_image = np.asanyarray(color_frame.get_data()) # color_frameをnumpy配列に変換
            # YOLOで物体検出
            results_ = self.model.track(color_image, persist=True, device=device, conf=CONFIDENCE)[0] # YOLOで物体検出 list object
            if use_gpu:
                results_ = results_.cpu()
            if PUB_IMG: # YOLOで処理した後の画像をpublishする場合
                cv_result = cv2.cvtColor(results_.plot(), cv2.COLOR_RGB2BGR)
            class_list = results_.names # 検出可能なクラス名のリスト
            results = results_.boxes # 検出された物体の情報
            rects = np.array(results.xyxy) # バウンディングボックスの座標
            if len(rects) > 0:
                if self.trace_num is not None:
                    # print(results.id)
                    index_arr = np.where(results.id == self.trace_num)[0]
                    if len(index_arr) == 0:
                        # print("reset trace_num")
                        self.trace_num = None
                        if PUB_IMG:
                            cv2.putText(cv_result, "LOST", (int(cv_result.shape[0]/2), int(cv_result.shape[1]/2)), cv2.FONT_HERSHEY_DUPLEX, 3.0, (225, 0, 0))
                    else:
                        index = index_arr[0]
                        rect = rects[index]
                        marker = (rect[0:2] + rect[2:4])*0.5
                        depth = depth_frame.get_distance(int(marker[0]), int(marker[1]))
                        ball_pos.x = float(marker[0]) # u
                        ball_pos.y = float(marker[1]) # v
                        ball_pos.z = depth # depth
                        if PUB_IMG:
                            track = self.track_history[index]
                            track.append(marker)
                            if len(track) > 20:
                                track.pop(0)
                            points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
                            cv2.polylines(cv_result, [points], isClosed=False, color=(225, 225, 0), thickness=2)
                            cv2.putText(cv_result, format(depth, ".2f"), (int(marker[0]), int(marker[1])), cv2.FONT_HERSHEY_DUPLEX, 3.0, (0, 255, 0))
                if self.trace_num is None:
                    min_depth = float('inf')
                    for i, rect in enumerate(rects):
                        label = class_list[int(results.cls[i])]
                        marker = np.array([0, 0])
                        if label == self.color: # パラメータで指定した色の物体の場合
                            marker = (rect[0:2] + rect[2:4])*0.5
                            depth = depth_frame.get_distance(int(marker[0]), int(marker[1]))
                            if depth < min_depth: # 最も近い物体をトレース
                                ball_pos.x = float(marker[0]) # u
                                ball_pos.y = float(marker[1]) # v
                                ball_pos.z = depth # depth
                                if results.id is not None:
                                    if results.id[i] is not None:
                                        self.trace_num = results.id[i]
                                min_depth = depth
                            if PUB_IMG:
                                cv2.putText(cv_result, format(depth, ".2f"), (int(marker[0]), int(marker[1])), cv2.FONT_HERSHEY_DUPLEX, 3.0, (0, 255, 0))
                    # print("set trace_num:", self.trace_num)
            if PUB_IMG:
                # resized_img = cv2.resize(cv_result, (640, 360))
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_result, "rgb8"))
            self.ball_pos_pub.publish(ball_pos)

    def start_detect_ball_callback(self, request, response):
        response = SetBool.Response()
        self.start_detect_flag = request.data
        response.success = True
        return response
    
    def get_silo_state_callback(self, request, response):
        response = GetSiloState.Response()
        silo_ball_color = [[], [], [], [], []] # それぞれの領域で検出されたボールの色を格納する配列
        silo_ball_pos = [[], [], [], [], []] # それぞれの領域で検出されたボールの位置を格納する配列
        # RealSenseの画像を取得
        frames = self.rspipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        color_image = np.asanyarray(color_frame.get_data()) # color_frameをnumpy配列に変換
        # YOLOで物体検出
        results_ = self.model.predict(color_image, device=device, conf=CONFIDENCE)[0]
        if use_gpu:
                    results_ = results_.cpu()
        if PUB_IMG: # YOLOで処理した後の画像をpublishする場合
            cv_result = cv2.cvtColor(results_.plot(), cv2.COLOR_RGB2BGR)
        class_list = results_.names
        result = results_.boxes
        rects = np.array(result.xyxy)
        for i, rect in enumerate(rects):
            label = class_list[int(result.cls[i])]
            if label == "blue" or label == "red": # 検出されたボールが青色または赤色の場合
                marker = (rect[0:2] + rect[2:4]) / 2
                depth = depth_frame.get_distance(marker[0], marker[1])
                if PUB_IMG:
                    cv2.drawMarker(cv_result, (int(marker[0]), int(marker[1])), (255, 255, 0), markerType=cv2.MARKER_CROSS, thickness=5)
                    cv2.putText(cv_result, format(depth, ".2f"), (int(marker[0]), int(marker[1])), cv2.FONT_HERSHEY_DUPLEX, 1.0, (255, 255, 255))
                if depth < DEPTH_LIMIT: # 検出されたボールが深度の閾値以下の場合
                    # SEPARATE_VALUEで指定された領域のどこに位置するかを判定
                    for j in range(5):
                        if PUB_IMG and j != 0:
                            # 領域を示す線を描画
                            cv2.line(cv_result, (SEPARATE_VALUE[j], 0), (SEPARATE_VALUE[j], COMMON_PIXEL[1]), (0, 255, 0), 2)
                        if SEPARATE_VALUE[j] <= marker[0] < SEPARATE_VALUE[j+1]: # j番目の領域に位置する場合
                            if len(silo_ball_pos[j]) != 3: # j番目の領域に3個未満のボールが検出された場合
                                silo_ball_color[j].append(label)
                                silo_ball_pos[j].append(marker[1]) # marker[1]は画像のy座標
                                if not PUB_IMG:
                                    break

        for i in range(5):
            if len(silo_ball_pos[i]) == 3: # 3つのボールが検出されているサイロ
                if i == 0:
                    response.silo1 = -1
                elif i == 1:
                    response.silo2 = -1
                elif i == 2:
                    response.silo3 = -1
                elif i == 3:
                    response.silo4 = -1
                elif i == 4:
                    response.silo5 = -1
            if len(silo_ball_pos[i]) == 0: # ボールが検出されていないサイロ
                if i == 0:
                    response.silo1 = 0
                elif i == 1:
                    response.silo2 = 0
                elif i == 2:
                    response.silo3 = 0
                elif i == 3:
                    response.silo4 = 0
                elif i == 4:
                    response.silo5 = 0
            if len(silo_ball_pos[i]) == 1: # 1つのボールが検出されているサイロ
                if silo_ball_color[i][0] == self.color: # 自分のチームのボールを検出した場合
                    if i == 0:
                        response.silo1 = 1
                    elif i == 1:
                        response.silo2 = 1
                    elif i == 2:
                        response.silo3 = 1
                    elif i == 3:
                        response.silo4 = 1
                    elif i == 4:
                        response.silo5 = 1
                else: # 相手チームのボールを検出した場合
                    if i == 0:
                        response.silo1 = 2
                    elif i == 1:
                        response.silo2 = 2
                    elif i == 2:
                        response.silo3 = 2
                    elif i == 3:
                        response.silo4 = 2
                    elif i == 4:
                        response.silo5 = 2
            if len(silo_ball_pos[i]) == 2: # 2つのボールが検出されているサイロ
                if silo_ball_pos[i][0] < silo_ball_pos[i][1]:
                    if silo_ball_color[i][0] == self.color: # 上側が自分のチームのボールの場合
                        if i == 0:
                            response.silo1 = 3
                        elif i == 1:
                            response.silo2 = 3
                        elif i == 2:
                            response.silo3 = 3
                        elif i == 3:
                            response.silo4 = 3
                        elif i == 4:
                            response.silo5 = 3
                    else: # 上側が相手チームのボールの場合
                        if i == 0:
                            response.silo1 = 4
                        elif i == 1:
                            response.silo2 = 4
                        elif i == 2:
                            response.silo3 = 4
                        elif i == 3:
                            response.silo4 = 4
                        elif i == 4:
                            response.silo5 = 4
                else:
                    if silo_ball_color[i][0] == self.color: # 上側が自分のチームのボールの場合
                        if i == 0:
                            response.silo1 = 3
                        elif i == 1:
                            response.silo2 = 3
                        elif i == 2:
                            response.silo3 = 3
                        elif i == 3:
                            response.silo4 = 3
                        elif i == 4:
                            response.silo5 = 3
                    else: # 上側が相手チームのボールの場合
                        if i == 0:
                            response.silo1 = 4
                        elif i == 1:
                            response.silo2 = 4
                        elif i == 2:
                            response.silo3 = 4
                        elif i == 3:
                            response.silo4 = 4
                        elif i == 4:
                            response.silo5 = 4
        if PUB_IMG:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_result, "rgb8"))
        return response
        
# *************************************************************************************************
# メイン関数
# *************************************************************************************************
def main(args=None):
    rclpy.init(args=args)
    node = DetectBallNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()