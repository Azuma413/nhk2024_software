# ライブラリのインポート
# *************************************************************************************************
# ROS関連のライブラリをインポート
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool
from ament_index_python.packages import get_package_share_directory

# 画像処理関連のモジュールのインポート
from cv_bridge import CvBridge
import cv2
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
WEIGHTS_PATH = get_package_share_directory("py2_nhk") + "/best_v4.pt" # 重みのパス
CONFIDENCE = 0.7 # YOLOの閾値
PUB_IMG = True # YOLOで処理した後の画像をpublishするかどうか.
TARGET_POS = np.array([320.0, 240.0]) # 目標位置の座標 x y
# TARGET_POS = np.array([320.0, 240.0]) # 目標位置の座標 x y
# *************************************************************************************************
# クラスの定義
# *************************************************************************************************
class TraceBallNode(Node):

    def __init__(self):
        super().__init__('trace_ball_node')
        # パラメータの宣言と取得
        self.declare_parameter("color", "blue")
        self.color = self.get_parameter('color').get_parameter_value().string_value # blue:右側 red:左側
        self.ball_err_pub_flag = False
        self.raw_image = None
        self.pub_error = Point()
        self.trace_num = None # トレースする物体の番号
        # ROSの設定
        self.create_subscription(Image, 'image_raw', self.image_raw_callback, 10)
        self.ball_error_pub = self.create_publisher(Point, 'ball_error', 10)
        self.create_service(SetBool, 'control_trace', self.control_trace_callback)
        self.image_pub = self.create_publisher(Image, 'trace_ball_image', 10)
        self.bridge = CvBridge() # OpenCVとROSの画像を変換するためのクラス
        # YOLOの設定
        self.model = YOLO(WEIGHTS_PATH)
        self.track_history = defaultdict(lambda: [])
        self.create_timer(0.1, self.timer_callback)
        print("セットアップが完了しました", flush=True)
        
    def image_raw_callback(self, msg):
        self.raw_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
    def control_trace_callback(self, request, response):
        response = SetBool.Response()
        self.ball_err_pub_flag = request.data
        if not self.ball_err_pub_flag: # flagがFalseの場合
            self.trace_num = None # トレースする物体の番号をリセットする
        response.success = True
        return response
    
    def timer_callback(self):
        cv_image = None
        if PUB_IMG:
            if self.raw_image is not None:
                cv_image = cv2.cvtColor(self.raw_image, cv2.COLOR_RGB2BGR)
            
        if self.ball_err_pub_flag: # flagがTrueの場合
            if self.raw_image is not None: # 画像が取得できた場合
                shape = self.raw_image.shape[:2]
                results_ = self.model.track(self.raw_image, persist=True, device=device, conf=CONFIDENCE)[0] # YOLOで物体検出 list object
                if use_gpu:
                    results_ = results_.cpu()
                if PUB_IMG:
                    cv_image = cv2.cvtColor(results_.plot(), cv2.COLOR_RGB2BGR)
                class_list = results_.names
                results = results_.boxes
                rects = np.array(results.xyxy)
                if len(rects) > 0: # 物体が検出された場合
                    if self.trace_num is not None: # トレースする物体の番号がNoneでない場合
                        index_arr = np.where(results.id == self.trace_num)[0]
                        if len(index_arr) == 0: # トレースしていた物体がロストした場合
                            print("トレースしていた物体をロストしました.", flush=True)
                            self.trace_num = None # トレースする物体の番号をリセットする
                            if PUB_IMG:
                                cv2.putText(cv_image, "LOST", (int(shape[0]/2), int(shape[1]/2)), cv2.FONT_HERSHEY_DUPLEX, 3.0, (225, 0, 0))
                        else: # トレースしていた物体が検出された場合
                            index = index_arr[0]
                            rect = rects[index]
                            marker = (rect[0:2] + rect[2:4])*0.5
                            error = (TARGET_POS - marker) / shape[0] # 標準化誤差 0~1
                            self.pub_error.x = error[0] # TARGET_POSを0として左を正とする
                            self.pub_error.y = error[1] # TARGET_POSを0として上を正とする
                            self.pub_error.z = 1.0 # 検出できた場合は正の値を返す
                            if PUB_IMG: # 軌跡を描画する
                                track = self.track_history[index]
                                track.append(marker)
                                if len(track) > 30:
                                    track.pop(0)
                                points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
                                cv2.polylines(cv_image, [points], isClosed=False, color=(225, 225, 0), thickness=2)
                    if self.trace_num is None: # トレースする物体の番号がNoneの場合
                        range = float('inf')
                        flag = False
                        for i, rect in enumerate(rects): # 検出された物体の数だけループ
                            label = class_list[int(results.cls[i])]
                            if label == "red" or label == "blue": # 赤か青のボールを検出した場合(サイロ対策)
                                flag = True
                                marker = (rect[0:2] + rect[2:4])*0.5
                                error = (TARGET_POS - marker) / shape[0] # 標準化誤差 0~1
                                err_range = np.linalg.norm(error)
                                if err_range < range: # 最も近い物体
                                    range = err_range
                                    if results.id is not None:
                                        if results.id[i] is not None:
                                            self.trace_num = results.id[i]
                                    self.pub_error.x = error[0]
                                    self.pub_error.y = error[1]
                                    self.pub_error.z = 1.0 # 検出できた場合は正の値を返す
                        if not flag: # 赤か青のボールを検出しなかった場合
                            self.pub_error.x = 0.0
                            self.pub_error.y = 0.0
                            self.pub_error.z = -1.0 # 検出できなかった場合は負の値を返す
                else: # 物体が検出されなかった場合
                    self.pub_error.x = 0.0
                    self.pub_error.y = 0.0
                    self.pub_error.z = -1.0 # 検出できなかった場合は負の値を返す
            else: # 画像が取得できなかった場合
                self.pub_error.x = 0.0
                self.pub_error.y = 0.0
                self.pub_error.z = -1.0 # 検出できなかった場合は負の値を返す
            self.ball_error_pub.publish(self.pub_error)
        if PUB_IMG:
            if cv_image is not None:
                cv2.drawMarker(cv_image, (int(TARGET_POS[0]), int(TARGET_POS[1])), (255, 0, 0), markerType=cv2.MARKER_CROSS, thickness=5)
                # resized_img = cv2.resize(cv_image, (640, 360))
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
# *************************************************************************************************
# メイン関数
# *************************************************************************************************
def main(args=None):
    rclpy.init(args=args)
    node = TraceBallNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# *************************************************************************************************
# 参考文献
# https://docs.ultralytics.com/ja/modes/track/