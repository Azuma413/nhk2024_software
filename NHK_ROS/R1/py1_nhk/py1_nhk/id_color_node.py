# R1のピックアップ部に設置されたカメラの画像からボールの有無，ボールの色を検出するノード
# もっとも値の大きい色を検出するように変更すべき
# ライブラリのインポート
# *************************************************************************************************
# ROS関連のライブラリをインポート
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool
from ament_index_python.packages import get_package_share_directory

# 画像処理関連のモジュールのインポート
from cv_bridge import CvBridge
import cv2

# その他のライブラリをインポート
import numpy as np
from collections import defaultdict

# *************************************************************************************************
# 定数の定義
# *************************************************************************************************
PUB_IMG = True # 動作確認用の画像をpublishするかどうか.
# 色相を0~360の範囲で指定
RED = [320, 20]
PURPLE = [270, 310]
BLUE = [180, 260]
REGION = [[0.3, 0.7], [0.3, 0.7]] # 色を検出する領域の割合[[x_min, x_max], [y_min, y_max]] なお，原点は左上
RED_THRESHOLD = 0.4 # 赤ボールがあると判定する閾値
BLUE_THRESHOLD = 0.4 # 青ボールがあると判定する閾値
PURPLE_THRESHOLD = 0.35 # 紫ボールがあると判定する閾値
ID_MIN_COUNT = 10 # n回以上連続してIDを検出した場合にIDをpublishする

def func(color):
    def is_color(region):
        if color[0] < color[1]:
            return (region >= int(color[0]/360*255)) & (region < int(color[1]/360*255)) # 下限より大きく，上限より小さいときTrue
        else:
            return (region >= int(color[0]/360*255)) | (region < int(color[1]/360*255)) # 上限より大きいか，下限より小さいときTrue
    return is_color

red_func = func(RED)
purple_func = func(PURPLE)
blue_func = func(BLUE)
# *************************************************************************************************
# クラスの定義
# *************************************************************************************************
class IdColorNode(Node):

    def __init__(self):
        super().__init__('id_color_node')
        # パラメータの宣言と取得
        self.declare_parameter("color", "blue")
        self.color = self.get_parameter('color').get_parameter_value().string_value # blue:右側 red:左側
        # 全体用の変数
        self.raw_image = None
        self.id_color_flag = False
        self.piror_color = "none" # 前回検出したボールの色
        self.id_continuous_count = 0 # 何回連続で現在のcolorを検出したかのカウント
        # ROSの設定
        self.create_subscription(Image, 'image_raw', self.image_raw_callback, 10)
        self.ball_color_pub = self.create_publisher(String, 'ball_color', 10)
        self.create_service(SetBool, 'control_id_color', self.control_id_color_callback)
        if PUB_IMG:
            self.image_pub = self.create_publisher(Image, 'id_color_image', 10)
        self.bridge = CvBridge() # OpenCVとROSの画像を変換するためのクラス
        print("start id_color_node")
        self.create_timer(0.1, self.timer_callback)
        
    def image_raw_callback(self, msg):
        self.raw_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # imageをOpenCVの画像に変換して保存
        
    def control_id_color_callback(self, request, response):
        response = SetBool.Response()
        self.id_color_flag = request.data # flagを設定
        response.success = True
        return response
    
    def timer_callback(self):
        if self.id_color_flag: # flagがTrueの場合
            color_msg = String()
            color_msg.data = "none"
            detect_flag = False # 何らかの色を検出したかどうかのフラグ  
            if self.raw_image is not None: # 画像が取得できた場合
                if PUB_IMG:
                    cv_image = cv2.cvtColor(self.raw_image, cv2.COLOR_RGB2BGR)
                # REGIONの領域を取り出す
                x_region_min = int(self.raw_image.shape[0]*REGION[1][0])
                x_region_max = int(self.raw_image.shape[0]*REGION[1][1])
                y_region_min = int(self.raw_image.shape[1]*REGION[0][0])
                y_region_max = int(self.raw_image.shape[1]*REGION[0][1])
                x_range = x_region_max - x_region_min
                y_range = y_region_max - y_region_min
                region_image = self.raw_image[x_region_min:x_region_max, y_region_min:y_region_max]
                hsv_frame = cv2.cvtColor(region_image, cv2.COLOR_BGR2HSV_FULL)
                h_frame = hsv_frame[:, :, 0]
                _, s_frame = cv2.threshold(hsv_frame[:, :, 1], 0, 255, cv2.THRESH_BINARY|cv2.THRESH_OTSU)
                _, v_frame = cv2.threshold(hsv_frame[:, :, 2], 0, 255, cv2.THRESH_BINARY|cv2.THRESH_OTSU)
                h_frame[(s_frame == 0) | (v_frame == 0)] = 0 # 彩度か明度が低い値は無視する
                # 赤判定
                mask = h_frame.copy()
                mask[red_func(mask) == False] = 0 # 条件に合致しない領域を黒にする
                mask[mask != 0] = 1 # 条件に合致する領域を1にする
                red_count = mask.sum() / (x_range * y_range) # 条件に合致する領域の割合を計算
                # print("red:", red_count)
                # 青判定
                mask = h_frame.copy()
                mask[blue_func(mask) == False] = 0 # 条件に合致しない領域を黒にする
                mask[mask != 0] = 1 # 条件に合致する領域を1にする
                blue_count = mask.sum() / (x_range * y_range) # 条件に合致する領域の割合を計算
                # print("blue:", blue_count)
                if red_count > RED_THRESHOLD: # 条件に合致する領域の割合が閾値を超えた場合
                    detect_flag = True
                    if self.piror_color == "red":
                        self.id_continuous_count += 1
                        if self.id_continuous_count > ID_MIN_COUNT:
                            color_msg.data = "red"
                    else:
                        self.id_continuous_count = 0
                        self.piror_color = "red"
                    if PUB_IMG:
                        cv2.putText(cv_image, f"red :{red_count:.2f}", (int(self.raw_image.shape[1]/2) - 200, int(self.raw_image.shape[0]/2)), cv2.FONT_HERSHEY_DUPLEX, 3.0, (225, 0, 0))
                # 青判定
                elif self.color == "blue":
                    mask = h_frame.copy()
                    mask[blue_func(mask) == False] = 0 # 条件に合致しない領域を黒にする
                    mask[mask != 0] = 1 # 条件に合致する領域を1にする
                    blue_count = mask.sum() / (x_range * y_range) # 条件に合致する領域の割合を計算
                    # print("blue:", blue_count)
                    if blue_count > BLUE_THRESHOLD: # 条件に合致する領域の割合が閾値を超えた場合
                        detect_flag = True
                        if self.piror_color == "blue":
                            self.id_continuous_count += 1
                            if self.id_continuous_count > ID_MIN_COUNT:
                                color_msg.data = "blue"
                        else:
                            self.id_continuous_count = 0
                            self.piror_color = "blue"
                        if PUB_IMG:
                            cv2.putText(cv_image, f"blue:{blue_count:.2f}", (int(self.raw_image.shape[1]/2) - 200, int(self.raw_image.shape[0]/2)), cv2.FONT_HERSHEY_DUPLEX, 3.0, (0, 0, 255))
                        
                if not detect_flag: # 赤/青ボールが検出できなかった場合
                    mask = h_frame.copy()
                    mask[purple_func(mask) == False] = 0 # 条件に合致しない領域を黒にする
                    mask[mask != 0] = 1 # 条件に合致する領域を1にする
                    purple_count = mask.sum() / (x_range * y_range) # 条件に合致する領域の割合を計算
                    # print("purple:", purple_count)
                    if purple_count > PURPLE_THRESHOLD: # 条件に合致する領域の割合が閾値を超えた場合
                        detect_flag = True
                        if self.piror_color == "purple":
                            self.id_continuous_count += 1
                            if self.id_continuous_count > ID_MIN_COUNT:
                                color_msg.data = "purple"
                        else:
                            self.id_continuous_count = 0
                            self.piror_color = "purple"
                        if PUB_IMG:
                            cv2.putText(cv_image, f"purp:{purple_count:.2f}", (int(self.raw_image.shape[1]/2) - 200, int(self.raw_image.shape[0]/2)), cv2.FONT_HERSHEY_DUPLEX, 3.0, (255, 0, 255))
                        
                if not detect_flag: # 何も検出できなかった場合
                    if PUB_IMG:
                        cv2.putText(cv_image, "none", (int(self.raw_image.shape[1]/2) - 100, int(self.raw_image.shape[0]/2)), cv2.FONT_HERSHEY_DUPLEX, 3.0, (0, 255, 0))
                
                if PUB_IMG:
                    # REGIONの領域を四角で囲む
                    cv2.rectangle(cv_image, (y_region_min, x_region_min), (y_region_max, x_region_max), (255, 255, 0), 2)
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
            self.ball_color_pub.publish(color_msg) # 検出したボールの色をpublishする
# *************************************************************************************************
# メイン関数
# *************************************************************************************************
def main(args=None):
    rclpy.init(args=args)
    node = IdColorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()