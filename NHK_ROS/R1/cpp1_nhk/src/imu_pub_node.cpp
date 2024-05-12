/*
キャリブレーションの終了判定を追加すること。
うまく動作したら,imu_setサービスは消す
*/
// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include <math.h>
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using Quaternion = geometry_msgs::msg::Quaternion;
using SetBool = std_srvs::srv::SetBool;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Float32 = std_msgs::msg::Float32;
using Bool = std_msgs::msg::Bool;
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
const bool PUB_POSESTAMPED = true; // デバッグ用設定
const float SLOPE_THRESHOLD = 20.0; // 傾斜を検出するための閾値
const int SLOPE_COUNT = 20; // 何回分の結果をプールして出力をならすか。
const float EPSILON_W = 0.0002;
const float EPSILON_V = 0.0002;
const int CALIB_COUNT =100;
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class IMUPubNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<Float32>::SharedPtr robot_yaw_pub;
    rclcpp::Publisher<PoseStamped>::SharedPtr robot_pose_pub;
    rclcpp::Publisher<Bool>::SharedPtr is_slope_pub;
    rclcpp::Subscription<Quaternion>::SharedPtr mros_output_imu_sub;
    rclcpp::Service<SetBool>::SharedPtr imu_set_srv;
    tf2::Quaternion init_imu_data, imu_data;
    std::string color = "";
    bool init_flag = false;
    bool auto_init_flag = false;
    float prior_theta = 0.0;
    bool prior_slope_flag = false;
    tf2::Quaternion prior_imu_diff;
    int count = 0; // 何回連続で傾斜を検出したか
    int count_for_calib = 0;

    public:
    IMUPubNode() : Node("imu_pub_node"){
        RCLCPP_INFO(this->get_logger(), "imu_pub_node is activated");
        // パラメータの宣言と取得
        auto parameter = this->declare_parameter<std::string>("color", "blue");
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }
        auto timer_callback = [this]() -> void{
            tf2::Quaternion imu_diff;
            // init_imu_dataが未設定の場合はimu_diffにimu_dataを代入
            if(!init_flag and !auto_init_flag){
                imu_diff = imu_data;
                if(PUB_POSESTAMPED){
                    PoseStamped pose;
                    pose.header.stamp = this->now();
                    pose.header.frame_id = "map";
                    pose.pose.orientation = tf2::toMsg(imu_diff);
                    robot_pose_pub->publish(pose);
                }

                tf2::Quaternion imu_d = imu_diff * prior_imu_diff.inverse();
                prior_imu_diff = imu_diff;
                if(abs(imu_d.w() - 1.0) < EPSILON_W and abs(imu_d.x()) < EPSILON_V and abs(imu_d.y()) < EPSILON_V and abs(imu_d.z()) < EPSILON_V){
                    count_for_calib++;
                }else{
                    count_for_calib = 0;
                }
                if(count_for_calib > CALIB_COUNT){
                    auto_init_flag = true;
                    init_flag = true;
                    RCLCPP_INFO(this->get_logger(), "キャリブレーションが終了しました");
                    init_imu_data = imu_data;
                    std::cout << "IMUの初期値を設定します" << std::endl;
                }

                return;
            }else{
                imu_diff = imu_data * init_imu_data.inverse(); // クォタニオンの差分を計算
            }

            // クォタニオンをオイラー角に変換
            // std::cout <<  << imu_diff.x() << " " << imu_diff.y() << " " << imu_diff.z() << " " << imu_diff.w() << std::endl;
            Float32 yaw;
            Bool is_slope;
            yaw.data = atan2(2.0 * (imu_diff.x() * imu_diff.y() + imu_diff.z() * imu_diff.w()), 1.0 - 2.0 * (imu_diff.y() * imu_diff.y() + imu_diff.z() * imu_diff.z()));
            float roll = asin(2.0 * (imu_diff.x() * imu_diff.z() - imu_diff.y() * imu_diff.w()));
            float pitch = asin(2.0 * (imu_diff.x() * imu_diff.w() + imu_diff.y() * imu_diff.z()));
            float theta = pow((pitch*pitch + roll*roll)/0.001, 3);
            // 急激な値の変動を抑制する
            double error = theta - prior_theta;
            if (error > 0){
                prior_theta += std::min(error, 0.2);
            }else{
                prior_theta += std::max(error, -5.0);
            }
            // std::cout << "傾斜度" << prior_theta << std::endl;
            if(prior_theta > SLOPE_THRESHOLD){
                count++;
            }else{
                count = 0;
            }

            if(count > SLOPE_COUNT){
                is_slope.data = true;
            }else{
                is_slope.data = false;
            }
            robot_yaw_pub->publish(yaw);
            is_slope_pub->publish(is_slope);
            if(PUB_POSESTAMPED){
                PoseStamped pose;
                pose.header.stamp = this->now();
                pose.header.frame_id = "map";
                pose.pose.orientation = tf2::toMsg(imu_diff);
                robot_pose_pub->publish(pose);
            }
        };
        
        auto mros_output_imu_callback = [this](const Quaternion::SharedPtr msg) -> void{
            // imu_dataを更新
            // std::cout << "callback" << msg->x << " " << msg->y << " " << msg->z << " " << msg->w << std::endl;
            imu_data.setW(msg->w);
            imu_data.setX(msg->x);
            imu_data.setY(msg->y);
            imu_data.setZ(msg->z);
        };

        auto imu_set_callback = [this](const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response) -> void{
            // if(!init_flag){
                init_flag = true;
                std::cout << "IMUの初期値を設定します" << std::endl;
                // init_imu_dataにimu_dataをコピー
                init_imu_data = imu_data;
            // }else if(auto_init_flag){
            //     std::cout << "既にIMUの初期値が自動で設定されています" << std::endl;
            // }else{
            //     std::cout << "既にIMUの初期値が設定されています" << std::endl;
            // }
            response->success = true;
        };

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        robot_yaw_pub = this->create_publisher<Float32>("robot_yaw", qos);
        mros_output_imu_sub = this->create_subscription<Quaternion>("mros_output_imu", qos, mros_output_imu_callback);
        robot_pose_pub = this->create_publisher<PoseStamped>("robot_pose", qos);
        is_slope_pub = this->create_publisher<Bool>("is_slope", qos);
        imu_set_srv = create_service<SetBool>("imu_set", imu_set_callback);
        timer = this->create_wall_timer(10ms, timer_callback);
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<IMUPubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}