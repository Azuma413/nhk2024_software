/*
エンコーダから受け取った値をIMUの情報を元にグローバル座標に補正してPublish
*/
// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using Point = geometry_msgs::msg::Point;
using Float32 = std_msgs::msg::Float32;

struct Point2D{
    double x;
    double y;
};
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************

// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class CalcDistanceNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<Point>::SharedPtr distance_pub;
    rclcpp::Subscription<Float32>::SharedPtr robot_yaw_sub;
    rclcpp::Subscription<Point>::SharedPtr mros_output_enc_sub;
    float euler_z = 0.0;
    Point distance;
    Point2D enc_r2 = {0.0, 0.0};
    std::string color = "";

    public:
    CalcDistanceNode() : Node("calc_distance_node"){
        RCLCPP_INFO(this->get_logger(), "calc_distance_node is activated");
        // パラメータの宣言と取得
        auto parameter = this->declare_parameter<std::string>("color", "blue");
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }

        auto timer_callback = [this]() -> void{
            distance_pub->publish(distance);
        };

        auto robot_yaw_callback = [this](const Float32& msg) -> void{
            // yaw角を取得
            euler_z = msg.data;
        };

        auto mros_output_enc_callback = [this](const Point& msg) -> void{
            // エンコーダーの値を取得[m]
            Point2D enc_diff = {msg.x - enc_r2.x, msg.y - enc_r2.y}; // エンコーダー値の前回との差分を取得
            enc_r2.x = msg.x; // ロボットから見て左を正とする軸における移動量
            enc_r2.y = msg.y; // ロボットから見て前を正とする軸における移動量
            double c = cos(euler_z);
            double s = sin(euler_z);
            // mm単位に変換してpublish
            distance.x += (enc_diff.x * c + enc_diff.y * s)*1000.0; // ロボットから見て左を正とする軸における移動量[mm]
            distance.y += (enc_diff.y * c - enc_diff.x * s)*1000.0; // ロボットから見て前を正とする軸における移動量
        };

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        distance_pub = this->create_publisher<Point>("distance", qos);
        robot_yaw_sub = this->create_subscription<Float32>("robot_yaw", qos, robot_yaw_callback);
        mros_output_enc_sub = this->create_subscription<Point>("mros_output_enc", qos, mros_output_enc_callback);
        timer = this->create_wall_timer(10ms, timer_callback);
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<CalcDistanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}