// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using Point = geometry_msgs::msg::Point;
using Twist = geometry_msgs::msg::Twist;
using SetBool = std_srvs::srv::SetBool;
using Float32 = std_msgs::msg::Float32;

struct Point2D{
    double x;
    double y;
};
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
const float P_ANGLE = 0.5; // 角度制御のPゲイン
const float D_ANGLE = 0.1; // 角度制御のDゲイン
const double MAX_VEL = 1.5; // 最大速度[m/s]
const float MAX_ANG_VEL = M_PI/3;
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class R1BaseNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    // rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<Point>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<Float32>::SharedPtr robot_yaw_sub;
    rclcpp::Subscription<Point>::SharedPtr robot_vel1_sub;
    rclcpp::Subscription<Point>::SharedPtr robot_vel2_sub;
    rclcpp::Service<SetBool>::SharedPtr robot_vel2_on_srv;
    float robot_yaw = 0.0; // ロボットのZ軸回転角　反時計回りを正とする[rad]
    float target_yaw = 0.0; // 目標角度[rad]
    Point2D target_vel = {0.0, 0.0}; // 目標速度[m/s] (左が正，前が正)右が正ではないことに注意
    bool robot_vel2_on = false;
    std::string color = "";
    float error_yaw = 0.0;

    public:
    R1BaseNode() : Node("r1_base_node"){
        RCLCPP_INFO(this->get_logger(), "r1_base_node is activated");
        // パラメータの宣言と取得
        auto parameter = this->declare_parameter<std::string>("color", "blue");
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }

        auto timer_callback = [this]() -> void{
            // robot_yawの値を用いて角度を補正しつつ，実際の速度を計算
            float c = cos(robot_yaw);
            float s = sin(robot_yaw);
            Point msg;
            float post_error = error_yaw;
            error_yaw = target_yaw - robot_yaw;
            if (error_yaw > M_PI){ // error_yawを-π~πの範囲にする
                error_yaw -= 2 * M_PI;
            }else if (error_yaw < -M_PI){
                error_yaw += 2 * M_PI;
            }
            msg.x = std::min(target_vel.x * c - target_vel.y * s, MAX_VEL); // フィールドに対して左を正とする。
            msg.y = std::min(target_vel.x * s + target_vel.y * c, MAX_VEL); // フィールドに対して前を正とする
            msg.z = std::min(P_ANGLE * (error_yaw) + D_ANGLE * (error_yaw - post_error), MAX_ANG_VEL); // PD制御 
            cmd_vel_pub->publish(msg);
        };

        auto robot_vel1_callback = [this](const Point& msg)->void{
            if (!robot_vel2_on){
                // 目標速度を取得
                target_vel.x = msg.x;
                target_vel.y = msg.y;
            }
            target_yaw = msg.z;
        };

        auto robot_vel2_callback = [this](const Point& msg)->void{
            if (robot_vel2_on){
                // 目標速度を取得
                target_vel.x = msg.x;
                target_vel.y = msg.y;
            }
        };

        auto robot_yaw_callback = [this](const Float32& msg)->void{
            // ロボットのZ軸回転角を取得
            robot_yaw = msg.data;
        };

        auto robot_vel2_on_callback = [this](const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response) -> void{
            robot_vel2_on = request->data;
            response->success = true;
        };
                
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        cmd_vel_pub = this->create_publisher<Point>("cmd_vel", qos);
        // cmd_vel_pub = this->create_publisher<Twist>("cmd_vel", qos);
        robot_yaw_sub = this->create_subscription<Float32>("robot_yaw", qos, robot_yaw_callback);
        robot_vel1_sub = this->create_subscription<Point>("robot_vel1", qos, robot_vel1_callback);
        robot_vel2_sub = this->create_subscription<Point>("robot_vel2", qos, robot_vel2_callback);
        robot_vel2_on_srv = this->create_service<SetBool>("robot_vel2_on", robot_vel2_on_callback);
        timer = this->create_wall_timer(10ms, timer_callback);
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<R1BaseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}