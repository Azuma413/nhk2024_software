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
const float ACCEL_LIMIT = 0.6/100.0; // 1tickあたり加速度制限 [m/s^2]/[Hz]
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class R2BaseNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<Point>::SharedPtr cmd_vel_pub;
    // rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<Float32>::SharedPtr robot_yaw_sub;
    rclcpp::Subscription<Point>::SharedPtr robot_vel1_sub;
    rclcpp::Subscription<Point>::SharedPtr robot_vel2_sub;
    rclcpp::Subscription<Point>::SharedPtr robot_vel3_sub;
    rclcpp::Subscription<Point>::SharedPtr robot_vel4_sub;
    rclcpp::Subscription<Point>::SharedPtr robot_vel5_sub;
    rclcpp::Service<SetBool>::SharedPtr robot_vel1_on_srv;
    rclcpp::Service<SetBool>::SharedPtr robot_vel2_on_srv;
    rclcpp::Service<SetBool>::SharedPtr robot_vel3_on_srv;
    rclcpp::Service<SetBool>::SharedPtr robot_vel4_on_srv;
    rclcpp::Service<SetBool>::SharedPtr robot_vel5_on_srv;
    float robot_yaw = 0.0; // ロボットのZ軸回転角　反時計回りを正とする[rad]
    float target_yaw = 0.0; // 目標角度[rad]
    Point2D target_vel = {0.0, 0.0}; // 目標速度[m/s] (左が正，前が正)右が正ではないことに注意
    Point prior_vel;
    bool robot_vel1_on = false;
    bool robot_vel2_on = false;
    bool robot_vel3_on = false;
    bool robot_vel4_on = false;
    bool robot_vel5_on = false;
    std::string color = "";
float error_yaw = 0.0;

    public:
    R2BaseNode() : Node("r2_base_node"){
        RCLCPP_INFO(this->get_logger(), "r2_base_node is activated");
        // パラメータの宣言と取得
        auto parameter = this->declare_parameter<std::string>("color", "blue");
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }

        auto timer_callback = [this]() -> void{
            Point msg;
            float post_error = error_yaw;
            error_yaw = target_yaw - robot_yaw;
            if (error_yaw > M_PI){ // error_yawを-π~πの範囲にする
                error_yaw -= 2 * M_PI;
            }else if (error_yaw < -M_PI){
                error_yaw += 2 * M_PI;
            }
            if(robot_vel1_on || robot_vel2_on || robot_vel3_on || robot_vel4_on || robot_vel5_on){
                // robot_yawの値を用いて角度を補正しつつ，実際の速度を計算
                float c = cos(robot_yaw);
                float s = sin(robot_yaw);

                msg.x = std::min(target_vel.x * c - target_vel.y * s, MAX_VEL); // フィールドに対して左を正とする。
                msg.y = std::min(target_vel.x * s + target_vel.y * c, MAX_VEL); // フィールドに対して前を正とする
            }else{
                msg.x = 0.0;
                msg.y = 0.0;
            }
            msg.z = std::min(P_ANGLE * (error_yaw) + D_ANGLE * (error_yaw - post_error), MAX_ANG_VEL); // PD制御

            // 減速には制限を設けない
            if(msg.x > 0){
                msg.x = std::min(msg.x, prior_vel.x + ACCEL_LIMIT);
            }else{
                msg.x = std::max(msg.x, prior_vel.x - ACCEL_LIMIT);
            }

            if(msg.y > 0){
                msg.y = std::min(msg.y, prior_vel.y + ACCEL_LIMIT);
            }else{
                msg.y = std::max(msg.y, prior_vel.y - ACCEL_LIMIT);
            }
            prior_vel = msg;
            cmd_vel_pub->publish(msg);
        };

        auto robot_vel1_callback = [this](const Point& msg)->void{
            if (robot_vel1_on){
                // 目標速度を取得
                target_vel.x = msg.x;
                target_vel.y = msg.y;
                target_yaw = msg.z;
            }
        };

        auto robot_vel2_callback = [this](const Point& msg)->void{
            if (robot_vel2_on){
                // 目標速度を取得
                target_vel.x = msg.x;
                target_vel.y = msg.y;
                target_yaw = msg.z;
            }
        };

        auto robot_vel3_callback = [this](const Point& msg)->void{
            if (robot_vel3_on){
                // 目標速度を取得
                target_vel.x = msg.x;
                target_vel.y = msg.y;
                target_yaw = msg.z;
            }
        };

        auto robot_vel4_callback = [this](const Point& msg)->void{
            if (robot_vel4_on){
                // 目標速度を取得
                target_vel.x = msg.x;
                target_vel.y = msg.y;
                target_yaw = msg.z;
            }
        };

        auto robot_vel5_callback = [this](const Point& msg)->void{
            if (robot_vel5_on){
                // 目標速度を取得
                target_vel.x = msg.x;
                target_vel.y = msg.y;
                target_yaw = msg.z;
            }
        };

        auto robot_yaw_callback = [this](const Float32& msg)->void{
            // ロボットのZ軸回転角を取得
            robot_yaw = msg.data;
        };

        auto robot_vel1_on_callback = [this](const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response) -> void{
            std::cout << "robot_vel1_on" << std::endl;
            robot_vel1_on = request->data;
            robot_vel2_on = false;
            robot_vel3_on = false;
            robot_vel4_on = false;
            robot_vel5_on = false;
            target_vel.x = 0.0;
            target_vel.y = 0.0;
            response->success = true;
        };

        auto robot_vel2_on_callback = [this](const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response) -> void{
            std::cout << "robot_vel2_on" << std::endl;
            robot_vel1_on = false;
            robot_vel2_on = request->data;
            robot_vel3_on = false;
            robot_vel4_on = false;
            robot_vel5_on = false;
            target_vel.x = 0.0;
            target_vel.y = 0.0;
            response->success = true;
        };

        auto robot_vel3_on_callback = [this](const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response) -> void{
            std::cout << "robot_vel3_on" << std::endl;
            robot_vel1_on = false;
            robot_vel2_on = false;
            robot_vel3_on = request->data;
            robot_vel4_on = false;
            robot_vel5_on = false;
            target_vel.x = 0.0;
            target_vel.y = 0.0;
            response->success = true;
        };

        auto robot_vel4_on_callback = [this](const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response) -> void{
            std::cout << "robot_vel4_on" << std::endl;
            robot_vel1_on = false;
            robot_vel2_on = false;
            robot_vel3_on = false;
            robot_vel4_on = request->data;
            robot_vel5_on = false;
            target_vel.x = 0.0;
            target_vel.y = 0.0;
            response->success = true;
        };

        auto robot_vel5_on_callback = [this](const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response) -> void{
            std::cout << "robot_vel5_on" << std::endl;
            robot_vel1_on = false;
            robot_vel2_on = false;
            robot_vel3_on = false;
            robot_vel4_on = false;
            robot_vel5_on = request->data;
            target_vel.x = 0.0;
            target_vel.y = 0.0;
            response->success = true;
        };
                
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        cmd_vel_pub = this->create_publisher<Point>("cmd_vel", qos);
        // cmd_vel_pub = this->create_publisher<Twist>("cmd_vel", qos);
        robot_yaw_sub = this->create_subscription<Float32>("robot_yaw", qos, robot_yaw_callback);
        robot_vel1_sub = this->create_subscription<Point>("robot_vel1", qos, robot_vel1_callback);
        robot_vel2_sub = this->create_subscription<Point>("robot_vel2", qos, robot_vel2_callback);
        robot_vel3_sub = this->create_subscription<Point>("robot_vel3", qos, robot_vel3_callback);
        robot_vel4_sub = this->create_subscription<Point>("robot_vel4", qos, robot_vel4_callback);
        robot_vel5_sub = this->create_subscription<Point>("robot_vel5", qos, robot_vel5_callback);
        robot_vel1_on_srv = this->create_service<SetBool>("robot_vel1_on", robot_vel1_on_callback);
        robot_vel2_on_srv = this->create_service<SetBool>("robot_vel2_on", robot_vel2_on_callback);
        robot_vel3_on_srv = this->create_service<SetBool>("robot_vel3_on", robot_vel3_on_callback);
        robot_vel4_on_srv = this->create_service<SetBool>("robot_vel4_on", robot_vel4_on_callback);
        robot_vel5_on_srv = this->create_service<SetBool>("robot_vel5_on", robot_vel5_on_callback);
        timer = this->create_wall_timer(10ms, timer_callback);
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<R2BaseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}