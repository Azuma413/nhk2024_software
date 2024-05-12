// こちらはコントローラーで足回りを動かすデモプログラムであり、実際のロボットには関係ない

// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
using Twist = geometry_msgs::msg::Twist;
using Joy = sensor_msgs::msg::Joy;

// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
const float VELOCITY = 0.5;
const float ANG_VEL = 0.7;
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class JoyControlNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<Joy>::SharedPtr joy_sub;
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
    Twist target_vel = Twist();
    std::string color = "";

    public:
    JoyControlNode() : Node("joy_control_node"){
        RCLCPP_INFO(this->get_logger(), "joy_control_node is activated");

        auto timer_callback = [this]() -> void{
            cmd_vel_pub->publish(target_vel);
        };

        auto joy_callback = [this](const Joy::SharedPtr msg)->void{
            // 目標速度を取得
            target_vel.linear.x = msg->axes[5] * VELOCITY;
            target_vel.linear.y = msg->axes[4] * VELOCITY;
            if (msg->buttons[4] == 1){
                target_vel.angular.z = ANG_VEL;
            }else if(msg->buttons[5] == 1){
                target_vel.angular.z = -ANG_VEL;
            }else{
                target_vel.angular.z = 0.0;
            }
        };
                
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        cmd_vel_pub = this->create_publisher<Twist>("cmd_vel_r2", qos);
        joy_sub = this->create_subscription<Joy>("joy", qos, joy_callback);
        timer = this->create_wall_timer(10ms, timer_callback);
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<JoyControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}