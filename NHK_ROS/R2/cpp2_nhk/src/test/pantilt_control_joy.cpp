// こちらはコントローラーで足回りを動かすデモプログラムであり、実際のロボットには関係ない

// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "actuator_custom_msgs/msg/set_position.hpp"

using namespace std::chrono_literals;
using Joy = sensor_msgs::msg::Joy;
using SetPosition = actuator_custom_msgs::msg::SetPosition;
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
const float VELOCITY = 0.03;
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class JoyControlNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<Joy>::SharedPtr joy_sub;
    rclcpp::Publisher<SetPosition>::SharedPtr set_pos_pub;
    SetPosition target_pos = SetPosition();
    std::string color = "";

    public:
    JoyControlNode() : Node("joy_control_node"){
        RCLCPP_INFO(this->get_logger(), "joy_control_node is activated");

        auto timer_callback = [this]() -> void{
            set_pos_pub->publish(target_pos);
        };

        auto joy_callback = [this](const Joy::SharedPtr msg)->void{
            // 目標速度を取得
            target_pos.position_1 -= msg->axes[5] * VELOCITY * 2.0;
            if (msg->buttons[4] == 1){
                target_pos.position_0 += VELOCITY;
            }else if(msg->buttons[5] == 1){
                target_pos.position_0 -= VELOCITY;
            }
        };
                
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        set_pos_pub = this->create_publisher<SetPosition>("set_pantilt", qos);
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