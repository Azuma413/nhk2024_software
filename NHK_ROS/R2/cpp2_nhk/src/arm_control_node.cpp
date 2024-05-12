// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "actuator_custom_msgs/msg/arm_control.hpp"
#include "actuator_custom_msgs/msg/set_position.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <math.h>

using namespace std::chrono_literals;
using ArmControl = actuator_custom_msgs::msg::ArmControl;
using SetPosition = actuator_custom_msgs::msg::SetPosition;
using Float32 = std_msgs::msg::Float32;
using Bool = std_msgs::msg::Bool;
using SetBool = std_srvs::srv::SetBool;
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
// アームの長さ
const float L0 = 382.0 - 50; // mm 第一関節の高さ - 補正値
const float L1 = 160.0; // mm 第一関節から第二関節までの長さ
const float L2 = 310.0; // mm 第二関節から第三関節までの長さ
const float L3 = 69.574; // mm 第三関節から吸着部分の中心までの長さ
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class ArmControlNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<SetPosition>::SharedPtr dxl_command_pub;
    rclcpp::Publisher<Float32>::SharedPtr mros_input_table_r2_pub;
    rclcpp::Publisher<Bool>::SharedPtr mros_input_air_r2_pub;
    rclcpp::Subscription<ArmControl>::SharedPtr arm_control1_sub;
    rclcpp::Subscription<ArmControl>::SharedPtr arm_control2_sub;
    rclcpp::Subscription<ArmControl>::SharedPtr arm_control3_sub;
    rclcpp::Service<SetBool>::SharedPtr arm_control1_on_srv;
    rclcpp::Service<SetBool>::SharedPtr arm_control2_on_srv;
    rclcpp::Service<SetBool>::SharedPtr arm_control3_on_srv;
    ArmControl arm_control_msg;
    bool arm_control1_on = false;
    bool arm_control2_on = false;
    bool arm_control3_on = false;
    std::string color = "";

    bool calc_rad(float r, float h, float& theta0, float& theta1){
        /*
        range 270-500 height 150-750ならほとんど破綻しない
        */
        float a0 = (r - L3)*(r - L3) + (h - L0)*(h - L0);
        if (a0 > (L1 + L2)*(L1 + L2) || a0 < (L1 - L2)*(L1 - L2)){
            // std::cout << "out of range" << std::endl;
            return false;
        }
        float a1 = (a0 + L1*L1 - L2*L2)/2;
        float a2 = sqrt(a0*L1*L1 - a1*a1);
        float xn, yn;
        if(h > L0){
            xn = (a1*(r - L3) + (h - L0)*a2)/a0; // 第２関節の座標
            yn = (a1*(h - L0) - (r - L3)*a2)/a0; // 第２関節の座標 - L0
        }else{
            xn = (a1*(r - L3) - (h - L0)*a2)/a0; // 第２関節の座標
            yn = (a1*(h - L0) + (r - L3)*a2)/a0; // 第２関節の座標 - L0
        }

        theta0 = atan2(yn, xn);
        theta1 = atan2(h - L0 - yn, r - xn);
        // thetaが可動域外の場合負の値を返すコードを追加する
        return true;
    }

    public:
    ArmControlNode() : Node("arm_control_node"){
        RCLCPP_INFO(this->get_logger(), "arm_control_node is activated");
        // パラメータの宣言と取得
        auto parameter = this->declare_parameter<std::string>("color", "blue");
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }

        auto timer_callback = [this]() -> void{
            if(!(arm_control1_on or arm_control2_on or arm_control3_on)) return;
            // 実際にアームに指令を送る
            SetPosition dxl_command_msg;
            Float32 mros_input_table_r2_msg;
            Bool mros_input_air_r2_msg;
            // 円筒座標の値をモーターの角度[ラジアン]に変換
            float theta0, theta1;
            // std::cout << "input value: " << arm_control_msg.range << ", " << arm_control_msg.height << std::endl;
            if(calc_rad(arm_control_msg.range, arm_control_msg.height, theta0, theta1)){
                // std::cout << "input is nominal.\ntheta0: " << theta0 << " theta1: " << theta1 << std::endl;
            }else{ // 可動範囲外の場合はモーターを動かさない
                std::cout << "無効な値の入力です（アームの可動範囲外）" << std::endl;
                return;
            }
            // ダイナミクセル
            dxl_command_msg.position_0 = theta0;
            dxl_command_msg.position_1 = theta1;
            dxl_command_pub->publish(dxl_command_msg);
            // 吸着
            mros_input_air_r2_msg.data = arm_control_msg.vacuum;
            mros_input_air_r2_pub->publish(mros_input_air_r2_msg);
            // ターンテーブル
            mros_input_table_r2_msg.data = arm_control_msg.angle;
            mros_input_table_r2_pub->publish(mros_input_table_r2_msg);
            // std::cout << "end timer callback" << std::endl;
        };
        
        auto arm_control1_callback = [this](const ArmControl& msg) -> void{
            // アームに関する制御情報を受け取る
            if (arm_control1_on){
                arm_control_msg = msg;
            }
        };

        auto arm_control2_callback = [this](const ArmControl& msg) -> void{
            // std::cout << "arm_control2_callback" << std::endl;
            // アームに関する制御情報を受け取る
            if (arm_control2_on){
                // std::cout << "arm_control2_callback if" << std::endl;
                arm_control_msg = msg;
            }
        };

        auto arm_control3_callback = [this](const ArmControl& msg) -> void{
            // アームに関する制御情報を受け取る
            if(arm_control3_on){
                arm_control_msg = msg;
            }
        };

        auto arm_control1_on_callback = [this](const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response) -> void{
            // アームの制御を有効にする
            std::cout << "control 1 on" << std::endl;
            arm_control1_on = request->data;
            arm_control2_on = false;
            arm_control3_on = false;
            response->success = true;
        };

        auto arm_control2_on_callback = [this](const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response) -> void{
            // アームの制御を有効にする
            std::cout << "control 2 on" << std::endl;
            arm_control2_on = request->data;
            arm_control1_on = false;
            arm_control3_on = false;
            response->success = true;
        };

        auto arm_control3_on_callback = [this](const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response) -> void{
            // アームの制御を有効にする
            std::cout << "control 3 on" << std::endl;
            arm_control3_on = request->data;
            arm_control1_on = false;
            arm_control2_on = false;
            response->success = true;
        };

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        dxl_command_pub = this->create_publisher<SetPosition>("set_arm_position", qos);
        mros_input_table_r2_pub = this->create_publisher<Float32>("mros_input_table", qos);
        mros_input_air_r2_pub = this->create_publisher<Bool>("mros_input_air", qos);
        arm_control1_sub = this->create_subscription<ArmControl>("arm_control1", qos, arm_control1_callback);
        arm_control2_sub = this->create_subscription<ArmControl>("arm_control2", qos, arm_control2_callback);
        arm_control3_sub = this->create_subscription<ArmControl>("arm_control3", qos, arm_control3_callback);
        arm_control1_on_srv = this->create_service<SetBool>("arm_control1_on", arm_control1_on_callback);
        arm_control2_on_srv = this->create_service<SetBool>("arm_control2_on", arm_control2_on_callback);
        arm_control3_on_srv = this->create_service<SetBool>("arm_control3_on", arm_control3_on_callback);
        timer = this->create_wall_timer(100ms, timer_callback);

        // アームの初期化
        arm_control_msg.range = 100;
        arm_control_msg.height = 100;
        arm_control_msg.angle = 0;
        arm_control_msg.vacuum = false;
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ArmControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}