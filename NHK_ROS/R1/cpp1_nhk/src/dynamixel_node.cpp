/*
稲を掴むハンドのダイナミクセルを動かすノード
*/
// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <cstdio>
#include <memory>
#include <string>
#include <map>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "actuator_custom_msgs/msg/set_position.hpp"
#include "actuator_custom_msgs/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "../include/dynamixel_workbench_toolbox/dynamixel_workbench.h"

#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

using namespace std::chrono_literals;
using SetPosition = actuator_custom_msgs::msg::SetPosition;
using GetPosition = actuator_custom_msgs::srv::GetPosition;

struct Point2D{
    float x;
    float y;
};
// ********************************************************************************************************************
// 定数等の定義 {1126.15, -210.32}
// ********************************************************************************************************************
const float DELTA_RAD = 0.5; // 1周期あたりの変化量[rad]
const std::vector<float> ID0_LIMIT = {-1180.0/180.0*M_PI, 200.0/180.0*M_PI}; // id0の上限と下限{min, max}[rad]
const std::vector<float> ID1_LIMIT = {-200.0/180.0*M_PI, 1120.0/180.0*M_PI}; // id1の上限と下限{min, max}[rad]
const std::vector<float> INIT_POS = {-76.77/180.0*M_PI, 27.42/180.0*M_PI}; // 初期位置[rad]
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class DynamixelNode : public rclcpp::Node{
    private:
    rclcpp::Subscription<SetPosition>::SharedPtr subscriber;
    rclcpp::Service<GetPosition>::SharedPtr server;
    rclcpp::TimerBase::SharedPtr timer;
    DynamixelWorkbench dxl_wb;
    std::vector<float> target_position = INIT_POS;
    uint8_t id0, id1;
    int id0_, id1_;
    std::string color = "";
    const char* log;

    public:
    DynamixelNode() : Node("dynamixel_node"){
        RCLCPP_INFO(this->get_logger(), "dynamixel_node is activated");
        // パラメータの宣言と取得
        auto parameter0 = this->declare_parameter<std::string>("color", "blue");
        auto parameter1 = this->declare_parameter<std::string>("device", "/dev/U2D2-R1");
        auto parameter2 = this->declare_parameter<int>("baudrate", 115200);
        auto parameter3 = this->declare_parameter<int>("id0", 0);
        auto parameter4 = this->declare_parameter<int>("id1", 1);

        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }

        std::string device_name_;
        if(!this->get_parameter("device", device_name_)){
            std::cout << "device パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "device: " << device_name_ << std::endl;
        }
        const char* device_name = device_name_.c_str();

        int baudrate_;
        if(!this->get_parameter("baudrate", baudrate_)){
            std::cout << "baudrate パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "baudrate: " << baudrate_ << std::endl;
        }
        uint32_t baudrate = (uint32_t)baudrate_;

        if(!this->get_parameter("id0", id0_)){
            std::cout << "id0 パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "id0: " << id0_ << std::endl;
        }
        id0 = (uint8_t)id0_;

        if(!this->get_parameter("id1", id1_)){
            std::cout << "id1 パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "id1: " << id1_ << std::endl;
        }
        id1 = (uint8_t)id1_;

        std::cout << "セットアップを開始します\n" << std::endl;

        dxl_wb.init(device_name, baudrate, &log);
        std::cout << "シリアルポートの初期化(" << device_name << ", " << baudrate << ")\n" << log << std::endl;
        
        uint16_t model_number;
        dxl_wb.ping(id0, &model_number, &log);
        std::cout << "ID " << id0_ << " へPingを送信\n" << log << std::endl;
        if(model_number == PRO_H42_20_S300_R_A){
            std::cout << "検出されたモデル: PRO_H42_20_S300_R_A" << std::endl;
        }else if(model_number == MX_106_2){
            std::cout << "検出されたモデル: MX_106_2" << std::endl;
        }else if(model_number == XM430_W350){
            std::cout << "検出されたモデル: XM430_W350" << std::endl;
        }else{
            std::cout << "検出されたモデル: " << model_number << std::endl;
        }

        dxl_wb.ping(id1, &model_number, &log);
        std::cout << "ID " << id1_ << " へPingを送信\n" << log << std::endl;
        if(model_number == PRO_H42_20_S300_R_A){
            std::cout << "検出されたモデル: PRO_H42_20_S300_R_A" << std::endl;
        }else if(model_number == MX_106_2){
            std::cout << "検出されたモデル: MX_106_2" << std::endl;
        }else if(model_number == XM430_W350){
            std::cout << "検出されたモデル: XM430_W350" << std::endl;
        }else{
            std::cout << "検出されたモデル: " << model_number << std::endl;
        }

        // dxl_wb.setPositionControlMode(id0, &log);
        dxl_wb.setExtendedPositionControlMode(id0, &log);
        std::cout << "ID " << id0_ << " を拡張位置制御モードに設定\n" << log << std::endl;
        // dxl_wb.setPositionControlMode(id1, &log);
        dxl_wb.setExtendedPositionControlMode(id1, &log);
        std::cout << "ID " << id1_ << " を拡張位置制御モードに設定\n" << log << std::endl;

        dxl_wb.torqueOn(id0, &log);
        std::cout << "ID " << id0_ << " のトルクをオン\n" << log << std::endl;
        dxl_wb.torqueOn(id1, &log);
        std::cout << "ID " << id1_ << " のトルクをオン\n" << log << std::endl;

        std::cout << "セットアップが完了しました\n" << std::endl;

        auto timer_callback = [this]() -> void{
            float present_position;
            dxl_wb.getRadian(id0, &present_position);
            if (target_position[0] - present_position > DELTA_RAD){ // target_positionがpresent_positionより十分大きい場合はDELTA_RADだけ増加
                dxl_wb.goalPosition(id0, present_position + DELTA_RAD, &log);
            }else if (target_position[0] - present_position < -DELTA_RAD){ // target_positionがpresent_positionより十分小さい場合はDELTA_RADだけ減少
                dxl_wb.goalPosition(id0, present_position - DELTA_RAD, &log);
            }else{ // target_positionがpresent_positionとほぼ同じ場合はtarget_positionに設定
                dxl_wb.goalPosition(id0, target_position[0], &log);
            }
            // std::cout << "ID " << (int)id0 << " のPositionを" << present_position + DELTA_RAD << "[rad]に設定\n" << log << std::endl;
            dxl_wb.getRadian(id1, &present_position);
            if (target_position[1] - present_position > DELTA_RAD){ // target_positionがpresent_positionより十分大きい場合はDELTA_RADだけ増加
                dxl_wb.goalPosition(id1, present_position + DELTA_RAD, &log);
            }else if (target_position[1] - present_position < -DELTA_RAD){ // target_positionがpresent_positionより十分小さい場合はDELTA_RADだけ減少
                dxl_wb.goalPosition(id1, present_position - DELTA_RAD, &log);
            }else{ // target_positionがpresent_positionとほぼ同じ場合はtarget_positionに設定
                dxl_wb.goalPosition(id1, target_position[1], &log);
            }
            // std::cout << "ID " << (int)id1 << " のPositionを" << present_position + DELTA_RAD << "[rad]に設定\n" << log << std::endl;
        };
        
        auto topic_callback = [this](const SetPosition::SharedPtr msg) -> void{
            Point2D position = range2rad(msg);
            if (position.x < ID0_LIMIT[0]){
                std::cout << "ID " << id0_ << " の目標値が可動域外です\n目標値を再設定します" << std::endl;
                target_position[0] = ID0_LIMIT[0];
            }else if(position.x > ID0_LIMIT[1]){
                std::cout << "ID " << id0_ << " の目標値が可動域外です\n目標値を再設定します" << std::endl;
                target_position[0] = ID0_LIMIT[1];
            }else{
                target_position[0] = position.x;
            }
            if (position.y < ID1_LIMIT[0]){
                std::cout << "ID " << id1_ << " の目標値が可動域外です\n目標値を再設定します" << std::endl;
                target_position[1] = ID1_LIMIT[0];
            }else if(position.y > ID1_LIMIT[1]){
                std::cout << "ID " << id1_ << " の目標値が可動域外です\n目標値を再設定します" << std::endl;
                target_position[1] = ID1_LIMIT[1];
            }else{
                target_position[1] = position.y;
            }
        };

        auto service_callback = [this](const std::shared_ptr<GetPosition::Request> request, std::shared_ptr<GetPosition::Response> response) -> void{
            float present_position;
            dxl_wb.getRadian(request->id, &present_position, &log);
            std::cout << "get_position サービスが呼び出されました\n" << log << std::endl;
            response->position = present_position;
        };

        this->declare_parameter("qos_depth", 10);
        int8_t qos_depth = 0;
        this->get_parameter("qos_depth", qos_depth);
        const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

        subscriber =this->create_subscription<SetPosition>("set_position", QOS_RKL10V, topic_callback);
        server = create_service<GetPosition>("get_position", service_callback);
        timer = this->create_wall_timer(5ms, timer_callback);
    }

    Point2D range2rad(const SetPosition::SharedPtr msg){
        Point2D rad;
        rad.x = INIT_POS[0] - msg->position_0*0.0929318;
        rad.y = INIT_POS[1] + msg->position_1*0.0929318;
        return rad;
    }

    ~DynamixelNode(){
        std::cout << "終了処理を開始します\n" << std::endl;
        target_position = INIT_POS;
        dxl_wb.goalPosition(id0, target_position[0], &log);
        std::cout << "ID " << (int)id0 << " のPositionを初期位置に設定\n" << log << std::endl;
        dxl_wb.goalPosition(id1, target_position[1], &log);
        std::cout << "ID " << (int)id1 << " のPositionを初期位置に設定\n" << log << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        dxl_wb.torqueOff(id0, &log);
        std::cout << "ID " << (int)id0 << " のトルクをオフ\n" << log << std::endl;
        dxl_wb.torqueOff(id1, &log);
        std::cout << "ID " << (int)id1 << " のトルクをオフ\n" << log << std::endl;
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamixelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}