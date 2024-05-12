/*
自動発射モードの実装
*/
// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "actuator_custom_msgs/action/call_auto_launch.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "actuator_custom_msgs/srv/set_table_rad.hpp"
#include "actuator_custom_msgs/srv/start_calc_pos.hpp"
#include "actuator_custom_msgs/msg/shoot_roller.hpp"
#include "actuator_custom_msgs/msg/shoot_air_status.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;
using String = std_msgs::msg::String;
using SetBool = std_srvs::srv::SetBool;
using StartCalcPos = actuator_custom_msgs::srv::StartCalcPos;
using Joy = sensor_msgs::msg::Joy;
using Point = geometry_msgs::msg::Point;
using ShootRoller = actuator_custom_msgs::msg::ShootRoller;
using ShootAirStatus = actuator_custom_msgs::msg::ShootAirStatus;
using SetTableRad = actuator_custom_msgs::srv::SetTableRad;
using CallAutoLaunch = actuator_custom_msgs::action::CallAutoLaunch;
using GoalHandleCAL = rclcpp_action::ServerGoalHandle<CallAutoLaunch>;

struct Point2D{
    double x;
    double y;
};

// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
Point2D RED_BLUE_TARGET = {1983, 1800}; // エリア2の左上/右上を原点としたときの，赤/青色のボールの目標位置
Point2D PURPLE_TARGET = {3768, 2750}; // 紫色のボールの目標位置
// R1のターンテーブルの可動範囲
const double TABLE_MIN = -M_PI/3 + 0.01;
const double TABLE_MAX = M_PI/3 - 0.01;
const double DUTY_CONTROL = 301.2541395838844; // モーターのデューティ比を計算するための定数

void calc_motor_duty(Point2D& robot_pos, Point2D& target, int& m1, int& m2, int& m3){
    // ロボットの位置と目標地点の距離を計算
    double sq_distance = sqrt(sqrt(pow(target.x - robot_pos.x, 2) + pow(target.y - robot_pos.y, 2)));
    double x = sq_distance / DUTY_CONTROL;
    if(x > 0.26){
        x = 0.26;
    }
    // モーターのデューティ比を計算
    m1 = x * 1000;//Duty*1000
    m2 = x * 1000;
    m3 = x * 3000;
}
void set_motor_duty(Point2D& robot_pos, int& color, int& m1, int& m2, int& m3){
    if (color == 1 || color == 2){
        if(robot_pos.x<3125){
            m1 = 0.188 * 1000;
            m2 = 0.188 * 1000;
            m3 = 0.625 * 1000;
            return;
        }else if(robot_pos.x<4125){
            m1 = 0.2 * 1000;
            m2 = 0.2 * 1000;
            m3 = 0.7 * 1000;
            return;
        }else{
            m1 = 0.234 * 1000;
            m2 = 0.234 * 1000;
            m3 = 0.7 * 1000;
            return;
        }
    }else if(color == 3){
        if(robot_pos.x<3125){
            m1 = 0.138  * 1000;
            m2 = 0.5 * 1000;
            m3 = 0.7 * 1000; 
            return;
        }else if(robot_pos.x<4125){
            m1 = 0.1  * 1000;
            m2 = 0.5 * 1000;
            m3 = 0.7 * 1000; 
            return;
        }else{
            m1 = 0.1  * 1000;
            m2 = 0.5 * 1000;
            m3 = 0.7 * 1000; 
            return;
        }
    }else{
        return;
    }
}
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class AutoLaunchNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<ShootAirStatus>::SharedPtr mros_input_air_shoot_pub;
    rclcpp::Publisher<ShootRoller>::SharedPtr mros_input_mcmd_pub;
    rclcpp::Subscription<String>::SharedPtr ball_color_sub;
    rclcpp::Subscription<Joy>::SharedPtr joy_sub;
    rclcpp::Subscription<Point>::SharedPtr robot_pos_sub;
    rclcpp::Client<SetTableRad>::SharedPtr table_rad_cli;
    rclcpp::Client<SetBool>::SharedPtr control_id_color_cli;
    rclcpp::Client<StartCalcPos>::SharedPtr start_calc_pos_cli;
    rclcpp_action::Server<CallAutoLaunch>::SharedPtr call_auto_launch_asrv;
    std::string color = "";
    int detect_color_id = 0;
    bool flag = true; // サーバーの起動確認用
    bool launch_flag = false; // 発射ボタンが押されたかどうかのフラグ
    ShootAirStatus air_status;
    ShootRoller roller;
    bool common_result = false;
    Point2D robot_pos;
    bool action_flag = false; // call action ?

    public:
    AutoLaunchNode() : Node("auto_launch_node"){
        RCLCPP_INFO(this->get_logger(), "auto_launch_node is activated");
        // パラメータの宣言と取得
        auto parameter = this->declare_parameter<std::string>("color", "blue");
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }

        auto timer_callback = [this]() -> void{
            if(action_flag){
                mros_input_air_shoot_pub->publish(air_status);
                mros_input_mcmd_pub->publish(roller);
            }
        };

        auto ball_color_callback = [this](const String::SharedPtr msg) -> void{
            color = msg->data;
            if(color == "none"){
                detect_color_id = 0;
            }else if(color == "blue"){
                detect_color_id = 1;
            }else if(color == "red"){
                detect_color_id = 2;
            }else if(color == "purple"){
                detect_color_id = 3;
            }else{
                detect_color_id = 0;
            }
        };

        auto joy_callback = [this](const Joy::SharedPtr msg) -> void{
            if(msg->buttons[1] == 1){
                launch_flag = true;
            }else{
                launch_flag = false;
            }
        };

        auto robot_pos_callback = [this](const Point::SharedPtr msg) -> void{
            robot_pos.x = msg->x; // mm
            robot_pos.y = msg->y;
        };

        auto call_auto_launch_goal = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const CallAutoLaunch::Goal> goal) -> rclcpp_action::GoalResponse{
            // 目標値を受け取った時の処理
            std::cout << "目標値を受け取りました" << std::endl;
            action_flag = true;
            async_start_calc_pos(true, 2, common_result);
            async_control_id_color(true, common_result);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // ACCEPT_AND_DEFER
        };

        auto call_auto_launch_cancel = [this](const std::shared_ptr<GoalHandleCAL> goal_handle) -> rclcpp_action::CancelResponse{
            // // キャンセルを受け取ったときの処理
            std::cout << "キャンセルを受け取りました" << std::endl;
            action_flag = false;
            async_start_calc_pos(false, 0, common_result);
            async_control_id_color(false, common_result);
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto call_auto_launch_accepted = [this](const std::shared_ptr<GoalHandleCAL> goal_handle) -> void{
            using namespace std::placeholders;
            std::cout << "アクションを開始します" << std::endl;
            std::thread(std::bind(&AutoLaunchNode::call_auto_launch_execute, this, _1), goal_handle).detach();
        };

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        mros_input_air_shoot_pub = this->create_publisher<ShootAirStatus>("mros_input_air_shoot", qos);
        mros_input_mcmd_pub = this->create_publisher<ShootRoller>("mros_input_mcmd", qos);
        ball_color_sub = this->create_subscription<String>("ball_color", qos, ball_color_callback);
        joy_sub = this->create_subscription<Joy>("joy", qos, joy_callback);
        robot_pos_sub = this->create_subscription<Point>("robot_pos", qos, robot_pos_callback);
        call_auto_launch_asrv = rclcpp_action::create_server<CallAutoLaunch>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "call_auto_launch",
            call_auto_launch_goal,
            call_auto_launch_cancel,
            call_auto_launch_accepted
        );

        control_id_color_cli = this->create_client<SetBool>("control_id_color");
        while(!control_id_color_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "control_id_color service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "control_id_color service is ready" << std::endl;
        flag = true;
        start_calc_pos_cli = this->create_client<StartCalcPos>("start_calc_pos");
        while(!start_calc_pos_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "start_calc_pos service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "start_calc_pos service is ready" << std::endl;
        flag = true;
        table_rad_cli = this->create_client<SetTableRad>("set_table_rad");
        while(!table_rad_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "set_table_rad service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "set_table_rad service is ready" << std::endl;
        // 状態の初期化
        air_status.ca = false; // catch ボールを掴むアームの部分のエアシリンダー true:掴む false:離す
        air_status.pi = true; // pick ボールを持ち上げる部分のエアシリンダー true:持ち上げる false:下げる
        air_status.gu = false; // guide ボールを射出するためのガイドのエアシリンダー true:ガイドを出す false:ガイドを引っ込める
        air_status.sh = false; // shoot ボールを射出機構に送るためのエアシリンダー true:射出機構に送る false:射出機構に送らない
        roller.m1 = 0; // rpm
        roller.m2 = 0;
        roller.m3 = 0;
        timer = this->create_wall_timer(100ms, timer_callback);
    }

    void call_auto_launch_execute(const std::shared_ptr<GoalHandleCAL> goal_handle){
        // thread内で実行される実際の処理部分
        auto result = std::make_shared<CallAutoLaunch::Result>();
        // エアシリンダーの初期化
        std::cout << "エアシリンダーを初期化します" << std::endl;
        air_status.ca = false; // catch ボールを掴むアームの部分のエアシリンダー true:掴む false:離す
        air_status.pi = true; // pick ボールを持ち上げる部分のエアシリンダー true:持ち上げる false:下げる
        air_status.gu = false; // guide ボールを射出するためのガイドのエアシリンダー true:ガイドを出す false:ガイドを引っ込める
        air_status.sh = false; // shoot ボールを射出機構に送るためのエアシリンダー true:射出機構に送る false:射出機構に送らない
        // ローラーの初期化
        std::cout << "ローラーを初期化します" << std::endl;
        roller.m1 = 0; // rpm
        roller.m2 = 0;
        roller.m3 = 0;
        // 無限ループ開始
        while(rclcpp::ok()){
            rclcpp::sleep_for(10ms);
            int color_id;
            if(goal_handle->is_canceling()){ // 途中でキャンセルされていないか確認
                std::cout << "アクションがキャンセルされました" << std::endl;
                result->result = false;
                action_flag = false;
                goal_handle->canceled(result);
                std::cout << "色認識を終了します" << std::endl;
                async_control_id_color(false, common_result);
                async_start_calc_pos(false, 0, common_result);
                return;
            }
            if(detect_color_id == 0){ // 何も認識されていない場合はスキップ
                continue;
            }else if(detect_color_id == 1 || detect_color_id == 2 || detect_color_id == 3){ // 認識された場合
                color_id = detect_color_id; // 認識された色のIDを保存
            }
            // アームを降ろす
            std::cout << "アームを降ろします" << std::endl;
            air_status.ca = false;
            air_status.pi = false;
            air_status.gu = false;
            air_status.sh = false;
            std::this_thread::sleep_for(1.0s);
            // ボールを掴む
            std::cout << "ボールを掴みます" << std::endl;
            air_status.ca = true;
            air_status.pi = false;
            air_status.gu = false;
            air_status.sh = false;
            // 0.5秒待つ
            std::this_thread::sleep_for(1.0s);
            // ボールを持ち上げる
            std::cout << "ボールを持ち上げます" << std::endl;
            air_status.ca = true;
            air_status.pi = true;
            air_status.gu = false;
            air_status.sh = false;
            std::this_thread::sleep_for(2.0s);
            // ボールの目標地点を設定
            Point2D target;
            if (color_id == 1 || color_id == 2){
                std::cout << "赤/青色のボールが認識されました" << std::endl;
                target = RED_BLUE_TARGET;
            }else if(color_id == 3){
                std::cout << "紫色のボールが認識されました" << std::endl;
                target = PURPLE_TARGET;
            }else{
                std::cout << "認識された色が不正です" << std::endl;
                continue;
            }
            // ロボットから目標地点までの角度を計算
            float angle = -atan2(target.x - robot_pos.x, target.y - (robot_pos.y - 4000.0)); // 引数の順番を間違えているわけではない。座標系の関係でこのようになる
            // ターンテーブルの可動範囲内に収める
            if(angle < TABLE_MIN){
                angle = TABLE_MIN;
            }else if(angle > TABLE_MAX){
                angle = TABLE_MAX;
            }
            // ターンテーブルを回転
            std::cout << "set table rad:" << angle*180.0/M_PI << std::endl;
            async_set_table_rad(angle, common_result);
            rclcpp::sleep_for(200ms);
            // ローラーを回す
            std::cout << "ローラーを回します" << std::endl;
            // calc_motor_duty(robot_pos, target, roller.m1, roller.m2, roller.m3);
            set_motor_duty(robot_pos, color_id, roller.m1, roller.m2, roller.m3);
            // ガイドを出す
            std::cout << "ガイドを出します" << std::endl;
            air_status.ca = true;
            air_status.pi = true;
            air_status.gu = true;
            air_status.sh = false;
            // 0.5秒待つ
            std::this_thread::sleep_for(1.0s);
            // joyのボタン入力を待つ
            while(rclcpp::ok()){
                rclcpp::sleep_for(10ms);
                if(goal_handle->is_canceling()){ // 途中でキャンセルされていないか確認
                    std::cout << "アクションがキャンセルされました" << std::endl;
                    result->result = false;
                    goal_handle->canceled(result);
                    std::cout << "色認識を終了します" << std::endl;
                    async_control_id_color(false, common_result);
                    async_start_calc_pos(false, 0, common_result);
                    air_status.pi = true; // pick ボールを持ち上げる部分のエアシリンダー true:持ち上げる false:下げる
                    air_status.gu = false; // guide ボールを射出するためのガイドのエアシリンダー true:ガイドを出す false:ガイドを引っ込める
                    air_status.sh = false; // shoot ボールを射出機構に送るためのエアシリンダー true:射出機構に送る false:射出機構に送らない
                    roller.m1 = 0; // rpm
                    roller.m2 = 0;
                    roller.m3 = 0;
                    std::this_thread::sleep_for(1.0s);
                    air_status.ca = false; // catch ボールを掴むアームの部分のエアシリンダー true:掴む false:離す
                    return;
                }
                // ボタンが押されたらループを抜ける
                if(launch_flag){
                    std::cout << "発射ボタンが押されました" << std::endl;
                    break;
                }
            }
            // ガイドを出す
            // std::cout << "ガイドを出します" << std::endl;
            // air_status.ca = true;
            // air_status.pi = true;
            // air_status.gu = true;
            // air_status.sh = false;
            // // 0.5秒待つ
            // std::this_thread::sleep_for(1.0s);
            // 射出機構に送る
            std::cout << "射出機構にボールを送ります" << std::endl;
            air_status.ca = true;
            air_status.pi = true;
            air_status.gu = true;
            air_status.sh = true;
            // 0.5秒待つ
            std::this_thread::sleep_for(1.0s);
            // ローラーを止める
            std::cout << "ローラーを止めます" << std::endl;
            roller.m1 = 0;
            roller.m2 = 0;
            roller.m3 = 0;
            // 初期状態に戻す
            std::cout << "初期状態に戻します" << std::endl;
            air_status.ca = false;
            air_status.pi = true;
            air_status.gu = false;
            air_status.sh = false;
        } // 無限ループ終了
        async_control_id_color(false, common_result);
        async_start_calc_pos(false, 0, common_result);
        if(rclcpp::ok()){ // アクションが成功した場合
            result->result = true;
            goal_handle->succeed(result);
        }
    }

    void async_control_id_color(bool set_flag, bool& myresult){
        auto request = std::make_shared<SetBool::Request>();
        request->data = set_flag;
        auto response_received_callback = [this, &myresult](rclcpp::Client<SetBool>::SharedFuture future) {
            myresult = future.get()->success;
            std::cout << "[control_id_color] レスポンスを受け取りました\nresult: " << myresult << std::endl;
        };
        auto future_result = control_id_color_cli->async_send_request(request);
        std::cout << "[control_id_color] リクエストを送信しました" << std::endl;
    }

    void async_start_calc_pos(bool start, int erea, bool& myresult){
        auto request = std::make_shared<StartCalcPos::Request>();
        request->start = start;
        request->erea = erea;
        auto response_received_callback = [this, &myresult](rclcpp::Client<SetBool>::SharedFuture future){
            myresult = future.get()->success;
            std::cout << "[start_calc_pos] レスポンスを受け取りました\nresult: " << myresult << std::endl;
        };
        auto future_result = start_calc_pos_cli->async_send_request(request);
        std::cout << "[start_calc_pos] リクエストを送信しました" << std::endl;
    }

    void async_set_table_rad(double rad, bool& myresult){
        auto request = std::make_shared<SetTableRad::Request>();
        request->rad = rad;
        auto response_received_callback = [this, &myresult](rclcpp::Client<SetTableRad>::SharedFuture future) {
            myresult = future.get()->result;
            std::cout << "[SetTableRad] レスポンスを受け取りました\nresult: " << myresult << std::endl;
        };
        auto future_result = table_rad_cli->async_send_request(request, response_received_callback);
        std::cout << "[SetTableRad] リクエストを送信しました" << std::endl;
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AutoLaunchNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}