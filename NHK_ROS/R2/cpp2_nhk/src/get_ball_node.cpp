// 拾う時
// range 300
// height 140


// start
// 300, 700
// end
// 300, 400

// ライブラリのインクルードなど
// 非同期を前提としてサービス通信内でループを使用できないかテストする。
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "actuator_custom_msgs/msg/arm_control.hpp"
#include "actuator_custom_msgs/action/call_get_ball.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using Point = geometry_msgs::msg::Point;
using Bool = std_msgs::msg::Bool;
using SetBool = std_srvs::srv::SetBool;
using ArmControl = actuator_custom_msgs::msg::ArmControl;
using CallGetBall = actuator_custom_msgs::action::CallGetBall;
using GoalHandleCGB = rclcpp_action::ServerGoalHandle<CallGetBall>;

struct Point2D{
    double x;
    double y;
};
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
float RANGE_P = 50; // 誤差によってアームを操作する際のPゲイン(range)y軸 [mm]
float RANGE_D = 50; 
float ANGLE_P = 0.05; // 誤差によってアームを操作する際のPゲイン(angle)x軸 [rad]
float ANGLE_D = 0.05;
float RANGE = 400; // mm bt_nodeによって設定されるボール検出の際のロボットからアームの吸着部分までの距離
float HEIGHT = 600; // mm bt_nodeによって設定されるボール検出の際の地面からアームの吸着部分までの高さ
float CATCH_HEIGHT = 100.0; //  mm ボールを取る際のアームの高さ
float CONTROL_HEIGHT = 500.0; // mm ボールを取る際にアームの位置を制御する最低高度
int TRY_NUM = 3; // ボールを取る際の試行回数
const float P_GAIN = 0.0005; // 速度制御のPゲイン
const float D_GAIN = 0.001; // 速度制御のDゲイン
const float TARGET_THRESHOLD = 30.0; // エンコーダの許容誤差[mm]
const float BALL_EPSILON = 0.05; // ボールの許容誤差(画面に対する割合)
const float ERR_P_VALUE = 0.5; // ボールの誤差に対する足回りのP制御のゲイン
const double MAX_VEL = 0.6; // 最大速度[m/s]
const int ARM_ADJUST_NUM = 50; // アームの調整回数(何周期でHEIGHTからCONTROL_HEIGHTに移行するか)

const bool use_vacuum_state = false; // 吸着をセンサーを用いて判定するかどうか
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class GetBallNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<Point>::SharedPtr robot_vel_pub;
    rclcpp::Publisher<ArmControl>::SharedPtr arm_control_pub;
    rclcpp::Subscription<Point>::SharedPtr ball_error_sub;
    rclcpp::Subscription<Point>::SharedPtr distance_sub;
    rclcpp::Subscription<Bool>::SharedPtr vacuum_state_sub;
    rclcpp::Client<SetBool>::SharedPtr control_trace_cli;
    rclcpp::Client<SetBool>::SharedPtr arm_control_on_cli;
    rclcpp::Client<SetBool>::SharedPtr robot_vel_on_cli;
    rclcpp_action::Server<CallGetBall>::SharedPtr call_get_ball_asrv;
    Point2D error; // ボールの誤差
    bool detect_ball = false; // ボールが検出されたかどうか
    std::string color = "";
    bool flag = true;
    ArmControl arm_control_msg;
    bool vacuum_state = false;
    Point robot_vel_msg;
    bool common_result;
    bool is_slope = false;

    Point2D distance_data = {0.0, 0.0}; // サブスクライブしたそのままの値
    Point2D old_distance = {0.0, 0.0}; // ある時点の移動距離
    void set_distance(void){
        old_distance = distance_data;
    }
    Point2D get_distance(void){
        Point2D distance;
        distance.x = distance_data.x - old_distance.x;
        distance.y = distance_data.y - old_distance.y;
        return distance;
    }

    public:
    GetBallNode() : Node("get_ball_node"){
        RCLCPP_INFO(this->get_logger(), "get_ball_node is activated");
        // パラメータの宣言と取得
        auto parameter = this->declare_parameter<std::string>("color", "blue");
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }

        auto timer_callback = [this]() -> void{
            robot_vel_pub->publish(robot_vel_msg);
            arm_control_pub->publish(arm_control_msg);
        };

        auto ball_error_callback = [this](const Point& msg) -> void{
            // 吸着部分に対するボールの誤差を取得
            if(msg.z < 0){
                std::cout << "ボールが検出されませんでした" << std::endl;
                detect_ball = false;
            }else{
                error.x = msg.x;
                error.y = msg.y;
                detect_ball = true;
            }
        };

        auto distance_callback = [this](const Point& msg) -> void{
            // 転がしエンコーダーの補正値を取得
            distance_data.x = msg.x;
            distance_data.y = msg.y;
        };

        auto vacuum_state_callback = [this](const Bool::SharedPtr msg) -> void{
            // 吸着の状態を取得
            vacuum_state = msg->data;
        };

        auto call_get_ball_goal = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const CallGetBall::Goal> goal) -> rclcpp_action::GoalResponse{
            // 目標値を受け取った時の処理
            std::cout << "目標値を受け取りました" << std::endl;
            arm_control_msg.angle = -M_PI/2;
            arm_control_msg.height = HEIGHT;
            arm_control_msg.range = RANGE;
            arm_control_msg.vacuum = false;
            async_arm_control_on(true, common_result);
            async_robot_vel_on(true, common_result);
            async_control_trace(true, common_result);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // ACCEPT_AND_DEFER
        };

        auto call_get_ball_cancel = [this](const std::shared_ptr<GoalHandleCGB> goal_handle) -> rclcpp_action::CancelResponse{
            // キャンセルを受け取ったときの処理
            std::cout << "キャンセルを受け取りました" << std::endl;
            async_arm_control_on(false, common_result);
            async_robot_vel_on(false, common_result);
            async_control_trace(false, common_result);
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto call_get_ball_accepted = [this](const std::shared_ptr<GoalHandleCGB> goal_handle) -> void{
            using namespace std::placeholders;
            std::thread(std::bind(&GetBallNode::call_get_ball_execute, this, _1), goal_handle).detach();
        };

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        robot_vel_pub = this->create_publisher<Point>("robot_vel1", qos);
        arm_control_pub = this->create_publisher<ArmControl>("arm_control1", qos);
        ball_error_sub = this->create_subscription<Point>("ball_error", qos, ball_error_callback);
        distance_sub = this->create_subscription<Point>("distance", qos, distance_callback);
        if(use_vacuum_state){
            vacuum_state_sub = this->create_subscription<Bool>("vacuum_state", qos, vacuum_state_callback);
        }
        call_get_ball_asrv = rclcpp_action::create_server<CallGetBall>(
            this->get_node_base_interface(), 
            this->get_node_clock_interface(), 
            this->get_node_logging_interface(), 
            this->get_node_waitables_interface(), 
            "call_get_ball", 
            call_get_ball_goal, 
            call_get_ball_cancel, 
            call_get_ball_accepted
        );
        control_trace_cli = create_client<SetBool>("control_trace");
        while(!control_trace_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "control_trace service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "control_trace service is available" << std::endl;
        flag = true;
        arm_control_on_cli = create_client<SetBool>("arm_control1_on");
        while(!arm_control_on_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "arm_control1_on service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "arm_control1_on service is available" << std::endl;
        flag = true;
        robot_vel_on_cli = create_client<SetBool>("robot_vel1_on");
        while(!robot_vel_on_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "robot_vel1_on service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "robot_vel1_on service is available" << std::endl;
        // arm_control_msgの初期化
        arm_control_msg.angle = -M_PI/2;
        arm_control_msg.height = HEIGHT;
        arm_control_msg.range = RANGE;
        arm_control_msg.vacuum = false;
        timer = this->create_wall_timer(10ms, timer_callback);
    }

    void call_get_ball_execute(const std::shared_ptr<GoalHandleCGB> goal_handle){
        // thread内で実行されるcall_get_ballアクションサーバーの本体部分
        rclcpp::Rate loop_rate(100); // 100Hz
        auto feedback = std::make_shared<CallGetBall::Feedback>();
        auto result = std::make_shared<CallGetBall::Result>();
        Point2D target_pos;
        Point2D prior_error = {0.0, 0.0};
        target_pos.x = goal_handle->get_goal()->x; // ボールの相対位置 [mm]
        target_pos.y = goal_handle->get_goal()->y;
        std::cout << "目標位置: (" << target_pos.x << ", " << target_pos.y << ")" << std::endl;
        rclcpp::sleep_for(1000ms);
        set_distance();
        while(rclcpp::ok()){
            if(goal_handle->is_canceling()){ // 途中でキャンセルされていないか確認
                result->result = false;
                goal_handle->canceled(result);
                async_arm_control_on(false, common_result);
                async_robot_vel_on(false, common_result);
                async_control_trace(false, common_result);
                return;
            }
            if(detect_ball){ // ボールが検出された場合
                // ボールの位置に向かってロボットを移動させる
                if(sqrt(pow(error.x, 2) + pow(error.y, 2)) < BALL_EPSILON){
                    // ボールが中央に近づいたらbreak
                    std::cout << "足回りの制御を終了します" << std::endl;
                    robot_vel_msg.x = 0;
                    robot_vel_msg.y = 0;
                    break;
                }
                if(error.y > 0){
                    robot_vel_msg.x = std::max(-error.y*ERR_P_VALUE, -MAX_VEL); // 最大でMAX_VEL[m/s]
                }else{
                    robot_vel_msg.x = std::min(-error.y*ERR_P_VALUE, MAX_VEL); // 最大でMAX_VEL[m/s]
                }
                if(error.x > 0){
                    robot_vel_msg.y = std::min(error.x*ERR_P_VALUE, MAX_VEL); // 最大でMAX_VEL[m/s]
                }else{
                    robot_vel_msg.y = std::max(error.x*ERR_P_VALUE, -MAX_VEL); // 最大でMAX_VEL[m/s]
                }
            }else{ // ボールが検出されなかった場合
                // P制御によってロボットを移動させる move_certain_node参照
                Point2D move_error = {target_pos.x - get_distance().x, target_pos.y - get_distance().y};
                if(sqrt(move_error.x*move_error.x + move_error.y*move_error.y) < TARGET_THRESHOLD){
                    break;
                }
                // P制御によってロボットを移動させる
                robot_vel_msg.x = move_error.x*P_GAIN + (prior_error.x - move_error.x)*D_GAIN; // 最大でMAX_VEL[m/s]
                robot_vel_msg.y = move_error.y*P_GAIN + (prior_error.y - move_error.y)*D_GAIN; // 最大でMAX_VEL[m/s]
                if(robot_vel_msg.x > 0){
                    robot_vel_msg.x = std::min(robot_vel_msg.x, MAX_VEL); // 最大でMAX_VEL[m/s]
                }else{
                    robot_vel_msg.x = std::max(robot_vel_msg.x, -MAX_VEL); // 最大でMAX_VEL[m/s]
                }
                if(robot_vel_msg.y > 0){
                    robot_vel_msg.y = std::min(robot_vel_msg.y, MAX_VEL); // 最大でMAX_VEL[m/s]
                }else{
                    robot_vel_msg.y = std::max(robot_vel_msg.y, -MAX_VEL); // 最大でMAX_VEL[m/s]
                }
                prior_error = move_error;
                std::this_thread::sleep_for(10ms);
            }
            loop_rate.sleep();
        }
        for(int i=0; i<TRY_NUM; i++){
            std::cout << "ボールを取得します: " << i << "回目" << std::endl;
            if(detect_ball){ // ボールが検出された場合
                std::cout << "吸着を開始します" << std::endl;
                arm_control_msg.vacuum = true; // 吸着をon
                // ボールの位置に向かってアームを動かす
                prior_error = {0.0, 0.0}; // 一旦リセット
                std::cout << "アームを動かします" << std::endl;
                for(int i = 0; i < ARM_ADJUST_NUM; i++){
                    arm_control_msg.height = HEIGHT - (HEIGHT - CONTROL_HEIGHT)/ARM_ADJUST_NUM*i;
                    arm_control_msg.range += RANGE_P*error.y + RANGE_D*(prior_error.y - error.y);
                    arm_control_msg.angle += ANGLE_P*error.x + ANGLE_D*(prior_error.x - error.x);
                    prior_error = error;
                    rclcpp::sleep_for(50ms);
                }
                arm_control_msg.height = CONTROL_HEIGHT;
                // ボールを取得する
                std::cout << "ボールを取得します" << std::endl;
                arm_control_msg.height = CATCH_HEIGHT + 50;
                rclcpp::sleep_for(500ms);
                while(arm_control_msg.height > CATCH_HEIGHT){
                    std::cout << "arm height: " << arm_control_msg.height << std::endl;
                    arm_control_msg.height -= 0.5;
                    rclcpp::sleep_for(50ms);
                }
                arm_control_msg.height = CATCH_HEIGHT;
                rclcpp::sleep_for(1500ms);
                // ボールが吸着されたかどうか判定する。
                if(!use_vacuum_state){
                    vacuum_state = true;
                }
                arm_control_msg.height = HEIGHT;
                if(vacuum_state){
                    std::cout << "ボールを取得しました" << std::endl;
                    arm_control_msg.height = 300;
                    rclcpp::sleep_for(1000ms);
                    break;
                }else{
                    std::cout << "ボールを取得できませんでした" << std::endl;
                    rclcpp::sleep_for(1s);
                }
            }else{
                // std::cout << "ボールが検出されませんでした" << std::endl;
            }
        }

        async_arm_control_on(false, common_result);
        async_robot_vel_on(false, common_result);
        async_control_trace(false, common_result);
        if(!vacuum_state){
            // ボールを取得できなかった場合
            result->result = false;
            goal_handle->abort(result);
            return;
        }
        if(rclcpp::ok()){ // アクションが成功した場合
            result->result = true;
            goal_handle->succeed(result);
        }
    }

    // trace_ball_nodeに対してボールの検出処理を要求
    void async_control_trace(bool send_data, bool& result){
        auto request = std::make_shared<SetBool::Request>();
        request->data = send_data;
        auto response_received_callback = [this, &result](rclcpp::Client<SetBool>::SharedFuture future) {
            result = future.get()->success;
            std::cout << "[control_trace] レスポンスを受け取りました\nresult: " << result << std::endl;
        };
        auto future_result = control_trace_cli->async_send_request(request);
        std::cout << "[control_trace] リクエストを送信しました" << std::endl;
    }

    // arm_control_nodeに対してアームの制御を要求
    void async_arm_control_on(bool send_data, bool& result){
        auto request = std::make_shared<SetBool::Request>();
        request->data = send_data;
        auto response_received_callback = [this, &result](rclcpp::Client<SetBool>::SharedFuture future) {
            result = future.get()->success;
            std::cout << "[arm_control1_on] レスポンスを受け取りました\nresult: " << result << std::endl;
        };
        auto future_result = arm_control_on_cli->async_send_request(request);
        std::cout << "[arm_control1_on] リクエストを送信しました" << std::endl;
    }

    // robot_vel_nodeに対してロボットの速度制御を要求
    void async_robot_vel_on(bool send_data, bool& result){
        auto request = std::make_shared<SetBool::Request>();
        request->data = send_data;
        auto response_received_callback = [this, &result](rclcpp::Client<SetBool>::SharedFuture future) {
            result = future.get()->success;
            std::cout << "[robot_vel1_on] レスポンスを受け取りました\nresult: " << result << std::endl;
        };
        auto future_result = robot_vel_on_cli->async_send_request(request);
        std::cout << "[robot_vel1_on] リクエストを送信しました" << std::endl;
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<GetBallNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}