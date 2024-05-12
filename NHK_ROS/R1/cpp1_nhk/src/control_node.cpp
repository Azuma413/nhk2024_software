/*
コントローラの値を受け取ってロボットを制御するノード
*/
// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "actuator_custom_msgs/srv/set_table_rad.hpp"
#include "actuator_custom_msgs/action/call_auto_launch.hpp"
#include "actuator_custom_msgs/action/call_take_rice.hpp"
#include "actuator_custom_msgs/action/call_move_certain.hpp"

using namespace std::chrono_literals;
using Point = geometry_msgs::msg::Point;
using SetBool = std_srvs::srv::SetBool;
using Bool = std_msgs::msg::Bool;
using Float64 = std_msgs::msg::Float64;
using Joy = sensor_msgs::msg::Joy;
using SetTableRad = actuator_custom_msgs::srv::SetTableRad;
using CallTakeRice = actuator_custom_msgs::action::CallTakeRice;
using GoalHandleCTR = rclcpp_action::ClientGoalHandle<CallTakeRice>;
using CallAutoLaunch = actuator_custom_msgs::action::CallAutoLaunch;
using GoalHandleCAL = rclcpp_action::ClientGoalHandle<CallAutoLaunch>;
using CallMoveCertain = actuator_custom_msgs::action::CallMoveCertain;
using GoalHandleCMC = rclcpp_action::ClientGoalHandle<CallMoveCertain>;
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
// R1のターンテーブルの可動域は正面を0として-60度から60度まで
const double TABLE_MIN = -M_PI/3;
const double TABLE_MAX = M_PI/3;
const double BASE_VEL_NORMAL = 0.2; // m/s 通常時の足回りの速度
const double BASE_VEL_FAST = 1.0; // m/s Xボタンを押している時の足回りの速度
const double TABLE_ANG_VEL = 0.003; // rad/tick 100hzならTABLE_ANG_VEL*100[rad/s]
const float CERTAIN_RANGE = 0.5; // m 固定距離移動の移動距離
const float BASE_MAX_ANG_VEL = 0.003; // rad/tick 100hzならBASE_MAX_ANG_VEL*100[rad/s]
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class ControlNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<Bool>::SharedPtr r2_start_pub;
    rclcpp::Publisher<Bool>::SharedPtr r2_stop_pub;
    rclcpp::Publisher<Float64>::SharedPtr mros_input_table_pub;
    rclcpp::Publisher<Point>::SharedPtr robot_vel_pub;
    rclcpp::Subscription<Joy>::SharedPtr joy_sub;
    rclcpp::Service<SetTableRad>::SharedPtr set_table_rad_srv;
    rclcpp::Client<SetBool>::SharedPtr imu_set_cli;
    rclcpp_action::Client<CallAutoLaunch>::SharedPtr call_auto_launch_acli;
    rclcpp_action::Client<CallMoveCertain>::SharedPtr call_move_certain_acli;
    rclcpp_action::Client<CallTakeRice>::SharedPtr call_take_rice_acli;
    GoalHandleCAL::SharedPtr auto_launch_gh;
    GoalHandleCMC::SharedPtr move_certain_gh;
    bool r2_start = false;
    bool r2_stop = false;
    bool common_result = false;
    Point robot_vel;
    std::string color = "";
    bool flag = true;
    double table_rad = 0.0;
    int a_button_state = 0; // Aボタンの状態を保持
    int b_button_state = 0; // Bボタンの状態を保持
    int y_button_state = 0; // Yボタンの状態を保持
    int lb_button_state = 0; // LBボタンの状態を保持
    int rb_button_state = 0; // RBボタンの状態を保持
    int back_button_state = 0; // BACKボタンの状態を保持
    int start_button_state = 0; // STARTボタンの状態を保持
    bool auto_launch = false; // 自動発射モードの状態を保持
    bool cross_up = false; // 十字キーの上の状態を保持
    bool cross_down = false; // 十字キーの下の状態を保持
    bool cross_left = false; // 十字キーの左の状態を保持
    bool cross_right = false; // 十字キーの右の状態を保持

    public:
    ControlNode() : Node("control_node"){
        RCLCPP_INFO(this->get_logger(), "control_node is activated");
        // パラメータの宣言と取得
        auto parameter = this->declare_parameter<std::string>("color", "blue");
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }
        auto timer_callback = [this]() -> void{
            Bool msg;
            msg.data = r2_start;
            r2_start_pub->publish(msg);
            msg.data = r2_stop;
            r2_stop_pub->publish(msg);
            robot_vel_pub->publish(robot_vel);
            Float64 table_msg;
            if(table_rad < TABLE_MIN){
                table_rad = TABLE_MIN;
            }else if(table_rad > TABLE_MAX){
                table_rad = TABLE_MAX;
            }
            table_msg.data = table_rad;
            mros_input_table_pub->publish(table_msg);
        };
        
        auto joy_callback = [this](const Joy::SharedPtr msg) -> void{
            /*
            左が1 右が-1
            上が1 下が-1
            msg->axes[0] 左スティック 左右
            msg->axes[1] 左スティック 上下
            msg->axes[2] 右スティック 左右
            msg->axes[3] 右スティック 上下
            msg->axes[4] 十字左右-移動
            msg->axes[5] 十字上下-移動
            msg->buttons[0] X-長押しで高速移動
            msg->buttons[1] A-稲を置く, 自動発射モード中は発射
            msg->buttons[2] B-稲を掴む
            msg->buttons[3] Y-自動発射モードon/of
            msg->buttons[4] LB-反時計回りに90度回転
            msg->buttons[5] RB-時計回りに90度回転
            msg->buttons[6] LT-ターンテーブル反時計回り
            msg->buttons[7] RT-ターンテーブル時計回り
            msg->buttons[8] BACK-R2の動作停止
            msg->buttons[9] START-IMUリセット,R2の動作開始
            msg->buttons[10]
            msg->buttons[11]
            */
            // 足回りの回転
            robot_vel.z += msg->axes[2]*BASE_MAX_ANG_VEL;
            if(msg->buttons[0] == 1){ // 高速移動
                robot_vel.x = BASE_VEL_FAST * msg->axes[0];
                robot_vel.y = BASE_VEL_FAST * msg->axes[1];
            }else{ // 通常移動
                robot_vel.x = BASE_VEL_NORMAL * msg->axes[0];
                robot_vel.y = BASE_VEL_NORMAL * msg->axes[1];
            }
            // 特定距離移動
            if(!cross_up and msg->axes[5] > 0.5){ // 上
                auto future_cancel = call_move_certain_acli->async_cancel_all_goals();
                send_call_move_certain(CERTAIN_RANGE, 0, common_result);
                cross_up = true;
            }else if(!cross_right and msg->axes[4] < -0.5){ // 右
                auto future_cancel = call_move_certain_acli->async_cancel_all_goals();
                send_call_move_certain(CERTAIN_RANGE, 1, common_result);
                cross_right = true;
            }else if(!cross_down and msg->axes[5] < -0.5){ // 下
                auto future_cancel = call_move_certain_acli->async_cancel_all_goals();
                send_call_move_certain(CERTAIN_RANGE, 2, common_result);
                cross_down = true;
            }else if(!cross_left and msg->axes[4] > 0.5){ // 左
                auto future_cancel = call_move_certain_acli->async_cancel_all_goals();
                send_call_move_certain(CERTAIN_RANGE, 3, common_result);
                cross_left = true;
            }
            if(cross_up and msg->axes[5] == 0){ // 十字キーの上が離されたとき
                cross_up = false;
            }
            if(cross_right and msg->axes[4] == 0){ // 十字キーの右が離されたとき
                cross_right = false;
            }
            if(cross_down and msg->axes[5] == 0){ // 十字キーの下が離されたとき
                cross_down = false;
            }
            if(cross_left and msg->axes[4] == 0){ // 十字キーの左が離されたとき
                cross_left = false;
            }
            // 自動発射モードの切り替え
            if(y_button_state != msg->buttons[3]){ // Yボタンの状態が切り替わったとき
                if(msg->buttons[3] == 1){
                    auto_launch = !auto_launch;
                    if (auto_launch){
                        std::cout << "auto_launch on" << std::endl;
                        send_call_auto_launch(common_result);
                    } else {
                        std::cout << "auto_launch off" << std::endl;
                        auto future_cancel = call_auto_launch_acli->async_cancel_all_goals();
                    }
                }
                y_button_state = msg->buttons[3];
            }
            // 稲を置く
            if(a_button_state != msg->buttons[1]){ // Aボタンの状態が切り替わったとき
                if(msg->buttons[1] == 1){
                    if(!auto_launch){
                        std::cout << "put rice" << std::endl;
                        send_call_take_rice(1, common_result); // 稲を置くのはmode=1
                    }
                }
                a_button_state = msg->buttons[1];
            }
            // 稲を掴む
            if(b_button_state != msg->buttons[2]){ // Bボタンの状態が切り替わったとき
                if(msg->buttons[2] == 1){
                    std::cout << "get rice" << std::endl;
                    send_call_take_rice(0, common_result); // 稲を掴むのはmode=0
                }
                b_button_state = msg->buttons[2];
            }
            // ロボットの回転
            if(lb_button_state != msg->buttons[4]){ // LBボタンの状態が切り替わったとき
                if(msg->buttons[4] == 1){
                    std::cout << "turn left" << std::endl;
                    robot_vel.z += M_PI/2;
                }
                lb_button_state = msg->buttons[4];
            }
            if(rb_button_state != msg->buttons[5]){ // RBボタンの状態が切り替わったとき
                if(msg->buttons[5] == 1){
                    std::cout << "turn right" << std::endl;
                    robot_vel.z -= M_PI/2;
                }
                rb_button_state = msg->buttons[5];
            }
            // ターンテーブルの回転
            table_rad += TABLE_ANG_VEL * (msg->buttons[7] - msg->buttons[6]);
            // R2の動作停止
            if (back_button_state != msg->buttons[8]){ // BACKボタンの状態が切り替わったとき
                if(msg->buttons[8] == 1){
                    std::cout << "r2_stop" << std::endl;
                    r2_stop = true;
                }else{
                    r2_stop = false;
                }
                back_button_state = msg->buttons[8];
            }
            // R2の動作開始とIMUリセット
            if (start_button_state != msg->buttons[9]){ // STARTボタンの状態が切り替わったとき
                if(msg->buttons[9] == 1){
                    std::cout << "r2_start" << std::endl;
                    r2_start = true;
                    bool send_data = true;
                    std::cout << "imu_set" << std::endl;
                    async_imu_set(send_data, common_result);
                    rclcpp::sleep_for(100ms);
                    robot_vel.z = 0.0;
                }
                start_button_state = msg->buttons[9];
            }
        };

        auto set_table_rad_callback = [this](const std::shared_ptr<SetTableRad::Request> request, std::shared_ptr<SetTableRad::Response> response) -> void{
            if(TABLE_MIN < request->rad and request->rad < TABLE_MAX){
                table_rad = request->rad;
                response->result = true;
            }else{
                response->result = false;
            }
        };

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        r2_start_pub = this->create_publisher<Bool>("r2_start", qos);
        r2_stop_pub = this->create_publisher<Bool>("r2_stop", qos);
        robot_vel_pub = this->create_publisher<Point>("robot_vel1", qos);
        mros_input_table_pub = this->create_publisher<Float64>("mros_input_table", qos);
        joy_sub = this->create_subscription<Joy>("joy", qos, joy_callback);
        set_table_rad_srv = this->create_service<SetTableRad>("set_table_rad", set_table_rad_callback);
        call_take_rice_acli = rclcpp_action::create_client<CallTakeRice>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "call_take_rice"
        );
        call_auto_launch_acli = rclcpp_action::create_client<CallAutoLaunch>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "call_auto_launch"
        );
        call_move_certain_acli = rclcpp_action::create_client<CallMoveCertain>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "call_move_certain"
        );
        imu_set_cli = this->create_client<SetBool>("imu_set");
        while(!imu_set_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "imu_set service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "imu_set service is ready" << std::endl;
        timer = this->create_wall_timer(10ms, timer_callback);
    }

    void send_call_take_rice(int mode, bool& myresult){
        auto goal_response_callback = [this, &myresult](GoalHandleCTR::SharedPtr goal_handle){
            if (goal_handle){
                std::cout << "[CallTakeRice] 目標値がサーバーに受け取られました" << std::endl;
            }else{
                std::cout << "[CallTakeRice] 目標値がサーバーから拒否されました" << std::endl;
                myresult = true;
            }
        };
        auto feedback_callback = [this](GoalHandleCTR::SharedPtr goal_handle, const std::shared_ptr<const CallTakeRice::Feedback> feedback){
            std::cout << "[CallTakeRice] 進行度: " << feedback->progress << std::endl;
        };
        auto result_callback = [this, &myresult](const GoalHandleCTR::WrappedResult& result){
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED){
                std::cout << "[CallTakeRice] 結果を受け取りました: " << result.result->result << std::endl;
                myresult = true;
            }else{
                std::cout << "[CallTakeRice] 目標値がサーバーから拒否されました" << std::endl;
                myresult = true;
            }
        };
        if(!this->call_take_rice_acli){ // action clientが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "[CallTakeRice] クライアントは利用可能ではありません");
            myresult = true;
            return;
        }
        if (!this->call_take_rice_acli->wait_for_action_server(10s)){ // action serverが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "[CallTakeRice] サーバーは利用可能ではありません");
            myresult = true;
            return;
        }
        auto goal = CallTakeRice::Goal(); // actionの目標値を設定
        goal.mode = mode;
        auto send_goal_options = rclcpp_action::Client<CallTakeRice>::SendGoalOptions();
        send_goal_options.goal_response_callback = goal_response_callback;
        send_goal_options.feedback_callback = feedback_callback;
        send_goal_options.result_callback = result_callback;
        auto future_goal_handle = call_take_rice_acli->async_send_goal(goal, send_goal_options);
        std::cout << "[CallTakeRice] リクエストを送信しました" << std::endl;
    }

    void send_call_auto_launch(bool& myresult){
        auto goal_response_callback = [this, &myresult](GoalHandleCAL::SharedPtr goal_handle){
            if (goal_handle) {
                std::cout << "[CallAutoLaunch] 目標値がサーバーに受け取られました" << std::endl;
            } else {
                std::cout << "[CallAutoLaunch] 目標値はサーバーによって拒否されました" << std::endl;
                myresult = true;
            }
            this->auto_launch_gh = goal_handle; // goal_handleを保持
        };
        auto feedback_callback = [this](GoalHandleCAL::SharedPtr, const std::shared_ptr<const CallAutoLaunch::Feedback> feedback){};
        auto result_callback = [this, &myresult](const GoalHandleCAL::WrappedResult & result){
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED){
                std::cout << "[CallAutoLaunch] 結果を受け取りました: " << result.result->result << std::endl;
                myresult = true;
            } else {
                std::cout << "[CallAutoLaunch] 結果を受け取りました: " << result.result->result << std::endl;
                myresult = true;
            }
        };
        if(!this->call_auto_launch_acli){ // action clientが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "[CallAutoLaunch] クライアントは利用可能ではありません");
            myresult = true;
            return;
        }
        if (!this->call_auto_launch_acli->wait_for_action_server(10s)){ // action serverが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "[CallAutoLaunch] サーバーは利用可能ではありません");
            myresult = true;
            return;
        }
        auto goal = CallAutoLaunch::Goal(); // actionの目標値を設定
        auto send_goal_options = rclcpp_action::Client<CallAutoLaunch>::SendGoalOptions();
        send_goal_options.goal_response_callback = goal_response_callback;
        send_goal_options.feedback_callback = feedback_callback;
        send_goal_options.result_callback = result_callback;
        auto future_goal_handle = call_auto_launch_acli->async_send_goal(goal, send_goal_options);
        std::cout << "[CallAutoLaunch] リクエストを送信しました" << std::endl;
    }

    void send_call_move_certain(float range, int direction, bool& myresult){
        auto goal_response_callback = [this, &myresult](GoalHandleCMC::SharedPtr goal_handle){
            if (goal_handle){
                std::cout << "[CallMoveCertain] 目標値がサーバーに受け取られました" << std::endl;
            }else{
                std::cout << "[CallMoveCertain] 目標値がサーバーから拒否されました" << std::endl;
                myresult = true;
            }
            this->move_certain_gh = goal_handle;
        };
        auto feedback_callback = [this](GoalHandleCMC::SharedPtr goal_handle, const std::shared_ptr<const CallMoveCertain::Feedback> feedback){
            std::cout << "[CallMoveCertain] 進行度: " << feedback->progress << std::endl;
        };
        auto result_callback = [this, &myresult](const GoalHandleCMC::WrappedResult& result){
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED){
                std::cout << "[CallMoveCertain] 結果を受け取りました: " << result.result->result << std::endl;
                myresult = true;
            }else{
                std::cout << "[CallMoveCertain] 結果を受け取りました: " << result.result->result << std::endl;
                myresult = true;
            }
        };
        if(!this->call_move_certain_acli){ // action clientが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "[CallMoveCertain] クライアントは利用可能ではありません");
            myresult = true;
            return;
        }
        if (!this->call_move_certain_acli->wait_for_action_server(10s)){ // action serverが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "[CallMoveCertain] サーバーは利用可能ではありません");
            myresult = true;
            return;
        }
        auto goal = CallMoveCertain::Goal(); // actionの目標値を設定
        goal.range = range;
        goal.direction = direction;
        auto send_goal_options = rclcpp_action::Client<CallMoveCertain>::SendGoalOptions();
        send_goal_options.goal_response_callback = goal_response_callback;
        send_goal_options.feedback_callback = feedback_callback;
        send_goal_options.result_callback = result_callback;
        auto future_goal_handle = call_move_certain_acli->async_send_goal(goal, send_goal_options);
        std::cout << "[CallMoveCertain] リクエストを送信しました" << std::endl;
    }

    void async_imu_set(bool& send_data, bool& myresult){
        auto request = std::make_shared<SetBool::Request>();
        request->data = send_data;
        auto response_received_callback = [this, &myresult](rclcpp::Client<SetBool>::SharedFuture future) {
            myresult = future.get()->success;
            std::cout << "[imu_set] レスポンスを受け取りました\nresult: " << myresult << std::endl;
            return;
        };
        auto future_result = imu_set_cli->async_send_request(request, response_received_callback);
        std::cout << "[imu_set] リクエストを送信しました" << std::endl;
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}