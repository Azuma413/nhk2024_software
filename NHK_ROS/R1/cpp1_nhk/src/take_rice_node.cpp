// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "actuator_custom_msgs/action/call_take_rice.hpp"
#include "actuator_custom_msgs/action/call_move_certain.hpp"
#include "actuator_custom_msgs/msg/set_position.hpp"
#include "actuator_custom_msgs/msg/hand_air_status.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using Bool = std_msgs::msg::Bool;
using SetPosition = actuator_custom_msgs::msg::SetPosition;
using HandAirStatus = actuator_custom_msgs::msg::HandAirStatus;
using CallTakeRice = actuator_custom_msgs::action::CallTakeRice;
using GoalHandleCTR = rclcpp_action::ServerGoalHandle<CallTakeRice>;
using CallMoveCertain = actuator_custom_msgs::action::CallMoveCertain;
using GoalHandleCMC = rclcpp_action::ClientGoalHandle<CallMoveCertain>;

struct Point2D{
    double x;
    double y;
};
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
const float GET_RICE_WIDTH = 0.625; // 稲を取る際に横に動く距離[m]
const float GET_RICE_BACK = 0.2; // 稲を取る際に後ろに動く距離[m]
const float PUT_RICE_WIDTH = 0.5; // 稲を置く際に横に動く距離[m]
const std::vector<float> GET_DXL_POS = {24.0, 24.0}; // 稲を取る際のダイナミクセルの位置[mm] {0:左, 1:右}
const std::vector<float> PUT_DXL_POS = {211.5, -38.5}; // 稲を置く際のダイナミクセルの初期位置[mm] {0:左, 1:右}
const std::vector<float> INIT_DXL_POS = {0.0, 0.0}; // ダイナミクセルの初期位置[rad] {0:左, 1:右}
const float PUT_POS_DIFF = 125.0; // 稲を置く際に，ダイナミクセルをずらす距離[mm]
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class TakeRiceNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<SetPosition>::SharedPtr dxl_command_pub;
    rclcpp::Publisher<HandAirStatus>::SharedPtr mros_input_air_hand_pub;
    rclcpp::Subscription<Bool>::SharedPtr r2_start_sub;
    rclcpp_action::Server<CallTakeRice>::SharedPtr call_take_rice_asrv;
    rclcpp_action::Client<CallMoveCertain>::SharedPtr call_move_certain_acli;
    SetPosition dxl_command_msg;
    HandAirStatus hand_air_status_msg;
    int get_rice_count = 0; // 稲を取った回数をカウント
    int put_rice_count = 0; // 稲を置いた回数をカウント
    std::string color = "";
    bool flag = true;
    bool common_result;

    public:
    TakeRiceNode() : Node("take_rice_node"){
        RCLCPP_INFO(this->get_logger(), "take_rice_node is activated");
        // パラメータの宣言と取得
        auto parameter = this->declare_parameter<std::string>("color", "blue");
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }

        auto timer_callback = [this]() -> void{
            dxl_command_pub->publish(dxl_command_msg);
            mros_input_air_hand_pub->publish(hand_air_status_msg);
        };
        
        auto call_take_rice_goal = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const CallTakeRice::Goal> goal) -> rclcpp_action::GoalResponse{
            // 目標値を受け取った時の処理
            std::cout << "目標値を受け取りました" << std::endl;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // ACCEPT_AND_DEFER
        };

        auto call_take_rice_cancel = [this](const std::shared_ptr<GoalHandleCTR> goal_handle) -> rclcpp_action::CancelResponse{
            // キャンセルを受け取ったときの処理
            std::cout << "キャンセルを受け取りました" << std::endl;
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto call_take_rice_accepted = [this](const std::shared_ptr<GoalHandleCTR> goal_handle) -> void{
            std::cout << "アクションを開始します" << std::endl;
            using namespace std::placeholders;
            std::thread(std::bind(&TakeRiceNode::call_take_rice_execute, this, _1), goal_handle).detach();
        };

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        dxl_command_pub = this->create_publisher<SetPosition>("set_position", qos);
        mros_input_air_hand_pub = this->create_publisher<HandAirStatus>("mros_input_air_hand", qos);
        call_take_rice_asrv = rclcpp_action::create_server<CallTakeRice>(
            this->get_node_base_interface(), 
            this->get_node_clock_interface(), 
            this->get_node_logging_interface(), 
            this->get_node_waitables_interface(), 
            "call_take_rice", 
            call_take_rice_goal, 
            call_take_rice_cancel, 
            call_take_rice_accepted
        );
        call_move_certain_acli = rclcpp_action::create_client<CallMoveCertain>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "call_move_certain"
        );
        //値の初期化
        dxl_command_msg.position_0 = INIT_DXL_POS[0];
        dxl_command_msg.position_1 = INIT_DXL_POS[1];
        hand_air_status_msg.a1 = true;
        hand_air_status_msg.a2 = true;
        hand_air_status_msg.a3 = true;
        hand_air_status_msg.a4 = true;
        hand_air_status_msg.a5 = true;
        hand_air_status_msg.a6 = true;
        timer = this->create_wall_timer(50ms, timer_callback);
    }

    void call_take_rice_execute(const std::shared_ptr<GoalHandleCTR> goal_handle){
        auto feedback = std::make_shared<CallTakeRice::Feedback>();
        auto result = std::make_shared<CallTakeRice::Result>();
        int mode = goal_handle->get_goal()->mode; // モードの取得
        if (mode == 0){ // 稲取得モード
            std::cout << "モード:稲取得" << std::endl;
            if(get_rice_count == 0){
                // ダイナミクセルを動かしてハンドの間隔を調節する
                std::cout << "ハンドの間隔を調節します:稲取得" << std::endl;
                dxl_command_msg.position_0 = GET_DXL_POS[0];
                dxl_command_msg.position_1 = GET_DXL_POS[1];
                // 0.7s待機する
                std::this_thread::sleep_for(700ms);
                // ハンドを開く
                std::cout << "ハンドを開きます" << std::endl;
                hand_air_status_msg.a1 = false;
                hand_air_status_msg.a2 = false;
                hand_air_status_msg.a3 = false;
                hand_air_status_msg.a4 = false;
                hand_air_status_msg.a5 = false;
                hand_air_status_msg.a6 = false;
                get_rice_count++; // 稲を取った回数をカウント
            }else if (get_rice_count == 1){
                // ハンドを閉じる
                std::cout << "ハンドを閉じます" << std::endl;
                hand_air_status_msg.a1 = true;
                hand_air_status_msg.a2 = false;
                hand_air_status_msg.a3 = true;
                hand_air_status_msg.a4 = false;
                hand_air_status_msg.a5 = true;
                hand_air_status_msg.a6 = false;
                common_result = false;
                // ダイナミクセルを動かしてハンドの間隔を調節する(念のため)
                std::cout << "ハンドの間隔を調節します:稲取得" << std::endl;
                dxl_command_msg.position_0 = GET_DXL_POS[0];
                dxl_command_msg.position_1 = GET_DXL_POS[1];
                // 0.7s待機する
                std::this_thread::sleep_for(700ms);
                // 後ろに下がる
                send_call_move_certain(GET_RICE_BACK, 0, common_result);
                while(common_result == false){
                    std::this_thread::sleep_for(10ms);
                }
                // 右に移動する
                common_result = false;
                send_call_move_certain(GET_RICE_WIDTH, 1, common_result);
                while(common_result == false){
                    std::this_thread::sleep_for(10ms);
                }
                // 前に進む
                common_result = false;
                send_call_move_certain(GET_RICE_BACK, 2, common_result);
                while(common_result == false){
                    std::this_thread::sleep_for(10ms);
                }
                get_rice_count++; // 稲を取った回数をカウント
            }else if(get_rice_count == 2){
                std::cout << "3回目の取得です" << std::endl;
                // ハンドを閉じる
                std::cout << "ハンドを閉じます" << std::endl;
                hand_air_status_msg.a1 = true;
                hand_air_status_msg.a2 = true;
                hand_air_status_msg.a3 = true;
                hand_air_status_msg.a4 = true;
                hand_air_status_msg.a5 = true;
                hand_air_status_msg.a6 = true;
                get_rice_count = 0; // 稲を取った回数をリセット
                // 0.7s待機する
                std::this_thread::sleep_for(700ms);
                // 後ろに下がる
                common_result = false;
                send_call_move_certain(GET_RICE_BACK, 0, common_result);
                while(common_result == false){
                    std::this_thread::sleep_for(10ms);
                }
                // ダイナミクセルを動かしてハンドの間隔を調節する(稲を置く位置に移動させる)
                std::cout << "ハンドの間隔を調節します:稲配置" << std::endl;
                dxl_command_msg.position_0 = PUT_DXL_POS[0];
                dxl_command_msg.position_1 = PUT_DXL_POS[1];
            }
        }else if (mode == 1){ // 稲配置モード
            std::cout << "モード:稲配置" << std::endl;
            std::cout << "ハンドの間隔を調節します:稲配置" << put_rice_count << std::endl;
            dxl_command_msg.position_0 = PUT_DXL_POS[0] - PUT_POS_DIFF*put_rice_count;
            dxl_command_msg.position_1 = PUT_DXL_POS[1] + PUT_POS_DIFF*put_rice_count;
            std::cout << "ハンドを開きます" << std::endl;
            if (put_rice_count == 0){
                // ハンドを開く
                hand_air_status_msg.a1 = true;
                hand_air_status_msg.a2 = true;
                hand_air_status_msg.a3 = false;
                hand_air_status_msg.a4 = true;
                hand_air_status_msg.a5 = true;
                hand_air_status_msg.a6 = false;
            }
            if (put_rice_count == 1){
                // ハンドを開く
                hand_air_status_msg.a1 = true;
                hand_air_status_msg.a2 = false;
                hand_air_status_msg.a3 = false;
                hand_air_status_msg.a4 = true;
                hand_air_status_msg.a5 = false;
                hand_air_status_msg.a6 = false;
            }
            if (put_rice_count == 2){
                // ハンドを開く
                hand_air_status_msg.a1 = false;
                hand_air_status_msg.a2 = false;
                hand_air_status_msg.a3 = false;
                hand_air_status_msg.a4 = false;
                hand_air_status_msg.a5 = false;
                hand_air_status_msg.a6 = false;
            }
            if (put_rice_count < 2){
                // 右に移動する
                common_result = false;
                send_call_move_certain(PUT_RICE_WIDTH, 1, common_result);
                rclcpp::sleep_for(500ms);
                dxl_command_msg.position_0 = PUT_DXL_POS[0] - PUT_POS_DIFF*(put_rice_count + 1.0);
                dxl_command_msg.position_1 = PUT_DXL_POS[1] + PUT_POS_DIFF*(put_rice_count + 1.0);
                while(common_result == false){
                    std::this_thread::sleep_for(10ms);
                }
                std::cout << "ハンドの間隔を調節します:稲配置" << put_rice_count + 1 << std::endl;
                put_rice_count++; // 稲を置いた回数をカウント
            }else{
                // 右に移動する
                common_result = false;
                send_call_move_certain(GET_RICE_BACK, 1, common_result);
                rclcpp::sleep_for(500ms);
                // dynamicsellを初期位置に戻す
                dxl_command_msg.position_0 = INIT_DXL_POS[0];
                dxl_command_msg.position_1 = INIT_DXL_POS[1];
                while(common_result == false){
                    std::this_thread::sleep_for(10ms);
                }
                hand_air_status_msg.a1 = true;
                hand_air_status_msg.a2 = true;
                hand_air_status_msg.a3 = true;
                hand_air_status_msg.a4 = true;
                hand_air_status_msg.a5 = true;
                hand_air_status_msg.a6 = true;
                put_rice_count = 0; // 稲を置いた回数をリセット
            }
        }else{ // その他
            std::cout << "モード:不明\nアクションを中断します" << std::endl;
            result->result = false;
            goal_handle->abort(result);
            return;
        }
        if(rclcpp::ok()){ // アクションが成功した場合
            result->result = true;
            goal_handle->succeed(result);
        }
    }

    void send_call_move_certain(float range, int direction, bool& myresult){
        auto goal_response_callback = [this, &myresult](GoalHandleCMC::SharedPtr goal_handle){
            if (goal_handle){
                std::cout << "[CallTakeRice] 目標値がサーバーに受け取られました" << std::endl;
            }else{
                std::cout << "[CallTakeRice] 目標値がサーバーから拒否されました" << std::endl;
                myresult = true;
            }
        };
        auto feedback_callback = [this](GoalHandleCMC::SharedPtr goal_handle, const std::shared_ptr<const CallMoveCertain::Feedback> feedback){
            std::cout << "[CallTakeRice] 進行度: " << feedback->progress << std::endl;
        };
        auto result_callback = [this, &myresult](const GoalHandleCMC::WrappedResult& result){
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED){
                std::cout << "[CallMoveCertain] 結果を受け取りました: " << result.result->result << std::endl;
                myresult = true;
            }else{
                std::cout << "[CallMoveCertain] 目標値がサーバーから拒否されました" << std::endl;
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
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TakeRiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}