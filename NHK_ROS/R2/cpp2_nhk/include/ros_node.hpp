// ライブラリのインクルードなど
// ********************************************************************************************************************
#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <inttypes.h>
#include "std_msgs/msg/bool.hpp"
#include "actuator_custom_msgs/msg/arm_control.hpp"
#include "actuator_custom_msgs/msg/set_position.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "actuator_custom_msgs/srv/get_silo_state.hpp"
#include "actuator_custom_msgs/action/get_ball_coord.hpp"
// #include "actuator_custom_msgs/action/call_line_trace.hpp"
#include "actuator_custom_msgs/action/call_ascend_slope.hpp"
#include "actuator_custom_msgs/action/call_get_ball.hpp"
#include "actuator_custom_msgs/action/call_put_ball.hpp"
#include "actuator_custom_msgs/action/set_move_target.hpp"

using namespace std::chrono_literals;
using Bool = std_msgs::msg::Bool;
using ArmControlMsg = actuator_custom_msgs::msg::ArmControl;
using SetPosition = actuator_custom_msgs::msg::SetPosition;
using SetBool = std_srvs::srv::SetBool;
using GetSiloState = actuator_custom_msgs::srv::GetSiloState;

using GetBallCoord = actuator_custom_msgs::action::GetBallCoord;
using GoalHandleGBC = rclcpp_action::ClientGoalHandle<GetBallCoord>;

// using CallLineTrace = actuator_custom_msgs::action::CallLineTrace;
// using GoalHandleCLT = rclcpp_action::ClientGoalHandle<CallLineTrace>;
using CallAscendSlope = actuator_custom_msgs::action::CallAscendSlope;
using GoalHandleCAS = rclcpp_action::ClientGoalHandle<CallAscendSlope>;

using CallGetBall = actuator_custom_msgs::action::CallGetBall;
using GoalHandleCGB = rclcpp_action::ClientGoalHandle<CallGetBall>;

using CallPutBall = actuator_custom_msgs::action::CallPutBall;
using GoalHandleCPB = rclcpp_action::ClientGoalHandle<CallPutBall>;

using SetMoveTarget = actuator_custom_msgs::action::SetMoveTarget;
using GoalHandleSMT = rclcpp_action::ClientGoalHandle<SetMoveTarget>;

struct Point2D{
    double x;
    double y;
};
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************

// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class BTNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<SetPosition>::SharedPtr set_pantilt_pub;
    rclcpp::Publisher<ArmControlMsg>::SharedPtr arm_control_pub;
    rclcpp::Subscription<Bool>::SharedPtr r2_start_sub;
    rclcpp::Client<GetSiloState>::SharedPtr get_silo_state_cli;
    rclcpp::Client<SetBool>::SharedPtr imu_set_cli;
    rclcpp::Client<SetBool>::SharedPtr arm_control_on_cli;
    // rclcpp_action::Client<CallLineTrace>::SharedPtr call_line_trace_acli;
    rclcpp_action::Client<CallAscendSlope>::SharedPtr call_ascend_slope_acli;
    rclcpp_action::Client<SetMoveTarget>::SharedPtr set_move_target_acli;
    rclcpp_action::Client<CallPutBall>::SharedPtr call_put_ball_acli;
    rclcpp_action::Client<CallGetBall>::SharedPtr call_get_ball_acli;
    rclcpp_action::Client<GetBallCoord>::SharedPtr get_ball_coord_acli;
    bool flag = true;

    public:
    bool is_retry = false;
    bool start_flag = false;
    std::string color = "";
    SetPosition pantilt_pos;
    ArmControlMsg arm_control_msg;
    BTNode() : Node("bt_node"){
        RCLCPP_INFO(this->get_logger(), "bt_node is activated");
        // パラメータの宣言と取得
        auto parameter1 = this->declare_parameter<std::string>("color", "blue");
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }
        auto parameter2 = this->declare_parameter<bool>("is_retry", false);
        if(!this->get_parameter("is_retry", is_retry)){
            std::cout << "is_retry パラメータの取得に失敗" << std::endl;
        }

        auto timer_callback = [this]() -> void {
            set_pantilt_pub->publish(pantilt_pos);
            arm_control_pub->publish(arm_control_msg);
        };

        auto r2_start_callback = [this](const Bool::SharedPtr msg) -> void {
            start_flag = msg->data;
        };

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        set_pantilt_pub = this->create_publisher<SetPosition>("set_pantilt", qos);
        arm_control_pub = this->create_publisher<ArmControlMsg>("arm_control2", qos);
        r2_start_sub = this->create_subscription<Bool>("r2_start", qos, r2_start_callback);
        // call_line_trace_acli = rclcpp_action::create_client<CallLineTrace>(
        //     this->get_node_base_interface(),
        //     this->get_node_graph_interface(),
        //     this->get_node_logging_interface(),
        //     this->get_node_waitables_interface(),
        //     "call_line_trace"
        // );
        call_ascend_slope_acli = rclcpp_action::create_client<CallAscendSlope>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "call_ascend_slope"
        );
        set_move_target_acli = rclcpp_action::create_client<SetMoveTarget>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "set_move_target"
        );
        call_put_ball_acli = rclcpp_action::create_client<CallPutBall>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "call_put_ball"
        );
        call_get_ball_acli = rclcpp_action::create_client<CallGetBall>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "call_get_ball"
        );
        get_ball_coord_acli = rclcpp_action::create_client<GetBallCoord>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "get_ball_coord"
        );
        get_silo_state_cli = this->create_client<GetSiloState>("get_silo_state");
        while(!get_silo_state_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "get_silo_state service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "get_silo_state service is available" << std::endl;
        flag = true;
        imu_set_cli = this->create_client<SetBool>("imu_set");
        while(!imu_set_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "imu_set service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "imu_set service is available" << std::endl;
        flag = true;
        arm_control_on_cli = this->create_client<SetBool>("arm_control2_on");
        while(!arm_control_on_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "arm_control2_on service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "arm_control2_on service is available" << std::endl;
        // 値の初期化
        pantilt_pos.position_0 = 0.0;
        pantilt_pos.position_1 = 0.0;
        arm_control_msg.range = 0.0;
        arm_control_msg.angle = 0.0;
        arm_control_msg.height = 0.0;
        arm_control_msg.vacuum = false;
        timer = this->create_wall_timer(10ms, timer_callback);
    }

    // detect_ball_nodeに対してサイロの状況を要求
    std::vector<int> sync_get_silo_state(void){
        std::vector<int> priorities(5, 0); // サイロの状況を格納する配列
        auto request = std::make_shared<GetSiloState::Request>();
        auto future_result = get_silo_state_cli->async_send_request(request);
        std::cout << "[imu_set] リクエストを送信しました" << std::endl;
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_result) == rclcpp::FutureReturnCode::SUCCESS) {
            priorities[0] = future_result.get()->silo1;
            priorities[1] = future_result.get()->silo2;
            priorities[2] = future_result.get()->silo3;
            priorities[3] = future_result.get()->silo4;
            priorities[4] = future_result.get()->silo5;
            return priorities;
        }
        std::cout << "[imu_set] レスポンスの取得に失敗しました" << std::endl;
        return priorities;
    }

    // imu_pub_nodeに対してimuのリセットを要求
    void async_imu_set(bool send_data, bool& result){
        auto request = std::make_shared<SetBool::Request>();
        request->data = send_data;
        auto response_received_callback = [this, &result](rclcpp::Client<SetBool>::SharedFuture future) {
            result = future.get()->success;
            std::cout << "[imu_set] レスポンスを受け取りました\nresult: " << result << std::endl;
        };
        auto future_result = imu_set_cli->async_send_request(request);
        std::cout << "[imu_set] リクエストを送信しました" << std::endl;
    }

    // arm_control_nodeに対してアームの制御を要求
    void async_arm_control_on(bool send_data, bool& result){
        auto request = std::make_shared<SetBool::Request>();
        request->data = send_data;
        auto response_received_callback = [this, &result](rclcpp::Client<SetBool>::SharedFuture future) {
            result = future.get()->success;
            std::cout << "[arm_control2_on] レスポンスを受け取りました\nresult: " << result << std::endl;
        };
        auto future_result = arm_control_on_cli->async_send_request(request);
        std::cout << "[arm_control2_on] リクエストを送信しました" << std::endl;
    }
    
    // bool send_call_line_trace(bool is_retry){
    //     auto feedback_callback = [this](GoalHandleCLT::SharedPtr, const std::shared_ptr<const CallLineTrace::Feedback> feedback){
    //         float progress = feedback->progress;
    //         RCLCPP_INFO(this->get_logger(), "Progress: %f", progress);
    //     };
    //     using namespace std::placeholders;
    //     if(!this->call_line_trace_acli){ // action clientが利用可能か確認
    //         RCLCPP_ERROR(this->get_logger(), "call_line_trace action client is not available");
    //         return false;
    //     }
    //     if (!this->call_line_trace_acli->wait_for_action_server(10s)){ // action serverが利用可能か確認
    //         RCLCPP_ERROR(this->get_logger(), "call_line_trace action server is not available after waiting");
    //         return false;
    //     }
    //     auto goal = CallLineTrace::Goal(); // actionの目標値を設定
    //     if(is_retry){
    //         goal.mode = 1;
    //     }else{
    //         goal.mode = 0;
    //     }
    //     auto send_goal_options = rclcpp_action::Client<CallLineTrace>::SendGoalOptions();
    //     send_goal_options.feedback_callback = feedback_callback;
    //     auto future_goal_handle = call_line_trace_acli->async_send_goal(goal, send_goal_options);
    //     if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_ERROR(this->get_logger(), "send goal call failed");
    //         return false;
    //     }
    //     GoalHandleCLT::SharedPtr goal_handle = future_goal_handle.get();
    //     if (!goal_handle) {
    //         RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    //         return false;
    //     }
    //     auto result_future = call_line_trace_acli->async_get_result(goal_handle);
    //     if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
    //         RCLCPP_ERROR(this->get_logger(), "get result call failed");
    //         return false;
    //     }
    //     switch (result_future.get().code){
    //         case rclcpp_action::ResultCode::SUCCEEDED:
    //             RCLCPP_INFO(this->get_logger(), "Goal was accepted");
    //             break;
    //         case rclcpp_action::ResultCode::ABORTED:
    //             RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    //             return false;
    //         case rclcpp_action::ResultCode::CANCELED:
    //             RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    //             return false;
    //         default:
    //             RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    //             return false;
    //     }
    //     return result_future.get().result->result;
    // }

    bool send_call_ascend_slope(int direction){
        auto feedback_callback = [this](GoalHandleCAS::SharedPtr, const std::shared_ptr<const CallAscendSlope::Feedback> feedback){
            float progress = feedback->progress;
            RCLCPP_INFO(this->get_logger(), "Progress: %f", progress);
        };
        using namespace std::placeholders;
        if(!this->call_ascend_slope_acli){ // action clientが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "call_ascend_slope action client is not available");
            return false;
        }
        if (!this->call_ascend_slope_acli->wait_for_action_server(10s)){ // action serverが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "call_ascend_slope action server is not available after waiting");
            return false;
        }
        auto goal = CallAscendSlope::Goal(); // actionの目標値を設定
        goal.direction = direction;
        auto send_goal_options = rclcpp_action::Client<CallAscendSlope>::SendGoalOptions();
        send_goal_options.feedback_callback = feedback_callback;
        auto future_goal_handle = call_ascend_slope_acli->async_send_goal(goal, send_goal_options);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "send goal call failed");
            return false;
        }
        GoalHandleCAS::SharedPtr goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }
        auto result_future = call_ascend_slope_acli->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "get result call failed");
            return false;
        }
        switch (result_future.get().code){
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was accepted");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return false;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return false;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return false;
        }
        return result_future.get().result->result;
    }

    // mm単位での移動目標を送信
    bool send_set_move_target(Point2D& point, int erea){
        auto feedback_callback = [this](GoalHandleSMT::SharedPtr, const std::shared_ptr<const SetMoveTarget::Feedback> feedback){
            float progress = feedback->progress;
            RCLCPP_INFO(this->get_logger(), "Progress: %f", progress);
        };
        using namespace std::placeholders;
        if(!this->set_move_target_acli){ // action clientが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "set_move_target action client is not available");
            return false;
        }
        if (!this->set_move_target_acli->wait_for_action_server(10s)){ // action serverが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "set_move_target action server is not available after waiting");
            return false;
        }
        auto goal = SetMoveTarget::Goal(); // actionの目標値を設定
        goal.x = point.x;
        goal.y = point.y;
        goal.erea = erea;
        auto send_goal_options = rclcpp_action::Client<SetMoveTarget>::SendGoalOptions();
        send_goal_options.feedback_callback = feedback_callback;
        auto future_goal_handle = set_move_target_acli->async_send_goal(goal, send_goal_options);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "send goal call failed");
            return false;
        }
        GoalHandleSMT::SharedPtr goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }
        auto result_future = set_move_target_acli->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "get result call failed");
            return false;
        }
        switch (result_future.get().code){
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was accepted");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return false;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return false;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return false;
        }
        return result_future.get().result->result;
    }

    bool send_call_put_ball(int id){
        auto feedback_callback = [this](GoalHandleCPB::SharedPtr, const std::shared_ptr<const CallPutBall::Feedback> feedback){
            float progress = feedback->progress;
            RCLCPP_INFO(this->get_logger(), "Progress: %f", progress);
        };
        using namespace std::placeholders;
        if(!this->call_put_ball_acli){ // action clientが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "call_put_ball action client is not available");
            return false;
        }
        if (!this->call_put_ball_acli->wait_for_action_server(10s)){ // action serverが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "call_put_ball action server is not available after waiting");
            return false;
        }
        auto goal = CallPutBall::Goal(); // actionの目標値を設定
        goal.id = id;
        auto send_goal_options = rclcpp_action::Client<CallPutBall>::SendGoalOptions();
        send_goal_options.feedback_callback = feedback_callback;
        auto future_goal_handle = call_put_ball_acli->async_send_goal(goal, send_goal_options);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "send goal call failed");
            return false;
        }
        GoalHandleCPB::SharedPtr goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }
        auto result_future = call_put_ball_acli->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "get result call failed");
            return false;
        }
        switch (result_future.get().code){
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was accepted");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return false;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return false;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return false;
        }
        return result_future.get().result->result;
    }

    bool send_call_get_ball(Point2D& point){
        auto feedback_callback = [this](GoalHandleCGB::SharedPtr, const std::shared_ptr<const CallGetBall::Feedback> feedback){
            float progress = feedback->progress;
            RCLCPP_INFO(this->get_logger(), "Progress: %f", progress);
        };
        using namespace std::placeholders;
        if(!this->call_get_ball_acli){ // action clientが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "call_get_ball action client is not available");
            return false;
        }
        if (!this->call_get_ball_acli->wait_for_action_server(10s)){ // action serverが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "call_get_ball action server is not available after waiting");
            return false;
        }
        auto goal = CallGetBall::Goal(); // actionの目標値を設定
        goal.x = point.x;
        goal.y = point.y;
        auto send_goal_options = rclcpp_action::Client<CallGetBall>::SendGoalOptions();
        send_goal_options.feedback_callback = feedback_callback;
        auto future_goal_handle = call_get_ball_acli->async_send_goal(goal, send_goal_options);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "send goal call failed");
            return false;
        }
        GoalHandleCGB::SharedPtr goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }
        auto result_future = call_get_ball_acli->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "get result call failed");
            return false;
        }
        switch (result_future.get().code){
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was accepted");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return false;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return false;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return false;
        }
        return result_future.get().result->result;
    }

    Point2D send_get_ball_coord(void){
        Point2D point;
        point.x = -1.0;
        point.y = -1.0;
        auto feedback_callback = [this](GoalHandleGBC::SharedPtr, const std::shared_ptr<const GetBallCoord::Feedback> feedback){
            float progress = feedback->progress;
            RCLCPP_INFO(this->get_logger(), "Progress: %f", progress);
        };
        using namespace std::placeholders;
        if(!this->get_ball_coord_acli){ // action clientが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "get_ball_coord action client is not available");
            return point;
        }
        if (!this->get_ball_coord_acli->wait_for_action_server(10s)){ // action serverが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "get_ball_coord action server is not available after waiting");
            return point;
        }
        auto goal = GetBallCoord::Goal(); // actionの目標値を設定
        auto send_goal_options = rclcpp_action::Client<GetBallCoord>::SendGoalOptions();
        send_goal_options.feedback_callback = feedback_callback;
        auto future_goal_handle = get_ball_coord_acli->async_send_goal(goal, send_goal_options);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "send goal call failed");
            return point;
        }
        GoalHandleGBC::SharedPtr goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return point;
        }
        auto result_future = get_ball_coord_acli->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "get result call failed");
            return point;
        }
        switch (result_future.get().code){
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was accepted");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return point;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return point;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return point;
        }
        point.x = result_future.get().result->x;
        point.y = result_future.get().result->y;
        return point;
    }
};

std::shared_ptr<BTNode> global_node;