#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "actuator_custom_msgs/action/call_move_certain.hpp"
#include "actuator_custom_msgs/action/call_take_rice.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using SetBool = std_srvs::srv::SetBool;
using String = std_msgs::msg::String;
using CallMoveCertain = actuator_custom_msgs::action::CallMoveCertain;
using GoalHandleCMC = rclcpp_action::ServerGoalHandle<CallMoveCertain>;
using CallTakeRice = actuator_custom_msgs::action::CallTakeRice;
using GoalHandleCTR = rclcpp_action::ClientGoalHandle<CallTakeRice>;

class TestNode2 : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<String>::SharedPtr string_pub;
    rclcpp::Client<SetBool>::SharedPtr set_bool_cli;
    rclcpp_action::Client<CallTakeRice>::SharedPtr call_take_rice_acli;
    rclcpp_action::Server<CallMoveCertain>::SharedPtr call_move_certain_asrv;
    int count = 0;

    public:
    TestNode2() : Node("test_node2"){
        RCLCPP_INFO(this->get_logger(), "test_node2 is activated");

        auto timer_callback = [this]() -> void{
            count ++;
            auto msg = String();
            msg.data = "Hello, world2! " + std::to_string(count);
            string_pub->publish(msg);
        };

        auto call_move_certain_goal = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const CallMoveCertain::Goal> goal) -> rclcpp_action::GoalResponse{
            std::cout << "目標値を受け取りました" << std::endl;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // ACCEPT_AND_DEFER
        };

        auto call_move_certain_cancel = [this](const std::shared_ptr<GoalHandleCMC> goal_handle) -> rclcpp_action::CancelResponse{
            std::cout << "キャンセルを受け取りました" << std::endl;
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto call_move_certain_accepted = [this](const std::shared_ptr<GoalHandleCMC> goal_handle) -> void{
            std::cout << "アクションを開始します" << std::endl;
            using namespace std::placeholders;
            std::thread(std::bind(&TestNode2::call_move_certain_execute, this, _1), goal_handle).detach();
        };

        string_pub = this->create_publisher<String>("string_pub2", 10);
        set_bool_cli = this->create_client<SetBool>("set_bool");
        call_move_certain_asrv = rclcpp_action::create_server<CallMoveCertain>(
            this->get_node_base_interface(), 
            this->get_node_clock_interface(), 
            this->get_node_logging_interface(), 
            this->get_node_waitables_interface(), 
            "call_move_certain", 
            call_move_certain_goal, 
            call_move_certain_cancel, 
            call_move_certain_accepted
        );
        call_take_rice_acli = rclcpp_action::create_client<CallTakeRice>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "call_take_rice"
        );
        timer = this->create_wall_timer(500ms, timer_callback);
    }

    void call_move_certain_execute(const std::shared_ptr<GoalHandleCMC> goal_handle){
        std::cout << "CallMoveCertainサービスが呼び出されました" << std::endl;
        float range = goal_handle->get_goal()->range; // 移動距離の取得
        int direction = goal_handle->get_goal()->direction; // 方向の取得 0上1右2下3左
        std::cout << "range: " << range << std::endl;
        std::cout << "direction: " << direction << std::endl;
        std::cout << "CallTakeRiceサービスを呼び出します" << std::endl;
        bool result_;
        send_call_take_rice(1, result_);
        while(result_ == false){
            std::cout << "CallTakeRiceサービスの結果を待っています" << std::endl;
            std::this_thread::sleep_for(0.5s);
        }
        std::cout << "CallTakeRiceサービスを呼び出しました\nSetBoolサービスを呼び出します" << std::endl;
        async_set_bool(true, result_);
        std::cout << "SetBoolサービスを呼び出しました" << std::endl;
        auto feedback = std::make_shared<CallMoveCertain::Feedback>();
        auto result = std::make_shared<CallMoveCertain::Result>();
        feedback->progress = 2.0;
        result->result = true;
        goal_handle->publish_feedback(feedback);
        goal_handle->succeed(result);
    }

    // 全てこの形態に統一したい
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

    void async_set_bool(bool send_data, bool& myresult){
        // 非同期関数
        auto request = std::make_shared<SetBool::Request>();
        request->data = send_data;
        auto response_received_callback = [this, &myresult](rclcpp::Client<SetBool>::SharedFuture future) {
            myresult = future.get()->success;
            std::cout << "[SetBool] レスポンスを受け取りました\nresult: " << myresult << std::endl;
        };
        auto future_result = set_bool_cli->async_send_request(request, response_received_callback);
        std::cout << "[SetBool] リクエストを送信しました" << std::endl;
    }

    void sync_set_bool(bool send_data, bool& myresult){
        // 同期関数
        auto request = std::make_shared<SetBool::Request>();
        request->data = send_data;
        auto future_result = set_bool_cli->async_send_request(request);
        std::cout << "[SetBool] リクエストを送信しました" << std::endl;
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_result) == rclcpp::FutureReturnCode::SUCCESS){
            // spin_until_future_completeを使っているのでスレッド内では用いないこと
            myresult = future_result.get()->success;
            std::cout << "[SetBool] レスポンスを受け取りました\nresult: " << myresult << std::endl;
        }
    }
};
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TestNode2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}