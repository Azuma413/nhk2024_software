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
using GoalHandleCMC = rclcpp_action::ClientGoalHandle<CallMoveCertain>;
using CallTakeRice = actuator_custom_msgs::action::CallTakeRice;
using GoalHandleCTR = rclcpp_action::ServerGoalHandle<CallTakeRice>;

class TestNode1 : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<String>::SharedPtr string_pub;
    rclcpp::Service<SetBool>::SharedPtr set_bool_srv;
    rclcpp_action::Client<CallMoveCertain>::SharedPtr call_move_certain_acli;
    rclcpp_action::Server<CallTakeRice>::SharedPtr call_take_rice_asrv;
    int count = 0;

    public:
    TestNode1() : Node("test_node1"){
        RCLCPP_INFO(this->get_logger(), "test_node1 is activated");

        auto timer_callback = [this]() -> void{
            auto msg = String();
            count ++;
            msg.data = "Hello, world! " + std::to_string(count);
            string_pub->publish(msg);
        };

        auto set_bool_callback = [this](const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response) -> void{
            response->success = true;
            std::cout << "SetBoolサービスが呼び出されました" << std::endl;
            response->message = "success";
        };

        auto call_take_rice_goal = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const CallTakeRice::Goal> goal) -> rclcpp_action::GoalResponse{
            std::cout << "目標値を受け取りました" << std::endl;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // ACCEPT_AND_DEFER
        };

        auto call_take_rice_cancel = [this](const std::shared_ptr<GoalHandleCTR> goal_handle) -> rclcpp_action::CancelResponse{
            std::cout << "キャンセルを受け取りました" << std::endl;
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto call_take_rice_accepted = [this](const std::shared_ptr<GoalHandleCTR> goal_handle) -> void{
            std::cout << "アクションを開始します" << std::endl;
            using namespace std::placeholders;
            std::thread(std::bind(&TestNode1::call_take_rice_execute, this, _1), goal_handle).detach();
        };

        string_pub = this->create_publisher<String>("string_pub1", 10);
        set_bool_srv = this->create_service<SetBool>("set_bool", set_bool_callback);
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
        timer = this->create_wall_timer(500ms, timer_callback);
    }

    void call_take_rice_execute(const std::shared_ptr<GoalHandleCTR> goal_handle){
        std::cout << "CallTakeRiceサービスが呼び出されました" << std::endl;
        int mode = goal_handle->get_goal()->mode;
        std::cout << "mode: " << mode << std::endl;
        auto feedback = std::make_shared<CallTakeRice::Feedback>();
        auto result = std::make_shared<CallTakeRice::Result>();
        for (float i = 0; i < 10; i++){
            feedback->progress = i / 10;
            goal_handle->publish_feedback(feedback);
            std::this_thread::sleep_for(1s);
        }
        result->result = true;
        goal_handle->publish_feedback(feedback);
        goal_handle->succeed(result);
    }

    bool send_call_move_certain(float range, int direction){
        // async
        auto feedback_callback = [this](GoalHandleCMC::SharedPtr, const std::shared_ptr<const CallMoveCertain::Feedback> feedback){};

        if(!this->call_move_certain_acli){ // action clientが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "call_move_certain action client は利用可能ではありません");
            return false;
        }
        if (!this->call_move_certain_acli->wait_for_action_server(10s)){ // action serverが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "call_move_certain action server は利用可能ではありません");
            return false;
        }
        auto goal = CallMoveCertain::Goal(); // actionの目標値を設定
        goal.range = range;
        goal.direction = direction;
        auto send_goal_options = rclcpp_action::Client<CallMoveCertain>::SendGoalOptions();
        send_goal_options.feedback_callback = feedback_callback;
        auto future_goal_handle = call_move_certain_acli->async_send_goal(goal, send_goal_options);

        if (future_goal_handle.wait_for(10s) == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Goal was not accepted by server");
            return false;
        }

        GoalHandleCMC::SharedPtr goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }
        auto result_future = call_move_certain_acli->async_get_result(goal_handle);
        if (result_future.wait_for(10s) == std::future_status::ready) {
            try{
                return result_future.get().result->result;
            } catch (const std::exception &e) {
                std::cout << "GetRobotPos ServiceからResultを得られませんでした" << std::endl;
                return false;
            }
        }else{
            std::cout << "Time out" << std::endl;
            return false;
        }
    }
};
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TestNode1>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}