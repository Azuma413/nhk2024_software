// range 280
// height 770

// ライブラリのインクルードなど
// 非同期を前提としてサービス通信内でループを使用できないかテストする。
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "actuator_custom_msgs/msg/arm_control.hpp"
#include "actuator_custom_msgs/action/call_put_ball.hpp"
#include "actuator_custom_msgs/action/set_move_target.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;
using SetBool = std_srvs::srv::SetBool;
using Point = geometry_msgs::msg::Point;
using ArmControl = actuator_custom_msgs::msg::ArmControl;
using CallPutBall = actuator_custom_msgs::action::CallPutBall;
using GoalHandleCPB = rclcpp_action::ServerGoalHandle<CallPutBall>;
using SetMoveTarget = actuator_custom_msgs::action::SetMoveTarget;
using GoalHandleSMT = rclcpp_action::ClientGoalHandle<SetMoveTarget>;

struct Point2D{
    double x;
    double y;
};
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
const float RANGE = 220; // mm bt_nodeによって設定されるボールを投入する際のアームの半径方向の長さ
const float HEIGHT = 740; // mm bt_nodeによって設定されるボールを投入する際のアームの高さ >715
const std::vector<Point2D> SILO_POS = {
    {6000 - 235 - RANGE, 500}, // id: 0
    {6000 - 235 - RANGE, 1250},
    {6000 - 235 - RANGE, 2000},
    {6000 - 235 - RANGE, 2750},
    {6000 - 235 - RANGE, 3500} // id: 4
};
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class PutBallNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<Point>::SharedPtr robot_vel_pub;
    rclcpp::Publisher<ArmControl>::SharedPtr arm_control_pub;
    rclcpp::Client<SetBool>::SharedPtr robot_vel_on_cli;
    rclcpp::Client<SetBool>::SharedPtr arm_control_on_cli;
    rclcpp_action::Server<CallPutBall>::SharedPtr call_put_ball_asrv;
    rclcpp_action::Client<SetMoveTarget>::SharedPtr set_move_target_acli;
    ArmControl arm_control_msg;
    Point robot_vel_msg;
    std::string color = "";
    bool flag = true;
    bool common_result;

    public:
    PutBallNode() : Node("put_ball_node"){
        RCLCPP_INFO(this->get_logger(), "put_ball_node is activated");
        // パラメータの宣言と取得
        auto parameter = this->declare_parameter<std::string>("color", "blue");
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }

        auto timer_callback = [this]() -> void{
            arm_control_pub->publish(arm_control_msg);
            robot_vel_pub->publish(robot_vel_msg);
        };

        auto call_put_ball_goal = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const CallPutBall::Goal> goal) -> rclcpp_action::GoalResponse{
            // 目標値を受け取った時の処理
            std::cout << "目標値を受け取りました" << std::endl;
            arm_control_msg.angle = M_PI/2;
            arm_control_msg.height = HEIGHT;
            arm_control_msg.range = RANGE;
            arm_control_msg.vacuum = true;
            async_arm_control_on(true, common_result);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // ACCEPT_AND_DEFER
        };

        auto call_put_ball_cancel = [this](const std::shared_ptr<GoalHandleCPB> goal_handle) -> rclcpp_action::CancelResponse{
            // キャンセルを受け取ったときの処理
            std::cout << "キャンセルを受け取りました" << std::endl;
            async_robot_vel5_on(false, common_result);
            async_arm_control_on(false, common_result);
            return rclcpp_action::CancelResponse::ACCEPT;
        };
        
        auto call_put_ball_accepted = [this](const std::shared_ptr<GoalHandleCPB> goal_handle) -> void{
            using namespace std::placeholders;
            std::thread(std::bind(&PutBallNode::call_put_ball_execute, this, _1), goal_handle).detach();
        };
        
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        robot_vel_pub = this->create_publisher<Point>("robot_vel5", qos);
        arm_control_pub = this->create_publisher<ArmControl>("arm_control3", qos);
        call_put_ball_asrv = rclcpp_action::create_server<CallPutBall>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "call_put_ball",
            call_put_ball_goal,
            call_put_ball_cancel,
            call_put_ball_accepted
        );
        set_move_target_acli = rclcpp_action::create_client<SetMoveTarget>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "set_move_target"
        );
        arm_control_on_cli = create_client<SetBool>("arm_control3_on");
        while(!arm_control_on_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "arm_control3_on service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "arm_control3_on service is available" << std::endl;
        flag = true;
        robot_vel_on_cli = create_client<SetBool>("robot_vel5_on");
        while(!robot_vel_on_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "robot_vel5_on service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "robot_vel5_on service is available" << std::endl;

        // arm_control_msgの初期化
        arm_control_msg.angle = M_PI/2;
        arm_control_msg.height = HEIGHT;
        arm_control_msg.range = RANGE;
        arm_control_msg.vacuum = true;
        timer = this->create_wall_timer(10ms, timer_callback);
    }

    void call_put_ball_execute(const std::shared_ptr<GoalHandleCPB> goal_handle){
        rclcpp::Rate loop_rate(100); // 100Hz
        auto feedback = std::make_shared<CallPutBall::Feedback>();
        auto result = std::make_shared<CallPutBall::Result>();
        Point2D target = SILO_POS[goal_handle->get_goal()->id]; // idが不正だった場合の処理を追加する
        std::cout << "目標サイロ: ID" << goal_handle->get_goal()->id << std::endl;
        // 目標位置まで移動
        bool smt_result = false;
        send_set_move_target(target, smt_result);
        while(!smt_result){
            loop_rate.sleep();
        }
        std::cout << "目標位置まで移動しました" << std::endl;
        async_robot_vel5_on(true, common_result);
        arm_control_msg.vacuum = false;
        rclcpp::sleep_for(500ms);
        // ボールをサイロに入れる
        std::cout << "back start" << std::endl;
        while(arm_control_msg.height > HEIGHT - 130){
            if(arm_control_msg.height < HEIGHT - 30){ // 70
                robot_vel_msg.x = -0.15;
                robot_vel_msg.y = 0.0;
            }
            arm_control_msg.height -= 5;
            arm_control_msg.range += 7;
            rclcpp::sleep_for(50ms);
        }
        rclcpp::sleep_for(400ms);
        robot_vel_msg.x = -0.3;
        robot_vel_msg.y = 0.0;
        rclcpp::sleep_for(2s);
        robot_vel_msg.x = 0.0;
        robot_vel_msg.y = 0.0;
        std::cout << "back end" << std::endl;
        async_arm_control_on(false, common_result);
        async_robot_vel5_on(false, common_result);
        if(rclcpp::ok()){ // アクションが成功した場合
            result->result = true;
            goal_handle->succeed(result);
        }
    }

    void async_arm_control_on(bool send_data, bool& result){
        // arm_control_nodeに対してアームの制御を要求
        auto request = std::make_shared<SetBool::Request>();
        request->data = send_data;
        auto response_received_callback = [this, &result](rclcpp::Client<SetBool>::SharedFuture future) {
            result = future.get()->success;
            std::cout << "[arm_control3_on] レスポンスを受け取りました\nresult: " << result << std::endl;
        };
        auto future_result = arm_control_on_cli->async_send_request(request);
        std::cout << "[arm_control3_on] リクエストを送信しました" << std::endl;
    }

    void async_robot_vel5_on(bool send_data, bool& myresult){
        // 非同期関数
        auto request = std::make_shared<SetBool::Request>();
        request->data = send_data;
        auto response_received_callback = [this, &myresult](rclcpp::Client<SetBool>::SharedFuture future) {
            myresult = future.get()->success;
            std::cout << "[robot_vel5_on] レスポンスを受け取りました\nresult: " << myresult << std::endl;
        };
        auto future_result = robot_vel_on_cli->async_send_request(request, response_received_callback);
        std::cout << "[robot_vel5_on] リクエストを送信しました" << std::endl;
    }

    void send_set_move_target(Point2D target, bool& myresult){
        auto goal_response_callback = [this, &myresult](GoalHandleSMT::SharedPtr goal_handle){
            if (goal_handle){
                std::cout << "[SetMoveTarget] 目標値がサーバーに受け取られました" << std::endl;
            }else{
                std::cout << "[SetMoveTarget] 目標値がサーバーから拒否されました" << std::endl;
                myresult = true;
            }
        };
        auto feedback_callback = [this](GoalHandleSMT::SharedPtr goal_handle, const std::shared_ptr<const SetMoveTarget::Feedback> feedback){
            std::cout << "[SetMoveTarget] 進行度: " << feedback->progress << std::endl;
        };
        auto result_callback = [this, &myresult](const GoalHandleSMT::WrappedResult& result){
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED){
                std::cout << "[SetMoveTarget] 結果を受け取りました: " << result.result->result << std::endl;
                myresult = true;
            }else{
                std::cout << "[SetMoveTarget] 目標値がサーバーから拒否されました" << std::endl;
                myresult = true;
            }
        };
        if(!this->set_move_target_acli){ // action clientが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "[SetMoveTarget] クライアントは利用可能ではありません");
            myresult = true;
            return;
        }
        if (!this->set_move_target_acli->wait_for_action_server(10s)){ // action serverが利用可能か確認
            RCLCPP_ERROR(this->get_logger(), "[SetMoveTarget] サーバーは利用可能ではありません");
            myresult = true;
            return;
        }
        auto goal = SetMoveTarget::Goal(); // actionの目標値を設定
        goal.x = target.x;
        goal.y = target.y;
        goal.erea = 3;
        auto send_goal_options = rclcpp_action::Client<SetMoveTarget>::SendGoalOptions();
        send_goal_options.goal_response_callback = goal_response_callback;
        send_goal_options.feedback_callback = feedback_callback;
        send_goal_options.result_callback = result_callback;
        auto future_goal_handle = set_move_target_acli->async_send_goal(goal, send_goal_options);
        std::cout << "[SetMoveTarget] リクエストを送信しました" << std::endl;
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PutBallNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
