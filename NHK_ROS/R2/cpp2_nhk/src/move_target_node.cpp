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
#include "actuator_custom_msgs/srv/start_calc_pos.hpp"
#include "actuator_custom_msgs/action/set_move_target.hpp"

using namespace std::chrono_literals;
using Point = geometry_msgs::msg::Point;
using SetBool = std_srvs::srv::SetBool;
using StartCalcPos = actuator_custom_msgs::srv::StartCalcPos;
using SetMoveTarget = actuator_custom_msgs::action::SetMoveTarget;
using GoalHandleSMT = rclcpp_action::ServerGoalHandle<SetMoveTarget>;

struct Point2D{
    double x;
    double y;
};
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
const float P_GAIN = 0.0015; // 速度制御のPゲイン
const float D_GAIN = 0.0003; // 速度制御のDゲイン
const float I_GAIN = 0.00000; // I
const float TARGET_THRESHOLD2 = 100.0; // 目標地点に到達したとみなす閾値[mm]
const float TARGET_THRESHOLD3 = 30.0; // 目標地点に到達したとみなす閾値[mm]
const float MAX_VEL = 1.0; // m/s
const float MIN_VEL = 0.0; // m/s
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class MoveTargetNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<Point>::SharedPtr robot_vel_pub;
    rclcpp::Subscription<Point>::SharedPtr robot_pos_sub;
    rclcpp::Client<SetBool>::SharedPtr robot_vel_on_cli;
    rclcpp::Client<StartCalcPos>::SharedPtr start_calc_pos_cli;
    rclcpp_action::Server<SetMoveTarget>::SharedPtr set_move_target_asrv;
    std::string color = "";
    Point robot_vel_msg;
    Point2D robot_pos; // [mm]
    bool flag = true;
    bool common_result;

    public:
    MoveTargetNode() : Node("move_target_node"){
        RCLCPP_INFO(this->get_logger(), "move_target_node is activated");
        // パラメータの宣言と取得
        auto parameter = this->declare_parameter<std::string>("color", "blue");
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }

        auto timer_callback = [this]() -> void{
            // 定期的に実行される処理
            robot_vel_pub->publish(robot_vel_msg);
        };

        auto robot_pos_callback = [this](const Point::SharedPtr msg) -> void{
            // ロボットの位置を受け取った時の処理
            robot_pos.x = msg->x;
            robot_pos.y = msg->y;
        };

        auto set_move_target_goal = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const SetMoveTarget::Goal> goal) -> rclcpp_action::GoalResponse{
            // 目標値を受け取った時の処理
            async_robot_vel2_on(true, common_result);
            // async_start_calc_pos(true, 2, common_result);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // ACCEPT_AND_DEFER
        };

        auto set_move_target_cancel = [this](const std::shared_ptr<GoalHandleSMT> goal_handle) -> rclcpp_action::CancelResponse{
            // キャンセルを受け取ったときの処理
            std::cout << "キャンセルを受け取りました" << std::endl;
            async_robot_vel2_on(false, common_result);
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto set_move_target_accepted = [this](const std::shared_ptr<GoalHandleSMT> goal_handle) -> void{
            using namespace std::placeholders;
            std::thread(std::bind(&MoveTargetNode::set_move_target_execute, this, _1), goal_handle).detach();
        };

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        robot_vel_pub = this->create_publisher<Point>("robot_vel2", qos);
        robot_pos_sub = this->create_subscription<Point>("robot_pos", qos, robot_pos_callback);
        set_move_target_asrv = rclcpp_action::create_server<SetMoveTarget>(
            this->get_node_base_interface(), 
            this->get_node_clock_interface(), 
            this->get_node_logging_interface(), 
            this->get_node_waitables_interface(), 
            "set_move_target", 
            set_move_target_goal, 
            set_move_target_cancel, 
            set_move_target_accepted
        );
        robot_vel_on_cli = create_client<SetBool>("robot_vel2_on");
        while(!robot_vel_on_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "robot_vel2_on service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "robot_vel2_on service is available" << std::endl;
        flag = true;
        start_calc_pos_cli = create_client<StartCalcPos>("start_calc_pos");
        while(!start_calc_pos_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "start_calc_pos service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "start_calc_pos service is available" << std::endl;
        timer = this->create_wall_timer(10ms, timer_callback);
    }

    void set_move_target_execute(const std::shared_ptr<GoalHandleSMT> goal_handle){
        // thread内で実行される実際の処理部分
        rclcpp::Rate loop_rate(100);
        auto feedback = std::make_shared<SetMoveTarget::Feedback>();
        auto result = std::make_shared<SetMoveTarget::Result>();
        Point2D target_pos;
        float prior_error = 0.0;
        float integral = 0.0;
        target_pos.x = goal_handle->get_goal()->x; // [mm]
        target_pos.y = goal_handle->get_goal()->y;
        common_result = false;
        async_start_calc_pos(true, goal_handle->get_goal()->erea, common_result);
        float target_threshold = 0.0;
        if(goal_handle->get_goal()->erea == 2){
            target_threshold = TARGET_THRESHOLD2;
        }else if(goal_handle->get_goal()->erea == 3){
            target_threshold = TARGET_THRESHOLD3;
        }else{
            std::cout << "invalid erea num input" << std::endl;
            if(goal_handle->is_canceling()){ // 途中でキャンセルされていないか確認
                result->result = false;
                goal_handle->canceled(result);
                async_robot_vel2_on(false, common_result);
                // async_start_calc_pos(false, 0, common_result);
                return;
            }
        }
        // while(!common_result){
        //     loop_rate.sleep();
        // }
        std::cout << "一時停止します" << std::endl;
        robot_vel_msg.x = 0.0;
        robot_vel_msg.y = 0.0;
        rclcpp::sleep_for(500ms);
        while(rclcpp::ok()){
            if(goal_handle->is_canceling()){ // 途中でキャンセルされていないか確認
                result->result = false;
                goal_handle->canceled(result);
                async_robot_vel2_on(false, common_result);
                // async_start_calc_pos(false, 0, common_result);
                return;
            }
            // goal_handle->publish_feedback(feedback);
            // std::cout << "target: " << target_pos.x << ", " << target_pos.y << std::endl;
            // std::cout << "robot : " << robot_pos.x << ", " << robot_pos.y << std::endl;
            float error = sqrt(pow(target_pos.x - robot_pos.x, 2) + pow(target_pos.y - robot_pos.y, 2));
            // std::cout << "error: " << error << std::endl;
            if(error < target_threshold){ // 目標地点に到達したら終了
                break;
            }
            integral += error;
            float vel = std::min(error*P_GAIN + integral*I_GAIN + (error - prior_error)*D_GAIN, MAX_VEL); // [m/s]
            vel = std::max(vel, MIN_VEL);
            float theta = atan2(robot_pos.x - target_pos.x, robot_pos.y - target_pos.y); // ロボットの目標角度を計算
            prior_error = error;
            robot_vel_msg.x = -vel*sin(theta);
            robot_vel_msg.y = -vel*cos(theta);
            loop_rate.sleep();
        }
        robot_vel_msg.x = 0.0;
        robot_vel_msg.y = 0.0;
        async_robot_vel2_on(false, common_result);
        // async_start_calc_pos(false, 0, common_result);
        if(rclcpp::ok()){ // アクションが成功した場合
            result->result = true;
            goal_handle->succeed(result);
        }
    }

    void async_robot_vel2_on(bool send_data, bool& myresult){
        // 非同期関数
        auto request = std::make_shared<SetBool::Request>();
        request->data = send_data;
        auto response_received_callback = [this, &myresult](rclcpp::Client<SetBool>::SharedFuture future) {
            myresult = future.get()->success;
            std::cout << "[robot_vel2_on] レスポンスを受け取りました\nresult: " << myresult << std::endl;
        };
        auto future_result = robot_vel_on_cli->async_send_request(request, response_received_callback);
        std::cout << "[robot_vel2_on] リクエストを送信しました" << std::endl;
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
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MoveTargetNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}