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
#include "actuator_custom_msgs/action/call_move_certain.hpp"

using namespace std::chrono_literals;
using Point = geometry_msgs::msg::Point;
using SetBool = std_srvs::srv::SetBool;
using CallMoveCertain = actuator_custom_msgs::action::CallMoveCertain;
using GoalHandleCMC = rclcpp_action::ServerGoalHandle<CallMoveCertain>;

struct Point2D{
    double x;
    double y;
};
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
const double MAX_VEL = 1.0; // 最大速度[m/s]
const float P_VALUE = 0.0015; // P制御の係数
const float I_VALUE = 0.00001; // I制御の係数
const float D_VALUE = 0.008; // D制御の係数
// const float D_VALUE = 0.0; // D制御の係数
const float EPSILON = 10.0; // 許容誤差[mm]
const float MIN_VEL = 0.0; // m/s
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class MoveCertainNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<Point>::SharedPtr robot_vel_pub;
    rclcpp::Subscription<Point>::SharedPtr distance_sub;
    rclcpp::Client<SetBool>::SharedPtr robot_vel2_on_cli;
    rclcpp_action::Server<CallMoveCertain>::SharedPtr call_move_certain_asrv;
    Point robot_vel_msg;
    std::string color = "";
    Point2D distance_data = {0.0, 0.0}; // サブスクライブしたそのままの値
    Point2D old_distance = {0.0, 0.0}; // ある時点の移動距離
    Point2D prior_error = {0.0, 0.0}; // 前回の誤差
    Point2D integral = {0.0, 0.0};
    bool flag = true;
    bool common_result;

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
    MoveCertainNode() : Node("move_certain_node"){
        RCLCPP_INFO(this->get_logger(), "move_certain_node is activated");
        // パラメータの宣言と取得
        auto parameter = this->declare_parameter<std::string>("color", "blue");
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }

        auto timer_callback = [this]() -> void{
            robot_vel_pub->publish(robot_vel_msg);
        };

        auto distance_callback = [this](const Point& msg) -> void{
            // 転がしエンコーダーの補正値を取得
            distance_data.x = msg.x;
            distance_data.y = msg.y;
        };

        auto call_move_certain_goal = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const CallMoveCertain::Goal> goal) -> rclcpp_action::GoalResponse{
            // 目標値を受け取った時の処理
            std::cout << "目標値を受け取りました" << std::endl;
            async_robot_vel2_on(true, common_result);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // ACCEPT_AND_DEFER
        };

        auto call_move_certain_cancel = [this](const std::shared_ptr<GoalHandleCMC> goal_handle) -> rclcpp_action::CancelResponse{
            // キャンセルを受け取ったときの処理
            std::cout << "キャンセルを受け取りました" << std::endl;
            async_robot_vel2_on(false, common_result);
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto call_move_certain_accepted = [this](const std::shared_ptr<GoalHandleCMC> goal_handle) -> void{
            std::cout << "アクションを開始します" << std::endl;
            using namespace std::placeholders;
            std::thread(std::bind(&MoveCertainNode::call_move_certain_execute, this, _1), goal_handle).detach();
        };

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        robot_vel_pub = this->create_publisher<Point>("robot_vel2", qos);
        distance_sub = this->create_subscription<Point>("distance", qos, distance_callback);
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
        robot_vel2_on_cli = this->create_client<SetBool>("robot_vel2_on");
        while(!robot_vel2_on_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "robot_vel2_on service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "robot_vel2_on service is available" << std::endl;
        timer = this->create_wall_timer(10ms, timer_callback);
    }

    void call_move_certain_execute(const std::shared_ptr<GoalHandleCMC> goal_handle){
        // thread内で実行されるcall_move_certainアクションサーバーの本体部分
        auto feedback = std::make_shared<CallMoveCertain::Feedback>();
        auto result = std::make_shared<CallMoveCertain::Result>();
        float range = goal_handle->get_goal()->range; // 移動距離の取得[m]
        int direction = goal_handle->get_goal()->direction; // 方向の取得 0上1右2下3左
        Point2D distance;
        if (direction == 0){
            // 上方向に移動
            std::cout << "上方向に移動します" << std::endl;
            distance.x = 0;
            distance.y = range*1000; // 単位を[m]から[mm]に変換
        }else if (direction == 1){
            // 右方向に移動
            std::cout << "右方向に移動します" << std::endl;
            distance.x = -range*1000;
            distance.y = 0;
        }else if (direction == 2){
            // 下方向に移動
            std::cout << "下方向に移動します" << std::endl;
            distance.x = 0;
            distance.y = -range*1000;
        }else if (direction == 3){
            // 左方向に移動
            std::cout << "左方向に移動します" << std::endl;
            distance.x = range*1000;
            distance.y = 0;
        }else{
            std::cout << "方向が不正です" << std::endl;
            result->result = false;
            goal_handle->abort(result);
            async_robot_vel2_on(false, common_result);
            return;
        }
        std::cout << "移動距離の計測を開始します" << std::endl;
        set_distance();
        Point2D error = {distance.x - get_distance().x, distance.y - get_distance().y};
        // 座標は右下を原点として，左がx軸正，上がy軸正，左側のフィールドではx軸が逆になる
        while(rclcpp::ok()){
            if(goal_handle->is_canceling()){
                std::cout << "アクションがキャンセルされました" << std::endl;
                result->result = false;
                goal_handle->canceled(result);
                async_robot_vel2_on(false, common_result);
                return;
            }
            Point2D error = {distance.x - get_distance().x, distance.y - get_distance().y};
            integral.x += (error.x) * 0.01 / 2;
            integral.y += (error.y) * 0.01 / 2;
            if(sqrt(error.x*error.x + error.y*error.y) < EPSILON){
                break;
            }
            // P制御によってロボットを移動させる
            std::cout << "target: " << distance.x << ", " << distance.y << "\ncurrent: " << get_distance().x << ", " << get_distance().y << std::endl;
            // robot_vel_msg.x = std::min(error.x*P_VALUE + (prior_error.x - error.x)*D_VALUE, MAX_VEL); // 最大でMAX_VEL[m/s]
            // robot_vel_msg.y = std::min(error.y*P_VALUE + (prior_error.y - error.y)*D_VALUE, MAX_VEL); // 最大でMAX_VEL[m/s]
            robot_vel_msg.x = error.x*P_VALUE + integral.x *I_VALUE + (error.x - prior_error.x)*D_VALUE; // 最大でMAX_VEL[m/s]
            robot_vel_msg.y = error.y*P_VALUE + integral.y *I_VALUE + (error.y - prior_error.y)*D_VALUE; // 最大でMAX_VEL[m/s]
            if(robot_vel_msg.x > MAX_VEL){
                robot_vel_msg.x = MAX_VEL;
            }else if(robot_vel_msg.x < -MAX_VEL){
                robot_vel_msg.x = -MAX_VEL;
            }
            if(robot_vel_msg.y > MAX_VEL){
                robot_vel_msg.y = MAX_VEL;
            }else if(robot_vel_msg.y < -MAX_VEL){
                robot_vel_msg.y = -MAX_VEL;
            }
            if(robot_vel_msg.x > 0 && robot_vel_msg.x < MIN_VEL){
                robot_vel_msg.x = MIN_VEL;
            }else if(robot_vel_msg.x < 0 && robot_vel_msg.x > -MIN_VEL){
                robot_vel_msg.x = -MIN_VEL;
            }
            if(robot_vel_msg.y > 0 && robot_vel_msg.y < MIN_VEL){
                robot_vel_msg.y = MIN_VEL;
            }else if(robot_vel_msg.y < 0 && robot_vel_msg.y > -MIN_VEL){
                robot_vel_msg.y = -MIN_VEL;
            }
            prior_error = error;
            std::this_thread::sleep_for(10ms);
        }
        // 十分にバックしたらロボットを停止
        std::cout << "ロボットを停止します" << std::endl;
        robot_vel_msg.x = 0;
        robot_vel_msg.y = 0;
        async_robot_vel2_on(false, common_result);

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
        auto future_result = robot_vel2_on_cli->async_send_request(request, response_received_callback);
        std::cout << "[robot_vel2_on] リクエストを送信しました" << std::endl;
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MoveCertainNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}