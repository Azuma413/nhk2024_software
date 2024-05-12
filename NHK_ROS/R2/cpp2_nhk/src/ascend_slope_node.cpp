// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "actuator_custom_msgs/action/call_ascend_slope.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using Point = geometry_msgs::msg::Point;
using CallAscendSlope = actuator_custom_msgs::action::CallAscendSlope;
using GoalHandleCAS = rclcpp_action::ServerGoalHandle<CallAscendSlope>;
using SetBool = std_srvs::srv::SetBool;
using Bool = std_msgs::msg::Bool;

struct Point2D{ // 座標を表す構造体
    double x;
    double y;
};

// struct TraceCommand{ // ライントレースのコマンドを表す構造体
//     int direction; // 0: 直進，1: 右，2: 後退，3: 左
//     double distance; // 移動距離 m
//     double speed = 2.0; // 最大速度 m/s
//     bool unuse_line_sensor = false; // ラインセンサーを使うかどうか(未実装)
//     bool use_imu_for_slope =false;
// };
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
const double FLAT_VEL = 1.0;
const double SLOPE_VEL = 1.0;

// const double MAX_VEL = 1.0; // 最大速度[m/s]
// const float P_VALUE = 1.0;//0.001; // P制御の係数
// const float D_VALUE = 0.005; // D制御の係数
// //const float EPSILON = 30.0; // 許容誤差[mm]
// const float EPSILON = 0.03; // 許容誤差[m]
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class AscendSlopeNode : public rclcpp::Node{
    private:
    // メンバ変数の宣言
    // ROS用
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<Point>::SharedPtr robot_vel_pub;
    rclcpp::Subscription<Point>::SharedPtr distance_sub;
    rclcpp::Subscription<Bool>::SharedPtr is_slope_sub;
    rclcpp::Client<SetBool>::SharedPtr robot_vel_on_cli;
    rclcpp_action::Server<CallAscendSlope>::SharedPtr call_ascend_slope_asrv;
    // パラメータから受け取ったフィールド情報を格納する変数
    std::string color = "";
    // その他の変数
    bool flag = true; // サービスが利用可能かどうかのフラグ(使いまわし)
    bool common_result; // サービスの結果を格納する変数(使いまわし)
    bool is_slope_data = false; // 傾斜を検出したかどうかのフラグ
    Point robot_vel_msg; // ロボットの速度を格納するメッセージ。値を入れるとtimer_callback内でpublishされる
    // 転がしエンコーダ用関数に必要な変数
    Point2D distance_data = {0.0, 0.0}; // サブスクライブしたそのままの値
    Point2D old_distance = {0.0, 0.0}; // ある時点の移動距離
    Point2D prior_error = {0.0, 0.0}; // 前回の誤差
    // 転がしエンコーダの計測を開始する関数
    void set_distance(void){
        old_distance = distance_data;
    }
    // 転がしエンコーダの計測結果を取得する関数
    Point2D get_distance(void){
        Point2D distance;
        distance.x = distance_data.x - old_distance.x;
        distance.y = distance_data.y - old_distance.y;
        return distance;
    }
    int call_count = 0;

    public:
    AscendSlopeNode() : Node("ascend_slope_node"){
        RCLCPP_INFO(this->get_logger(), "ascend_slope_node is activated");
        // パラメータの宣言と取得
        auto parameter = this->declare_parameter<std::string>("color", "blue");
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }

        // n[ms]ごとに呼び出されるコールバック関数
        auto timer_callback = [this]() -> void{
            robot_vel_pub->publish(robot_vel_msg);
        };

        // 転がしエンコーダの値を取得するコールバック関数
        auto distance_callback = [this](const Point& msg) -> void{
            distance_data.x = msg.x;
            distance_data.y = msg.y;
        };

        auto is_slope_callback = [this](const Bool& msg) -> void{
            is_slope_data = msg.data;
        };

        // アクションサーバのコールバック関数1：クライアントから目標値を受け取ったタイミングで実行される
        auto call_ascend_slope_goal = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const CallAscendSlope::Goal> goal) -> rclcpp_action::GoalResponse{
            // 目標値を受け取った時の処理
            std::cout << "目標値を受け取りました" << std::endl;
            async_robot_vel3_on(true, common_result);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // ACCEPT_AND_DEFER
        };

        // アクションサーバのコールバック関数2：クライアントからキャンセルを受け取ったタイミングで実行される
        auto call_ascend_slope_cancel = [this](const std::shared_ptr<GoalHandleCAS> goal_handle) -> rclcpp_action::CancelResponse{
            // キャンセルを受け取ったときの処理
            std::cout << "キャンセルを受け取りました" << std::endl;
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        // アクションサーバのコールバック関数3：アクションを開始するタイミングで実行される
        auto call_ascend_slope_accepted = [this](const std::shared_ptr<GoalHandleCAS> goal_handle) -> void{
            // 目標値に基づいてアクションを開始する（別のスレッドでcall_ascend_slope_execute関数を実行）
            using namespace std::placeholders;
            std::cout << "アクションを開始します" << std::endl;
            std::thread{std::bind(&AscendSlopeNode::call_ascend_slope_execute, this, _1), goal_handle}.detach(); // メンバ関数をbindする際には，関数名の前にクラス名を明示的に指定する
        };

        // ノードの初期化 ノード同士の接続についてはメインのReadMeを参照
        rclcpp::QoS qos(rclcpp::KeepLast(10)); // QOSの設定
        robot_vel_pub = this->create_publisher<Point>("robot_vel3", qos);
        distance_sub = this->create_subscription<Point>("distance", qos, distance_callback);
        is_slope_sub = this->create_subscription<Bool>("is_slope", qos, is_slope_callback);
        call_ascend_slope_asrv = rclcpp_action::create_server<CallAscendSlope>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "call_ascend_slope",
            call_ascend_slope_goal,
            call_ascend_slope_cancel,
            call_ascend_slope_accepted
        );
        robot_vel_on_cli = this->create_client<SetBool>("robot_vel3_on");
        while(!robot_vel_on_cli->wait_for_service(1s) && rclcpp::ok()){ // サービスが利用可能になるまで待機
            if(flag){
                std::cout << "robot_vel3_on service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "robot_vel3_on service is available" << std::endl;
        timer = this->create_wall_timer(10ms, timer_callback);
    } // コンストラクタここまで

    // CallAscendSlopeアクションサーバの実行関数。ここに機能を実装している
    void call_ascend_slope_execute(const std::shared_ptr<GoalHandleCAS>& goal_handle){
        // thread内で実行される実際の処理部分
        call_count++;
        rclcpp::Rate loop_rate(100); // ループの周期を設定(100Hz)
        auto feedback = std::make_shared<CallAscendSlope::Feedback>(); // アクションの進捗を格納する変数
        auto result = std::make_shared<CallAscendSlope::Result>(); // アクションの結果を格納する変数
        int direction = goal_handle->get_goal()->direction;
        //Point2D distance; // 移動距離を格納する変数
        float move_vel = 0.0;
        bool on_slope = false;
        bool after_slope = false;

        // 
        while(rclcpp::ok()){
            if(goal_handle->is_canceling()){
                std::cout << "アクションがキャンセルされました" << std::endl;
                result->result = false;
                goal_handle->canceled(result);
                async_robot_vel3_on(false, common_result);
                return;
            }

            if(is_slope_data and !after_slope) move_vel = SLOPE_VEL;
            else if(after_slope) move_vel = 0.0;
            else move_vel = FLAT_VEL;

            switch(direction){
                case 0:
                    robot_vel_msg.x = 0.0;
                    robot_vel_msg.y = move_vel;
                    break;
                case 1:
                    robot_vel_msg.x = -move_vel;
                    robot_vel_msg.y = 0.0;
                    break;
                case 2:
                    robot_vel_msg.x = 0.0;
                    robot_vel_msg.y = -move_vel;
                    break;
                case 3:
                    robot_vel_msg.x = move_vel;
                    robot_vel_msg.y = 0.0;
                    break;
                default:break;
            }

            std::cout << "x: " << robot_vel_msg.x << ", " << "y: " << robot_vel_msg.y << std::endl;

            if(is_slope_data and !on_slope) {
                std::cout << "坂に乗りました" << std::endl;
                on_slope = true;
            }

            if(!is_slope_data and on_slope){
                std::cout << "坂を超えました" << std::endl;
                if(call_count == 1){
                    std::cout << "左に移動します" << std::endl;
                    robot_vel_msg.x = move_vel/2;
                    rclcpp::sleep_for(2000ms);
                }else{
                    rclcpp::sleep_for(500ms);
                }
                break;
                after_slope = true;
            }

            std::this_thread::sleep_for(10ms);

        }
        std::cout << "ロボットを停止します" << std::endl;
        robot_vel_msg.x = 0;
        robot_vel_msg.y = 0;
        // dircectionにの方向に進んで坂に登ったら停止
        
        if(rclcpp::ok()){ // アクションが成功した場合
            result->result = true;
            goal_handle->succeed(result);
        }
    }

    // 非同期でr2_base_nodeのサービスを呼び出し，このノードからロボットの速度を制御できるようにする関数
    // send_dataがtureなら制御できるようになる。falseなら制御できなくなる。myresultには非同期でサービスの結果が格納される
    void async_robot_vel3_on(bool send_data, bool& myresult){
        auto request = std::make_shared<SetBool::Request>();
        request->data = send_data;
        auto response_received_callback = [this, &myresult](rclcpp::Client<SetBool>::SharedFuture future) {
            myresult = future.get()->success;
            std::cout << "[robot_vel3_on] レスポンスを受け取りました\nresult: " << myresult << std::endl;
        };
        auto future_result = robot_vel_on_cli->async_send_request(request, response_received_callback);
        std::cout << "[robot_vel3_on] リクエストを送信しました" << std::endl;
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AscendSlopeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}