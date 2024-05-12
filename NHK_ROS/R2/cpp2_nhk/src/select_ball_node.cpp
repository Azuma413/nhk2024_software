// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "actuator_custom_msgs/action/get_ball_coord.hpp"
#include "actuator_custom_msgs/srv/get_position.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "actuator_custom_msgs/srv/start_calc_pos.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::chrono_literals;
using GetPosition = actuator_custom_msgs::srv::GetPosition;
using SetBool = std_srvs::srv::SetBool;
using StartCalcPos = actuator_custom_msgs::srv::StartCalcPos;
using Point = geometry_msgs::msg::Point;
using Float32 = std_msgs::msg::Float32;
using GetBallCoord = actuator_custom_msgs::action::GetBallCoord;
using GoalHandleGBC = rclcpp_action::ServerGoalHandle<GetBallCoord>;
using PointStamped = geometry_msgs::msg::PointStamped;

struct Point2D{
    double x;
    double y;
};
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
// カメラの設定
const float FOV_H = 90.0; //87.0; // 水平方向の視野角
const float FOV_V = 65.0; //58.0; // 垂直方向の視野角
const int WIDTH = 1280; // 画像の横幅
const int HEIGHT = 720; // 画像の縦幅
const float CAMERA_HEIGHT = 685.0; // カメラの高さ[mm] (仮)
const float CAMERA_PITCH = 60.0/180.0*M_PI; // btによって設定されるカメラの角度
const float CAMERA_YAW = -90.0/180.0*M_PI; // btによって設定されるカメラの角度
// ボールの設定
const float BALL_RADIUS = 85; // ボールの半径[mm]
// ダイナミクセルの設定
const int PAN_DXL_ID = 1; // Pitch軸のダイナミクセルのID
const int TILT_DXL_ID = 0; // Yaw軸のダイナミクセルのID
// その他設定
const float THRESHOLD = 500; // ボールを同一とみなすための距離の閾値[mm]
const float TARGET_X = 3550.0; // 坂の上のぎりぎりのx座標。
const float P_GAIN = 0.001; // 速度制御のPゲイン
const float D_GAIN = 0.0005; // 速度制御のDゲイン
const float TARGET_THRESHOLD = 50.0; // 目標地点に到達したとみなす閾値[mm]
const float MAX_VEL = 0.5; // m/s
const float SLOPE_HEIGHT = 100.0; // 坂の高さ[mm]
const float BALL_UPDATE_WEIGHT = 0.3; // ボールの位置情報の更新時の重み。1に近いほど新しい情報を重視する

const bool DEBUG_MODE = true; // デバッグモードの設定 
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class SelectBallNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<Point>::SharedPtr robot_vel4_pub;
    rclcpp::Publisher<PointStamped>::SharedPtr ball_pos_pub; // debug用
    rclcpp::Subscription<Float32>::SharedPtr robot_yaw_sub;
    rclcpp::Subscription<Point>::SharedPtr robot_pos_sub;
    rclcpp::Subscription<Point>::SharedPtr detect_ball_pos_sub;
    rclcpp::Client<GetPosition>::SharedPtr get_pantilt_cli;
    rclcpp::Client<SetBool>::SharedPtr start_detect_ball_cli;
    rclcpp::Client<StartCalcPos>::SharedPtr start_calc_pos_cli;
    rclcpp::Client<SetBool>::SharedPtr robot_vel4_on_cli;
    rclcpp_action::Server<GetBallCoord>::SharedPtr get_ball_coord_asrv;
    float robot_yaw = 0.0; // ロボットのyaw角を保存[rad]
    std::string color = ""; // フィールドの色を保存
    bool flag = true; // サービスが利用可能かどうかのフラグ
    Point2D robot_pos; // ロボットの座標を保存
    Point ball_pos; // subscribeしたボールの座標を保存
    Point2D ball_pos_map = {-1.0, -1.0}; // マップ上のボールの座標を保存
    Point robot_vel_msg;
    bool common_result = false; // サービスの結果を格納する変数

    public:
    SelectBallNode() : Node("select_ball_node"){
        RCLCPP_INFO(this->get_logger(), "select_ball_node is activated");
        // パラメータの宣言と取得
        auto parameter = this->declare_parameter<std::string>("color", "blue");
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }

        auto timer_callback = [this]() -> void{
            robot_vel4_pub->publish(robot_vel_msg);
        };

        auto robot_yaw_callback = [this](const Float32& msg) -> void{
            // yawの更新
            robot_yaw = msg.data;
        };

        auto robot_pos_callback = [this](const Point& msg) -> void{
            // ロボットの座標の更新
            robot_pos.x = msg.x; // [mm]
            robot_pos.y = msg.y;
        };

        auto detect_ball_pos_callback = [this](const Point& msg) -> void{
            // ボールの座標の更新
            // std::cout << "detect_ball_pos: (" << msg.x << ", " << msg.y << ", " << msg.z << ")" << std::endl;
            if(msg.z != 0.0){
                ball_pos = msg; // (u, v)[pixel]
                ball_pos.z = msg.z*1000; // depth [m]->[mm]
            }
        };

        auto get_ball_coord_goal = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const GetBallCoord::Goal> goal) -> rclcpp_action::GoalResponse{
            // 目標値を受け取った時の処理
            std::cout << "目標値を受け取りました" << std::endl;
            async_robot_vel4_on(true, common_result);
            async_start_calc_pos(true, 3, common_result);
            async_start_detect_ball(true, common_result);
            rclcpp::sleep_for(200ms);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // ACCEPT_AND_DEFER
        };

        auto get_ball_coord_cancel = [this](const std::shared_ptr<GoalHandleGBC> goal_handle) -> rclcpp_action::CancelResponse{
            // キャンセルを受け取ったときの処理
            std::cout << "キャンセルを受け取りました" << std::endl;
            async_robot_vel4_on(false, common_result);
            async_start_calc_pos(false, 0, common_result);
            async_start_detect_ball(false, common_result);
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto get_ball_coord_accepted = [this](const std::shared_ptr<GoalHandleGBC> goal_handle) -> void{
            using namespace std::placeholders;
            std::cout << "アクションを開始します" << std::endl;
            std::thread{std::bind(&SelectBallNode::get_ball_coord_execute, this, _1), goal_handle}.detach(); // メンバ関数をbindする際には，関数名の前にクラス名を明示的に指定する
        };

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        robot_vel4_pub = this->create_publisher<Point>("robot_vel4", qos);
        if(DEBUG_MODE){
            ball_pos_pub = this->create_publisher<PointStamped>("ball_pos", qos);
        }
        robot_yaw_sub = this->create_subscription<Float32>("robot_yaw", qos, robot_yaw_callback);
        robot_pos_sub = this->create_subscription<Point>("robot_pos", qos, robot_pos_callback);
        detect_ball_pos_sub = this->create_subscription<Point>("detect_ball_pos", qos, detect_ball_pos_callback);
        get_ball_coord_asrv = rclcpp_action::create_server<GetBallCoord>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "get_ball_coord",
            get_ball_coord_goal,
            get_ball_coord_cancel,
            get_ball_coord_accepted
        );
        // get_pantilt_cli = this->create_client<GetPosition>("get_pantilt");
        // while(!get_pantilt_cli->wait_for_service(1s) && rclcpp::ok()){
        //     if(flag){
        //         std::cout << "get_pantilt service is not available" << std::endl;
        //         flag = false;
        //     }
        // }
        // std::cout << "get_pantilt service is available" << std::endl;
        // flag = true;
        start_detect_ball_cli = this->create_client<SetBool>("start_detect_ball");
        while(!start_detect_ball_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "start_detect_ball service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "start_detect_ball service is available" << std::endl;
        flag = true;
        start_calc_pos_cli = this->create_client<StartCalcPos>("start_calc_pos");
        while(!start_calc_pos_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "start_calc_pos service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "start_calc_pos service is available" << std::endl;
        flag = true;
        robot_vel4_on_cli = this->create_client<SetBool>("robot_vel4_on");
        while(!robot_vel4_on_cli->wait_for_service(1s) && rclcpp::ok()){
            if(flag){
                std::cout << "robot_vel4_on service is not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "robot_vel4_on service is available" << std::endl;
        robot_vel_msg.x = 0.0;
        robot_vel_msg.y = 0.0;
        timer = this->create_wall_timer(10ms, timer_callback);
    }

    void get_ball_coord_execute(const std::shared_ptr<GoalHandleGBC>& goal_handle){
        // thread内で実行される実際の処理部分
        rclcpp::Rate loop_rate(50);
        auto feedback = std::make_shared<GetBallCoord::Feedback>();
        auto result = std::make_shared<GetBallCoord::Result>();
        Point2D target_pos;
        float prior_error = 0.0;
        target_pos.x = TARGET_X;
        // actionを受け取ったら，detect_ball_posの値からボールの座標を計算し，目標地点を補正する。robot_vel4をpublishして斜面のぎりぎりまで移動する。位置をボールのy座標に合わせる。
        while(rclcpp::ok()){
            if(goal_handle->is_canceling()){ // 途中でキャンセルされていないか確認
                result->x = -1.0;
                result->y = -1.0;
                goal_handle->canceled(result);
                async_robot_vel4_on(false, common_result);
                async_start_calc_pos(false, 0, common_result);
                async_start_detect_ball(false, common_result);
                return;
            }
            if(!calc_ball_pos()) continue; // ボールの座標を計算
            if(DEBUG_MODE){
                PointStamped ball_pos_msg;
                ball_pos_msg.header.frame_id = "map";
                ball_pos_msg.header.stamp = this->now();
                ball_pos_msg.point.x = -(ball_pos_map.x - robot_pos.x)/1000.0; // ボールの相対座標をpublish [m]
                ball_pos_msg.point.y = (ball_pos_map.y - robot_pos.y)/1000.0;
                // std::cout << "ball_pos: (" << ball_pos_msg.point.x << ", " << ball_pos_msg.point.y << ")" << std::endl;
                ball_pos_pub->publish(ball_pos_msg);
            }
            target_pos.y = ball_pos_map.y;
            // std::cout << "target_pos: (" << target_pos.x << ", " << target_pos.y << ")" << std::endl;
            // std::cout << "robot_pos: (" << robot_pos.x << ", " << robot_pos.y << ")" << std::endl;

            // ロボットの現在位置と目標位置から速度を計算する move_target_node参照
            float error = sqrt(pow(target_pos.x - robot_pos.x, 2) + pow(target_pos.y - robot_pos.y, 2));
            if(error < TARGET_THRESHOLD){ // 目標地点に到達したら終了
                std::cout << "目標地点に到達しました" << std::endl;
                break;
            }
            float vel = std::min(error*P_GAIN + (error - prior_error)*D_GAIN, MAX_VEL); // [m/s]
            std::cout << "vel: " << vel << std::endl;
            float theta = atan2(robot_pos.x - target_pos.x, robot_pos.y - target_pos.y); // ロボットの目標角度を計算
            prior_error = error;
            robot_vel_msg.x = -vel*sin(theta);
            robot_vel_msg.y = -vel*cos(theta);
            std::cout << "robot_vel: (" << robot_vel_msg.x << ", " << robot_vel_msg.y << ")" << std::endl;
            loop_rate.sleep();
        }
        robot_vel_msg.x = 0.0;
        robot_vel_msg.y = 0.0;
        async_robot_vel4_on(false, common_result);
        async_start_calc_pos(false, 0, common_result);
        async_start_detect_ball(false, common_result);
        if(rclcpp::ok()){ // アクションが成功した場合
            std::cout << "success action" << std::endl;
            result->x = ball_pos_map.x - robot_pos.x; // ボールの相対座標を返す [mm]
            result->y = ball_pos_map.y - robot_pos.y;
            goal_handle->succeed(result);
        }
    }

    // ボールの座標を計算する
    bool calc_ball_pos(void){
        // カメラからボールまでの距離を２通りの方法で求める
        float camera_pitch, camera_yaw;
        // if(!sync_get_pantilt(PAN_DXL_ID, camera_pitch) || !sync_get_pantilt(TILT_DXL_ID, camera_yaw)){
        //     std::cout << "パンチルトの角度を取得できませんでした" << std::endl;
        //     camera_pitch = CAMERA_PITCH;
        //     camera_yaw = CAMERA_YAW;
        // }
        camera_pitch = CAMERA_PITCH;
        camera_yaw = CAMERA_YAW;
        // std::cout << "ball_pos: (" << ball_pos.x << ", " << ball_pos.y << ", " << ball_pos.z << ")" << std::endl;
        // 1. 深度情報を使わない方法
        // float omega = (0.5 - ball_pos.y/HEIGHT)*FOV_V*M_PI/180.0; // 垂直方向の角度[rad] カメラの方向からどれだけ上下にずれているか
        // float x1 = (CAMERA_HEIGHT + SLOPE_HEIGHT - BALL_RADIUS)*tan(camera_pitch + omega); // カメラからボールまでの距離[mm]
        // 2. 画像上の座標を使わない方法
        float x2 = sqrt(pow(ball_pos.z + BALL_RADIUS, 2.0) - pow(CAMERA_HEIGHT + SLOPE_HEIGHT - BALL_RADIUS, 2.0)); // カメラからボールまでの距離[mm] 坂の上から観測するので100mm分あげる
        // float x2 = (ball_pos.z - (CAMERA_HEIGHT + SLOPE_HEIGHT - BALL_RADIUS)*cos(camera_pitch))/sin(camera_pitch); // カメラからボールまでの距離[mm] 坂の上から観測するので100mm分あげる
        // std::cout << "x2_1:" << ball_pos.z + BALL_RADIUS << std::endl;
        // std::cout << "x2_2:" << (CAMERA_HEIGHT + SLOPE_HEIGHT - BALL_RADIUS) << std::endl;
        // std::cout << "camera_pitch: " << camera_pitch << "\ncamera_yaw: " << camera_yaw << std::endl;
        // std::cout << "omega: " << omega << std::endl;
        // std::cout << "x1: " << x1 << "\nx2: " << x2 << std::endl;
        // ボールまでの距離はx1とx2の平均値とする
        // float range = (x1 + x2)/2.0;
        float range = x2;
        if(range < 0 or range > 10000 or std::isnan(range)){
            return false;
        }

        // std::cout << "range: " << range << std::endl;
        // ボールの座標を計算
        float theta = (WIDTH/2 - ball_pos.x)*FOV_H*M_PI/(WIDTH*180.0); // 水平方向の角度[rad] カメラの方向からどれだけ左右にずれているか
        float local_x = -range*sin(robot_yaw + camera_yaw + theta); // ロボットを原点とするボールのx座標[mm]
        float local_y = range*cos(robot_yaw + camera_yaw + theta); // ロボットを原点とするボールのy座標[mm]
        // ボールの座標をグローバル座標に変換
        Point2D ball;
        ball.x = robot_pos.x - local_x;
        ball.y = robot_pos.y + local_y;
        // std::cout << "ball: (" << ball.x << ", " << ball.y << ")" << std::endl;
        if(ball_pos_map.x < 0 and ball_pos_map.y < 0){ // 1回目はそのまま更新
            ball_pos_map = ball; // [mm]
        }else{ // 2回目以降は既に計算されたボールとの距離を比較し，ある程度近いボールは平均値を取る
            float distance = sqrt(pow(ball.x - ball_pos_map.x, 2) + pow(ball.y - ball_pos_map.y, 2));
            if(distance < THRESHOLD){ // 閾値以内のボールは同一とみなして平均を取る
                ball_pos_map.x = BALL_UPDATE_WEIGHT*ball.x + (1.0 - BALL_UPDATE_WEIGHT)*ball_pos_map.x;
                ball_pos_map.y = BALL_UPDATE_WEIGHT*ball.y + (1.0 - BALL_UPDATE_WEIGHT)*ball_pos_map.y;
                // ball_pos_map.x = (ball_pos_map.x + ball.x)/2.0;
                // ball_pos_map.y = (ball_pos_map.y + ball.y)/2.0;
            }else{ // 閾値以内のボールがない場合はそのまま更新
                ball_pos_map = ball;
            }
        }
        return true;
    }

    bool sync_get_pantilt(int id, float& rad){
        auto request = std::make_shared<GetPosition::Request>();
        request->id = id;
        auto future_result = get_pantilt_cli->async_send_request(request);
        if (future_result.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            try{
                auto response = future_result.get();
                rad = response->position;
                return true;
            } catch (const std::exception &e) {
                std::cout << "GetPantilt ServiceからResultを得られませんでした" << std::endl;
                return false;
            }
        }else{
            std::cout << "Time out" << std::endl;
            return false;
        }
    }

    void async_start_detect_ball(bool send_data, bool& myresult){
        auto request = std::make_shared<SetBool::Request>();
        request->data = send_data;
        auto response_received_callback = [this, &myresult](rclcpp::Client<SetBool>::SharedFuture future){
            myresult = future.get()->success;
            std::cout << "[start_detect_ball] レスポンスを受け取りました\nresult: " << myresult << std::endl;
        };
        auto future_result = start_detect_ball_cli->async_send_request(request);
        std::cout << "[start_detect_ball] リクエストを送信しました" << std::endl;
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

    void async_robot_vel4_on(bool send_data, bool& myresult){
        auto request = std::make_shared<SetBool::Request>();
        request->data = send_data;
        auto response_received_callback = [this, &myresult](rclcpp::Client<SetBool>::SharedFuture future){
            myresult = future.get()->success;
            std::cout << "[robot_vel4_on] レスポンスを受け取りました\nresult: " << myresult << std::endl;
        };
        auto future_result = robot_vel4_on_cli->async_send_request(request);
        std::cout << "[robot_vel4_on] リクエストを送信しました" << std::endl;
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SelectBallNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}