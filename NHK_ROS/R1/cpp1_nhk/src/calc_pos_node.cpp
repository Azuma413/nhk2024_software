/*
自己位置推定ノード
*/
// ライブラリのインクルードなど
// ********************************************************************************************************************
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
// #include "std_srvs/srv/set_bool.hpp"
#include "actuator_custom_msgs/srv/start_calc_pos.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/channel_float32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::chrono_literals;
using Point = geometry_msgs::msg::Point;
using LaserScan = sensor_msgs::msg::LaserScan;
// using SetBool = std_srvs::srv::SetBool;
using StartCalcPos = actuator_custom_msgs::srv::StartCalcPos;
using Float32 = std_msgs::msg::Float32;
using PointCloud = sensor_msgs::msg::PointCloud;
using Point32 = geometry_msgs::msg::Point32;
using ChannelFloat32 = sensor_msgs::msg::ChannelFloat32;

struct Point2D{
    double x;
    double y;
};
// ********************************************************************************************************************
// 定数の定義
// ********************************************************************************************************************
const std::vector<Point2D> CORNER2 = {{0,0}, {0,3000}, {3800,3000}, {3800,3500}, {4800,3500}, {4800,4000}, {5875,4000}, {5875,0}}; // エリア2の頂点座標
const std::vector<Point2D> CORNER3 = {{0,0}, {0,4000}, {5875,4000}, {5875,0}, {0, 0}}; // エリア3の頂点座標
const float INTERVAL = 500.0; // 頂点座標を補間する際の間隔
const float r1_lidar_width = 630.0; // R1のLiDARの幅
const float r2_lidar_width = 434.0; // R2のLiDARの幅
const float NORM_LIMIT = 10.0; // update_box関数で使用する。boxの移動量がこの値以下になったら終了
const float MINIMUM_THRESHOLD = 200.0; // calc_threshold関数の戻り値はこの値を下回らない [mm]
const float ERROR_DISCOUNT = 0.85; // calc_threshold関数のパラメータ
const float COUNT_DISCOUNT = 8.5; // calc_threshold関数のパラメータ
const int MINIMUM_UPDATE_COUNT = 10; // update_box関数の最低更新回数
const float MAX_RANGE = 6.0; // 外れ値として扱う距離の下限[m]
const float MINIMUM_RANGE = 0.5; // 点群として扱う距離の下限[m]
const int RESET_POINT_RATE = 0.5;
// 頂点座標を補間する関数
std::vector<Point2D> interpolate(const std::vector<Point2D>& corners, const float interval=500.0){
    std::vector<Point2D> new_corners;
    for(int i = 0; i < corners.size()-1; i++){
        new_corners.push_back(corners[i]);
        double diff_x = corners[i+1].x - corners[i].x;
        double diff_y = corners[i+1].y - corners[i].y;
        int num = (int)(sqrt(diff_x*diff_x + diff_y*diff_y) / interval); // 2点間の距離を求めて補間する点の数を決める
        for(int j = 1; j < num; j++){
            new_corners.push_back({corners[i].x + diff_x * j / num, corners[i].y + diff_y * j / num});
        }
    }
    new_corners.push_back(corners.back()); // 最後の点を追加
    return new_corners;
}
const bool PUB_PCL = true;
// ********************************************************************************************************************
// クラスの定義 
// ********************************************************************************************************************
class CalcPosNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<Point>::SharedPtr robot_pos_pub;
    rclcpp::Publisher<PointCloud>::SharedPtr pcl_pub; // debug用
    rclcpp::Publisher<PointCloud>::SharedPtr box_pub; // debug用
    rclcpp::Publisher<PointCloud>::SharedPtr ignore_pub; // debug用
    rclcpp::Subscription<Float32>::SharedPtr robot_yaw_sub; 
    rclcpp::Subscription<Point>::SharedPtr distance_sub;
    rclcpp::Subscription<LaserScan>::SharedPtr scan1_sub;
    rclcpp::Subscription<LaserScan>::SharedPtr scan2_sub;
    // rclcpp::Service<SetBool>::SharedPtr start_calc_pos_srv;
    rclcpp::Service<StartCalcPos>::SharedPtr start_calc_pos_srv;
    float euler_z = 0.0; // ロボットのyaw角を格納する変数
    LaserScan scan1_data; // LiDARのデータを格納する変数
    LaserScan scan2_data; // LiDARのデータを格納する変数
    std::string color = ""; // フィールドの色
    int robot_num = 1; // ロボットの番号
    int erea_num = 0;
    std::vector<Point2D> corners;
    std::vector<Point2D> corners2; // 自己位置推定に使用するエリアの補完済み頂点座標
    std::vector<Point2D> corners3; // 自己位置推定に使用するエリアの補完済み頂点座標
    Point robot_pos; // ロボットの位置を格納する変数
    bool calc_pos_flag = false; // 位置推定を行うかどうかのフラグ
    Point2D max_corner = {0,0}; // エリアの最大座標
    Point2D distance_data = {0.0, 0.0}; // サブスクライブしたそのままの値
    Point2D old_distance = {0.0, 0.0}; // ある時点の移動距離
    int count = 0;
    float lidar_width = 0.0; // LiDARの幅
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
    CalcPosNode() : Node("calc_pos_node"){
        RCLCPP_INFO(this->get_logger(), "calc_pos_node is activated");
        // パラメータの宣言と取得
        auto parameter1 = this->declare_parameter<std::string>("color", "blue");
        auto parameter2 = this->declare_parameter<int>("robot_num", 1);
        if(!this->get_parameter("color", color)){
            std::cout << "color パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "color: " << color << std::endl;
        }
        if(!this->get_parameter("robot_num", robot_num)){
            std::cout << "robot_num パラメータの取得に失敗" << std::endl;
        }else{
            std::cout << "robot_num: " << robot_num << std::endl;
        }
        max_corner = {5875,4000};
        if(robot_num == 1){
            lidar_width = r1_lidar_width;
        }else if(robot_num == 2){
            lidar_width = r2_lidar_width;
        }
        corners2 = interpolate(CORNER2, INTERVAL);
        corners3 = interpolate(CORNER3, INTERVAL);

        auto timer_callback = [this]() -> void{
            if(!calc_pos_flag){
                return;
            }
            if(erea_num != 2 and erea_num != 3){
                return;
            }
            
            Point2D distance = get_distance(); // 移動距離を取得
            robot_pos.x += distance.x;
            robot_pos.y += distance.y;
            count++;
            set_distance(); // 移動距離をリセット

            if(count >= 10){ // 10回に1回の割合で位置推定を行う
                count = 0;
                // 2つのLiDARのデータを統合してデカルト座標系に変換 ついでに[m]->[mm]としている
                std::vector<Point2D> point_list, ignore_points;
                int start_idx = (int)((-M_PI/2 - scan1_data.angle_min)/scan1_data.angle_increment);
                int end_idx = start_idx + (int)(M_PI/scan1_data.angle_increment);
                for(int i = start_idx; i < end_idx; i++){
                    if(scan1_data.ranges[i] > MAX_RANGE || scan1_data.ranges[i] < MINIMUM_RANGE){
                        continue;
                    }
                    double theta = (scan1_data.angle_min + scan1_data.angle_increment*i) - M_PI/2 - euler_z;
                    point_list.push_back({-scan1_data.ranges[i]*sin(theta)*1000.0 + lidar_width*cos(euler_z)/2, scan1_data.ranges[i]*cos(theta)*1000.0 - lidar_width*sin(euler_z)/2});
                }
                start_idx = (int)((-M_PI/2 - scan2_data.angle_min)/scan2_data.angle_increment);
                end_idx = start_idx + (int)(M_PI/scan2_data.angle_increment);
                for(int i = start_idx; i < end_idx; i++){
                    if(scan2_data.ranges[i] > MAX_RANGE || scan2_data.ranges[i] < MINIMUM_RANGE){
                        continue;
                    }
                    double theta = (scan2_data.angle_min + scan2_data.angle_increment*i) - M_PI/2 - euler_z;
                    point_list.push_back({scan2_data.ranges[i]*sin(theta)*1000.0 - lidar_width*cos(euler_z)/2, -scan2_data.ranges[i]*cos(theta)*1000.0 + lidar_width*sin(euler_z)/2});
                }

                if(ignore_points.size()/point_list.size() > RESET_POINT_RATE){
                    robot_pos.z = -1.0;
                }

                if(robot_pos.z < 0.0){ // 初回のみ点群の平均をとる
                    double x = 0.0;
                    double y = 0.0;
                    for(auto p : point_list){
                        x += p.x;
                        y += p.y;
                    }
                    robot_pos.x = x / point_list.size() + max_corner.x/2.0;
                    robot_pos.y = y / point_list.size() + max_corner.y/2.0;
                    robot_pos.z = 1.0;
                }

                // robot_posを中心としてcornerを配置する。
                std::vector<Point2D> box;
                for(auto c : corners){
                    box.push_back({c.x - robot_pos.x, c.y - robot_pos.y});
                }

                // ロボットの位置を推定
                float max_error = 6000.0;
                int count = 0;
                bool done = false;
                while(!done){
                    ignore_points.clear();
                    float threshold = calc_threshold(max_error, count, MINIMUM_THRESHOLD, ERROR_DISCOUNT, COUNT_DISCOUNT);
                    done = update_box(box, max_error, ignore_points, point_list, threshold, NORM_LIMIT);
                    count++;
                    if(count < MINIMUM_UPDATE_COUNT){
                        done = false;
                    }
                }

                // どちらのロボットもエリアの右下の角を原点とする。x軸は左方向，y軸は前方向
                robot_pos.x = -box[0].x;
                robot_pos.y = -box[0].y;

                if(PUB_PCL){
                    int points_size = point_list.size();
                    PointCloud pcl, pcl2, pcl3;
                    Point32 xyz = Point32();
                    pcl.header.frame_id = "map";
                    pcl.header.stamp = this->now();
                    pcl2.header = pcl.header;
                    pcl3.header = pcl.header;
                    xyz.z = 0.0;
                    for(int i = 0; i < points_size; i++){
                        xyz.x = -point_list[i].x/1000.0; // [mm]->[m]
                        xyz.y = point_list[i].y/1000.0;
                        pcl.points.push_back(xyz);
                    }
                    for(int i = 0; i < box.size(); i++){
                        xyz.x = -box[i].x/1000.0; // [mm]->[m]
                        xyz.y = box[i].y/1000.0;
                        pcl2.points.push_back(xyz);
                    }
                    for(int i = 0; i < ignore_points.size(); i++){
                        xyz.x = -ignore_points[i].x/1000.0; // [mm]->[m]
                        xyz.y = ignore_points[i].y/1000.0;
                        pcl3.points.push_back(xyz);
                    }
                    pcl_pub->publish(pcl);
                    box_pub->publish(pcl2);
                    ignore_pub->publish(pcl3);
                }
            }

            robot_pos_pub->publish(robot_pos);
        };

        auto robot_yaw_callback = [this](const Float32& msg) -> void{
            // yaw角を取得
            euler_z = msg.data;
        };

        auto distance_callback = [this](const Point::SharedPtr msg) -> void{
            distance_data.x = msg->x; // [mm]
            distance_data.y = msg->y;
        };

        auto scan1_callback = [this](const LaserScan& msg) -> void{
            scan1_data = msg;
        };

        auto scan2_callback = [this](const LaserScan& msg) -> void{
            scan2_data = msg;
        };

        auto start_calc_pos_callback = [this](const std::shared_ptr<StartCalcPos::Request> request, std::shared_ptr<StartCalcPos::Response> response) -> void{
            calc_pos_flag = request->start;
            // エリアが同じならば前回の位置から推定を行う。ないほうがいいかも。
            // if(erea_num == request->erea){
            //     response->result = true;
            //     return;
            // }
            if(request->erea == 2){
                std::cout << "start calc erea 2 pos" << std::endl;
                erea_num = request->erea;
                corners = corners2;
            }else if(request->erea == 3){
                std::cout << "start calc erea 3 pos" << std::endl;
                erea_num = request->erea;
                corners = corners3;
            }else{
                std::cout << "stop calc pos" << request->erea << std::endl;
            }
            robot_pos.z = -1.0; // 初期化
            response->result = true;
        };


        rclcpp::QoS qos(rclcpp::KeepLast(10));
        robot_pos_pub = this->create_publisher<Point>("robot_pos", qos);
        if(PUB_PCL){
            pcl_pub = this->create_publisher<PointCloud>("pcl", qos);
            box_pub = this->create_publisher<PointCloud>("box", qos);
            ignore_pub = this->create_publisher<PointCloud>("ignore", qos);
        }
        distance_sub = this->create_subscription<Point>("distance", qos, distance_callback);
        robot_yaw_sub = this->create_subscription<Float32>("robot_yaw", qos, robot_yaw_callback);
        scan1_sub = this->create_subscription<LaserScan>("scan1", qos, scan1_callback);
        scan2_sub = this->create_subscription<LaserScan>("scan2", qos, scan2_callback);
        start_calc_pos_srv = this->create_service<StartCalcPos>("start_calc_pos", start_calc_pos_callback);
        timer = this->create_wall_timer(20ms, timer_callback);
    }

    // 点群に対してboxをフィットさせる関数
    bool update_box(std::vector<Point2D>& box, float& max_error, std::vector<Point2D>& ignore_points, const std::vector<Point2D>& point_list, const float threshold=1000.0, const float norm_limit=5.0){
        max_error = 0.0;
        std::vector<float> diff = {0.0, 0.0}; // x, y座標のそれぞれのずれを保存するリスト
        // std::vector<Point2D> ignore_points; // デバッグ用に無視された点群を保存する
        for(auto point : point_list){ // 点群から点を1つずつ取り出す
            std::vector<float> distances; // boxの各点との距離を格納するリスト
            for(auto b : box){ // boxの各点との距離を計算してdistancesに格納
                distances.push_back(sqrt((b.x - point.x)*(b.x - point.x) + (b.y - point.y)*(b.y - point.y)));
            }
            std::vector<int> closest_indices; // 最も近い2点のインデックスを格納するリスト
            for(int i = 0; i < 2; i++){ // 最も近い2点のインデックスをclosest_indicesに格納
                closest_indices.push_back(std::distance(distances.begin(), std::min_element(distances.begin(), distances.end())));
                distances[closest_indices[i]] = 1000000.0; // 最も距離が近い点を無視するために大きな値を代入
            }
            std::vector<Point2D> closest_points; // 最も近い2点を格納するリスト
            for(auto i : closest_indices){ // boxの最も近い2点をclosest_pointsに格納
                closest_points.push_back(box[i]);
            }
            if(closest_points[0].x == closest_points[1].x){ // x座標が同じ場合
                // point.yがclosest_points[0].yとclosest_points[1].yの間にあるかどうかを判定
                if(std::min(closest_points[0].y, closest_points[1].y) < point.y && point.y < std::max(closest_points[0].y, closest_points[1].y)){
                    float error = closest_points[0].x - point.x; // x座標のずれ 絶対値ではないことに注意
                    if(error*error < threshold*threshold){ // ノイズへの対処
                        diff[0] += error;
                        error = fabs(error); // 絶対値を取る
                        if(error > max_error){
                            max_error = error;
                        }
                    }else{ // 外れ値の場合は無視
                        if(PUB_PCL){
                            ignore_points.push_back(point);
                        }
                    }
                }else{
                    if(PUB_PCL){
                        ignore_points.push_back(point);
                    }
                }
            }else if(closest_points[0].y == closest_points[1].y){ // y座標が同じ場合
                if(std::min(closest_points[0].x, closest_points[1].x) < point.x && point.x < std::max(closest_points[0].x, closest_points[1].x)){
                    float error = closest_points[0].y - point.y; // y座標のずれ
                    if(error*error < threshold*threshold){
                        diff[1] += error;
                        error = fabs(error);
                        if(error > max_error){
                            max_error = error;
                        }
                    }else{
                        if(PUB_PCL){
                            ignore_points.push_back(point);
                        }
                    }
                }else{
                    if(PUB_PCL){
                        ignore_points.push_back(point);
                    }
                }
            }else{ // x座標もy座標も異なる場合。これは箱から十分に離れていると考えられる
                if(PUB_PCL){
                    ignore_points.push_back(point);
                }
            }
        }
        diff[0] /= point_list.size(); // 平均を取ってdiffを標準化
        diff[1] /= point_list.size();
        for(auto& b : box){ // boxにdiffを適用
            b.x -= diff[0];
            b.y -= diff[1];
        }
        bool done = false;
        if(sqrt(diff[0]*diff[0] + diff[1]*diff[1]) < norm_limit){
            done = true;
        }
        return done;
    }

    // update_box関数で使用する閾値を計算する関数
    float calc_threshold(float max_error, int count, float minimum_threshold=200.0, float error_discount=0.8, float count_discount=8.0){
        return std::max(max_error*error_discount - count_discount*count*count, minimum_threshold);
    }
};
// ********************************************************************************************************************
// メイン関数
// ********************************************************************************************************************
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<CalcPosNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}