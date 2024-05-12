#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using namespace std::chrono_literals;
using PoseSamped = geometry_msgs::msg::PoseStamped;
using Quaternion = geometry_msgs::msg::Quaternion;

class QuatNode : public rclcpp::Node{
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<PoseSamped>::SharedPtr publisher;
    rclcpp::Subscription<Quaternion>::SharedPtr subscription;
    
    public:
    QuatNode() : Node("quat_node"){
        std::cout << "call QuatNode!" << std::endl;
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        
        auto callback = [this](const Quaternion::SharedPtr msg) -> void {
            auto message = PoseSamped();
            message.pose.orientation = *msg;
            message.header.stamp = this->now();
            message.header.frame_id = "map";
            publisher->publish(message);
        };
        
        publisher = this->create_publisher<PoseSamped>("quaternion", qos);
        subscription = this->create_subscription<Quaternion>("mros_output_imu", qos, callback);
    }
};

int main(int argc, char* argv[]){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<QuatNode>());
  rclcpp::shutdown();
  return 0;
}