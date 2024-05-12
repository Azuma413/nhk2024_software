#include "behaviortree_cpp/bt_factory.h"
#include "../include/action_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "../include/ros_node.hpp"

using namespace MyActionNodes;
using namespace BT;

// フィールド情報をノードに渡す方法を考える
int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  global_node = std::make_shared<BTNode>();
  BehaviorTreeFactory factory;
  factory.registerNodeType<ArmControl>("ArmControl");
  factory.registerNodeType<GetBall>("GetBall");
  factory.registerNodeType<AscendSlope>("AscendSlope");
  factory.registerNodeType<MoveTarget>("MoveTarget");
  factory.registerNodeType<PanTilt>("PanTilt");
  factory.registerNodeType<PutBall>("PutBall");
  factory.registerNodeType<RecogSilo>("RecogSilo");
  factory.registerNodeType<SelectBall>("SelectBall");
  factory.registerNodeType<StartProcess>("StartProcess");
  std::string package_path = ament_index_cpp::get_package_share_directory("cpp2_nhk");
  factory.registerBehaviorTreeFromFile(package_path + "/config/main_bt.xml");
  BT::Tree tree = factory.createTree("MainBT");
  printTreeRecursively(tree.rootNode());
  NodeStatus status = NodeStatus::RUNNING;

  while(status == NodeStatus::RUNNING && rclcpp::ok()){
    rclcpp::spin_some(global_node);
    status = tree.tickOnce();
  }

  rclcpp::shutdown();
  return 0;
}