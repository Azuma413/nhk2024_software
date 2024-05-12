#pragma once
#include <memory>
#include <string>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "ros_node.hpp"
#include <rclcpp/executors.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <nlohmann/json.hpp>

using namespace BT;
using json = nlohmann::json;

namespace BT{
    template <> inline Point2D convertFromString(StringView str){
        auto parts = splitString(str, ';');
        if (parts.size() != 2){
            throw RuntimeError("invalid input");
        } else {
            Point2D output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            return output;
        }
    }
}

bool common_result = false;

namespace MyActionNodes{
// "ArmControl"
// input_port "arm_angle_degree" int
// input_port "arm_range_mm" float
// input_port "arm_height_mm" float
// input_port "is_vacuum" bool
class ArmControl : public StatefulActionNode
{
    public:
        ArmControl(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { InputPort<int>("arm_angle_degree"),
                        InputPort<float>("arm_range_mm"),
                        InputPort<float>("arm_height_mm"),
                        InputPort<bool>("is_vacuum")
                    };
        }

        NodeStatus onStart() override{
            std::cout << "\ncall ArmControl" << std::endl;
            Expected<int> msg1 = getInput<int>("arm_angle_degree");
            if (!msg1){
                throw BT::RuntimeError("missing required input [arm_angle_degree]: ", msg1.error() );
            }
            arm_angle_degree = msg1.value();
            std::cout << "[arm_angle_degree]:" << arm_angle_degree << std::endl;
            Expected<float> msg2 = getInput<float>("arm_range_mm");
            if (!msg2){
                throw BT::RuntimeError("missing required input [arm_range_mm]: ", msg2.error() );
            }
            arm_range_mm = msg2.value();
            std::cout << "[arm_range_mm]:" << arm_range_mm << std::endl;
            Expected<float> msg3 = getInput<float>("arm_height_mm");
            if (!msg3){
                throw BT::RuntimeError("missing required input [arm_height_mm]: ", msg3.error() );
            }
            arm_height_mm = msg3.value();
            std::cout << "[arm_height_mm]:" << arm_height_mm << std::endl;
            Expected<bool> msg4 = getInput<bool>("is_vacuum");
            if (!msg4){
                throw BT::RuntimeError("missing required input [is_vacuum]: ", msg4.error() );
            }
            is_vacuum = msg4.value();
            std::cout << "[is_vacuum]:" << is_vacuum << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            global_node->async_arm_control_on(true, common_result);
            global_node->arm_control_msg.angle = arm_angle_degree/180.0*M_PI;
            global_node->arm_control_msg.range = arm_range_mm;
            global_node->arm_control_msg.height = arm_height_mm;
            global_node->arm_control_msg.vacuum = is_vacuum;
            return NodeStatus::SUCCESS;
        }

        void onHalted() override{
            std::cout << "interrupt ArmControl Node" << std::endl;
        }
    private:
        int arm_angle_degree;
        float arm_range_mm;
        float arm_height_mm;
        bool is_vacuum;
};

// "GetBall"
// input_port "ball_pos" Point2D
class GetBall : public StatefulActionNode
{
    public:
        GetBall(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { InputPort<double>("ball_pos_x"),
                        InputPort<double>("ball_pos_y") };
        }

        NodeStatus onStart() override{
            std::cout << "\ncall GetBall" << std::endl;
            Expected<double> msg1 = getInput<double>("ball_pos_x");
            if (!msg1){
                throw BT::RuntimeError("missing required input [ball_pos_x]: ", msg1.error() );
            }
            ball_pos.x = msg1.value();
            Expected<double> msg2 = getInput<double>("ball_pos_y");
            if (!msg2){
                throw BT::RuntimeError("missing required input [ball_pos_y]: ", msg2.error() );
            }
            ball_pos.y = msg2.value();
            std::cout << "[ball_pos]:" << ball_pos.x << ", " << ball_pos.y << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            if(global_node->send_call_get_ball(ball_pos)){
                return NodeStatus::SUCCESS;
            }else{
                return NodeStatus::FAILURE;
            }
        }

        void onHalted() override{
            std::cout << "interrupt GetBall Node" << std::endl;
        }
    private:
        Point2D ball_pos;
};

// "LineTrace"
// input_port "is_retry" bool
// class LineTrace : public StatefulActionNode
// {
//     public:
//         LineTrace(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

//         static PortsList providedPorts()
//         {
//             return { InputPort<bool>("is_retry") };
//         }

//         NodeStatus onStart() override{
//             std::cout << "\ncall LineTrace" << std::endl;
//             Expected<bool> msg1 = getInput<bool>("is_retry");
//             if (!msg1){
//                 throw BT::RuntimeError("missing required input [is_retry]: ", msg1.error() );
//             }
//             is_retry = msg1.value();
//             std::cout << "[is_retry]:" << is_retry << std::endl;
//             return NodeStatus::RUNNING;
//         }

//         NodeStatus onRunning() override{
//             if(global_node->send_call_line_trace(is_retry)){
//                 return NodeStatus::SUCCESS;
//             }else{
//                 return NodeStatus::FAILURE;
//             }
//         }

//         void onHalted() override{
//             std::cout << "interrupt LineTrace Node" << std::endl;
//         }
//     private:
//         bool is_retry;
// };

// "AscendSlope"
// input_port "move_direction" int
class AscendSlope : public StatefulActionNode
{
    public:
        AscendSlope(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { InputPort<int>("move_direction") };
        }

        NodeStatus onStart() override{
            std::cout << "\ncall AscendSlope" << std::endl;
            Expected<int> msg1 = getInput<int>("move_direction");
            if (!msg1){
                throw BT::RuntimeError("missing required input [move_direction]: ", msg1.error() );
            }
            direction = msg1.value();
            std::cout << "[move_direction]:" << direction << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            if(global_node->send_call_ascend_slope(direction)){
                return NodeStatus::SUCCESS;
            }else{
                return NodeStatus::FAILURE;
            }
        }

        void onHalted() override{
            std::cout << "interrupt AscendSlope Node" << std::endl;
        }
    private:
        int direction;
};

// "MoveTarget"
// input_port "target_x_m" float
// input_port "target_y_m" float
class MoveTarget : public StatefulActionNode
{
    public:
        MoveTarget(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { InputPort<float>("target_x_m"),
                        InputPort<float>("target_y_m"),
                        InputPort<int>("erea") };
        }

        NodeStatus onStart() override{
            std::cout << "\ncall MoveTarget" << std::endl;
            Expected<float> msg1 = getInput<float>("target_x_m");
            if (!msg1){
                throw BT::RuntimeError("missing required input [target_x_m]: ", msg1.error() );
            }
            target_x_m = msg1.value();
            std::cout << "[target_x_m]:" << target_x_m << std::endl;
            Expected<float> msg2 = getInput<float>("target_y_m");
            if (!msg2){
                throw BT::RuntimeError("missing required input [target_y_m]: ", msg2.error() );
            }
            target_y_m = msg2.value();
            std::cout << "[target_y_m]:" << target_y_m << std::endl;
            Expected<int> msg3 = getInput<int>("erea");
            if (!msg3){
                throw BT::RuntimeError("missing required input [erea]: ", msg3.error() );
            }
            erea = msg3.value();
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            Point2D target_pos;
            target_pos.x = target_x_m * 1000.0; // m -> mm
            target_pos.y = target_y_m * 1000.0;
            if(global_node->send_set_move_target(target_pos, erea)){
                return NodeStatus::SUCCESS;
            }else{
                return NodeStatus::FAILURE;
            }
        }

        void onHalted() override{
            std::cout << "interrupt MoveTarget Node" << std::endl;
        }
    private:
        float target_x_m;
        float target_y_m;
        int erea;
};

// "PanTilt"
// input_port "pan_degree" int
// input_port "tilt_degree" int
class PanTilt : public StatefulActionNode
{
    public:
        PanTilt(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { InputPort<int>("pan_degree"),
                        InputPort<int>("tilt_degree") };
        }

        NodeStatus onStart() override{
            std::cout << "\ncall PanTilt" << std::endl;
            Expected<int> msg1 = getInput<int>("pan_degree");
            if (!msg1){
                throw BT::RuntimeError("missing required input [pan_degree]: ", msg1.error() );
            }
            pan_degree = msg1.value();
            std::cout << "[pan_degree]:" << pan_degree << std::endl;
            Expected<int> msg2 = getInput<int>("tilt_degree");
            if (!msg2){
                throw BT::RuntimeError("missing required input [tilt_degree]: ", msg2.error() );
            }
            tilt_degree = msg2.value();
            std::cout << "[tilt_degree]:" << tilt_degree << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            global_node->pantilt_pos.position_0 = pan_degree/180.0*M_PI;
            global_node->pantilt_pos.position_1 = tilt_degree/180.0*M_PI;
            return NodeStatus::SUCCESS;
        }

        void onHalted() override{
            std::cout << "interrupt PanTilt Node" << std::endl;
        }
    private:
        int pan_degree;
        int tilt_degree;
};

// "PutBall"
// input_port "silo_id" int
class PutBall : public StatefulActionNode
{
    public:
        PutBall(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { InputPort<int>("silo_id") };
        }

        NodeStatus onStart() override{
            std::cout << "\ncall PutBall" << std::endl;
            Expected<int> msg1 = getInput<int>("silo_id");
            if (!msg1){
                throw BT::RuntimeError("missing required input [silo_id]: ", msg1.error() );
            }
            silo_id = msg1.value();
            std::cout << "[silo_id]:" << silo_id << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            if(global_node->send_call_put_ball(silo_id)){
                return NodeStatus::SUCCESS;
            }else{
                return NodeStatus::FAILURE;
            }
        }

        void onHalted() override{
            std::cout << "interrupt PutBall Node" << std::endl;
        }
    private:
        int silo_id;
};

// "RecogSilo" 何回かサービスを呼んで，結果の和が最も多いサイロを選ぶ？
// output_port "recommend_id" int
class RecogSilo : public StatefulActionNode
{
    public:
        RecogSilo(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { OutputPort<int>("recommend_id") };
        }

        NodeStatus onStart() override{
            std::cout << "\ncall RecogSilo" << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            // std::vector<int> priorities(5, 0);
            // priorities = global_node->sync_get_silo_state();
            // // prioritiesの中から最も大きい値をもつidを選ぶ
            int max_id = 0;
            // int max_val = 0;
            // for(int i=0; i<5; i++){
            //     if(priorities[i] > max_val){
            //         max_id = i;
            //         max_val = priorities[i];
            //     }
            // }

            // 2次ビデオでは固定入力
            max_id = (count)%3 + 1;
            count++;
            setOutput("recommend_id", max_id);
            return NodeStatus::SUCCESS;
        }

        void onHalted() override{
            std::cout << "interrupt RecogSilo Node" << std::endl;
        }
    private:
        int count = 0;
};

// "SelectBall"
// output_port "recommend_ball_pos" Point2D
class SelectBall : public StatefulActionNode
{
    public:
        SelectBall(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { OutputPort<double>("recommend_ball_pos_x"),
                        OutputPort<double>("recommend_ball_pos_y")};
        }

        NodeStatus onStart() override{
            std::cout << "\ncall SelectBall" << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            Point2D ball_pos = global_node->send_get_ball_coord();
            std::cout << "[ball_pos]:" << ball_pos.x << ", " << ball_pos.y << std::endl;
            // if(ball_pos.x < 0 && ball_pos.y < 0){
            //     return NodeStatus::FAILURE;
            // }
            setOutput("recommend_ball_pos_x", ball_pos.x);
            setOutput("recommend_ball_pos_y", ball_pos.y);
            return NodeStatus::SUCCESS;
        }

        void onHalted() override{
            std::cout << "interrupt SelectBall Node" << std::endl;
        }
};

// "StartProcess"
// input_port "wait_time_ms" int
// output_port "is_retry" bool
class StartProcess : public StatefulActionNode
{
    public:
        StartProcess(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { InputPort<int>("wait_time_ms"),
                        OutputPort<std::string>("is_retry") };
        }

        NodeStatus onStart() override{
            std::cout << "\ncall StartProcess" << std::endl;
            Expected<int> msg1 = getInput<int>("wait_time_ms");
            if (!msg1){
                throw BT::RuntimeError("missing required input [wait_time_ms]: ", msg1.error() );
            }
            wait_time_ms = msg1.value();
            std::cout << "[wait_time_ms]:" << wait_time_ms << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_ms));
            if(global_node->start_flag){
                // global_node->async_imu_set(true, common_result);
                if(global_node->is_retry){
                    setOutput("is_retry", "true");
                }else{
                    setOutput("is_retry", "false");
                }
                return NodeStatus::SUCCESS;         
            }
            return NodeStatus::RUNNING;
        }

        void onHalted() override{
            std::cout << "interrupt StartProcess Node" << std::endl;
        }
    private:
        int wait_time_ms;
};
} // namespace MyActionNodes