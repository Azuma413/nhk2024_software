//
// Created by emile on 23/08/18.
//

//TODO:修正必須!!!!!!


#include <can_utils.h>


NUM_OF_DEVICES num_of_devices;
MCMD_HandleTypedef mcmd_handlers[(NUM_OF_MCMD3+NUM_OF_MCMD4)*2];
CAN_Device air_devices[NUM_OF_AIR];

C620_DeviceInfo c620_dev_info_global[8];
const uint8_t num_of_c620=4;//TODO: 足回りonly


//actuator_msgs__msg__DeviceInfo CAN_Device_to_DeviceInfo(const CAN_Device* can_device){
//    actuator_msgs__msg__DeviceInfo device;
//    device.node_id = can_device->node_id;
//    device.device_num = can_device->device_num;
//    switch(can_device->node_type){
//        case NODE_MCMD1:
//            device.node_type.node_type = actuator_msgs__msg__NodeType__NODE_MCMD1;
//            break;
//        case NODE_MCMD2:
//            device.node_type.node_type = actuator_msgs__msg__NodeType__NODE_MCMD2;
//            break;
//        case NODE_MCMD3:
//            device.node_type.node_type = actuator_msgs__msg__NodeType__NODE_MCMD3;
//            break;
//        case NODE_MCMD4:
//            device.node_type.node_type = actuator_msgs__msg__NodeType__NODE_MCMD4;
//            break;
//        case NODE_AIR:
//            device.node_type.node_type = actuator_msgs__msg__NodeType__NODE_AIR;
//            break;
//        case NODE_SERVO:
//            device.node_type.node_type = actuator_msgs__msg__NodeType__NODE_SERVO;
//            break;
//        default:
//            break;
//    }
//    return device;
//}
//
//CAN_Device DeviceInfo_to_CAN_Device(const actuator_msgs__msg__DeviceInfo* device_info){
//    CAN_Device can_device;
//    can_device.node_id = device_info->node_id;
//    can_device.device_num = device_info->device_num;
//    switch(device_info->node_type.node_type){
//        case actuator_msgs__msg__NodeType__NODE_MCMD1:
//            can_device.node_type = NODE_MCMD1;
//            break;
//        case actuator_msgs__msg__NodeType__NODE_MCMD2:
//            can_device.node_type = NODE_MCMD2;
//            break;
//        case actuator_msgs__msg__NodeType__NODE_MCMD3:
//            can_device.node_type = NODE_MCMD3;
//            break;
//        case actuator_msgs__msg__NodeType__NODE_MCMD4:
//            can_device.node_type = NODE_MCMD4;
//            break;
//        case actuator_msgs__msg__NodeType__NODE_AIR:
//            can_device.node_type = NODE_AIR;
//            break;
//        case actuator_msgs__msg__NodeType__NODE_SERVO:
//            can_device.node_type = NODE_SERVO;
//            break;
//        default:
//            break;
//    }
//    return can_device;
//}
//
//
//uint8_t MCMD_FB_TYPE_to_ActuatorMsg(MCMD_FB_TYPE fb_type){
//    uint8_t ans = 0;
//    switch(fb_type){
//        case MCMD_FB_DUTY:
//            ans = actuator_msgs__msg__ActuatorFeedback__FB_DUTY;
//            break;
//        case MCMD_FB_POS:
//            ans = actuator_msgs__msg__ActuatorFeedback__FB_POS;
//            break;
//        case MCMD_FB_VEL:
//            ans = actuator_msgs__msg__ActuatorFeedback__FB_VEL;
//            break;
//        default:
//            break;
//    }
//    return ans;
//}
//
//
//actuator_msgs__msg__DeviceInfo C620_Device_to_DeviceInfo(C620_DeviceInfo* c620_device_info){
//    actuator_msgs__msg__DeviceInfo ans;
//    ans.node_type.node_type = actuator_msgs__msg__NodeType__NODE_C620;
//    ans.node_id = actuator_msgs__msg__DeviceInfo__NODE_ID_C620;
//    ans.device_num = c620_device_info->device_id;
//    return ans;
//}
//
//
//actuator_msgs__msg__ActuatorMultipleFeedback Get_C620_ActuatorMultiFB(C620_DeviceInfo* c620_device_info, uint8_t act_fb_type){
//    actuator_msgs__msg__ActuatorMultipleFeedback act_fb;
//    act_fb.device = C620_Device_to_DeviceInfo(c620_device_info);
//    act_fb.fb_type = act_fb_type;
//    act_fb.input = c620_device_info->ctrl_param._req_value;
//    switch (act_fb.fb_type) {
//        case actuator_msgs__msg__ActuatorFeedback__FB_CURRENT:
//            act_fb.output = Get_C620_FeedbackData(c620_device_info).current;
//            break;
//        case actuator_msgs__msg__ActuatorFeedback__FB_POS:
//            act_fb.output = Get_C620_FeedbackData(c620_device_info).position;
//            break;
//        case actuator_msgs__msg__ActuatorFeedback__FB_VEL:
//            act_fb.output = Get_C620_FeedbackData(c620_device_info).velocity;
//            break;
//        default:
//            break;
//    }
//    return act_fb;
//}
