//
// Created by emile on 2021/10/13.
//
#include "CAN_System_Def.h"
#include "main.h"


/**
 * @brief CANのIDを設定する. 上16bitのみを使用
 * @param CAN_Device*
 * @param cmd
 */
uint64_t Make_CAN_ID_from_CAN_Device(CAN_Device* _can_device, uint8_t cmd){  // mainからmcmdなどへの送信
    uint8_t node_type = (uint8_t)(_can_device->node_type) & (0b111);
    return (((node_type&0b111)<<11) | (((_can_device->node_id)&0b111)<<8) | (((_can_device->device_num)&0b111)<<5)
            | (cmd&0b11111) );
}


/**
 * @brief CANのIDを設定する. 上16bitのみを使用
 * @param node_type
 * @param node_id
 * @param device_num
 * @param cmd
 */
uint64_t Make_CAN_ID(Node_Type node_type, uint8_t node_id, uint8_t device_num, uint8_t cmd){
    return ( (((uint8_t)node_type & 0b111) << 11) | (((node_id)&0b111) << 8) | ((device_num&0b111) << 5)
             | (cmd&0b11111) );
}

CAN_Device Extract_CAN_Device(uint64_t can_id) {  // CAN_IDからCAN_Deviceを抽出する
    CAN_Device ans;
    ans.device_num = ((can_id>>5) & 0b111);
    ans.node_id = ((can_id>>8) & 0b111);
    ans.node_type = (Node_Type)( (can_id>>11) & 0b111);
    return ans;
}

uint8_t Extract_CAN_CMD(uint64_t can_id){ return ( can_id & 0b11111); }  // cmdを抽出
