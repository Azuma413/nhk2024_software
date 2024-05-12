/*
 * CAN_System_Def.h
 *
 *  Created on: 9/25, 2021
 *      Author: Emile
 */

#ifndef INC_CAN_SYSTEM_CAN_SYSTEM_DEF_H_
#define INC_CAN_SYSTEM_CAN_SYSTEM_DEF_H_

#include <main.h>
#include <stdio.h>

typedef enum{
	NODE_MAIN = 0,  // メイン基板
	NODE_MCMD1,  // device 0 : vnhモタドラ, device 1 : 100Aモタドラ
	NODE_MCMD2,  // device 0,1 : vnhモタドラ
	NODE_MCMD3,  // device 0,1 : 100Aモタドラ
	NODE_SERVO,  // servoの基板
	NODE_AIR,	// エアシリンダ
	NODE_MCMD4, // device 0,1 100A モタドラ
	NODE_UI,
	NODE_OTHER,
	NODE_TYPES	 //nodeの種類数:必ず列挙体の最後にする
}Node_Type;

typedef struct{
	Node_Type node_type;  // 基板の種類
	uint8_t node_id;      // 基板の番号
	uint8_t device_num;   // 動かすデバイス(motorとか)の番号
}CAN_Device;

//typedef struct{
//	float present_ctrl_quantity;		// 現在の制御量
//	uint8_t done;										// (bool)目標値に到達したか.
//	uint8_t already_read;						// (bool)この値を取得したか
//}CAN_Feedback;

uint64_t Make_CAN_ID_from_CAN_Device(CAN_Device* _can_device, uint8_t cmd);  // CAN_IDを生成する
uint64_t Make_CAN_ID(Node_Type node_type, uint8_t node_id, uint8_t device_num, uint8_t cmd);  // CAN_IDを生成する
CAN_Device Extract_CAN_Device(uint64_t can_id);  // CAN_IDからCAN_Deviceを抽出する
uint8_t Extract_CAN_CMD(uint64_t can_id);  // CAN_IDからcmdを抽出する

/*
 *  Base ID : 14bit
 *  Data fieldのData : Max 64bit
 *
 * メインからモタドラなどへの送信
 *---------------------------
 * BIT     |内容
 * --------|-----------------
 * [13:11] |Destination NodeType
 * --------|-----------------
 * [10:8]  |Destination NodeID
 * --------|-----------------
 * [7:5]   | DeviceNum
 * --------|-----------------
 * [4:0]   | CommandType
 *---------------------------
 *
 * Destinationのところだけを取ってくるなら,
 * MaskIdHigh = MAKE_MAIN_CANID(0b111, 0b111, 0) << 5; とすればok.
 * 16bitの前14bitがIDに対応する. この14bitをいいかんじに操作してやる.
 *モタドラなどからメインへの送信
 *---------------------------
 * BIT     |内容
 * --------|-----------------
 * [13:11] |Source NodeType
 * --------|-----------------
 * [10:8]  |Source NodeID
 * --------|-----------------
 * [7:5]   | DeviceNum
 * --------|-----------------
 * [4:0]   | CommandType
 *---------------------------
 *
 * フローを書く
 */


#define CMD_AWAKE 0x0

#endif /* INC_CAN_SYSTEM_CAN_SYSTEM_DEF_H_ */
