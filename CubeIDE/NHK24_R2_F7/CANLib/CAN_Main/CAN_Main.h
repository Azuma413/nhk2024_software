/*
 * CAN_Main.h
 *
 *  Created on: Feb 6, 2020
 *      Author: Eater
 */

//reference https://hsdev.co.jp/stm32-can/

#ifndef _CAN_MAIN_H_
#define _CAN_MAIN_H_

#include "CAN_AirCylinder_Def.h"
#include "CAN_System_Def.h"
#include "MCMD_CAN_Def.h"
#include "CAN_Servo_Def.h"
#include "main.h"

#define CAN_TXBUFFER_SIZE	(512)

#define AWAKE_CMD (0)
#define FB_CMD (1)


typedef struct{
	uint8_t mcmd1;
	uint8_t mcmd2;
	uint8_t mcmd3;
	uint8_t servo;
	uint8_t air;
	uint8_t mcmd4;
	uint8_t other;
} NUM_OF_DEVICES;

void CAN_SystemInit(CAN_HandleTypeDef* _hcan);  // CANの設定と初期化
void CAN_WaitConnect(NUM_OF_DEVICES *num_of); // 全ての基板の接続が確認されるまで待つ

void CANLib_WhenTxMailbox0_1_2CompleteCallbackCalled(CAN_HandleTypeDef* phcan);
void CANLib_WhenTxMailbox0_1_2AbortCallbackCalled(CAN_HandleTypeDef* phcan);

void WhenCANRxFifo0MsgPending(CAN_HandleTypeDef *hcan,NUM_OF_DEVICES *num_of);
//void WhenCANRxFifo1MsgPending(CAN_HandleTypeDef *hcan);

////////////////////////////////////////////////////////////////////////////////////////////
////MCMD
void MCMD_ChangeControl(MCMD_HandleTypedef* hmcmd); // - (can_device)のMCMDのモーターを制御モードにする.
void MCMD_init(MCMD_HandleTypedef* hmcmd);          // - (can_device)のMCMDのモーターに(mode)の初期化命令を送信する.
void MCMD_Calib(MCMD_HandleTypedef* hmcmd);          // - (can_device)のMCMDに原点合わせをさせる.
void MCMD_Control_Enable(MCMD_HandleTypedef* hmcmd); // - (can_device)のMCMDのモーターを制御モードにする.
void MCMD_Control_Disable(MCMD_HandleTypedef* hmcmd); // - (can_device)のMCMDのモーターを制御モードを止める.
void MCMD_SetTarget(MCMD_HandleTypedef* hmcmd, float target); // - (can_device)のMCMDのモーターのPID制御の目標値を変更する.
MCMD_Feedback_Typedef Get_MCMD_Feedback(CAN_Device* can_device);

////ServoDriver
void ServoDriver_Init(CAN_Device* can_device, CANServo_Param_Typedef* param); // Servo基盤のパラメータを初期化する
void ServoDriver_SendValue(CAN_Device* can_device, float angle); // Servo基盤に目標値を送る


////AirCylinder
void AirCylinder_Init(CAN_Device* can_device, Air_PortStatus_Typedef param);
void AirCylinder_SendOutput(CAN_Device* can_device, Air_PortStatus_Typedef param);
// void AirCylinder_SendOutput_withTimeLimit(CAN_Device* can_device, AIRCYLINDER_OUTPUT output, uint16_t time_ms);

#endif /* INC_CAN_SYSTEM_CAN_MAIN_H_ */
