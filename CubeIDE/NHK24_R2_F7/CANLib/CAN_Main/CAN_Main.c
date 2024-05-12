/*
 * CAN_Main.c
 *
 *  Created on: Feb 6, 2020
 *      Author: Eater
 */
#include "CAN_Main.h"
#include <stdio.h>
#include <string.h>

CAN_HandleTypeDef* _canlib_phcan;
volatile uint8_t all_node_detected = 0;

uint8_t num_detected[NODE_TYPES] = {0};
uint8_t node_id_list[NODE_TYPES][0b111];

MCMD_Feeback_Main_typedef _feedback_table_mcmd1[10];
MCMD_Feeback_Main_typedef _feedback_table_mcmd2[10];
MCMD_Feeback_Main_typedef _feedback_table_mcmd3[10];
MCMD_Feeback_Main_typedef _feedback_table_mcmd4[10];

typedef struct{
	uint32_t ExtId; // 18bit
	uint32_t DLC;
	uint8_t bytes[8];
} CANTxBuf;

CANTxBuf buffer[CAN_TXBUFFER_SIZE];
uint32_t readpoint = 0;
uint32_t writepoint = 0;
uint8_t isfull = 0;

HAL_StatusTypeDef PushTx8Bytes(uint32_t ExtId, uint8_t *bytes, uint32_t size){
	buffer[writepoint].DLC = size;
	buffer[writepoint].ExtId = ExtId;
	for (uint8_t i = 0; i < size; i++)buffer[writepoint].bytes[i] = bytes[i];

	if (isfull == 1)readpoint = (readpoint + 1) & (CAN_TXBUFFER_SIZE - 1);
	writepoint = (writepoint + 1) & (CAN_TXBUFFER_SIZE - 1);

	if (writepoint == readpoint){
		isfull = 1;
	}

	return HAL_OK;
}

HAL_StatusTypeDef PopSendTx8Bytes(CAN_HandleTypeDef* phcan){
	CAN_TxHeaderTypeDef txHeader;
	uint32_t txMailbox;

	txHeader.RTR = CAN_RTR_DATA; // Data frame
	txHeader.IDE = CAN_ID_EXT;	 // CAN Extend ID
	txHeader.TransmitGlobalTime = DISABLE;

	while (HAL_CAN_GetTxMailboxesFreeLevel(phcan) > 0){
		if (isfull == 0 && readpoint == writepoint)break;

		txHeader.DLC = buffer[readpoint].DLC;
		txHeader.ExtId = buffer[readpoint].ExtId;

		HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(phcan, &txHeader, buffer[readpoint].bytes, &txMailbox);
		if (ret != HAL_OK)return ret;

		readpoint = (readpoint + 1) & (CAN_TXBUFFER_SIZE - 1);
		isfull = 0;
	}

	return HAL_OK;
}

void CANLib_WhenTxMailbox0_1_2CompleteCallbackCalled(CAN_HandleTypeDef *phcan){
	if(phcan != _canlib_phcan)return;
    PopSendTx8Bytes(_canlib_phcan);
}

void CANLib_WhenTxMailbox0_1_2AbortCallbackCalled(CAN_HandleTypeDef *phcan){
    if(phcan != _canlib_phcan)return;
	PopSendTx8Bytes(_canlib_phcan);
}

void WhenCANRxFifo0MsgPending(CAN_HandleTypeDef *phcan, NUM_OF_DEVICES *num_of){ // Fifo0MsgPendingで呼び出すこと. CAN受信時に呼び出される関数
//	if(phcan != _canlib_phcan)return;
    CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];
	if (HAL_CAN_GetRxMessage(_canlib_phcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK){
		// Reception Error
		printf("GetRxMessage error\n\r");
		Error_Handler();
	}
	// awakeコマンドを受信した場合
	CAN_Device can_device = Extract_CAN_Device(rxHeader.ExtId);
	uint8_t extracted_cmd = Extract_CAN_CMD(rxHeader.ExtId);
	if(extracted_cmd == AWAKE_CMD){
		for(uint8_t i = 0; i < num_detected[can_device.node_type]; i++){
			if(node_id_list[can_device.node_type][i] == rxData[0])return;
		}
		node_id_list[can_device.node_type][num_detected[can_device.node_type]] = rxData[0];
		num_detected[can_device.node_type] += 1;
	}else if(extracted_cmd == FB_CMD){
		if (can_device.node_type == NODE_MCMD1){
			memcpy(&(_feedback_table_mcmd1[can_device.node_id].feedback_motor[can_device.device_num]),
                   rxData, sizeof(MCMD_Feedback_Typedef));
		}else if(can_device.node_type == NODE_MCMD2){
			memcpy(&(_feedback_table_mcmd2[can_device.node_id].feedback_motor[can_device.device_num]),
				   rxData, sizeof(MCMD_Feedback_Typedef));
		}else if(can_device.node_type == NODE_MCMD3){
			memcpy(&(_feedback_table_mcmd3[can_device.node_id].feedback_motor[can_device.device_num]),
				   rxData, sizeof(MCMD_Feedback_Typedef));
		}else if(can_device.node_type == NODE_MCMD4){
			memcpy(&(_feedback_table_mcmd4[can_device.node_id].feedback_motor[can_device.device_num]),
				   rxData, sizeof(MCMD_Feedback_Typedef));
		}else{
			printf("Error\n\r");
		}
	}else{

    }
	if (num_detected[NODE_MCMD1] == num_of->mcmd1 && num_detected[NODE_MCMD2] == num_of->mcmd2 &&
		    num_detected[NODE_MCMD3] == num_of->mcmd3 && num_detected[NODE_SERVO] == num_of->servo &&
		    num_detected[NODE_AIR] == num_of->air && num_detected[NODE_MCMD4]==num_of->mcmd4 &&num_detected[NODE_OTHER] == num_of->other){
		all_node_detected = 1;
	}
}

HAL_StatusTypeDef SendBytes(uint32_t ExtId, uint8_t *bytes, uint32_t size){ // 命令を送信する関数
	uint32_t quotient = size / 8;
	uint32_t remainder = size - (8 * quotient);
	HAL_StatusTypeDef ret;

	for (uint8_t i = 0; i < quotient; i++){
		ret = PushTx8Bytes(ExtId, bytes + i * 8, 8);
		if (ret != HAL_OK){
			Error_Handler();
			return ret;
		}
	}

	if(remainder != 0){
		ret = PushTx8Bytes(ExtId, bytes + quotient * 8, remainder);
		if (ret != HAL_OK){
			Error_Handler();
			return ret;
		}
	}

	ret = PopSendTx8Bytes(_canlib_phcan);
	if (ret != HAL_OK){
		Error_Handler();
		return ret;
	}

	return HAL_OK;
}

/**
 * @brief CANのシステムをアクティベートする. 具体的にはメッセージフィルターの設定とhal_can_start.
 *
 * @param _hcan
 * @param can_param
 */
void CAN_SystemInit(CAN_HandleTypeDef *_hcan){ // CANの初期化
	CAN_FilterTypeDef sFilterConfig;
    _canlib_phcan = _hcan;

	all_node_detected = 0;
	for (uint8_t type = 0; type < NODE_TYPES; type++){
		num_detected[type] = 0;
		for (uint8_t i = 0; i < 0b111; i++){
			node_id_list[type][i] = 0xff;
		}
	}

	//フィルタバンク設定
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	// FIFO0に初期化用のフィルタを設定
    // CAN2をつかうならSlaveStartFilterBank以上の値をFilterBankに設定する必要がある
	sFilterConfig.FilterBank = 15;
	sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_MCMD1, 0, 0, MCMD_CMD_AWAKE) >> 13; // 上16bit
	sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_MCMD1, 0, 0, 0b11111) >> 13;
	sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_MCMD1, 0, 0, MCMD_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
	sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_MCMD1, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
	if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
		/* Filter configuration Error */
		Error_Handler();
	}

	// FIFO0に初期化用のフィルタを設定
	sFilterConfig.FilterBank = 16;
	sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_MCMD2, 0, 0, MCMD_CMD_AWAKE) >> 13; // 上16bit
	sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_MCMD2, 0, 0, 0b11111) >> 13;
	sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_MCMD2, 0, 0, MCMD_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
	sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_MCMD2, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
	if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
		/* Filter configuration Error */
		Error_Handler();
	}

	// FIFO0に初期化用のフィルタを設定
	sFilterConfig.FilterBank = 17;
	sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_MCMD3, 0, 0, MCMD_CMD_AWAKE) >> 13;				   // 上16bit
	sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_MCMD3, 0, 0, 0b11111) >> 13;					   // TODO : 治す
	sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_MCMD3, 0, 0, MCMD_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
	sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_MCMD3, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
	if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
		/* Filter configuration Error */
		Error_Handler();
	}

	// FIFO0にfeedback用のフィルタを設定
	sFilterConfig.FilterBank = 18;
	sFilterConfig.FilterIdHigh = Make_CAN_ID(0, 0, 0, MCMD_CMD_FB) >> 13; // 上16bit
	sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(0, 0, 0, 0b11111) >> 13;
	sFilterConfig.FilterIdLow = (Make_CAN_ID(0, 0, 0, MCMD_CMD_FB) & 0x1FFF) << 3 | 0b100; // 下16bit
	sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(0, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
	if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
		/* Filter configuration Error */
		Error_Handler();
	}

	// FIFO0に初期化用のフィルタを設定
	sFilterConfig.FilterBank = 19;
	sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_SERVO, 0, 0, SERVO_CMD_AWAKE) >> 13; // 上16bit
	sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_SERVO, 0, 0, 0b11111) >> 13;
	sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_SERVO, 0, 0, SERVO_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
	sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_SERVO, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
	if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
		/* Filter configuration Error */
		Error_Handler();
	}

	// FIFO0に初期化用のフィルタを設定
	sFilterConfig.FilterBank = 20;
	sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_AIR, 0, 0, AIR_CMD_AWAKE) >> 13; // 上16bit
	sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_AIR, 0, 0, 0b11111) >> 13;
	sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_AIR, 0, 0, AIR_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
	sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_AIR, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
	if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
		/* Filter configuration Error */
		Error_Handler();
	}

	// FIFO0に初期化用のフィルタを設定
	sFilterConfig.FilterBank = 21;
	sFilterConfig.FilterIdHigh = Make_CAN_ID(NODE_MCMD4, 0, 0, AIR_CMD_AWAKE) >> 13; // 上16bit
	sFilterConfig.FilterMaskIdHigh = Make_CAN_ID(NODE_MCMD4, 0, 0, 0b11111) >> 13;
	sFilterConfig.FilterIdLow = (Make_CAN_ID(NODE_MCMD4, 0, 0, AIR_CMD_AWAKE) & 0x1FFF) << 3 | 0b100; // 下16bit
	sFilterConfig.FilterMaskIdLow = (Make_CAN_ID(NODE_MCMD4, 0, 0, 0b11111) & 0x1FFF) << 3 | 0b100;
	if (HAL_CAN_ConfigFilter(_canlib_phcan, &sFilterConfig) != HAL_OK){
		/* Filter configuration Error */
		Error_Handler();
	}

	if (HAL_CAN_Start(_canlib_phcan) != HAL_OK){
		printf(" -> Start Error\n");
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(_canlib_phcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){
		printf(" -> FIFO0 CAN_Activation error\n\r");
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(_canlib_phcan, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK){
		printf(" -> FIFO0 CAN_Activation error\n\r");
		Error_Handler();
	}
}

/**
 * @brief CANの全デバイスの接続が確認されるまで待つ.
 *
 */
void CAN_WaitConnect(NUM_OF_DEVICES *num_of){ // 他のデバイスが接続されるのを待つ
	if(num_of->mcmd1 == 0 && num_of->mcmd2 == 0 && num_of->mcmd3 == 0 && num_of->mcmd4 == 0 && num_of->air == 0
		&& num_of->servo == 0 && num_of->other == 0){
		all_node_detected = 1;
	}
	while (all_node_detected == 0){
		printf("Waiting CAN_NODES Wake Up...\n\r");
		HAL_Delay(500);
	}
	for (uint8_t i = 0; i < num_of->mcmd1; i++)
		printf("MCMD1 No.%d\n\r", node_id_list[NODE_MCMD1][i]);
	for (uint8_t i = 0; i < num_of->mcmd2; i++)
		printf("MCMD2 No.%d\n\r", node_id_list[NODE_MCMD2][i]);
	for (uint8_t i = 0; i < num_of->mcmd3; i++)
		printf("MCMD3 No.%d\n\r", node_id_list[NODE_MCMD3][i]);
	for (uint8_t i = 0; i < num_of->mcmd4; i++)
		printf("MCMD4 No.%d\n\r", node_id_list[NODE_MCMD4][i]);
	for (uint8_t i = 0; i < num_of->servo; i++)
		printf("Servo No.%d\n\r", node_id_list[NODE_SERVO][i]);
	for (uint8_t i = 0; i < num_of->air; i++)
		printf("Air No.%d\n\r", node_id_list[NODE_AIR][i]);
	for (uint8_t i = 0; i < num_of->other; i++)
		printf("Others No.%d\n\r", node_id_list[NODE_OTHER][i]);
}

//// MCMD
void MCMD_ChangeControl(MCMD_HandleTypedef *hmcmd){ // Ctrl typeを変更する.
	float fdata[2];
	fdata[0] = hmcmd->ctrl_param.PID_param.kp;
	fdata[1] = hmcmd->ctrl_param.PID_param.ki;
	SendBytes(Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_CHANGE_CTRL1), (uint8_t *)&fdata, sizeof(fdata));
	fdata[0] = hmcmd->ctrl_param.PID_param.kd;
	fdata[1] = hmcmd->ctrl_param.accel_limit_size;
	SendBytes(Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_CHANGE_CTRL2), (uint8_t *)&fdata, sizeof(fdata));

    fdata[0] = hmcmd->ctrl_param.PID_param.kff;
    fdata[1] = hmcmd->ctrl_param.gravity_compensation_gain;
    SendBytes(Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_CHANGE_CTRL3), (uint8_t *)&fdata, sizeof(fdata));

	uint8_t bdata[6];
	bdata[0] = hmcmd->ctrl_param.ctrl_type;
	bdata[1] = hmcmd->ctrl_param.accel_limit;
	bdata[2] = hmcmd->ctrl_param.feedback;
	bdata[3] = hmcmd->ctrl_param.timup_monitor;
	bdata[4] = hmcmd->fb_type;
    bdata[5] = hmcmd->ctrl_param.gravity_compensation; // TODO : new
	SendBytes(Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_CHANGE_CTRL4), (uint8_t *)&bdata, sizeof(bdata));
}

void MCMD_init(MCMD_HandleTypedef *hmcmd){
	uint8_t bdata[4];
	bdata[0] = hmcmd->enc_dir;
	bdata[1] = hmcmd->rot_dir;
	bdata[2] = hmcmd->calib;
	bdata[3] = hmcmd->limit_sw_type;
	SendBytes(Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_INIT1), bdata, sizeof(bdata));
	float fdata[2];
	fdata[0] = hmcmd->offset;
	fdata[1] = hmcmd->calib_duty;
	SendBytes(Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_INIT2), (uint8_t *)&fdata, sizeof(fdata));

	fdata[0] = hmcmd->quant_per_unit;
	fdata[1] = 0;
	SendBytes(Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_INIT3), (uint8_t *)&fdata, sizeof(fdata));
	HAL_Delay(50); // これないと動かない(なぜ?)
	MCMD_ChangeControl(hmcmd);
}

void MCMD_Calib(MCMD_HandleTypedef *hmcmd){
	uint8_t bdata[4];
	SendBytes(Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_CALIB), bdata, sizeof(bdata));
}

void MCMD_Control_Enable(MCMD_HandleTypedef *hmcmd){
	uint8_t bdata[4];
	SendBytes(Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_ENABLE), bdata, sizeof(bdata));
}

void MCMD_Control_Disable(MCMD_HandleTypedef *hmcmd){
	uint8_t bdata[4];
	SendBytes(Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_DISABLE), bdata, sizeof(bdata));
}

void MCMD_SetTarget(MCMD_HandleTypedef *hmcmd, float target){
	SendBytes(Make_CAN_ID_from_CAN_Device(&(hmcmd->device), MCMD_CMD_SET_TARGET), (uint8_t *)&target, sizeof(target));
}

MCMD_Feedback_Typedef Get_MCMD_Feedback(CAN_Device *can_device){
	MCMD_Feedback_Typedef ans;
	if (can_device->node_type == NODE_MCMD1){
		ans.fb_type = _feedback_table_mcmd1[(can_device->node_id)].feedback_motor[(can_device->device_num)].fb_type;
		ans.status = _feedback_table_mcmd1[(can_device->node_id)].feedback_motor[(can_device->device_num)].status;
		ans.value = _feedback_table_mcmd1[(can_device->node_id)].feedback_motor[(can_device->device_num)].value;
		return ans;
	}else if (can_device->node_type == NODE_MCMD2){
		return _feedback_table_mcmd2[(can_device->node_id)].feedback_motor[(can_device->device_num)];
	}else if (can_device->node_type == NODE_MCMD3){
		ans.fb_type = _feedback_table_mcmd3[(can_device->node_id)].feedback_motor[(can_device->device_num)].fb_type;
		ans.status = _feedback_table_mcmd3[(can_device->node_id)].feedback_motor[(can_device->device_num)].status;
		ans.value = _feedback_table_mcmd3[(can_device->node_id)].feedback_motor[(can_device->device_num)].value;
		return ans;
	}else if (can_device->node_type == NODE_MCMD4){
		ans.fb_type = _feedback_table_mcmd4[(can_device->node_id)].feedback_motor[(can_device->device_num)].fb_type;
		ans.status = _feedback_table_mcmd4[(can_device->node_id)].feedback_motor[(can_device->device_num)].status;
		ans.value = _feedback_table_mcmd4[(can_device->node_id)].feedback_motor[(can_device->device_num)].value;
		return ans;
	}else{
		ans.fb_type = 0;
		ans.status = 0;
		ans.value = 0.0f;
		printf("get feed back error\n\r");
		return ans;
	}
}

////servo
void ServoDriver_Init(CAN_Device *can_device, CANServo_Param_Typedef *param){
	float fdata[2];
	fdata[0] = param->pulse_width_min;
	fdata[1] = param->pulse_width_max;
	SendBytes(Make_CAN_ID_from_CAN_Device(can_device, SERVO_CMD_INIT1), (uint8_t *)fdata, sizeof(fdata));
	fdata[0] = param->pwm_frequency;
	fdata[1] = param->angle_range;
	SendBytes(Make_CAN_ID_from_CAN_Device(can_device, SERVO_CMD_INIT2), (uint8_t *)fdata, sizeof(fdata));
	fdata[0] = param->angle_offset;
	fdata[1] = 0.0f;
	SendBytes(Make_CAN_ID_from_CAN_Device(can_device, SERVO_CMD_INIT3), (uint8_t *)fdata, sizeof(fdata));
}

void ServoDriver_SendValue(CAN_Device *can_device, float angle){
	if (SendBytes(Make_CAN_ID_from_CAN_Device(can_device, SERVO_CMD_SET_TARGET), (uint8_t *)(&angle), sizeof(float)) != HAL_OK){
		Error_Handler();
	}
}

/* AirCylinderは初期化の際に, ON状態, もしくはOFF状態のどちらにするかは
 * 機構の初期状態に依存するので, 最初にON or OFFを指定してあげないといけない.
 * 従って, Air_Cylinder()とAirCylinder_SendOutput()の中身は実質同じとなっているが,
 * 初期化するという操作をすべてのデバイスに共通のものとして導入するために,
 * 異なる関数名を用いて明示的にしてある.
 * また, AirCylinderの基板の方にも, 最初は必ず初期化処理(AIR_CMD_INIT)が来るものとして定義してある.
 */

////AirCylinder
void AirCylinder_Init(CAN_Device *can_device, Air_PortStatus_Typedef param){
	if (SendBytes(Make_CAN_ID_from_CAN_Device(can_device, AIR_CMD_INIT), (uint8_t *)(&param), sizeof(Air_PortStatus_Typedef)) != HAL_OK){
		Error_Handler();
	}
}

void AirCylinder_SendOutput(CAN_Device *can_device, Air_PortStatus_Typedef param){
	if (SendBytes(Make_CAN_ID_from_CAN_Device(can_device, AIR_CMD_OUTPUT), (uint8_t *)(&param), sizeof(Air_PortStatus_Typedef)) != HAL_OK){
		Error_Handler();
	}
}

/* 実装予定なし
void AirCylinder_SendOutput_withTimeLimit(CAN_Device* can_device, AIRCYLINDER_OUTPUT output, uint16_t time_ms){  // TODO : 未完成
	AirCylinder_CMD_Typedef cmd;
	cmd.cylinder_num=can_device->device_num;
	cmd.output=output;
	cmd.time_limit=time_ms;
	cmd.time_limit_enable=aircylinder_time_limit_enable;
	if(SendBytes(Make_CAN_ID_from_CAN_Device(can_device,air_cmd_output), (uint8_t*)(&cmd), sizeof(AirCylinder_CMD_Typedef))!= HAL_OK){
		Error_Handler();
	}
}
*/
