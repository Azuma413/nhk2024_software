/*
 * c620_utils.h
 *
 *  Created on: 2024/02/11
 *      Author: ryose
 */

#ifndef INC_C620_UTILS_H_
#define INC_C620_UTILS_H_

#include "CAN_C620.h"
#include "CAN_C620_Def.h"
#include "math.h"
#include "stdio.h"

#define RADIUS 24.0

extern const uint8_t num_of_c620;

extern C620_DeviceInfo c620_dev_info_global[8];
extern CAN_HandleTypeDef hcan1;

void Linearmovement(C620_DeviceInfo omni_info_array[num_of_c620], float v_x, float v_y, float v_theta);
void Rotation_POS(C620_DeviceInfo omni_info_array[num_of_c620], float angle);
void Rotation_Current(C620_DeviceInfo omni_info_array[num_of_c620], float angle);

#endif /* INC_C620_UTILS_H_ */
