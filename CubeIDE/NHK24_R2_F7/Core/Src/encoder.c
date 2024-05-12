/*
 * encoder.c
 *
 *  Created on: 2024/02/18
 *      Author: ryose
 */

#include "encoder.h"
#include "stdio.h"


void EncoderUpdateData(Enc_HandleTypedef* enc_state) {
	int32_t present_cnt = __HAL_TIM_GET_COUNTER(enc_state->Init.htim);
	int32_t diff_cnt;
	volatile float diff;

	diff_cnt = enc_state->Init.cnt_dir * (present_cnt - enc_state->prev_cnt);
	diff = (float)diff_cnt * enc_state->Init.value_per_pulse;

	enc_state->vel = diff / enc_state->Init.update_freq;
	enc_state->pos += diff;

	enc_state->prev_cnt = present_cnt;
	if ((present_cnt > (ENC_CNT_PERIOD-ENC_CNT_MARGIN)) || (present_cnt < ENC_CNT_MARGIN)) {
		__HAL_TIM_SET_COUNTER(enc_state->Init.htim, ENC_CNT_RESET);
		enc_state->prev_cnt = ENC_CNT_RESET;
	}
}

void EncoderEnable(Enc_HandleTypedef* enc_state) {
	printf("Encoder : Enable\n\r");
	HAL_TIM_Encoder_Start((enc_state->Init.htim), TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(enc_state->Init.htim, ENC_CNT_RESET);
	enc_state->prev_cnt = ENC_CNT_RESET;
	enc_state->pos = 0;
}

void EncoderDisable(Enc_HandleTypedef* enc_state) {
	printf("Encoder : Disable\n\r");
	HAL_TIM_Encoder_Stop(enc_state->Init.htim, TIM_CHANNEL_ALL);
}
