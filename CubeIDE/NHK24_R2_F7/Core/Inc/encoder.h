/*
 * encoder.h
 *
 *  Created on: 2020/03/09
 *      Author: nabeya11
 */

#ifndef ENCODER_H_
#define ENCODER_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Exported macro ------------------------------------------------------------*/
#define __ENC_GET_POSITION(__ENC_STRUCT__) ((__ENC_STRUCT__)->pos)
#define __ENC_GET_VELOCITY(__ENC_STRUCT__) ((__ENC_STRUCT__)->vel)

#define __ENC_SET_POSITION(__ENC_STRUCT__, __VAULE__) ((__ENC_STRUCT__)->pos = (__VAULE__))

/* Exported constants --------------------------------------------------------*/
#define ENC_CNT_PERIOD			65536
//リセット値
#define ENC_CNT_RESET			(ENC_CNT_PERIOD / 2)
#define ENC_CNT_MARGIN 5000

//正回転方向の指定
typedef enum{
	ENC_FW_CNTUP			=  1,		//正回転はカウントアップ
	ENC_FW_CNTDOWN	= -1		//正回転はカウントダウン
}Enc_FW;

//値の変換先 meter/radian
typedef enum{
	ENC_CNVT_METER,
	ENC_CNVT_RADIAN
}Enc_Cnvt;

/* Exported types ------------------------------------------------------------*/
typedef struct{
	TIM_HandleTypeDef* htim;//TIMHandler
	float update_freq;	//割り込み間隔(s)
	//hard parameter
	Enc_FW cnt_dir;				//1.カウント方向
	float value_per_pulse;	//2.一パルス当たりの進度
}Enc_InitTypedef;

typedef struct{
	Enc_InitTypedef Init;
	//encoder data
	int32_t prev_cnt;
	float pos;			//a.位置
	float vel;			//b.速度
}Enc_HandleTypedef;

/* Exported functions prototypes ---------------------------------------------*/
void EncoderUpdateData(Enc_HandleTypedef *);
void EncoderEnable(Enc_HandleTypedef *);
void EncoderDisable(Enc_HandleTypedef *);


#ifdef __cplusplus
}
#endif
#endif /* ENCODER_H_ */
