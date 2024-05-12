/*
 * c620_utils.c
 *
 *  Created on: 2024/02/11
 *      Author: ryose
 */

#include "c620_utils.h"

//TODO:４輪のみ実装->n輪
//[cm/s] ver
//void Linearmovement(C620_DeviceInfo c620_dev_info_global_array[num_of_c620], float v_x, float v_y, float v_theta){
//	  C620_SetTarget(&c620_dev_info_global_array[0], sqrt(2.0)*(v_x+v_y)/4-RADIUS*v_theta);
//	  C620_SetTarget(&c620_dev_info_global_array[1], sqrt(2.0)*(-v_x+v_y)/4-RADIUS*v_theta);
//	  C620_SetTarget(&c620_dev_info_global_array[2], sqrt(2.0)*(-v_x+v_y)/4+RADIUS*v_theta);
//	  C620_SetTarget(&c620_dev_info_global_array[3], sqrt(2.0)*(v_x+v_y)/4+RADIUS*v_theta);
//}

//[m/s] ver
void Linearmovement(C620_DeviceInfo c620_dev_info_global_array[num_of_c620], float v_x, float v_y, float v_theta){
	  C620_SetTarget(&c620_dev_info_global_array[0], (sqrt(2.0)*(v_x+v_y)/4)*100-RADIUS*v_theta);
	  C620_SetTarget(&c620_dev_info_global_array[1], (sqrt(2.0)*(-v_x+v_y)/4)*100-RADIUS*v_theta);
	  C620_SetTarget(&c620_dev_info_global_array[2], (sqrt(2.0)*(-v_x+v_y)/4)*100+RADIUS*v_theta);
	  C620_SetTarget(&c620_dev_info_global_array[3], (sqrt(2.0)*(v_x+v_y)/4)*100+RADIUS*v_theta);
}


void Rotation_POS(C620_DeviceInfo c620_dev_info_global_array[num_of_c620], float angle){//正面を0として反時計回りが正
	  C620_FeedbackData previous_data[num_of_c620];
	  C620_FeedbackData c620_fb[num_of_c620];
	  printf("Rotation:Start\n\r");
	  for(int i=0; i<num_of_c620; i++){
		  C620_ControlDisable(&c620_dev_info_global[i]);
		  printf("C620 Control:Disabled(VEL)\n\r");
		  c620_dev_info_global[i].ctrl_param.rotation = C620_ROT_CW;//機体の回転が反時計回りで正になるように
		  c620_dev_info_global[i].ctrl_param.pid_vel.kp = 5.0f;//位置制御の場合はpid_velに速度制御用のgainを設定する
		  c620_dev_info_global[i].ctrl_param.pid_vel.ki = 0.0f;
		  c620_dev_info_global[i].ctrl_param.pid_vel.kd = 0.0f;
		  c620_dev_info_global[i].ctrl_param.pid_vel.kff = 0.0f;

		  c620_dev_info_global[i].ctrl_param.pid.kp = 5.0f;//位置制御用
		  c620_dev_info_global[i].ctrl_param.pid.ki = 0.0f;
		  c620_dev_info_global[i].ctrl_param.pid.kd = 0.0f;
		  c620_dev_info_global[i].ctrl_param.pid.kff = 0.0f;

		  C620_ChangeControl(&c620_dev_info_global[i], C620_CTRL_POS);
		  previous_data[i]= Get_C620_FeedbackData(&c620_dev_info_global_array[i]);
		  C620_SetTarget(&c620_dev_info_global[i], RADIUS*angle+previous_data[i].position);
		  //C620_SetTarget(&c620_dev_info_global[i], 50.0f);
		  C620_ControlEnable(&c620_dev_info_global[i]);
	  }
	  printf("C620 Control:Enabled(POS)\n\r");
	  float order[num_of_c620];
	  for(int i=0; i<num_of_c620; i++){
		  order[i]=RADIUS*angle+previous_data[i].position;
	  }


	  while(fabs(order[0]-Get_C620_FeedbackData(&c620_dev_info_global_array[0]).position)>0.10
		 || fabs(order[1]-Get_C620_FeedbackData(&c620_dev_info_global_array[1]).position)>0.10
	 	 || fabs(order[2]-Get_C620_FeedbackData(&c620_dev_info_global_array[2]).position)>0.10
		 || fabs(order[3]-Get_C620_FeedbackData(&c620_dev_info_global_array[3]).position)>0.10){
		  C620_SendRequest(c620_dev_info_global, num_of_c620, 1000.0f, &hcan1);//制御する
		  for(int i=0; i<num_of_c620; i++){
			  c620_fb[i]=Get_C620_FeedbackData(&c620_dev_info_global[i]);//フィードバックを受け取る
			  printf("omni%d : %d\n\r", i, (int)(c620_fb[i].position*1000000));
		  }
	  }
	  printf("Rotation:Complete\n\r");
	  for(int i=0; i<num_of_c620; i++){
	      C620_ControlDisable(&c620_dev_info_global[i]);
	      printf("C620 Control:Disabled(POS)\n\r");
		  c620_dev_info_global[0].ctrl_param.rotation = C620_ROT_ACW;//半時計周り
		  c620_dev_info_global[1].ctrl_param.rotation = C620_ROT_ACW;//半時計周り
		  c620_dev_info_global[2].ctrl_param.rotation = C620_ROT_CW;//時計周り
		  c620_dev_info_global[3].ctrl_param.rotation = C620_ROT_CW;//時計周り
		  c620_dev_info_global[i].ctrl_param.pid_vel.kp = 0.5f;
		  c620_dev_info_global[i].ctrl_param.pid_vel.ki = 0.0f;
		  c620_dev_info_global[i].ctrl_param.pid_vel.kd = 0.0f;
		  c620_dev_info_global[i].ctrl_param.pid_vel.kff = 0.0f;

		  c620_dev_info_global[i].ctrl_param.pid.kp = 5.0f;
		  c620_dev_info_global[i].ctrl_param.pid.ki = 0.0f;
		  c620_dev_info_global[i].ctrl_param.pid.kd = 0.0f;
		  c620_dev_info_global[i].ctrl_param.pid.kff = 0.0f;

	      C620_ChangeControl(&c620_dev_info_global[i], C620_CTRL_VEL);
	  	  C620_ControlEnable(&c620_dev_info_global[i]);
	  }
	  printf("C620 Control:Enabled(VEL)\n\r");
}
void Rotation_Current(C620_DeviceInfo c620_dev_info_global_array[num_of_c620], float angle){//正面を0として反時計回りが正
	  C620_FeedbackData previous_data[num_of_c620];
	  C620_FeedbackData c620_fb[num_of_c620];
	  printf("Rotation:Start\n\r");
	  for(int i=0; i<num_of_c620; i++){
		  C620_ControlDisable(&c620_dev_info_global[i]);
		  printf("C620 Control:Disabled(VEL)\n\r");
		  c620_dev_info_global[i].ctrl_param.rotation = C620_ROT_CW;//機体の回転が反時計回りで正になるように
		  C620_ChangeControl(&c620_dev_info_global[i], C620_CTRL_CURRENT);
		  previous_data[i]= Get_C620_FeedbackData(&c620_dev_info_global_array[i]);
		  C620_SetTarget(&c620_dev_info_global[i], 1.0f);
		  C620_ControlEnable(&c620_dev_info_global[i]);
	  }
	  printf("C620 Control:Enabled(CURRENT)\n\r");
	  float order[num_of_c620];
	  for(int i=0; i<num_of_c620; i++){
		  order[i]=RADIUS*angle+previous_data[i].position;
	  }
	  while(fabs(order[0]-Get_C620_FeedbackData(&c620_dev_info_global_array[0]).position)>0.10
		 || fabs(order[1]-Get_C620_FeedbackData(&c620_dev_info_global_array[1]).position)>0.10
		 || fabs(order[2]-Get_C620_FeedbackData(&c620_dev_info_global_array[2]).position)>0.10
		 || fabs(order[3]-Get_C620_FeedbackData(&c620_dev_info_global_array[3]).position)>0.10){
		  C620_SendRequest(c620_dev_info_global, num_of_c620, 1000.0f, &hcan1);//制御する
		  for(int i=0; i<num_of_c620; i++){
			  c620_fb[i]=Get_C620_FeedbackData(&c620_dev_info_global[i]);//フィードバックを受け取る
			  printf("omni%d : %d\n\r", i, (int)(c620_fb[i].position*1000000));
		  }
	  }
	  printf("Rotation:Complete\n\r");
	  for(int i=0; i<num_of_c620; i++){
	      C620_ControlDisable(&c620_dev_info_global[i]);
	      printf("C620 Control:Disabled(POS)\n\r");
		  c620_dev_info_global[0].ctrl_param.rotation = C620_ROT_ACW;//半時計周り
		  c620_dev_info_global[1].ctrl_param.rotation = C620_ROT_ACW;//半時計周り
		  c620_dev_info_global[2].ctrl_param.rotation = C620_ROT_CW;//時計周り
		  c620_dev_info_global[3].ctrl_param.rotation = C620_ROT_CW;//時計周り
		  c620_dev_info_global[i].ctrl_param.pid_vel.kp = 0.5f;
		  c620_dev_info_global[i].ctrl_param.pid_vel.ki = 0.0f;
		  c620_dev_info_global[i].ctrl_param.pid_vel.kd = 0.0f;
		  c620_dev_info_global[i].ctrl_param.pid_vel.kff = 0.0f;

		  c620_dev_info_global[i].ctrl_param.pid.kp = 5.0f;
		  c620_dev_info_global[i].ctrl_param.pid.ki = 0.0f;
		  c620_dev_info_global[i].ctrl_param.pid.kd = 0.0f;
		  c620_dev_info_global[i].ctrl_param.pid.kff = 0.0f;

	      C620_ChangeControl(&c620_dev_info_global[i], C620_CTRL_VEL);
	  	  C620_ControlEnable(&c620_dev_info_global[i]);
	  }
	  printf("C620 Control:Enabled(VEL)\n\r");
}

