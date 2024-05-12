/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point.h>


#include <usart.h>

#include "can_utils.h"
#include "can.h"

#include "math.h"

#include "c620_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

rcl_publisher_t publisher_enc;

//TODO:publisher

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
uint32_t LEDTaskBuffer[ 128 ];
osStaticThreadDef_t LEDTaskControlBlock;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .cb_mem = &LEDTaskControlBlock,
  .cb_size = sizeof(LEDTaskControlBlock),
  .stack_mem = &LEDTaskBuffer[0],
  .stack_size = sizeof(LEDTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for C620Timer */
osTimerId_t C620TimerHandle;
osStaticTimerDef_t C620TimerControlBlock;
const osTimerAttr_t C620Timer_attributes = {
  .name = "C620Timer",
  .cb_mem = &C620TimerControlBlock,
  .cb_size = sizeof(C620TimerControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void pub_timer_callback_enc(rcl_timer_t * timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
    geometry_msgs__msg__Point feedback_msg;

    if (timer != NULL) {

        feedback_msg.x = 0.0f;
        feedback_msg.y = 0.0f;
//        feedback_msg.z.data = ;     //zはいらない
        RCSOFTCHECK(rcl_publish(&publisher_enc, &feedback_msg, NULL));

    }
}

void subscription_callback_air(const void * msgin)
{
	 // Cast received message to used type
	  const std_msgs__msg__Bool * air = (const std_msgs__msg__Bool *)msgin;
	  static Air_PortStatus_Typedef air_status[2];

	  MCMD_SetTarget(&(mcmd_handlers[1]), 0.4f);

	  air_status[0] = (air->data)? AIR_OFF : AIR_ON;
	  air_status[1] = (air->data)? AIR_OFF : AIR_ON;

	  if(air->data) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

	  int i=0;
	  for(int j=0;j<2;j++){       //TODO: j<6は手動
		  air_devices[i].device_num = j;
		  AirCylinder_SendOutput(&air_devices[i], air_status[j]);
	  }


}

void subscription_callback_table(const void * msgin)
{
	 // Cast received message to used type
	  const std_msgs__msg__Float64 * zrot = (const std_msgs__msg__Float64 *)msgin;
	  static float zrot_rad;
	  zrot_rad = zrot->data;

	  MCMD_SetTarget(&(mcmd_handlers[0]), zrot_rad);

}

void subscription_callback(const void * msgin)
{
	 // Cast received message to used type
	  const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *)msgin;
	  static float v_x,v_y,v_theta;

	  v_x = twist->linear.x;
	  v_y = twist->linear.y;
	  v_theta = twist->angular.z;

	  Linearmovement(c620_dev_info_global, v_x, v_y, v_theta);

	  if(v_x == 100.0) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

}

//void subscription_callback_ems(const void * msgin)
//{
//	 // Cast received message to used type
//	  const std_msgs__msg__Bool * ems = (const std_msgs__msg__Bool *)msgin;
//
//
//
//}

//subscriber template

//void subscription_callback(const void * msgin)
//{
//	 // Cast received message to used type
//	  const std_msgs__msg__Int32MultiArray * rotvels = (const std_msgs__msg__Int32MultiArray *)msgin;
//
//
//
//}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartLEDTask(void *argument);
void C620TimerCallback(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of C620Timer */
  C620TimerHandle = osTimerNew(C620TimerCallback, osTimerPeriodic, NULL, &C620Timer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
	 rmw_uros_set_custom_transport(
			true,
			(void *) &huart3,
			cubemx_transport_open,
			cubemx_transport_close,
			cubemx_transport_write,
			cubemx_transport_read);

	// micro-ROS connection check
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);  // LD3 (RED) -> ON
	while(1) {
		rmw_ret_t ping_result = rmw_uros_ping_agent(1000, 5);  // ping Agent
		if(ping_result == RMW_RET_OK){
			break;
		}
	}
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);  // LD3 (RED) -> OFF


	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		printf("Error on default allocators (line %d)\n", __LINE__);
	}
	printf("start Micro-ROS Task\n");

	// micro-ROS app
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	rclc_support_t support;
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rcl_node_t node;
	rcl_node_options_t node_ops = rcl_node_get_default_options();

	// // node setting
	// RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));  //create init_options
	// RCCHECK(rclc_node_init_default(&node, "f7_mros_node_r2", "", &support));  // create node

	// node setting
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	RCCHECK(rcl_init_options_set_domain_id(&init_options, 30)); // ROS_DOMAIN_IDの設定。今回は123としてる。
	rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
	RCCHECK(rclc_node_init_with_options(&node, "f7_mros_node_r2", "", &support, &node_ops));

	// create executor
	rclc_executor_t executor;
	unsigned int num_handlers = 4; // TODO : 忘れずに変更
	RCCHECK(rclc_executor_init(&executor, &support.context, num_handlers, &allocator));

	// create subscriber for air
	rcl_subscription_t subscriber_air;
	const char* sub_name_air= "mros_input_air_r2";
	std_msgs__msg__Bool actuator_msg_air;
	RCCHECK(rclc_subscription_init_default(&subscriber_air, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), sub_name_air));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_air, &actuator_msg_air, &subscription_callback_air, ON_NEW_DATA));

	// create subscriber for table
	rcl_subscription_t subscriber_table;
	const char* sub_name_table = "mros_input_table_r2";
	std_msgs__msg__Float64 actuator_msg_table;
	RCCHECK(rclc_subscription_init_default(&subscriber_table, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), sub_name_table));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_table, &actuator_msg_table, &subscription_callback_table, ON_NEW_DATA));

	// create subscriber for foots
	rcl_subscription_t subscriber_vel;
	const char* sub_name_f = "cmd_vel_r2";
	geometry_msgs__msg__Twist actuator_msg_f;
	RCCHECK(rclc_subscription_init_default(&subscriber_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), sub_name_f));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_vel, &actuator_msg_f, &subscription_callback, ON_NEW_DATA));

    // publisher for enc
    const char* topic_name_pub_enc = "mros_output_enc_r2";
    RCCHECK(rclc_publisher_init_default(&publisher_enc, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), topic_name_pub_enc));
    rcl_timer_t timer_enc;
    RCCHECK(rclc_timer_init_default(&timer_enc, &support, RCL_MS_TO_NS(45), pub_timer_callback_enc));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_enc));

	//rclc_executor_spin(&executor);
  	while(1){
          // エグゼキューターを実行してリクエストを処理
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
          osDelay(10);
    //      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
  	}

	// free resources
	RCCHECK(rclc_executor_fini(&executor));
	RCCHECK(rcl_subscription_fini(&subscriber_air, &node));
	RCCHECK(rcl_subscription_fini(&subscriber_table, &node));
	RCCHECK(rcl_subscription_fini(&subscriber_vel, &node));
	RCCHECK(rcl_publisher_fini(&publisher_enc, &node));
	RCCHECK(rcl_timer_fini(&timer_enc));
	RCCHECK(rcl_node_fini(&node));

	//  /* Infinite loop */
	//  for(;;)
	//  {
	//    osDelay(1);
	//  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void *argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */
  for(;;)
  {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);  // LD2 (Blue)
      osDelay(10);
      C620_SendRequest(c620_dev_info_global, 4, 1000.0f, &hcan1);
  }
  /* USER CODE END StartLEDTask */
}

/* C620TimerCallback function */
void C620TimerCallback(void *argument)
{
  /* USER CODE BEGIN C620TimerCallback */
   // C620_SendRequest(c620_dev_info_global, 4, 1000.0f, &hcan1);
  /* USER CODE END C620TimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

