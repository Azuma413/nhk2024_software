#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/quaternion.h>

#include <SparkFunMPU9250-DMP.h>
#define SerialPort Serial
#define LED_PIN 2
MPU9250_DMP imu;

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

#define LED_PIN 2 //マイコン上の青いLEDを指定
#define AD_PIN 34 //ADCのピンを指定

//エラーハンドリング用のマクロ
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//micro-ROS関連で必要となる変数を宣言しておく
rcl_publisher_t publisher_imu,publisher_line;
geometry_msgs__msg__Quaternion msg_imu,msg_line;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

//micro-ROS関連でエラーが出るとマイコン上の青色LEDが点滅するように設定している
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


void setup() {
  set_microros_transports();
  
  //LEDの設定
  pinMode(LED_PIN, OUTPUT);
  //ADCの設定
  analogSetAttenuation(ADC_6db); //追加
  pinMode(AD_PIN, ANALOG); //追加
  
  delay(1000);

  SerialPort.begin(115200);
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(500);
    }
  }

  delay(1000);


//IMU setting
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              10); // Set DMP FIFO rate to 10 Hz



  // allocator = rcl_get_default_allocator();

  // //create init_options
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // // create node
  // RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // // create publisher
  // RCCHECK(rclc_publisher_init_default(
  //   &publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //   "micro_ros_arduino_node_publisher"));

  // // create timer,
  // const unsigned int timer_timeout = 1000;
  // RCCHECK(rclc_timer_init_default(
  //   &timer,
  //   &support,
  //   RCL_MS_TO_NS(timer_timeout),
  //   timer_callback));

  // // create executor
  // RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));

//micro-ROSの設定
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  //初期化オプションの作成
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 30)); //ROS_DOMAIN_IDを設定できる。今回は123としている。
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  //ノードの作成
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  //publisherの作成
  RCCHECK(rclc_publisher_init_best_effort( //TODO:best_effortの必要性
    &publisher_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "/mros_output_imu"));

  msg.data = {0,0,0,0};

  RCCHECK(rclc_publisher_default(
    &publisher_line,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "/mros_output_line"));

  msg.data = {0,0,0,0};

  //セットアップが完了するとLEDが点灯する
  digitalWrite(LED_PIN, HIGH);
}



void printIMUData(void)
{  
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);
  SerialPort.println(String(q0,6) + "," + String(q1,6) + "," + String(q2,6) + "," + String(q3,6));
}

void get_imudata()
{
  msg_imu.w = imu.calcQuat(imu.qw);
  msg_imu.x = imu.calcQuat(imu.qx);
  msg_imu.y = imu.calcQuat(imu.qy);
  msg_imu.z = imu.calcQuat(imu.qz);
}

// calurate line
float calc_prob(vector<float> ph_data){
  for(auto ph_data:x) total_p += x;
  return total_p;
}

void get_linedata()
{
  //TODO:存在確率の計算実装→その際配列を使うのは適切か？
  // msg_line.w = calc_prob();
  // msg_line.x = calc_prob();
  // msg_line.y = calc_prob();
  // msg_line.z = calc_prob();
}

void loop() 
{
  //imu
  if ( imu.fifoAvailable() )
  {
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      imu.computeEulerAngles();
      get_imudata();
      RCSOFTCHECK(rcl_publish(&publisher_imu, &msg_imu, NULL));
      printIMUData();
    }
  }

  //line
  get_linedata();
  RCSOFTCHECK(rcl_publish(&publisher_line, &msg_line, NULL));
}
