#include <micro_ros_arduino.h>
#include <define.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>

rclc_executor_t executor;

void setup() {
  Serial.begin(115200); // シリアル通信を初期化
  encoder_open();  
  send_setup();

  delay(1000);  // 一秒待機
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(100);
}
