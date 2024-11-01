#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <PID_v1_bc.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <rosidl_runtime_c/string_functions.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/twist.h>

#include "quaternion.h"
#include "define.h"

nav_msgs__msg__Odometry odom_msg;             //オドメトリ
geometry_msgs__msg__TransformStamped odom_tf; //tf変換
std_msgs__msg__Int32MultiArray enc_msg;       //エンコーダー情報
geometry_msgs__msg__Twist vel_msg;            //速度指令値

rcl_publisher_t odom_pub;
rcl_publisher_t tf_broadcaster;
rcl_publisher_t enc_pub;
rcl_subscription_t subscriber;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

int count_l,count_r;
int last_count_l,last_count_r;

double left_distance;
double right_distance;

// PID制御用の変数
double r_vel_cmd;
double l_vel_cmd;
double r_vel;
double l_vel;
double r_pwm;
double l_pwm;

int abc,def;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //オドメトリ計算
    calculate_odometry();
    //エンコーダーデータを格納
    enc_msg.data.data[0] = r_pwm;
    enc_msg.data.data[1] = count_r;
    rcl_publish(&enc_pub, &enc_msg, NULL);
    rcl_publish(&odom_pub, &odom_msg, NULL);
    rcl_publish(&tf_broadcaster, &odom_tf, NULL);
  }
}

void setup() {
  Serial.begin(115200); // シリアル通信を初期化
  encoder_open();
  set_microros_transports();
  delay(2000);

  //micro-ROSのセットアップ
  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "ESP32_node", "", &support);

  //サブスクライバとパブリッシャーの宣言
  rclc_publisher_init_default(
    &odom_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom"
  );

  rclc_publisher_init_default(
    &tf_broadcaster,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TransformStamped),
    "/tf"
  );

  rclc_publisher_init_default(
    &enc_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/encoder"
  );

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  );



  const unsigned int timer_timeout = 100;

  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback
  );

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &vel_msg, &cmd_vel_Callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  odometry_set();
  cmd_vel_set();

  delay(2000);

}

void loop() {
  delay(10);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
