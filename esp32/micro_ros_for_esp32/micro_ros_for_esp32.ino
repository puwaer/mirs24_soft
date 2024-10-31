#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <rosidl_runtime_c/string_functions.h>
#include <tf2_msgs/msg/tf_message.h>
//#include <std_msgs/msg/int32_multi_array.h>

#include "quaternion.h"
#include "define.h"

rcl_publisher_t odom_pub;
rcl_publisher_t tf_broadcaster;
rcl_publisher_t enc_pub;
nav_msgs__msg__Odometry odom_msg;             //オドメトリ
geometry_msgs__msg__TransformStamped odom_tf; //tf変換
std_msgs__msg__Int32MultiArray enc_msg;       //エンコーダー情報
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
int enc_l, enc_r;
int count_l,count_r;
int last_count_l,last_count_r;

double x_ = 0;
double y_ = 0;
double theta_ = 0;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //オドメトリ計算
    calculate_odometry();
    //エンコーダーデータを格納
    enc_msg.data.data[0] = count_l;
    enc_msg.data.data[1] = count_r;
    rcl_publish(&enc_pub, &enc_msg, NULL);
    rcl_publish(&odom_pub, &odom_msg, NULL);
    rcl_publish(&tf_broadcaster, &odom_tf, NULL);
  }
}

void calculate_odometry(){

  //エンコーダーの変化量を計算
  int delta_left = count_l - last_count_l;
  int delta_right = count_r - last_count_r;

  last_count_l = count_l;
  last_count_r = count_r;

  // 車輪の回転角度（ラジアン）を計算
  double delta_left_rad = (delta_left / COUNTS_PER_REV) * 2.0 * PI;
  double delta_right_rad = (delta_right / COUNTS_PER_REV) * 2.0 * PI;

  // それぞれの車輪の移動距離を計算
  double left_distance = delta_left_rad * WHEEL_RADIUS;
  double right_distance = delta_right_rad * WHEEL_RADIUS;

  // ロボットの移動距離と回転量を計算
  double delta_distance = (left_distance + right_distance) / 2.0;
  double delta_theta = (right_distance - left_distance) / WHEEL_BASE;

  // ロボットの位置と姿勢を更新
  x_ += delta_distance * cos(theta_);
  y_ += delta_distance * sin(theta_);
  theta_ += delta_theta;

  // オドメトリメッセージの作成
  //odom_msg.header.stamp = this->get_clock()->now();
  //odom_msg.header.frame_id = "odom";
  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");

  // 位置を設定
  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  //odom_msg.pose.pose.position.z = 0.0;

  // オリエンテーション（四元数）
  Quaternion q = toQuaternion(theta_, 0, 0);
  odom_msg.pose.pose.orientation.x = q.x;
  odom_msg.pose.pose.orientation.y = q.y;
  odom_msg.pose.pose.orientation.z = q.z;
  odom_msg.pose.pose.orientation.w = q.w;

  //odom_msg.child_frame_id = "base_link";
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");


  // 速度を設定（今回は仮で0としていますが、実際はエンコーダの変化量から計算します）
  odom_msg.twist.twist.linear.x = delta_distance / 0.1;  // 0.1秒周期なので0.1で割る
  odom_msg.twist.twist.angular.z = delta_theta / 0.1;

  // オドメトリをパブリッシュ
  //odom_pub_->publish(odom_msg);

  // TF変換のパブリッシュ

  //odom_tf.header.stamp = this->get_clock()->now();
  rosidl_runtime_c__String__assign(&odom_tf.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&odom_tf.child_frame_id, "base_link");
  //odom_tf.header.frame_id = "odom";
  //odom_tf.child_frame_id = "base_link";

  odom_tf.transform.translation.x = x_;
  odom_tf.transform.translation.y = y_;
  //odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation.x = q.x;
  odom_tf.transform.rotation.y = q.y;
  odom_tf.transform.rotation.z = q.z;
  odom_tf.transform.rotation.w = q.w;

  //tf_broadcaster_->sendTransform(odom_tf);
}

void setup() {
  Serial.begin(115200); // シリアル通信を初期化
  encoder_open();
  set_microros_transports();
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "ESP32_odom_node", "", &support);

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

  const unsigned int timer_timeout = 1000;

  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  enc_msg.data.size = 2; // メッセージ配列のサイズを3に設定
  enc_msg.data.data = (int32_t *)malloc(enc_msg.data.size * sizeof(int32_t)); // 配列のメモリを確保
  enc_msg.data.data[0] = 0;
  enc_msg.data.data[1] = 0;
}

void loop() {
  delay(100);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
