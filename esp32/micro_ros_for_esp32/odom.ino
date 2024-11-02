double x_ = 0;
double y_ = 0;
double theta_ = 0;


void calculate_odometry(){

  //エンコーダーの変化量を計算
  int32_t delta_left = count_l - last_count_l;
  int32_t delta_right = count_r - last_count_r;

  last_count_l = count_l;
  last_count_r = count_r;

  // 車輪の回転角度（ラジアン）を計算
  double delta_left_rad = (delta_left / COUNTS_PER_REV) * 2.0 * PI;
  double delta_right_rad = (delta_right / COUNTS_PER_REV) * 2.0 * PI;

  // それぞれの車輪の移動距離を計算
  left_distance = delta_left_rad * WHEEL_RADIUS;
  right_distance = delta_right_rad * WHEEL_RADIUS;

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

void odometry_set(){
  enc_msg.data.size = 2; // メッセージ配列のサイズを3に設定
  enc_msg.data.data = (int32_t *)malloc(enc_msg.data.size * sizeof(int32_t)); // 配列のメモリを確保
  enc_msg.data.data[0] = 0;
  enc_msg.data.data[1] = 0;
}