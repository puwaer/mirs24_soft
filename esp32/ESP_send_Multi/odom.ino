rcl_publisher_t publisher;
std_msgs__msg__Int32MultiArray msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
int enc_l, enc_r;

void error_loop(){
  while(1){
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    msg.data.data[0] = enc_l;
    msg.data.data[1] = enc_r;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

void send_setup() {
  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "LR_encoder"));

    
  // create timer,
  const unsigned int timer_timeout = 500;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data.size = 2; // メッセージ配列のサイズを3に設定
  msg.data.data = (int32_t *)malloc(msg.data.size * sizeof(int32_t)); // 配列のメモリを確保
  msg.data.data[0] = 0;
  msg.data.data[1] = 0;
}
