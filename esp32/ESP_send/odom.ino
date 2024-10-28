rcl_publisher_t l_publisher;
rcl_publisher_t r_publisher;
std_msgs__msg__Int32 l_msg;
std_msgs__msg__Int32 r_msg;
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
    
    l_msg.data = count_l;
    RCSOFTCHECK(rcl_publish(&l_publisher, &l_msg, NULL));
    r_msg.data = count_r;
    RCSOFTCHECK(rcl_publish(&r_publisher, &r_msg, NULL));     

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
    &l_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "L_encoder"));

  
  RCCHECK(rclc_publisher_init_default(
    &r_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "R_encoder"));


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

  l_msg.data = 0;
  r_msg.data = 0;
}
