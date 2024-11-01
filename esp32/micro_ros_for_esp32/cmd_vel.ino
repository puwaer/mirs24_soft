

// PIDオブジェクトを作成
PID R_PID(&r_vel, &r_pwm, &r_vel_cmd, KP, KI, KD, DIRECT);
PID L_PID(&l_vel, &l_pwm, &l_vel_cmd, KP, KI, KD, DIRECT);

// PWM設定
const int r_Channel = 0;      // PWMチャンネル
const int l_Channel = 1;
const int pwmFrequency = 5000; // PWM周波数 (例: 5kHz)
const int pwmResolution = 8;   // PWM分解能 (8ビット = 0-255)

// cmd_velメッセージのコールバック関数
void cmd_vel_Callback(const void * msgin) {
  const geometry_msgs__msg__Twist * vel_msg = (const geometry_msgs__msg__Twist *)msgin;

  // linear.x と angular.z のデータを取得
  float linear_x = vel_msg->linear.x;
  float angular_z = vel_msg->angular.z;

  //目標速度計算
  r_vel_cmd = linear_x + WHEEL_BASE / 2 * angular_z;
  l_vel_cmd = linear_x - WHEEL_BASE / 2 * angular_z;

  // センサーからの入力を取得 (例: 現在の速度)
  l_vel = left_distance / 0.1;
  r_vel = right_distance / 0.1;
 
  // PID計算を実行
  R_PID.Compute();
  L_PID.Compute();

  abc = r_pwm;
  def = l_pwm;
 
  // 出力結果を利用してPWM出力を設定
  r_pwm = constrain(r_pwm, -255, 255); // 出力を0-255の範囲に制限
  l_pwm = constrain(l_pwm, -255, 255); // 出力を0-255の範囲に制限

  if(r_pwm >= 0){
    digitalWrite(PIN_DIR_R, LOW);
  }else{
    r_pwm *= -1;
    digitalWrite(PIN_DIR_R,HIGH);
  }
  if(l_pwm >= 0){
    digitalWrite(PIN_DIR_L, HIGH);
  }else{
    l_pwm *= -1;
    digitalWrite(PIN_DIR_L,LOW);
  }
  ledcWrite(r_Channel, uint8_t(r_pwm));          // PWM出力
  ledcWrite(l_Channel, uint8_t(l_pwm));          // PWM出力
}

void cmd_vel_set() {
  // ledcのPWM設定
  pinMode(PIN_DIR_R, OUTPUT);
  pinMode(PIN_DIR_L, OUTPUT);
  ledcSetup(r_Channel, pwmFrequency, pwmResolution);
  ledcSetup(l_Channel, pwmFrequency, pwmResolution);
  ledcAttachPin(PIN_PWM_R, r_Channel);
  ledcAttachPin(PIN_PWM_L, l_Channel);
 
  // PID制御を開始
  R_PID.SetMode(AUTOMATIC);
  L_PID.SetMode(AUTOMATIC);
}
