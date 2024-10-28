int count_l = 0;
int count_r = 0;

#define PIN_ENC_A_L 34
#define PIN_ENC_B_L 35
#define PIN_ENC_A_R 32
#define PIN_ENC_B_R 25

#define PIN_DIR_R   26
#define PIN_PWM_R   27
#define PIN_DIR_L   14

#define PIN_PWM_L   12
#define PIN_SW      21
#define PIN_LED     19
#define PIN_BATT    18

// 割り込みに使用する変数 (volatileをつけて宣言)


void setup() {
  Serial.begin(115200);  // シリアル通信を初期化
  encoder_open();

  delay(1000);  // 一秒待機
}

void loop() {
//  count_l++;
  Serial.print("count_l: ");  // "enc_l: "というテキストを表示
  Serial.println(count_l);    // enc_lの値を表示
  delay(1000);                // 一秒待機
}