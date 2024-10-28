#define PIN_ENC_A_L  15
#define PIN_ENC_B_L  2

#define PIN_ENC_A_R  3
#define PIN_ENC_B_R  7

#define PIN_DIR_R    8
#define PIN_PWM_R    9
#define PIN_DIR_L   12
#define PIN_PWM_L   11
#define PIN_SW      10
#define PIN_LED     13
#define PIN_BATT    19
// 割り込みに使用する変数 (volatileをつけて宣言)
int count_l = 0;
int count_r = 0;





void encoder_get(int *cnt_l, int *cnt_r) {
  *cnt_l = count_l;
  *cnt_r = count_r;

  // エンコーダ回転方向の補正
  *cnt_l *= -1;
  //*cnt_r *= -1;
}

void encoder_reset() {
  count_l = 0;
  count_r = 0;
}

static void enc_change_l() {
  int a_curr, b_curr;
  static int a_prev = LOW, b_prev = LOW;

  a_curr = digitalRead(PIN_ENC_A_L);
  b_curr = digitalRead(PIN_ENC_B_L);

  // 正転 : [L, H]→(L, L)→[H, L]→(H, H)→[L, H]
  if (a_prev ==  LOW && b_prev == HIGH && a_curr == HIGH && b_curr ==  LOW) count_l++;
  if (a_prev == HIGH && b_prev ==  LOW && a_curr ==  LOW && b_curr == HIGH) count_l++;

  // 逆転 : [L, L]→(L, H)→[H, H]→(H, L)→[L, L]
  if (a_prev ==  LOW && b_prev ==  LOW && a_curr == HIGH && b_curr == HIGH) count_l--;
  if (a_prev == HIGH && b_prev == HIGH && a_curr ==  LOW && b_curr ==  LOW) count_l--;

  a_prev = a_curr;
  b_prev = b_curr;
}

static void enc_change_r() {
  int a_curr, b_curr;
  static int a_prev = LOW, b_prev = LOW;

  a_curr = digitalRead(PIN_ENC_A_R);
  b_curr = digitalRead(PIN_ENC_B_R);

  // 正転 : [L, H]→(L, L)→[H, L]→(H, H)→[L, H]
  if (a_prev ==  LOW && b_prev == HIGH && a_curr == HIGH && b_curr ==  LOW) count_r++;
  if (a_prev == HIGH && b_prev ==  LOW && a_curr ==  LOW && b_curr == HIGH) count_r++;

  // 逆転 : [L, L]→(L, H)→[H, H]→(H, L)→[L, L]
  if (a_prev ==  LOW && b_prev ==  LOW && a_curr == HIGH && b_curr == HIGH) count_r--;
  if (a_prev == HIGH && b_prev == HIGH && a_curr ==  LOW && b_curr ==  LOW) count_r--;

  a_prev = a_curr;
  b_prev = b_curr;
}


void encoder_open() {
  pinMode(PIN_ENC_A_L, INPUT);
  pinMode(PIN_ENC_B_L, INPUT);
  pinMode(PIN_ENC_A_R, INPUT);
  pinMode(PIN_ENC_B_R, INPUT);
  digitalWrite(PIN_ENC_A_L, HIGH);
  digitalWrite(PIN_ENC_B_L, HIGH);
  digitalWrite(PIN_ENC_A_R, HIGH);
  digitalWrite(PIN_ENC_B_R, HIGH);
  attachInterrupt(PIN_ENC_A_L, enc_change_l, CHANGE);
  //attachInterrupt(1, enc_change_r, CHANGE);
}