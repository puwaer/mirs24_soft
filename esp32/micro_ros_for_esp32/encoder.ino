// ガチ不要
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
  int a_curr = digitalRead(PIN_ENC_A_L);
  int b_curr = digitalRead(PIN_ENC_B_L);

  if(a_curr == b_curr){
    count_l++;
  }else{
    count_l--;
  }
}

static void enc_change_r() {
  int a_curr = digitalRead(PIN_ENC_A_R);
  int b_curr = digitalRead(PIN_ENC_B_R);

  if(a_curr == b_curr){
    count_r++;
  }else{
    count_r--;
  }
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
  attachInterrupt(PIN_ENC_A_R, enc_change_r, CHANGE);
}