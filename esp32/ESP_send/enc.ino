int count_l = 0;
int count_r = 0;

void encoder_reset() {
  count_l = 0;
  count_r = 0;
}

static void enc_change_l() {
  int a_curr, b_curr;

  a_curr = digitalRead(PIN_ENC_A_L);
  b_curr = digitalRead(PIN_ENC_B_L);

  if(a_curr == b_curr){ //正転
    count_l++;
  }else{  // 逆正転
    count_l--;
  }
}

static void enc_change_r() {
  int a_curr, b_curr;

  a_curr = digitalRead(PIN_ENC_A_R);
  b_curr = digitalRead(PIN_ENC_B_R);

  if(a_curr == b_curr){ //正転
    count_l++;
  }else{  // 逆転 
    count_l--;
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