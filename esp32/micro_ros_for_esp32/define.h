#define PIN_ENC_A_L 34
#define PIN_ENC_B_L 35
#define PIN_ENC_A_R 32
#define PIN_ENC_B_R 25

//足回り用モータードライバー用
#define PIN_DIR_R   26
#define PIN_PWM_R   27
#define PIN_DIR_L   14
#define PIN_PWM_L   12

#define PIN_BATT    18

//足回り速度制御用
#define RKP 80.0
#define RKI 30.0
#define RKD 8.0
#define LKP 100.0
#define LKI 30.0
#define LKD 8.0

//車体数値
#define COUNTS_PER_REV    4096.0
#define WHEEL_RADIUS      0.04  //ホイール径
#define WHEEL_BASE       0.38  //車輪間幅