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


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

