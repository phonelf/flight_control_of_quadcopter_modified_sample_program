#define SAFE

//-------PID Config----------
#define ROLL_PID_KP 0.250
#define ROLL_PID_KI 0.950
#define ROLL_PID_KD 0.011
#define ROLL_PID_MIN -200.0
#define ROLL_PID_MAX 200.0

#define PITCH_PID_KP 0.250
#define PITCH_PID_KI 0.950
#define PITCH_PID_KD 0.011
#define PITCH_PID_MIN -200.0
#define PITCH_PID_MAX 200.0

#define YAW_PID_KP 0.680
#define YAW_PID_KI 0.500
#define YAW_PID_KD 0.0001
#define YAW_PID_MIN 100.0
#define YAW_PID_MAX 100.0

//-------------------------

//-------RX Config----------
#define THROTTLE_RMIN 1000
#define THROTTLE_SAFE_SHUTOFF 1120
#define THROTTLE_RMAX 1900
#define THROTTLE_RMID 1470

#define ROLL_RMIN THROTTLE_RMIN
#define ROLL_RMAX THROTTLE_RMAX
#define ROLL_WMIN -30
#define ROLL_WMAX 30

#define PITCH_RMIN THROTTLE_RMIN
#define PITCH_RMAX THROTTLE_RMAX
#define PITCH_WMIN -30
#define PITCH_WMAX 30

#define YAW_RMIN THROTTLE_RMIN
#define YAW_RMAX THROTTLE_RMAX
#define YAW_WMIN -30
#define YAW_WMAX 30

//-------IMU Config-----------
#define ADDR_SLAVE_I2C 2
#define PACKET_SIZE 12

//-------Debug Config---------
#define DEBUG_OUTPUT
#define DEBUG_ANGLES
#define DEBUG_PID
#define DEBUG_RX
#define DEBUG_MOTORS
#define DEBUG_LOOP_TIME
//............................
#define DEBUG_ANGLES_OUTPUT_Z
#define DEBUG_RX_OUTPUT_YAW
#define DEBUG_PID_OUTPUT_YAW
//----------------------------

//-------Motor PWM Levels
#define MOTOR_ZERO_LEVEL 1000
#define MOTOR_ARM_START 1500
#define MOTOR_MAX_LEVEL 2000

//-------RX PINS-------------
#define RX_PINS_OFFSET 2
#define PIN_RX_ROLL 2
#define PIN_RX_PITCH 3
#define PIN_RX_THROTTLE 4
#define PIN_RX_YAW 5

//-------MOTOR PINS-----------
#define PIN_MOTOR0 6
#define PIN_MOTOR1 9
#define PIN_MOTOR2 10
#define PIN_MOTOR3 11

//-------LED PINS-------------
#define PIN_LED 13

//-----Supplementary definition-----
//.........Write the original constant as a definition...
#define SAMPLE_TIME_DEF 10
//#define ABLE_YAW_APPEND
#define RX_NOISE_AND_SENSITIVITY 20
//..................................
//.....Auto balance.................
#define AUTOMATIC_FLIGHT_BALANCE
#define AUTO_BALANCE_VIRTUALLY_SET_POINT_ROLL 0
#define AUTO_BALANCE_VIRTUALLY_SET_POINT_PITCH 0
#define AUTO_BALANCE_VIRTUALLY_SET_POINT_YAW 0
#define AUTO_BALANCE_VIRTUALLY_RECEIVE_DATA_THROTTLE 1650
//..................................
//.....MCU6050......................
#define NEW_MCU6050_GET
#define TRIAXIALSENSORTEST_INO
//#define GY_87_V8
//#define MCU6050_GY87_PRINTLN
//..................................
//....Change PID parameters online........
//#define ONLINE_PARAMETER_ADJUSTMENT_PIN_GROUP
#ifdef ONLINE_PARAMETER_ADJUSTMENT_PIN_GROUP
#undef RX_PINS_OFFSET
#undef PIN_RX_ROLL
#undef PIN_RX_PITCH
#undef PIN_RX_THROTTLE
#undef PIN_RX_YAW
#define RX_PINS_OFFSET A0
#define PIN_RX_ROLL A0
#define PIN_RX_PITCH A1
#define PIN_RX_THROTTLE A2
#define PIN_RX_YAW A3
//....Pin definition........
#define PARAMETER_ADJUSTMENT_INTERRUPT_PIN 2
#define PARAMETER_ADJUSTMENT_DATA 7
//....data processing........
#define TOTAL_DATA_BITS 361
#define DATA_TRANSMISSION_START_TRIGGER_AMOUNT 9
#define NUMBER_OF_PARAMETERS 9
#define NUMBER_OF_TEXTS_FOR_EACH_PARAMETER 5
#define DATA_TRANSMISSION_START_DELAY 100000
#define SHOW_OUT_OF_RANGE_ERROR_MSG
#endif
//..................................
