
#include "Configuration.h"
#include <Math.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <Servo.h>
#include <Wire.h>

#ifdef NEW_MCU6050_GET
#ifdef TRIAXIALSENSORTEST_INO
#include <MPU6050.h>
#endif
#ifdef GY_87_V8
#include "I2Cdev.h"     //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
#include "MPU6050.h"    //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include "QMC5883L.h"   //https://github.com/dthain/QMC5883L
#include "SFE_BMP180.h" //https://github.com/sparkfun/BMP180_Breakout/tree/master/Libraries/Arduino
#endif
#endif

// Angles
float angleX, angleY, angleZ = 0.0;

// RX Signals
int throttle = THROTTLE_RMIN;
volatile int rx_values[4]; // ROLL, PITCH, THROTTLE, YAW

// PID variables
double pid_roll_in, pid_roll_out, pid_roll_setpoint = 0;
double pid_pitch_in, pid_pitch_out, pid_pitch_setpoint = 0;
double pid_yaw_in, pid_yaw_out, pid_yaw_setpoint = 0;

// Motors
int m0, m1, m2, m3; // Front, Right, Back, Left

// Track loop time.
unsigned long prev_time = 0;

//I brought the things that were originally at PID here for everyone to use
PID roll_controller(&pid_roll_in, &pid_roll_out, &pid_roll_setpoint, ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD, REVERSE);
PID pitch_controller(&pid_pitch_in, &pid_pitch_out, &pid_pitch_setpoint, PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD, REVERSE);
PID yaw_controller(&pid_yaw_in, &pid_yaw_out, &pid_yaw_setpoint, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD, DIRECT);

void setup()
{
#ifdef DEBUG_OUTPUT
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("Debug Output ON");
#endif

    motors_initialize();
    leds_initialize();
    rx_initialize();
    pid_initialize();
    motors_arm();

    //wait for IMU YAW  to settle before beginning??? ~20s

#ifdef ONLINE_PARAMETER_ADJUSTMENT_PIN_GROUP
    attachInterrupt(digitalPinToInterrupt(PARAMETER_ADJUSTMENT_INTERRUPT_PIN), &Online_update, CHANGE);
#endif

#ifdef NEW_MCU6050_GET
#ifdef TRIAXIALSENSORTEST_INO
    tr_ino_init();
#endif
#ifdef GY_87_V8
    V8_init();
#endif
#endif
}

void loop()
{
    imu_update();
    control_update();

#ifdef DEBUG_OUTPUT
    debug_process();
#endif

    prev_time = micros();
}
