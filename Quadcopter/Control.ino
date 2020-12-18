
void control_update()
{
#ifdef AUTOMATIC_FLIGHT_BALANCE
    throttle = map(AUTO_BALANCE_VIRTUALLY_RECEIVE_DATA_THROTTLE, THROTTLE_RMIN, THROTTLE_RMAX, MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
#else
    throttle = map(rx_values[2], THROTTLE_RMIN, THROTTLE_RMAX, MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
#endif

    setpoint_update();
    pid_update();
    pid_compute();

#ifdef ABLE_YAW_APPEND
    m0 = throttle + pid_pitch_out + pid_yaw_out;
    m1 = throttle + pid_roll_out - pid_yaw_out;
    m2 = throttle - pid_pitch_out + pid_yaw_out;
    m3 = throttle - pid_roll_out - pid_yaw_out;
#else
    // yaw control disabled for stabilization testing...
    m0 = throttle + pid_pitch_out; //+ pid_yaw_out;
    m1 = throttle + pid_roll_out;  //- pid_yaw_out;
    m2 = throttle - pid_pitch_out; //+ pid_yaw_out;
    m3 = throttle - pid_roll_out;  //- pid_yaw_out;
#endif

#ifdef SAFE
    if (throttle < THROTTLE_SAFE_SHUTOFF)
    {
        m0 = m1 = m2 = m3 = MOTOR_ZERO_LEVEL;
    }
#endif

    update_motors(m0, m1, m2, m3);
}

void setpoint_update()
{
#ifdef AUTOMATIC_FLIGHT_BALANCE

    pid_roll_setpoint = AUTO_BALANCE_VIRTUALLY_SET_POINT_ROLL;

    pid_pitch_setpoint = AUTO_BALANCE_VIRTUALLY_SET_POINT_PITCH;

    pid_yaw_setpoint = AUTO_BALANCE_VIRTUALLY_SET_POINT_YAW;
#else
                                   // here we allow +- 20 for noise and sensitivity on the RX controls...
    //~RX_NOISE_AND_SENSITIVITY=20
    // ROLL rx at mid level?
    if (rx_values[0] > THROTTLE_RMID - RX_NOISE_AND_SENSITIVITY && rx_values[0] < THROTTLE_RMID + RX_NOISE_AND_SENSITIVITY)
        pid_roll_setpoint = 0;
    else
        pid_roll_setpoint = map(rx_values[0], ROLL_RMIN, ROLL_RMAX, ROLL_WMIN, ROLL_WMAX);
    //PITCH rx at mid level?
    if (rx_values[1] > THROTTLE_RMID - RX_NOISE_AND_SENSITIVITY && rx_values[1] < THROTTLE_RMID + RX_NOISE_AND_SENSITIVITY)
        pid_pitch_setpoint = 0;
    else
        pid_pitch_setpoint = map(rx_values[1], PITCH_RMIN, PITCH_RMAX, PITCH_WMIN, PITCH_WMAX);
    //YAW rx at mid level?
    if (rx_values[3] > THROTTLE_RMID - RX_NOISE_AND_SENSITIVITY && rx_values[3] < THROTTLE_RMID + RX_NOISE_AND_SENSITIVITY)
        pid_yaw_setpoint = 0;
    else
        pid_yaw_setpoint = map(rx_values[3], YAW_RMIN, YAW_RMAX, YAW_WMIN, YAW_WMAX);
#endif
}
