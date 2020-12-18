

void pid_initialize()
{
    roll_controller.SetOutputLimits(ROLL_PID_MIN, ROLL_PID_MAX);
    pitch_controller.SetOutputLimits(PITCH_PID_MIN, PITCH_PID_MAX);
    yaw_controller.SetOutputLimits(YAW_PID_MIN, YAW_PID_MAX);
    roll_controller.SetMode(AUTOMATIC);
    pitch_controller.SetMode(AUTOMATIC);
    yaw_controller.SetMode(AUTOMATIC);
    roll_controller.SetSampleTime(SAMPLE_TIME_DEF);
    pitch_controller.SetSampleTime(SAMPLE_TIME_DEF);
    yaw_controller.SetSampleTime(SAMPLE_TIME_DEF);
}

void pid_update()
{
    pid_roll_in = angleX;
    pid_pitch_in = angleY;
    pid_yaw_in = angleZ;
}

void pid_compute()
{
    roll_controller.Compute();
    pitch_controller.Compute();
    yaw_controller.Compute();
}
