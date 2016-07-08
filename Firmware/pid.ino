PID roll_controller(&pid_roll_in,   &pid_roll_out,  &pid_roll_setpoint,  5.0, 0.0, 0.0, REVERSE);


void pid_initialize() {
  roll_controller.SetOutputLimits(ROLL_PID_MIN,ROLL_PID_MAX);
  roll_controller.SetMode(AUTOMATIC);
  roll_controller.SetSampleTime(10);
}

void pid_update(){
  pid_roll_in = roll_angle;
}

void pid_compute() {
   roll_controller.Compute();
}
