void control_update(){
  throttle = map(input[2], THROTTLE_RMIN, THROTTLE_RMAX, MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);

  setpoint_update();
  pid_update();
  pid_compute();

  mR = throttle + pid_roll_out;
  mL = throttle - pid_roll_out;
  motors_update(mR, mL); 
}


void setpoint_update(){
  // here we allow +- 20 for noise and sensitivity on the RX controls...
  // ROLL rx at mid level?
  if(input[0] > THROTTLE_RMID - 20 && input[0] < THROTTLE_RMID + 20)
    pid_roll_setpoint = 0;
  else
    pid_roll_setpoint = map(input[0],ROLL_RMIN,ROLL_RMAX,ROLL_WMIN,ROLL_WMAX);
}

