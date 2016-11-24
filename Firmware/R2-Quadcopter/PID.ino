//fonction qui calcule l'output du controlleur
void pid_compute(){
  //executer le pid et en tirer le resultat
  pid_roll_out = ROLL_PID_KP*roll_error + ROLL_PID_KI*Integral_roll_error + ROLL_PID_KD*Derivative_roll_error;
  pid_pitch_out = PITCH_PID_KP*pitch_error + PITCH_PID_KI*Integral_pitch_error + PITCH_PID_KD*Derivative_pitch_error;
  pid_yaw_out = YAW_PID_KP*yaw_error + YAW_PID_KI*Integral_yaw_error + YAW_PID_KD*Derivative_yaw_error;

  if(pid_roll_out > ROLL_WMAX) pid_roll_out = ROLL_WMAX;
  if(pid_pitch_out > PITCH_WMAX) pid_pitch_out = PITCH_WMAX;
  if(pid_yaw_out > YAW_WMAX) pid_yaw_out = YAW_WMAX;
  //Ici on borne l'output total pour ne pas avoir de très gros accoups

  //actualiser le setpoint, les erreurs et calculer les termes int et der
  pid_setpoint_update();
  pid_errors_update();
  pid_calculate_Int_and_Der();

  //passer de la mesure anterieure a la nouvelle
  last_roll_error = roll_error;
  last_pitch_error = pitch_error;
  last_yaw_error = yaw_error;
}

//calculer les termes der et int pour l'intervalle delta-temps
void pid_calculate_Int_and_Der(){
  if(delta_temps >= sampletime){ //Si le temps de mesure atteint sampletime
    Integral_roll_error += roll_error;//pour nous delta_temps est dt
    Integral_pitch_error += pitch_error;//ce qui permet d'integrer
    Integral_yaw_error += yaw_error;//pour chacun des axes

    if(Integral_roll_error > ROLL_WMAX) Integral_roll_error = ROLL_WMAX; //ne pas laisser l'ouput
    else if(Integral_roll_error < ROLL_WMIN) Integral_roll_error = ROLL_WMIN; //depasser l'output max

    if(Integral_pitch_error > PITCH_WMAX) Integral_pitch_error = PITCH_WMAX;
    else if(Integral_pitch_error < PITCH_WMIN) Integral_pitch_error = PITCH_WMIN;

    if(Integral_yaw_error > YAW_WMAX) Integral_yaw_error = YAW_WMAX;
    else if(Integral_yaw_error < YAW_WMIN) Integral_yaw_error = YAW_WMIN;
    //Ici nous avons borné le terme integral pour qu'il n'exagere pas


    Derivative_roll_error = (roll_error - last_roll_error);//le delta-temps est inclus dans le debut du loop
    Derivative_pitch_error = (pitch_error - last_pitch_error);
    Derivative_yaw_error = (yaw_error - last_yaw_error);
  }
}

//fonction qui actualise les erreurs
void pid_errors_update(){
  //calculate the errors
  roll_error = pid_roll_setpoint - pid_roll_in;
  pitch_error = pid_pitch_setpoint - pid_pitch_in;
  yaw_error = pid_yaw_setpoint - pid_yaw_in;
}

//fonction qui prend la position du joystick
void pid_setpoint_update(){
  //Ici on regarde si le stick est au milieu et on laisse une marge
  //d'erreur pour que quand on le lache, il donne une erraur nulle.
  if (input[0] > THROTTLE_RMID - 20 && input[0] < THROTTLE_RMID + 20)
    pid_roll_setpoint = 0;
  else
    pid_roll_setpoint = map(input[0], ROLL_RMIN, ROLL_RMAX, ROLL_WMIN, ROLL_WMAX);
  //PITCH rx at mid level? +-20
  if (input[1] > THROTTLE_RMID - 20 && input[1] < THROTTLE_RMID + 20)
    pid_pitch_setpoint = 0;
  else
    pid_pitch_setpoint = map(input[1], PITCH_RMIN, PITCH_RMAX, PITCH_WMIN, PITCH_WMAX);
  //YAW rx mid +-20
  if (input[3] > THROTTLE_RMID - 20 && input[3] < THROTTLE_RMID + 20)
    pid_yaw_setpoint = 0;
  else
    pid_yaw_setpoint = map(input[3], YAW_RMIN, YAW_RMAX, YAW_WMIN, YAW_WMAX);
}
