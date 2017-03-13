// Fonction qui calcule l'output du controlleur
void pid_compute() {
  // Executer le PID et en tirer le resultat
  if (!auto_stabilisation_mode) {
    pid_roll_out = ROLL_PID_KP * roll_error + ROLL_PID_KI * Integral_roll_error * dt + ROLL_PID_KD * Derivative_roll_error / dt;
    if(pid_roll_out > ROLL_PID_MAX)pid_roll_out = ROLL_PID_MAX;
    else if(pid_roll_out < ROLL_PID_MIN)pid_roll_out = ROLL_PID_MIN;
    
    pid_pitch_out = PITCH_PID_KP * pitch_error + PITCH_PID_KI * Integral_pitch_error * dt + PITCH_PID_KD * Derivative_pitch_error / dt;
    if(pid_pitch_out > PITCH_PID_MAX)pid_pitch_out = PITCH_PID_MAX;
    else if(pid_pitch_out < PITCH_PID_MIN)pid_pitch_out = PITCH_PID_MIN;
    
    pid_yaw_out = YAW_PID_KP * yaw_error + YAW_PID_KI * Integral_yaw_error * dt + YAW_PID_KD * Derivative_yaw_error / dt;
    if(pid_yaw_out > YAW_PID_MAX)pid_yaw_out = YAW_PID_MAX;
    else if(pid_yaw_out < YAW_PID_MIN)pid_yaw_out = YAW_PID_MIN;
    
    pid_setpoint_update();
    pid_errors_update();
    pid_calculate_Int_and_Der();
  }
  else if (auto_stabilisation_mode) {
    pid_roll_out = ROLL_PID_KP * AS_roll_error + ROLL_PID_KI * AS_Integral_roll_error * dt + ROLL_PID_KD * AS_Derivative_roll_error / dt;
    if(pid_roll_out > ROLL_PID_MAX)pid_roll_out = ROLL_PID_MAX;
    else if(pid_roll_out < ROLL_PID_MIN)pid_roll_out = ROLL_PID_MIN;
    
    pid_pitch_out = PITCH_PID_KP * AS_pitch_error + PITCH_PID_KI * AS_Integral_pitch_error * dt + PITCH_PID_KD * AS_Derivative_pitch_error / dt;
    if(pid_pitch_out > PITCH_PID_MAX)pid_pitch_out = PITCH_PID_MAX;
    else if(pid_pitch_out < PITCH_PID_MIN)pid_pitch_out = PITCH_PID_MIN;

    pid_yaw_out = YAW_PID_KP * yaw_error + YAW_PID_KI * Integral_yaw_error * dt + YAW_PID_KD * Derivative_yaw_error / dt;//On laisse le yaw comme avant car il ne doit pas etre affecté pas l'auto_dajust
    if(pid_yaw_out > YAW_PID_MAX)pid_yaw_out = YAW_PID_MAX;
    else if(pid_yaw_out < YAW_PID_MIN)pid_yaw_out = YAW_PID_MIN;
    
    pid_setpoint_update();
    AS_errors_update();
    pid_calculate_AS_Int_and_Der();
  }

  if (pid_roll_out > ROLL_PID_MAX) pid_roll_out = ROLL_PID_MAX;
  else if (pid_roll_out < ROLL_PID_MIN) pid_roll_out = ROLL_PID_MIN;
  if (pid_pitch_out > PITCH_PID_MAX) pid_pitch_out = PITCH_PID_MAX;             // Ajout de restrictions pour maximum et minimums
  else if (pid_pitch_out < PITCH_PID_MIN) pid_pitch_out = PITCH_PID_MIN;
  if (pid_yaw_out > YAW_PID_MAX) pid_yaw_out = YAW_PID_MAX;
  else if (pid_yaw_out < YAW_PID_MIN) pid_yaw_out = YAW_PID_MIN;
  // Ici on borne l'output total pour ne pas avoir de très gros accoups
}

// Fonction qui prend la position du joystick
void pid_setpoint_update() {
  /* Ici on regarde si le stick est au milieu et on laisse une marg
   * d'erreur pour que quand on le lache, il donne une erraur nulle. */
  if (input[0] > THROTTLE_RMID - 20 && input[0] < THROTTLE_RMID + 20)
    pid_roll_setpoint = 0;
  else
    pid_roll_setpoint = map(input[0], ROLL_RMIN, ROLL_RMAX, ROLL_WMIN, ROLL_WMAX);
  // PITCH rx at mid level? +-20ms
  if (input[1] > THROTTLE_RMID - 20 && input[1] < THROTTLE_RMID + 20)
    pid_pitch_setpoint = 0;
  else
    pid_pitch_setpoint = map(input[1], PITCH_RMIN, PITCH_RMAX, PITCH_WMIN, PITCH_WMAX);
  //YAW rx mid +-20ms
  if (input[3] > THROTTLE_RMID - 20 && input[3] < THROTTLE_RMID + 20)
    pid_yaw_setpoint = 0;
  else
    pid_yaw_setpoint = map(input[3], YAW_RMIN, YAW_RMAX, YAW_WMIN, YAW_WMAX);
}

// Calculer les termes der et int pour l'intervalle delta-temps
void pid_calculate_Int_and_Der() {
  // Partie integrale
  Integral_roll_error += (last_roll_error + roll_error);
  Integral_pitch_error += (last_pitch_error + pitch_error); // Commutativité de l'addition
  Integral_yaw_error += (last_yaw_error + yaw_error);

  if (Integral_roll_error > ROLL_WMAX) Integral_roll_error = ROLL_WMAX;      //Ne pas laisser l'ouput
  else if (Integral_roll_error < ROLL_WMIN) Integral_roll_error = ROLL_WMIN; //depasser l'output max

  if (Integral_pitch_error > PITCH_WMAX) Integral_pitch_error = PITCH_WMAX;
  else if (Integral_pitch_error < PITCH_WMIN) Integral_pitch_error = PITCH_WMIN;

  if (Integral_yaw_error > YAW_WMAX) Integral_yaw_error = YAW_WMAX;
  else if (Integral_yaw_error < YAW_WMIN) Integral_yaw_error = YAW_WMIN;
  // Ici nous avons borné le terme integral pour qu'il n'exagere pas

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  // Partie dérivée
  Derivative_roll_error = (roll_error - last_roll_error);
  Derivative_pitch_error = (pitch_error - last_pitch_error);
  Derivative_yaw_error = (yaw_error - last_yaw_error);

  if (Derivative_roll_error > ROLL_WMAX) Derivative_roll_error = ROLL_WMAX;      //Ne pas laisser l'ouput
  else if (Derivative_roll_error < ROLL_WMIN) Derivative_roll_error = ROLL_WMIN; //depasser l'output max

  if (Derivative_pitch_error > PITCH_WMAX) Derivative_pitch_error = PITCH_WMAX;
  else if (Derivative_pitch_error < PITCH_WMIN) Derivative_pitch_error = PITCH_WMIN;

  if (Derivative_yaw_error > YAW_WMAX) Derivative_yaw_error = YAW_WMAX;
  else if (Derivative_yaw_error < YAW_WMIN) Derivative_yaw_error = YAW_WMIN;
  // Ici nous avons borné le terme Derivative pour qu'il n'exagere pas

  delay(dt);
  // Ici on laisse passer le temps dt pour obtenir la prochaine mesure

  last_roll_error = roll_error;
  last_pitch_error = pitch_error;
  last_yaw_error = yaw_error;
  // Ici on passe les mesures du dernier cycle pour les prochaines
}

// Fonction qui actualise les erreurs
void pid_errors_update() {
  // Calculer les erreurs
  roll_error = pid_roll_setpoint - pid_roll_speed_in;
  pitch_error = pid_pitch_setpoint - pid_pitch_speed_in;
  yaw_error = pid_yaw_setpoint - pid_yaw_speed_in;
}

// Auto-Stabilisation
void pid_calculate_AS_Int_and_Der() {
  // Partie intégrale
  AS_Integral_roll_error += (AS_roll_error + AS_last_roll_error);
  AS_Integral_pitch_error += (AS_pitch_error + AS_last_pitch_error); //commutativite de l'addition
  Integral_yaw_error += (last_yaw_error + yaw_error);

  if (AS_Integral_roll_error > ROLL_WMAX) AS_Integral_roll_error = ROLL_WMAX;      //Ne pas laisser l'ouput
  else if (AS_Integral_roll_error < ROLL_WMIN) AS_Integral_roll_error = ROLL_WMIN; //depasser l'output max

  if (AS_Integral_pitch_error > PITCH_WMAX) AS_Integral_pitch_error = PITCH_WMAX;
  else if (AS_Integral_pitch_error < PITCH_WMIN) AS_Integral_pitch_error = PITCH_WMIN;

  if (Integral_yaw_error > YAW_WMAX) Integral_yaw_error = YAW_WMAX;
  else if (Integral_yaw_error < YAW_WMIN) Integral_yaw_error = YAW_WMIN;
  //Ici nous avons borné le terme integral pour qu'il n'exagere pas


  //////////////////////////////////////////////////////////////////////////////////////////////////////


  // Partie derivée
  AS_Derivative_roll_error = (AS_roll_error - AS_last_roll_error);
  AS_Derivative_pitch_error = (AS_pitch_error - AS_last_pitch_error);
  Derivative_yaw_error = (yaw_error - last_yaw_error);

  if (AS_Derivative_roll_error > ROLL_WMAX) AS_Derivative_roll_error = ROLL_WMAX;      //Ne pas laisser l'ouput
  else if (AS_Derivative_roll_error < ROLL_WMIN) AS_Derivative_roll_error = ROLL_WMIN; //depasser l'output max

  if (AS_Derivative_pitch_error > PITCH_WMAX) AS_Derivative_pitch_error = PITCH_WMAX;
  else if (AS_Derivative_pitch_error < PITCH_WMIN) AS_Derivative_pitch_error = PITCH_WMIN;

  if (Derivative_yaw_error > YAW_WMAX) Derivative_yaw_error = YAW_WMAX;
  else if (Derivative_yaw_error < YAW_WMIN) Derivative_yaw_error = YAW_WMIN;
  // Ici nous avons borné le terme dérivée pour qu'il n'exagere pas

  delay(dt);
  // Ici on laisse passer le temps dt pour obtenir la prochaine mesure

  AS_last_roll_error = AS_roll_error;
  AS_last_pitch_error = AS_pitch_error;
  last_yaw_error = yaw_error;
  // Ici on passe les mesures du dernier cycle pour les prochaines
}

void AS_errors_update() {
  AS_roll_error = pid_roll_setpoint - pid_roll_angle_in * AS_K;
  AS_pitch_error = pid_pitch_setpoint - pid_pitch_angle_in * AS_K;
  yaw_error = pid_yaw_setpoint - pid_yaw_speed_in; // Meme erreur que pour mode accro
}
