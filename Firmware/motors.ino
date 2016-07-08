Servo motor_right;
Servo motor_left;

void motors_initialisation(){
  motor_right.attach(MOTOR_PIN_RIGHT);
  motor_left.attach(MOTOR_PIN_LEFT);
  motor_right.writeMicroseconds(MOTOR_ZERO_LEVEL);
  motor_left.writeMicroseconds(MOTOR_ZERO_LEVEL);
}

void motors_set_to_zero(){
  motor_right.writeMicroseconds(MOTOR_ZERO_LEVEL);
  motor_left.writeMicroseconds(MOTOR_ZERO_LEVEL);
}

void motors_update(int mR, int mL){
  motor_right.writeMicroseconds(mR);
  motor_left.writeMicroseconds(mL);
}

