#include "config.h"
#include <Math.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//mise en place des variables

Servo motor_right;
Servo motor_left;
Servo motor_front;
Servo motor_back;


// PID variables
bool auto_stabilisation_mode = false; //activer ou pas le mode auto-stabilisé: utiliser le switch de input 5 ou 6
int dt = 3;//notre dt est de 100 millisecondes
double pid_roll_speed_in, pid_roll_angle_in,   pid_roll_out,   pid_roll_setpoint,  roll_error,  Integral_roll_error,  Derivative_roll_error,  last_roll_error, AS_roll_error, AS_Integral_roll_error, AS_Derivative_roll_error, AS_last_roll_error = 0;
double pid_pitch_speed_in, pid_pitch_angle_in,  pid_pitch_out,  pid_pitch_setpoint, pitch_error, Integral_pitch_error, Derivative_pitch_error, last_pitch_error, AS_pitch_error, AS_Integral_pitch_error, AS_Derivative_pitch_error, AS_last_pitch_error = 0;
double pid_yaw_speed_in,    pid_yaw_out,       pid_yaw_setpoint,   yaw_error,   Integral_yaw_error,   Derivative_yaw_error,   last_yaw_error = 0;

// MOTORS
int mR, mL, mF, mB;

//RX
int throttle = THROTTLE_RMIN;
volatile int input[4];
unsigned long timer[4];
byte last_channel[4];

//IMU variables
float roll_speed, roll_angle;
float pitch_speed, pitch_angle;
float yaw_speed, yaw_angle;
Adafruit_BNO055 bno = Adafruit_BNO055();


////fonctions de lecture du gyro et accel

//Initiation du IMU
void bno_initialisation() {
  bno.begin();
  bno.setExtCrystalUse(true);
}

//calculer inclinaison
void bno_get_values() {
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  roll_speed = gyroscope.x() * 180 / 3.14159265359;
  pitch_speed = gyroscope.y() * 180 / 3.14159265359;
  yaw_speed = gyroscope.z() * 180 / 3.14159265359;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  roll_angle = euler.x();
  pitch_angle = euler.y();

  pid_roll_speed_in = map(roll_speed, -180, 180, ROLL_WMIN, ROLL_WMAX);
  pid_pitch_speed_in = map(pitch_speed, -180, 180, PITCH_WMIN, PITCH_WMAX);
  pid_yaw_speed_in = map(yaw_speed, -180, 180, YAW_WMIN, YAW_WMAX);

  pid_roll_angle_in = map(roll_angle, -180, 180, ROLL_WMIN, ROLL_WMAX);
  pid_pitch_angle_in = map(pitch_angle, -180, 180, PITCH_WMIN, PITCH_WMAX);
}


////fonctions de controle

//calculer l'output et l'ecrire
void control_update() {
  //fait correspondre les valeurs du joystick aux valeurs des moteurs
  throttle = map(input[2], THROTTLE_RMIN, THROTTLE_RMAX, MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);

  //calculer les valeurs a écrire aux moteurs
  pid_compute();

  //ecrire les valeurs aux moteurs
  mR = throttle + pid_roll_out + pid_yaw_out;///////moteurs opposés/////
  mL = throttle - pid_roll_out + pid_yaw_out;///////////////////////////
  mF = throttle + pid_pitch_out - pid_yaw_out;//////moteurs opposés/////
  mB = throttle - pid_pitch_out - pid_yaw_out;//////////////////////////

  motor_right.writeMicroseconds(mR - 100);
  motor_left.writeMicroseconds(mL - 100);
  motor_front.writeMicroseconds(mF - 100);
  motor_back.writeMicroseconds(mB - 100);
}

//fonction qui empeche les moteurs de tourner
void motors_set_to_zero() {
  motor_right.writeMicroseconds(MOTOR_ZERO_LEVEL);
  motor_left.writeMicroseconds(MOTOR_ZERO_LEVEL);
  motor_front.writeMicroseconds(MOTOR_ZERO_LEVEL);
  motor_back.writeMicroseconds(MOTOR_ZERO_LEVEL);
}

////fonction radio

//fonction qui lit les interruptspour une execution plus rapide(pas encore)
void rx_initialize() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
}

//fonction qui lit les valeurs de la radio
void rx_read() {
  //TODO: Try with interrupts
  input[0] = pulseIn(A0, HIGH);
  input[1] = pulseIn(A1, HIGH);
  input[2] = pulseIn(A2, HIGH);
  input[3] = pulseIn(A3, HIGH);
}


//fonctions de debugging
void print_rx_values() {
  Serial.print(input[0]);//Print values in millis
  Serial.print(" - ");
  Serial.print(input[1]);
  Serial.print(" - ");
  Serial.print(input[2]);
  Serial.print(" - ");
  Serial.println(input[3]);
}
void print_pitch_and_roll() {
  Serial.print(mR - 100);
  Serial.print(", ");
  Serial.print(mL - 100);
  Serial.print(", ");
  Serial.print(roll_speed);
  Serial.print("     …     ");
  Serial.print(mF - 100);
  Serial.print(", ");
  Serial.print(mB - 100);
  Serial.print(", ");
  Serial.println(pitch_speed);
}
void print_yaw_and_motors() {
  Serial.print(mR - 100);
  Serial.print(", ");
  Serial.print(mL - 100);
  Serial.print(", ");
  Serial.print(mF - 100);
  Serial.print(", ");
  Serial.print(mB - 100);
  Serial.print(", ");
  Serial.println(yaw_speed);
}
void print_pid_values() {
  Serial.print(input[0]);
  Serial.print(", ");
  Serial.println(pid_roll_out);
}

void print_that_bitch() {
  print_pid_values();
}

//foncition qui s'execute une fois et au début
void setup() {
  Serial.begin(9600);
  bno_initialisation();
  rx_initialize();
  motor_right.attach(MOTOR_PIN_RIGHT);
  motor_left.attach(MOTOR_PIN_LEFT);
  motor_front.attach(MOTOR_PIN_FRONT);
  motor_back.attach(MOTOR_PIN_BACK);
}

//fonction qui s'execute a chaque cycle
void loop() {
  rx_read();
  bno_get_values();
  control_update();
  print_yaw_and_motors();

}
