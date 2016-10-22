#include "config.h"
#include <Math.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



Servo motor_right;
Servo motor_left;


// PID variables
double pid_roll_in,   pid_roll_out,   pid_roll_setpoint = 0;
double pid_pitch_in,  pid_pitch_out,  pid_pitch_setpoint = 0;
double pid_yaw_in,    pid_yaw_out,    pid_yaw_setpoint = 0;

// MOTORS
int mR, mL, mF, mB;

//rx
int throttle = THROTTLE_RMIN;
volatile int input[4];

unsigned long timer[4];
byte last_channel[4];


float roll_angle;

Adafruit_BNO055 bno = Adafruit_BNO055();

PID roll_controller(&pid_roll_in,   &pid_roll_out,  &pid_roll_setpoint,  5.0, 0.0, 0.0, REVERSE);


void bno_initialisation() {
  bno.begin();
  bno.setExtCrystalUse(true);
}

void bno_get_values() {
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  roll_angle = gyroscope.x() * 180 / 3.14159265359;
}

void control_update() {
  throttle = map(input[2], THROTTLE_RMIN, THROTTLE_RMAX, MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);

  setpoint_update();
  pid_update();
  pid_compute();

  mR = throttle + pid_roll_out;
  mL = throttle - pid_roll_out;
}

void motors_set_to_zero() {
  motor_right.writeMicroseconds(MOTOR_ZERO_LEVEL);
  motor_left.writeMicroseconds(MOTOR_ZERO_LEVEL);
}

void pid_initialize() {
  roll_controller.SetOutputLimits(ROLL_PID_MIN, ROLL_PID_MAX);
  roll_controller.SetMode(AUTOMATIC);
  roll_controller.SetSampleTime(5);
}

void pid_update() {
  pid_roll_in = roll_angle;
}

void pid_compute() {
  roll_controller.Compute();
}

void setpoint_update() {
  // here we allow +- 20 for noise and sensitivity on the RX controls...
  // ROLL rx at mid level?
  if (input[0] > THROTTLE_RMID - 20 && input[0] < THROTTLE_RMID + 20)
    pid_roll_setpoint = 0;
  else
    pid_roll_setpoint = map(input[0], ROLL_RMIN, ROLL_RMAX, ROLL_WMIN, ROLL_WMAX);
}

void rx_initialize() {

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
}

void print_that_bitch() {
  print_values();
}

void rx_read()
{
  input[0] = pulseIn(A0, HIGH);
  input[1] = pulseIn(A1, HIGH);
  input[2] = pulseIn(A2, HIGH);
  input[3] = pulseIn(A3, HIGH);
}

void print_values() {
  Serial.print(input[0]);//Ici on print les valeurs en microsecondes
  Serial.print(" - ");
  Serial.print(input[1]);
  Serial.print(" - ");
  Serial.print(input[2]);
  Serial.print(" - ");
  Serial.println(input[3]);
}

void setup() {
  Serial.begin(9600);
  bno_initialisation();
  pid_initialize();
  rx_initialize();
  motor_right.attach(MOTOR_PIN_RIGHT);
  motor_left.attach(MOTOR_PIN_LEFT);
}

void loop() {
  rx_read();
  bno_get_values();
  control_update();
  motor_right.writeMicroseconds(mR - 100);
  motor_left.writeMicroseconds(mL - 100);
  Serial.print(mR - 100);
  Serial.print(", ");
  Serial.print(mL - 100);
  Serial.print(", ");
  Serial.println(roll_angle);

}

