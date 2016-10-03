#include "config.h"
#include <Math.h>
#include <PID_v1.h>
#include <Servo.h> 
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"


// PID variables
double pid_roll_in,   pid_roll_out,   pid_roll_setpoint = 0;
double pid_pitch_in,  pid_pitch_out,  pid_pitch_setpoint = 0;
double pid_yaw_in,    pid_yaw_out,    pid_yaw_setpoint = 0;

// MOTORS
int mR, mL;

//rx
int throttle=THROTTLE_RMIN;
volatile int input[4];

float roll_angle;

void setup(){
  mpu_initialisation();
  pid_initialize();
  rx_initialize();
  motors_set_to_zero();
}

void loop(){
  mpu_get_values();
  control_update();
//  Serial.print(mR);
//  Serial.print(", ");
//  Serial.print(mL);
//  Serial.print(", ");
//  Serial.println(roll_angle);

}

