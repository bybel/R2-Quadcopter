#include "config.h"
#include <Math.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Mise en place des variables
Servo esc_1; //FRONT RIGHT (avant droit)
Servo esc_2; //BACK LEFT (arrière gauche)
Servo esc_3; //FRONT LEFT (avant gauche)
Servo esc_4; //BACK RIGHT (arrière droit)

// PID
bool auto_stabilisation_mode = false; // Activer ou pas le mode auto-stabilisé
int dt = 3;
double pid_roll_speed_in, pid_roll_angle_in,   pid_roll_out,   pid_roll_setpoint,  roll_error,  Integral_roll_error,  Derivative_roll_error,  last_roll_error, AS_roll_error, AS_Integral_roll_error, AS_Derivative_roll_error, AS_last_roll_error = 0;
double pid_pitch_speed_in, pid_pitch_angle_in,  pid_pitch_out,  pid_pitch_setpoint, pitch_error, Integral_pitch_error, Derivative_pitch_error, last_pitch_error, AS_pitch_error, AS_Integral_pitch_error, AS_Derivative_pitch_error, AS_last_pitch_error = 0;
double pid_yaw_speed_in,    pid_yaw_out,       pid_yaw_setpoint,   yaw_error,   Integral_yaw_error,   Derivative_yaw_error,   last_yaw_error = 0;

// MOTORS
int motorFR, motorBL, motorFL, motorBR;

// RX
int throttle = THROTTLE_RMIN;
volatile int input[5];
volatile unsigned long chrono_start0, chrono_start1, chrono_start2, chrono_start3,chrono_start4;
volatile int last_interrupt_time0, last_interrupt_time1, last_interrupt_time2, last_interrupt_time3, last_interrupt_time4;

// IMU
float roll_speed, roll_angle;
float pitch_speed, pitch_angle;
float yaw_speed, yaw_angle;
const float pi = 3.14159265359;
Adafruit_BNO055 bno = Adafruit_BNO055();

//// Fonctions de lecture du gyro et accel

//Initiation du IMU
void bno_initialisation() {
  bno.begin();
  bno.setExtCrystalUse(true);
}

// Calculer inclinaison
void bno_get_values() {
  if (auto_stabilisation_mode == false) {
    imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    roll_speed = gyroscope.x() * 180 / pi;
    pitch_speed = gyroscope.y() * 180 / pi;
    yaw_speed = gyroscope.z() * 180 / pi;
    pid_roll_speed_in = roll_speed;
    pid_pitch_speed_in = pitch_speed;
    pid_yaw_speed_in = yaw_speed;
  }
  else {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    roll_angle = euler.z();
    pitch_angle = euler.y();
    pid_roll_angle_in = map(roll_angle, -180, 180, ROLL_WMIN, ROLL_WMAX);
    pid_pitch_angle_in = map(pitch_angle, -180, 180, PITCH_WMIN, PITCH_WMAX);
  }
}


//// Fonctions de controle

// Calculer l'output et l'ecrire
void control_update() {
  // Fait correspondre les valeurs du joystick aux valeurs des moteurs
  throttle = map(input[2], THROTTLE_RMIN, THROTTLE_RMAX, MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);

  // Calculer les valeurs a écrire aux moteurs
  pid_compute();

  // Ecrire les valeurs aux moteurs
  motorFR = throttle + pid_pitch_out + pid_roll_out - pid_yaw_out;
  motorFL = throttle + pid_pitch_out - pid_roll_out + pid_yaw_out;
  motorBL = throttle - pid_pitch_out - pid_roll_out - pid_yaw_out;
  motorBR = throttle - pid_pitch_out + pid_roll_out + pid_yaw_out;
  
  if(input[4] > 1500){
    esc_1.writeMicroseconds(motorFR);
    esc_2.writeMicroseconds(motorFL);
    esc_3.writeMicroseconds(motorBL);
    esc_4.writeMicroseconds(motorBR);
  }
  if (input[4] < 1500) {
    esc_1.writeMicroseconds(1000);
    esc_3.writeMicroseconds(1000);
    esc_2.writeMicroseconds(1000);
    esc_4.writeMicroseconds(1000);
  }
}

// Fonction qui empeche les moteurs de tourner
void motors_set_to_zero() {
  esc_1.writeMicroseconds(MOTOR_ZERO_LEVEL);
  esc_3.writeMicroseconds(MOTOR_ZERO_LEVEL);
  esc_2.writeMicroseconds(MOTOR_ZERO_LEVEL);
  esc_4.writeMicroseconds(MOTOR_ZERO_LEVEL);
}

//// Fonctions radio

//TODO: fonction qui lit les interrupts pour une execution plus rapide(pas encore)
void rx_initialize() {
  attachInterrupt(A0, calcSignal0, CHANGE);
  attachInterrupt(A1, calcSignal1, CHANGE);
  attachInterrupt(A2, calcSignal2, CHANGE);
  attachInterrupt(A3, calcSignal3, CHANGE);
  attachInterrupt(A4, calcSignal4, CHANGE);
}

// Fonctions qui lisent les valeurs de la radio
void calcSignal0() 
{
    last_interrupt_time0 = micros(); 

    if(digitalRead(A0) == HIGH) 
    { 
        chrono_start0 = micros();
    } 
    else
    { 
        if(chrono_start0 != 0)
        { 
            input[0] = ((volatile int)micros() - chrono_start0);
            chrono_start0 = 0;
        }
    } 
}

void calcSignal1() 
{
    last_interrupt_time1 = micros(); 

    if(digitalRead(A1) == HIGH) 
    { 
        chrono_start1 = micros();
    } 
    else
    { 
        if(chrono_start1 != 0)
        { 
            input[1] = ((volatile int)micros() - chrono_start1);
            chrono_start1 = 0;
        }
    } 
}

void calcSignal2() 
{
    last_interrupt_time2 = micros(); 

    if(digitalRead(A2) == HIGH) 
    { 
        chrono_start2 = micros();
    } 
    else
    { 
        if(chrono_start2 != 0)
        { 
            input[2] = ((volatile int)micros() - chrono_start2);
            chrono_start2 = 0;
        }
    } 
}

void calcSignal3() 
{
    last_interrupt_time3 = micros(); 

    if(digitalRead(A3) == HIGH) 
    { 
        chrono_start3 = micros();
    } 
    else
    { 
        if(chrono_start3 != 0)
        { 
            input[3] = ((volatile int)micros() - chrono_start3);
            chrono_start3 = 0;
        }
    } 
}

void calcSignal4() 
{
    last_interrupt_time4 = micros(); 

    if(digitalRead(A4) == HIGH) 
    { 
        chrono_start4 = micros();
    } 
    else
    { 
        if(chrono_start4 != 0)
        { 
            input[4] = ((volatile int)micros() - chrono_start4);
            chrono_start4 = 0;
        }
    } 
}

// Fonction qui s'execute une fois et au début
void setup() {
  Serial.begin(9600);
  bno_initialisation();
  rx_initialize();
  esc_1.attach(MOTOR_PIN_FRONT_RIGHT);
  esc_2.attach(MOTOR_PIN_FRONT_LEFT);
  esc_3.attach(MOTOR_PIN_BACK_LEFT);
  esc_4.attach(MOTOR_PIN_BACK_RIGHT);
}

// Fonctions de debug
void print_motors() {
  Serial.print(motorFR);
  Serial.print(",");
  Serial.print(motorFL);
  Serial.print(",");
  Serial.print(motorBL);
  Serial.print(",");
  Serial.print(motorBR);
  Serial.print(",");
  Serial.print(pid_roll_speed_in);
  Serial.print(",");
  Serial.print(pid_pitch_speed_in);
  Serial.print(",");
  Serial.println(pid_yaw_speed_in);
}

void print_pid_out() {
  Serial.print(pid_roll_out);
  Serial.print(",");
  Serial.print(pid_pitch_out);
  Serial.print(",");
  Serial.println(pid_yaw_out);
}

void print_pid_setpoints() {
  Serial.print(pid_roll_setpoint);
  Serial.print(",");
  Serial.print(pid_pitch_setpoint);
  Serial.print(",");
  Serial.println(pid_yaw_setpoint);
}
void print_imu() {
  Serial.print(pid_roll_speed_in);
  Serial.print(",");
  Serial.print(pid_pitch_speed_in);
  Serial.print(",");
  Serial.println(pid_yaw_speed_in);
}

void print_rx() {
  Serial.print(input[0]);
  Serial.print(",");
  Serial.print(input[1]);
  Serial.print(",");
  Serial.print(input[2]);
  Serial.print(",");
  Serial.println(input[3]);
}

// Fonction qui s'execute a chaque cycle
void loop() {
  bno_get_values();
  control_update();
  print_motors();
}
