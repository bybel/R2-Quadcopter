////////////////////////////////Definition des pins//////////////////////////////////

////////RX///////
#define RX_PIN_ROLL A0
#define RX_PIN_PITCH A1
#define RX_PIN_THROTTLE A2
#define RX_PIN_YAW A3

////////Motors////////
#define MOTOR_PIN_FRONT_RIGHT 6
#define MOTOR_PIN_BACK_LEFT 5
#define MOTOR_PIN_FRONT_LEFT 9
#define MOTOR_PIN_BACK_RIGHT 10

////////////////////////////////Definition des constantes///////////////////////////////////////

/////////////PID////////////////
#define ROLL_PID_KP  1
#define ROLL_PID_KI  0.03 
#define ROLL_PID_KD  1
#define ROLL_PID_MIN  -200.0
#define ROLL_PID_MAX  200.0

#define PITCH_PID_KP  1
#define PITCH_PID_KI  0.03             ////toutes les K a verifier et tester
#define PITCH_PID_KD  1
#define PITCH_PID_MIN  -200.0
#define PITCH_PID_MAX  200.0

#define YAW_PID_KP  1
#define YAW_PID_KI  0.03
#define YAW_PID_KD  0
#define YAW_PID_MIN  -100.0
#define YAW_PID_MAX  100.0

#define ROLL_WMIN  -30//////////
#define ROLL_WMAX  30//////////

#define PITCH_WMIN  -30//////////    valeurs a mapper pour le setpoint
#define PITCH_WMAX  30//////////

#define YAW_WMIN  -30//////////
#define YAW_WMAX  30//////////

#define AS_K 1 //auto_adjust --> coef de 1

////////Config RX///////////////////
#define THROTTLE_RMIN  1100
#define THROTTLE_SAFE_SHUTOFF 1120
#define THROTTLE_RMAX  1850
#define THROTTLE_RMID  1500

#define ROLL_RMIN  THROTTLE_RMIN
#define ROLL_RMAX  THROTTLE_RMAX


#define PITCH_RMIN  THROTTLE_RMIN
#define PITCH_RMAX  THROTTLE_RMAX


#define YAW_RMIN  THROTTLE_RMIN
#define YAW_RMAX  THROTTLE_RMAX

/////////////Variable des moteurs////////////
#define MOTOR_ZERO_LEVEL  1000
#define MOTOR_ARM_START  1100
#define MOTOR_MAX_LEVEL  2000
