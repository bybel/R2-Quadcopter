////////////////////////////////Definition des pins//////////////////////////////////

////////RX///////
#define RX_PIN_ARM A4
#define RX_PIN_ROLL A0
#define RX_PIN_PITCH A1
#define RX_PIN_THROTTLE A2
#define RX_PIN_YAW A3

////////Motors////////
#define MOTOR_PIN_FRONT_RIGHT 4 //ESC1
#define MOTOR_PIN_BACK_LEFT 3 //ESC2
#define MOTOR_PIN_FRONT_LEFT 1 //ESC3
#define MOTOR_PIN_BACK_RIGHT 0 //ESC4

////////////////////////////////Definition des constantes///////////////////////////////////////

/////////////PID////////////////
#define ROLL_PID_KP  0.3
#define ROLL_PID_KI  0
#define ROLL_PID_KD  0.7
#define ROLL_PID_MIN  -100.0
#define ROLL_PID_MAX  100.0

#define PITCH_PID_KP  0.3
#define PITCH_PID_KI  0             ////toutes les K a verifier et tester
#define PITCH_PID_KD  0.7
#define PITCH_PID_MIN  -100.0
#define PITCH_PID_MAX  100.0

#define YAW_PID_KP  0.7
#define YAW_PID_KI  0
#define YAW_PID_KD  0
#define YAW_PID_MIN  -100.0
#define YAW_PID_MAX  100.0

////////////////////////LEVEL PIDS
#define ROLL_PID_LEVEL_KP  0.4
#define ROLL_PID_LEVEL_KI  0.01
#define ROLL_PID_LEVEL_KD  2.2

#define PITCH_PID_LEVEL_KP  0.4
#define PITCH_PID_LEVEL_KI  0.01             ////toutes les K a verifier et tester
#define PITCH_PID_LEVEL_KD  2.2

////////Config RX///////////////////
#define THROTTLE_RMIN  1000
#define THROTTLE_WMIN  1100
#define THROTTLE_SAFE_SHUTOFF 1120
#define THROTTLE_RMAX  2000
#define THROTTLE_WMAX  1850
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
