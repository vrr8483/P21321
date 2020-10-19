#define MAX_ANALOG_WRITE (255)
#define DEBUG_MOTORS_ENABLED (1)
//Motor PWM Pins
#define LEFT_TOP_PWM_PIN (2)
#define LEFT_BOT_PWM_PIN (3)
#define RIGHT_TOP_PWM_PIN (4)
#define RIGHT_BOT_PWM_PIN (5)
//Speeds
#define INITIAL_MOTOR_SPEED (0)
#define MIN_MOTOR_SPEED (0)
#define MAX_MOTOR_SPEED (50)
//Wheels - is this even needed?
#define NUM_WHEELS (4)
#define WHEEL_SIZE (6)//Only freedom units here y'all
#define CM_PER_INCH (2.54)
#define PI (3.14)
#define MAX_SPEED_CM_PER_S (100)//Speed in metric, wheels in customary...neat
#define MAX_SPEED_REVS_PER_S (TICKS_PER_REV  * MAX_SPEED_CM_PER_S) / WHEEL_SIZE / PI / CM_PER_INCH)

//PID LOOP GAINS
#define PROPORTIONAL_GAIN (0.1)
#define INTEGRAL_GAIN (0.0)

void initPWM();
void setMotorSpeed(int motor, int speed);