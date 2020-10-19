#include "Arduino.h"
#include "motors.h"
#include "Debug.h"


const int pwmPins[] = {LEFT_TOP_PWM_PIN, LEFT_BOT_PWM_PIN, RIGHT_TOP_PWM_PIN, RIGHT_BOT_PWM_PIN};
//Motor flags and values
uint8_t debugFlag = 0;
//PWM of each pin, 0 to 100
uint8_t pwmValues[4] = {0,0,0,0};
//requested speed of the motors [leftTop, leftBot, rightTop, rightBot]
uint16_t requestedSpeed[] = {INITIAL_MOTOR_SPEED, INITIAL_MOTOR_SPEED, INITIAL_MOTOR_SPEED, INITIAL_MOTOR_SPEED};
//actual speed of the motors [leftTop, leftBot, rightTop, rightBot]
uint16_t actualSpeed[] = {INITIAL_MOTOR_SPEED, INITIAL_MOTOR_SPEED, INITIAL_MOTOR_SPEED, INITIAL_MOTOR_SPEED};
//Sum of speed error
static uint16_t errorSpeedSum[] = {0, 0, 0, 0};

//Encoder values
extern volatile uint32_t encoderTicks[4];
uint32_t encoderLastRead[4] = {0,0,0,0};
//


void setDebugFlag(uint8_t v)
{
    debugFlag = v;
}

void setPWMValues(uint8_t leftTop, uint8_t leftBot, uint8_t rightTop, uint8_t rightBot)
{
    pwmValues[0] = leftTop;
    pwmValues[1] = leftBot;
    pwmValues[2] = rightTop;
    pwmValues[3] = rightBot;
}

void incrementPWMValues(int * incrementValues)
{
    for(uint8_t i = 0; i < NUM_WHEELS; i++)
    {
        pwmValues[i] += *(incrementValues + i);
    }
}

/*
**Initializes the PWM pins to output mode
*/
void initPWM(){
    for(int i = 0; i < sizeof(pwmPins); i++){
        pinMode(pwmPins[i], OUTPUT);
    }
}

/*
** Calculates and sets the PWM of the given pin
** @param   pin - pin to set PWM of
**          dutyCycle - dutyCycle to set pin to, 0-100
*/
void setDutyCycle(int pin, int dutyCycle){

    //Need to calculate what to write to the pin, 255 is 100%, 127 is 50%, etc...
    int toWrite = ((dutyCycle / 100) * MAX_ANALOG_WRITE);

    analogWrite(pin, toWrite);
}

/*
**Sets the speed of the given motor to the given value, as long as that value is within range
**@param    motor - Motor to set the speed of [left, right]
**          speed - Speed to set the motor to as long as it is within bounds
*/
void setMotorSpeed(int motor, int speed){

    //Make sure speed is in bounds
    if(speed < MIN_MOTOR_SPEED){
        speed = MIN_MOTOR_SPEED;
        //Debug is enabled, log out a warning because this shouldn't happen
        if(DEBUG_ENABLED && DEBUG_MOTORS_ENABLED){
            Serial_Printf("WARNING - Motor %d set to speed %d which is below the min speed of %d\r\n",
                motor, speed, MAX_MOTOR_SPEED);
        }
    }
    else if(speed > MAX_MOTOR_SPEED){
        speed = MAX_MOTOR_SPEED;

        //Debug is enabled, log out a warning because this shouldn't happen
        if(DEBUG_ENABLED && DEBUG_MOTORS_ENABLED){
            Serial_Printf("WARNING - Motor %d set to speed %d which exceeds the max speed of %d\r\n",
                motor, speed, MAX_MOTOR_SPEED);
        }
    }
    
    requestedSpeed[motor] = speed;

}

/**
 * Handles motor fluctuations by reading the encoders and setting the motor PWM based on expected speed vs. actual speed.
 */
void handleMotors()
{
    //Handle all four motors
    int deltaPWM[] = {0, 0, 0, 0};
    for(int i = 0; i < sizeof(pwmPins); i++){
        uint16_t currentTime = millis();
        //Calculate actual speed (ticks/sec)
        actualSpeed[i] = encoderTicks[i] * 1000 / (currentTime - encoderLastRead[i]);
        
        //Calculate Error
        uint16_t errorSpeed = requestedSpeed[i] - actualSpeed[i];
        errorSpeedSum[i] += errorSpeed; 

        //PID LOOP change in PWM = Kp*error + Ki*sum(error)
        deltaPWM[i] = PROPORTIONAL_GAIN * errorSpeed + INTEGRAL_GAIN * errorSpeedSum[i]; 

        encoderTicks[i] = 0;
        encoderLastRead[i] = currentTime;

    }
    //Set new PWM Values
    incrementPWMValues(deltaPWM);
}