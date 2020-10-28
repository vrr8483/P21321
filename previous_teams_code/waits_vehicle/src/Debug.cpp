#include "Arduino.h"
#include "Debug.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "sensor.h"

uint32_t sampleTime = 0;
uint32_t testVector = random(0,65535);
uint32_t testVectorb = random(0,65535);
void testPlots(){
    if(millis() > sampleTime + 200){
        testVector += random(-500,500);
        //testVectorb += random(-1000,1000);
        sampleTime = millis();
        Serial_Printf("a%lu,%lu\n\r", testVector, sampleTime);
        //Serial_Printf("b%lu,%lu\n\r", testVectorb, sampleTime);

    }
}

/*
**Initializes and flushes the serial
*/
void initSerial()
{
    Serial.begin(BAUD_RATE);
    Serial.flush();
}

/*
**Mirrors printf functionality, but to the serial.
**Stole this from my RTE project, we'l see if it works in arduino.
**@params   str - String to write to serial
**          ... - Format parameters
*/
void Serial_Printf(const char * str, ...)
{
    char buffer[255];

    va_list argptr;
    va_start(argptr, str);

    vsnprintf(buffer, sizeof(buffer), str, argptr);
    va_end(argptr);

    Serial.write(buffer);
}

char getChar()
{
    return Serial.read();
}

uint8_t characterAvailable()
{
    return Serial.available();
}

void printPrompt()
{
    Serial_Printf(">>");
}

/**
 * Lists the different commands available to the debugger
 */
void printUsage()
{
    Serial_Printf("Welcome to the WAITS vehicle debug terminal!\r\n");
    Serial_Printf("Available Commands:\r\n");
    //Serial_Printf("\t(D | d) - Enter debug mode, defaults to off.\r\n");
    //Serial_Printf("\t(X | x) - Exit Debug mode, continue as normal.\r\n" );
    //Serial_Printf("\t(S | s) [leftSpeed rightSpeed] - Set the speed of each motor.\r\n");
    //Serial_Printf("\t(P | p) [leftPWM rightPWM] - Set the PWM for each motor.\r\n");
    Serial_Printf("\t(A | a) [pwm] - Set the PWM value of the actuator.\r\n");
    Serial_Printf("\t(D | d) [dir] - Set direction of actuator, 0 forward else reverse.\r\n");
    Serial_Printf("\t(X | x) - Stop both wheels (PWM and Speed are set to 0).\r\n");
}

void putChar(char c)
{
    Serial.write(c);
}

/**
 * Handles the debug command sent through the terminal.
 * @param str - Null terminated string entered through the terminal
 *      D | d - Enter debug mode, defaults to off and required to run terminal commands
 *      X | x - Exit debug mode
 *      S | s [leftSpeed rightSpeed] - Set the speed of each motor
 *      P | p [leftPWM rightPWM] - Set the PWM for each motor
 *      A | a [PWM] - Set the PWM for the sensor
 *      B | b - Stop both wheels and sensor
 */
void handleCommand(char * str)
{
    //Split the inputted string into tokens
    char * tokens[5];
    int i = 0;
    char * token = strtok(str, " ");
    //Tokenize the command into something parsable.
    while(token != NULL && i < 5){
        //Put the tokens in an array for easy parsing
        tokens[i] = token;
        i++;
        token = strtok(NULL, " ");
    }

    //Parse commands and do whatever
    char * command = tokens[0];
    int params[4] = {0,0,0,0};
    if(i > 0){
        for(int k = 0; k < i; k++){
            char * ptr;
            params[k] = strtol(tokens[k+1], &ptr, 10);
        }  
    }
    //Debug command
    /*
    if(command[0] == 'D' || command[0] == 'd'){
        Serial_Printf("\r\nEntering debug mode!\r\n");
    }
    //Exit debug command
    if(command[0] == 'X' || command[0] == 'x'){
        Serial_Printf("\r\nExiting debug mode!\r\n");
    }
    */
    //Set Speed command
    if(command[0] == 'S' || command[0] == 's'){
        //Serial_Printf("\r\nSetting motor speed to Left , Right: %d , %d!\r\n", params[0], params[1]);
    }
    //Set PWM command
    if(command[0] == 'P' || command[0] == 'p'){
        //Serial_Printf("\r\nSetting PWM to LeftTop , LeftBot , RightTop, RightBot: %d , %d , %d , %d!\r\n",
          //  params[0], params[1], params[2], params[3] );
    }
    //Set actuator PWM
    if(command[0] == 'A' || command[0] =='a'){
        //Serial_Printf("\r\nSetting actuator PWM to %d", params[0]);
        setSensorPWM(params[0]);
    }
    //Set actuator direction
    if(command[0] == 'D' || command[0] =='d'){
        Serial_Printf("\r\nSetting actuator direction to %d\n", params[0]);
        setSensorDirection(params[0]);
    }
    //Stop wheels command
    if(command[0] == 'X' || command[0] == 'x'){
        //Serial_Printf("\r\nStopping all motors and actuator!\r\n");
        setSensorPWM(0);
    }

}
