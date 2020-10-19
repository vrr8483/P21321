#include "MainLoop.h"
#include "Debug.h"
#include "sensor.h"

/**
 * Main loop of the program
 */
void MainLoop(){
    VEHICLE_STATE_t vehicleState;
    SENSOR_STATE_t sensorState;
    MovementCommand_t movementCommand;

    initSerial();
    initSensor();

    int strIndex = 0;
    char str[100];
    //printUsage();
    //printPrompt();

    //Main program loop
    while(1) {

        //testPlots();//Test vectors
        reportCurrentSense();
        reportDistanceSense();

        //Sensor Task

        //Debug task, check if character is available before calling on UART
        if(characterAvailable()){
            //Get character from UART, this function blocks
            char c = getChar();

            //Deal with the script adding an '!' to the front of all commands so that it can filter out
            //What it writes to COM. And by deal with it I mean throw it away.
            if(c == 'q'){
                //NOP
            }
            //Newline on Windows, end the string and handle the input
            else if(c == 0x0D){
                //Null terminate string
                str[strIndex] = 0x00;
                //Handle the input
                //Serial_Printf(str);
                handleCommand(str);
                //Serial_Printf("Recieved: %s\n\r", str);

                //Setup for next command
                strIndex = 0;
                //printPrompt();

            }
            //Handle backspaces
            /*
            else if (c == 0x7F){
                if(strIndex > 0){
                    strIndex--;
                    putChar(c);
                }
            }*/
            else{
                str[strIndex] = c;
                strIndex++;
                //putChar(c);

            }


        }
    }
}