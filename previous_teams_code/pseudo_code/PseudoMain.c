//Includes
#include "PseudoMain.h" //Includes all of the #defines

//Initialization Functions, should be in their respective files
initCLK()
{
    //Set Clock Speed
    //Enable Clock
}

initGPIO()
{
    //Enable GPIO clocks
    //Set used GPIO pins to output, input, alternate functions
    //Enable GPIO if needed
}

initPWM()
{
    //Enable PWM Clocks
    //Enable TIM
    //Setup TIM
    //Enable Required Channels
    //Set CCRs
}

int pseudo_main()
{

    //Declare and Initialize Variables
    uint8_t hardStopFlag = 0;
    VEHICLE_STATE_t vehicleState = INITIALIZATION_STATE;
    SENSOR_STATE_t sensorState;
    ERROR_STATE_t errorState = NO_ERROR_STATE;
    MovementCommand_t movementCommand;

    //Initialization Code
    initCLK();  //System Clock, if required by board
    initGPIO(); //GPIO for both PWM and I/O
    initPWM();  //Motor & Servo Control, possibly the linear actuator
    //Main Program Loop of WAITS
    while (1)
    {

        //Hard Stop Engaged, hold in hard stop state
        if (hardStopFlag)
        {
            //Engage the break on the motors
            breakMotors();
            //Put the sensor in a safe state
            sensorState = SENSOR_STATE_SAFE; //That's a placeholder
        }

        //Check for user input flags
        if (userInputFlag)
        {
            //Get and decode the message into something usable
            //No clue what this message will look like
            MESSAGE_t m = messageDecode();
            //Emergency stop was hit, stop sensor now.
            if (m == ESTOP)
            {
                vehicleState = ESTOP_STATE;
            }
            //Something is blocking input (sensing?), provide some feedback if possible
            else if (vehicleState == SENSING_STATE || vehicleState == ESTOP_STATE || vehicleState == ERROR_STATE)
            {
                userFeedback();
                //throw away the message?
            }
            //Nothing is blocking, go ahead and handle input
            else if (m == MOVEMENT_COMMAND)
            {
                //Store the movement command and set to MOVEMENT_STATE
                getMovementCommand(movementCommand, m);
                vehicleState = MOVEMENT_STATE;
            }
            else if (m == SENSING_COMMAND)
            {
                vehicleState = SENSING_STATE;
            }
            //else its noise, trash it
        }

        //The sensor is currently running, do the associated tasks and logic with that
        if (sensingFlag)
        {
            //Handle sensing, in a non-blocking manner preferably
            switch (sensorState)
            {
            case SENSING_STATE_0:
                handleState0();
                break;
            case SENSING_STATE_1:
                handleState1();
                break;
            //Add more states as needed
            default:
                //may or may not be useful
                break;
            }
        }

        //Monitor components to make sure everything is operating within parameters
        if (MONITORING_ENABLED)
        {
            errorState = NO_ERROR_STATE;
            if (tempOutOfRange())
            {
                errorState = SOFT_ERROR_STATE;
            }
            if (currentOutOfRange())
            {
                errorState = SOFT_ERROR_STATE;
            }
            if (speedOutOfRange())
            {
                errorState = HARD_ERROR_STATE;
            }
            if (sensorOutOfRange())
            {
                errorState = SENSOR_ERROR_STATE;
            }
        }

        //Debug is enabled, perhaps it takes info from the UART?
        if (debugFlag && UARTFlag)
        {
            //Debug is enabled & character on the UART
            char command = getChar();
            handleDebug(command);
        }

        switch (vehicleState)
        {
        case MOVEMENT_STATE:
            //moving code
            break;
        case SENSING_STATE:
            //sensing code
            break;
        case ERROR_STATE:
            //Error code
            break;
        case ESTOP_STATE:
            //ESTOP CODE
            break;
        default:
            break;
        }
    }
}