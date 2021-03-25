/**
 * Lists out all of the structs and #defines in main
 */

#define ESTOP (0)
#define MONITORING_ENABLED (1)

typedef unsigned char uint8_t;

typedef enum
{
    INITIALIZATION_STATE,
    MOVEMENT_STATE,
    SENSING_STATE,
    ERROR_STATE,
    ESTOP_STATE,

} VEHICLE_STATE_t;

//Different type of errors that can occur, some more managable than others
typedef enum
{
    //Vehicle is fine, everything is fine, just act normal and everything will be fine
    NO_ERROR_STATE,
    //Need to cut motors and sensor NOW, user must retrieve vehicle
    HARD_ERROR_STATE,
    //Disable sensor, vehicle can still move perfectly fine 
    SENSOR_ERROR_STATE,
    //Vehicle can manage to move in reduced state, sensor disabled
    SOFT_ERROR_STATE,
    //Low battery, send indication to user, perhaps disable sensor?
    LOW_BATTERY_ERROR_STATE,
} ERROR_STATE_t;

typedef enum
{
    SENSING_STATE_0,
    SENSING_STATE_1,
    SENSOR_STATE_SAFE,
} SENSOR_STATE_t;

typedef struct
{

} MESSAGE_t;

typedef struct
{

} MovementCommand_t;