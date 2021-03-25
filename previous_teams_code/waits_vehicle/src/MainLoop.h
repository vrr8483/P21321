#define ESTOP (0)
#define MONITORING_ENABLED (1)

typedef unsigned char uint8_t;
typedef unsigned int  uint16_t;
typedef unsigned long uint32_t;

typedef enum
{
    INITIALIZATION_STATE,
    MOVEMENT_STATE,
    SENSING_STATE,
    ERROR_STATE,
    ESTOP_STATE,

} VEHICLE_STATE_t;

typedef enum
{
    //Placeholders
    SENSOR_STATE_SAFE,
    SENSING_STATE_0,
    SENSING_STATE_1,
} SENSOR_STATE_t;

typedef struct
{
    //Insert message members here
} MESSAGE_t;

typedef struct
{
    //Insert command members here
} MovementCommand_t;

void MainLoop();