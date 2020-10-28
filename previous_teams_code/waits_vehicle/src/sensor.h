#include "Arduino.h"
#include "Debug.h"
//Pin Mappings
#define SENSOR_FORCE_ANALOG_PIN (A7)
#define SENSOR_DISTANCE_ANALOG_PIN (A6)
#define SENSOR_PWM_PIN (4)
#define ACTUATOR_INA (32)
#define ACTUATOR_INB (33)
#define ACTUATOR_ENA (34)
#define ACTUATOR_ENB (35)
//Constants
#define SENSOR_SAMPLE_RATE (20)//milliseconds
#define SENSOR_RESULT_TIME (200)//milliseconds
#define MAX_ANALOG_WRITE (255)
#define PASS_THRESHOLD (1000)//This number is made up, please replace it
#define SENSOR_MAX_DISTANCE (1024)
//Sampling rates
#define CURRENT_SENSE_SAMPLING_RATE (100)
#define DISTANCE_SENSE_SAMPLING_RATE (100)

#define DRILL_PWM_PIN (2)


//Function declarations
void initSensor();
void setSensorPWM(uint8_t dutyCycle);
void setDrillPWM(uint8_t dutyCycle);
void testSensor(uint8_t dutyCycle, uint8_t direction);
void setSensorDirection(uint8_t direction);
void reportCurrentSense();
void reportDistanceSense();
void reportDistanceCurrentSense();
