#include "sensor.h"


//Global variable to store the last time the sensor was sampled
uint32_t lastSampled = 0;
//Current and distance last sample time
uint32_t currentLastSampled = 0;
uint32_t distanceLastSampled = 0;
//PWM Variables
uint8_t  sensorPWMValue = 0;
uint8_t  sensorDirection = 0;
//Sensor current draw array
//Digital filter for force derivative
int forceFilter[] = {-1, -2, 0, 2, 1};
uint16_t forceSamples[sizeof(forceFilter)];
uint8_t  forceSampleIndex = 0;
uint8_t  forceSampleMiddleIndex = sizeof(forceFilter) / 2;
//Sensor distance array
//Digital filter for distance derivative
int distanceFilter[] = {-1, -2, 0, 2, 1};
uint16_t distanceSamples[sizeof(distanceFilter)];
uint8_t  distanceSampleIndex = 0;
uint8_t  distanceMiddleIndex = sizeof(distanceFilter) / 2;
uint8_t  passCounter = 0;
uint8_t  failCounter = 0;
uint8_t  forceInitialized = 0;
uint8_t  distanceInitialized = 0;


/**
 * Initializes the servo to be on whatever pins.
 * Feedback pins are also initialized here if they need it.
 */
void initSensor()
{
    //Initialize PWM pin
    pinMode(SENSOR_PWM_PIN, OUTPUT);
    //Initialize direction pins
    pinMode(ACTUATOR_INA, OUTPUT);
    pinMode(ACTUATOR_INB, OUTPUT);
    pinMode(ACTUATOR_ENA, OUTPUT);
    pinMode(ACTUATOR_ENB, OUTPUT);

    //Initialize current and distance sense pins
    //Might not be required for analog pins??
    pinMode(SENSOR_DISTANCE_ANALOG_PIN, INPUT);
    pinMode(SENSOR_FORCE_ANALOG_PIN, INPUT);

    //Enable actuator
    analogWrite(ACTUATOR_ENA, 255);
    analogWrite(ACTUATOR_ENB, 255);


}

/**
 * Reports the current sense pin to serial OUTPUT
 * Uses format:
 * i<sample>,<time>
 */
void reportCurrentSense() {
    uint32_t currentTime = millis();
    //only report every sample period
    if( currentTime > currentLastSampled + CURRENT_SENSE_SAMPLING_RATE){
        //Sample the current and report
        currentLastSampled = currentTime;//set time to current time
        uint32_t currentSample = analogRead(SENSOR_FORCE_ANALOG_PIN);
        Serial_Printf("i%lu,%lu\n\r", currentSample, currentTime);
    }
}

/**
 * Reports the distance sense pin to serial OUTPUT
 * Uses format:
 * d<sample>,<time>
 */
void reportDistanceSense() {
    uint32_t currentTime = millis();
    //only report every sample period
    if( currentTime > distanceLastSampled + DISTANCE_SENSE_SAMPLING_RATE){
        //Sample the current and report
        distanceLastSampled = currentTime;//set time to current time
        uint32_t distanceSample = analogRead(SENSOR_DISTANCE_ANALOG_PIN);
        Serial_Printf("d%lu,%lu\n\r", distanceSample, currentTime);
    }
}

/**
 * @param direction - 0 for forward, else for reverse
 */
void setSensorDirection(uint8_t direction){
    if(direction) {
        analogWrite(ACTUATOR_INA, 255);
        analogWrite(ACTUATOR_INB, 0);

    }
    else {
        analogWrite(ACTUATOR_INA, 0);
        analogWrite(ACTUATOR_INB, 255);
    }
}

void setSensorPWM(uint8_t dutyCycle)
{
    //Check for out of bounds, something is wrong
    if(dutyCycle > 100){
        dutyCycle = 0;//Just don't set it
    }
    //Need to calculate what to write to the pin, 255 is 100%, 127 is 50%, etc...
    sensorPWMValue = ((dutyCycle / 100.0) * MAX_ANALOG_WRITE);
    //Serial_Printf("Setting duty cycle to %d\n\r", sensorPWMValue);

    analogWrite(SENSOR_PWM_PIN, sensorPWMValue);
}

/**
 * Sample the sensor, taking the derivative of the distance and force
 * @return bool * - Boolean array [pass, fail], return of [0,0] means still sensing
 */
uint8_t * sampleSensor()
{
    //Initialize return value [pass, fail]
    uint8_t retval[2] = {0,0};
    //Find the current time, compare it to the last sampeld time
    uint32_t currentTime = millis();


    //Time to sample
    if(currentTime > lastSampled + SENSOR_SAMPLE_RATE){

        //Sample starts here!
        lastSampled = currentTime;
        
        //Sample the force and distance
        uint16_t forceSample = analogRead(SENSOR_FORCE_ANALOG_PIN);
        uint16_t distanceSample = analogRead(SENSOR_DISTANCE_ANALOG_PIN);
        long distanceDerivative = 0;
        long forceDerivative = 0;


        if(!forceInitialized){
            forceSamples[forceSampleIndex] = forceSample;
            forceSampleIndex++;
            if(forceSampleIndex > sizeof(forceSamples) - 1){
                forceInitialized = 1;
            }
        }
        else{
            //Calculate the derivative of the force
            for(int i = 1; i < sizeof(forceFilter); i++){
                //Rolling calculation of derivative, so we only need 1 loop
                forceDerivative += (long) forceSamples[i] * forceFilter[i-1];
                forceSamples[i-1] = forceSamples[i];
            }
            //calculate last sample
            forceSamples[sizeof(forceFilter)] = forceSample;
            forceDerivative += (long) forceSample * forceFilter[sizeof(forceFilter - 1)];
        }
        //Calculate the derivative of the distance
        if(!distanceInitialized){
            distanceSamples[distanceSampleIndex] = distanceSample;
            distanceSampleIndex++;
            if(distanceSampleIndex > sizeof(distanceSamples) - 1){
                distanceInitialized = 1;
            }
        }
        else{
            distanceSamples[distanceSampleIndex] = distanceSample;
            for(int i = 1; i < sizeof(distanceFilter); i++){
                //Rolling calculation, only need one loop, literal microseconds saved
                distanceDerivative += (long) distanceSamples[i] * distanceFilter[i-1];
                distanceSamples[i-1] = distanceSamples[i];
            }
            //calculate last sample
            distanceSamples[sizeof(distanceFilter)] = distanceSample;
            distanceDerivative += (long) distanceSample * distanceFilter[sizeof(distanceFilter - 1)];
        }


        //Calculate pass/fail
        //Must pass/fail for duration of SENSOR_RESULT_TIME, which should limit noise
        //Calculate pass
        if(forceSample > PASS_THRESHOLD){
            passCounter++;
        }
        //One fail in a string of passes doesn't reset the counter but has an effect
        else if(passCounter > 0){
            passCounter--;
        }
        else {
            passCounter = 0;
        }

        //Calculate failure
        if(forceDerivative < 0 || distanceSample >= SENSOR_MAX_DISTANCE ){
            failCounter++;
        }
        //One pass in a string of failures doesn't reset the counter, but still has an effect
        else if(failCounter > 0){
            failCounter--;
        }
        else {
            failCounter = 0;
        }

        //Check for pass or fail, set the flags if so
        if(passCounter > SENSOR_RESULT_TIME / SENSOR_SAMPLE_RATE){
            retval[0] = 1;
        }
        if(failCounter > SENSOR_RESULT_TIME / SENSOR_SAMPLE_RATE){
            retval[1] = 1;
        }
    }

    return retval;
}

void testSensor(uint8_t dutyCycle, uint8_t direction) {
    setSensorPWM(dutyCycle);
}