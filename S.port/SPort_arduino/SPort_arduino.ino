#include <SPort.h>                  //Include the SPort library

SPortHub hub(0x12, 10);        //Hardware ID 0x12, SW pin 10
//SPortHub hub(0x12, Serial3); //DOES NOT WORK
SimpleSPortSensor sensor1(0x5900);
SimpleSPortSensor sensor2(0x5901);

void setup() {
  hub.registerSensor(sensor1);       //Add sensor to the hub
  hub.registerSensor(sensor2);
  hub.begin();                      //Start listening
}

void loop() {
  sensor1.value = sensor1.value + 1;              //Set the sensor value
  if (sensor1.value > 69000){ //nice
    sensor1.value = 0;
  }
  sensor2.value = 0;
  hub.handle();                     //Handle new data
}
