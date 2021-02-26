#include <SPort.h>                  //Include the SPort library

#define SENSOR_CMD_HEADER (0x53)
#define SENSOR_CMD_FOOTER (0x45)

bool endianness_reverse = false;

struct sensor_cmd_type {
  uint8_t header;
  uint16_t sensor_id;
  uint32_t value;
  uint8_t footer;

  bool verify_cmd(){
    return header == SENSOR_CMD_HEADER && footer == SENSOR_CMD_FOOTER;
  }
};

union sensor_cmd_packet_type {
  uint8_t bytes[8];
  sensor_cmd_type cmd;
};

sensor_cmd_packet_type sensor_cmd_packet = {0};

SPortHub hub(0x12, 10);        //Hardware ID 0x12, SW pin 10
//SPortHub hub(0x12, Serial3); //DOES NOT WORK
const uint16_t sensor1_id = 0x5900;
const uint16_t sensor2_id = 0x5958;
SimpleSPortSensor sensor1(sensor1_id);
SimpleSPortSensor sensor2(sensor2_id);

int buffer_position = 0;

void setup() {
  Serial.begin(100000, SERIAL_8E2);
  hub.registerSensor(sensor1);       //Add sensor to the hub
  hub.registerSensor(sensor2);
  hub.begin();                      //Start listening
}

void loop() {

  while (Serial.available()){
    int new_data = Serial.read();
    if (new_data == -1){
      //no data available. shouldnt happen.
      break;
    }
    
    uint8_t new_byte = new_data & 0xFF;

    if (buffer_position > 0 && buffer_position < 8){
      // currently reading out packet data
      sensor_cmd_packet.bytes[buffer_position++] = new_byte;
    } else if (buffer_position == 0 && new_byte == SENSOR_CMD_HEADER){
      //no current packet, just looking for a new header
      sensor_cmd_packet.bytes[0] = new_byte;
      buffer_position = 1;
    }

    if (buffer_position < 8){
      //still reading out packet
      continue;
    }

    //packet data size reached
    if (sensor_cmd_packet.cmd.verify_cmd()){

      //process packet
      if (endianness_reverse){
        // reverse sensor id byte-wise
        uint8_t tmp = sensor_cmd_packet.bytes[1];
        sensor_cmd_packet.bytes[1] = sensor_cmd_packet.bytes[2];
        sensor_cmd_packet.bytes[2] = tmp;

        //reverse value byte-wise
        tmp = sensor_cmd_packet.bytes[3];
        sensor_cmd_packet.bytes[3] = sensor_cmd_packet.bytes[6];
        sensor_cmd_packet.bytes[6] = tmp;
        tmp = sensor_cmd_packet.bytes[4];
        sensor_cmd_packet.bytes[4] = sensor_cmd_packet.bytes[5];
        sensor_cmd_packet.bytes[5] = tmp;
      }

      //check ID
      switch (sensor_cmd_packet.cmd.sensor_id){
        case sensor1_id:
          sensor1.value = sensor_cmd_packet.cmd.value;
          break;
        case sensor2_id:
          sensor2.value = sensor_cmd_packet.cmd.value;
          break;
        default:
          Serial.print(sensor_cmd_packet.cmd.sensor_id);
          Serial.println(" did not match any sensor IDs.");
          break;
      }

      //time for new packet
      buffer_position = 0;
      
    } else { 
      //invalid packet, look for header possibly inside this packet

      bool found_header = false;
      for (int i = 1; i < 8; ++i){
        if (sensor_cmd_packet.bytes[i] == SENSOR_CMD_HEADER){
          found_header = true;
          for (int j = 0; i+j < 8; ++j){
            //copy bytes, starting with header, 
            //backwards in the buffer 
            //so that the header is aligned to the start.
            sensor_cmd_packet.bytes[j] = sensor_cmd_packet.bytes[i+j];
          }

          //set buffer position to point to next needed byte
          buffer_position = 8 - i;
        }
      }

      if (!found_header){
        //no header in this packet, just reset and try again
        buffer_position = 0;
        Serial.println("Invalid packet.");
      }
    } //end packet validation

  } //end while serial available
  
  /*sensor1.value = sensor1.value + 1;              //Set the sensor value
  if (sensor1.value > 69000){ //nice
    sensor1.value = 0;
  }
  sensor2.value = 0;*/
  hub.handle();                     //Handle new data
}
