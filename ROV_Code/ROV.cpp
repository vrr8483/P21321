#include <cstdio>
#include <ctime>
#include <chrono>
#include <string>
#include <unistd.h>

#include <fcntl.h>
#include <asm/termbits.h>
#include <asm/ioctls.h>
#include <sys/ioctl.h>

#include "SBUS.h"

#include <csignal>

#include <wiringPi.h>

#include "PCA9685.h"

using namespace std::chrono;

//channels as defined in the profile being used on the transmitter. 
//these are channel indices, which are 0-indexed, so keep that in 
//mind since the channels shown on the transmitter start at 1.
#define LEFT_STEER_CHANNEL (0)
#define LEFT_THROTTLE_CHANNEL (1)
#define RIGHT_THROTTLE_CHANNEL (2)
#define RIGHT_STEER_CHANNEL (3)
#define SA_UP_CHANNEL (4)
#define SA_DOWN_CHANNEL (5)
#define SD_ON_CHANNEL (6)

#define SENSOR_CMD_HEADER (0x53)
#define SENSOR_CMD_FOOTER (0x45)

#define INA_WPI_PIN (3)
#define ENA_WPI_PIN (4)
#define ENB_WPI_PIN (5)
#define INB_WPI_PIN (6)

struct sensor_cmd_type {
  uint8_t header = SENSOR_CMD_HEADER;
  uint16_t sensor_id;
  uint32_t value;
  uint8_t footer = SENSOR_CMD_FOOTER;

  bool verify_cmd(){
    return header == SENSOR_CMD_HEADER && footer == SENSOR_CMD_FOOTER;
  }
}__attribute__((packed));
//the __attribute__((packed)) part tells the compiler 
//NOT to use word-aligning optimization for this struct, 
//which is key for its use in a union.


union sensor_cmd_packet_type {
  uint8_t bytes[8];
  sensor_cmd_type cmd;
    sensor_cmd_packet_type(){
        for (uint8_t i = 0; i < 8; ++i){
            bytes[i] = 0;
        }
        cmd = {0};
	cmd.header = SENSOR_CMD_HEADER;
	cmd.footer = SENSOR_CMD_FOOTER;
    }
};

SBUS sbus;

uint32_t value = 0;

PCA9685 pca{};

//exit fxn
void signalHandler( int signum ) {
   
	pca.set_pwm(0, 0, 0);
	pca.set_pwm(1, 0, 0);

	digitalWrite(ENA_WPI_PIN, 0);
	digitalWrite(ENB_WPI_PIN, 0);
	digitalWrite(INA_WPI_PIN, 0);
	digitalWrite(INB_WPI_PIN, 0);

   	exit(signum);  
}

int send_sensor_cmd(uint16_t id, uint32_t new_val){
	//send command to sensor
	sensor_cmd_type cmd;
	cmd.sensor_id = id;
	cmd.value = new_val;

	//this stores the packet in big-endian format. 
	//Raspbian is little-endian, as is Arduino. 
	//Use endianness_reverse = true (line 6 in 
	////frsky_arduino.ino as of the time of writing) 
	////to reverse the endianness of the packet inside arduino.
	/*uint8_t packet_bytes[8];
	packet_bytes[0] = cmd.header;
	packet_bytes[1] = cmd.sensor_id >> 8;   
	packet_bytes[2] = cmd.sensor_id & 0xFF;   
	packet_bytes[3] = (cmd.value >> 24) & 0xFF;
	packet_bytes[4] = (cmd.value >> 16) & 0xFF;
	packet_bytes[5] = (cmd.value >> 8) & 0xFF;
	packet_bytes[6] = cmd.value & 0xFF;
	packet_bytes[7] = cmd.footer;*/
    	
    	sensor_cmd_packet_type sensor_packet;
    	sensor_packet.cmd = cmd;
    	//debugging printouts
    	/*for (uint8_t i = 0; i < 8; ++i){
		printf("%d: %x\t", i, sensor_packet.bytes[i]);
    	}
    	printf("\n");
    	printf("Header: %x\t", sensor_packet.cmd.header);
    	printf("ID: %x\t", sensor_packet.cmd.sensor_id);
    	printf("Val: %x\t", sensor_packet.cmd.value);
    	printf("Footer: %x\t", sensor_packet.cmd.footer);
    	printf("\n");*/

	int packet_size = sizeof(sensor_cmd_packet_type);
    	
    	if (write(sbus._fd, sensor_packet.bytes, packet_size) != packet_size){
        	fprintf(stderr, "Failed to send sensor command.\n");
		return -1;
    	}
	return 0;
}

void onPacket(sbus_packet_t packet){

	//printf("Callback called\n");
    	//printf("now: %ld, lastPrint: %ld\n", now, lastPrint);
	milliseconds period = milliseconds(100);
	static milliseconds lastms = duration_cast<milliseconds>(
			system_clock::now().time_since_epoch()
			);
	milliseconds ms = duration_cast<milliseconds>(
			system_clock::now().time_since_epoch()
			);

	/*if (ms - lastms > period){
    		lastms = ms;
        	//lastPrint = now;
		for (int c = 0; c < 16; ++c){
			printf("ch%d: %u\t", c+1, packet.channels[c]);
		}
		printf("ch17: %u\tch18: %u\t", packet.ch17, packet.ch18);
		printf(
			"%s\tFailsafe: %s\t", 
			packet.frameLost ? "Frame lost" : "Frame fine",
			packet.failsafe ? "active" : "inactive"
		);
		printf("\n");
		send_sensor_cmd(0x5958, value++);
		send_sensor_cmd(0x5900, value/10);
    	}*/

	//set LED PWM to the adjusted value of packet.channels[0]
	int max_PWM = 25;
	int min_PWM = 0;
	int max_throttle = 1811;
	int nominal_throttle = 992;
	int PWM = (int)(
			((max_PWM - min_PWM)*1.0/(max_throttle - nominal_throttle))*
			(packet.channels[RIGHT_THROTTLE_CHANNEL] - nominal_throttle)
			);
	if (PWM < 0) PWM = 0;
	int PCA_PWM = (int)((4095.0/100)*PWM);
	//int adjusted_val = (int)(1.0*packet.channels[0]*(4095.0/1811));
	pca.set_pwm(0, 0, PCA_PWM);

	//adjusted_val =  (int)(1.0*packet.channels[1]*(4095.0/1811));
	//pca.set_pwm(1, 0, adjusted_val);

	bool A_enabled = packet.channels[SA_DOWN_CHANNEL] > 1000;
	bool B_enabled = packet.channels[SA_UP_CHANNEL] > 1000;

	digitalWrite(INA_WPI_PIN, A_enabled);
	digitalWrite(INB_WPI_PIN, B_enabled);

	bool drill_on = packet.channels[SD_ON_CHANNEL] > 1000;
	if (drill_on){
		pca.set_pwm(1, 0, 4095);
	} else {
		pca.set_pwm(1, 0, 0);
	}

}

int main(int argc, char* argv[])
{
	wiringPiSetup();

	pinMode(ENA_WPI_PIN, OUTPUT);
	pinMode(ENB_WPI_PIN, OUTPUT);
	pinMode(INA_WPI_PIN, OUTPUT);
	pinMode(INB_WPI_PIN, OUTPUT);

	std::signal(SIGINT, signalHandler);

	digitalWrite(ENA_WPI_PIN, 1);
	digitalWrite(ENB_WPI_PIN, 1);

	pca.set_pwm_freq(1500.0);
	pca.set_pwm(0, 0, 0);

	const int num_usb_ports = 4;
	const std::string port_base = "/dev/ttyUSB";

	//https://www.geeksforgeeks.org/getopt-function-in-c-to-parse-command-line-arguments/
	int opt;
	std::string arduino_filepath = port_base + std::to_string(0);

	while ((opt = getopt(argc, argv, ":p:")) != -1){
		switch(opt){
		case 'p':
			printf("Using port: %s\n", optarg);
			arduino_filepath = optarg;	
			break;
		case ':':
			fprintf(stderr, "Option needs a value\n");
			break;
		case '?':
			fprintf(stderr, "Unknown option: %c\n", optopt);
			break;
		}
	}

    sbus.onPacket(onPacket);

    sbus_err_t err = sbus.install(arduino_filepath.c_str(), true);
    
    if (err == SBUS_ERR_OPEN){
	//loop through /dev/ttyUSB<n> and attempt to find a working USB port
 
	printf("Could not open device at %s.\n"
		"Attempting to find device by searching devices that match %s<n> (searching indices from 0 to %d)...\n", 
		arduino_filepath.c_str(), port_base.c_str(), num_usb_ports - 1);

	for (int i = 0; i < num_usb_ports; ++i){
		std::string port_i = port_base + std::to_string(i);
		err = sbus.install(port_i.c_str(), true);
		if (err == SBUS_OK){
			printf("Found device on %s\n", port_i.c_str());
			break;
		}
	}
	if (err != SBUS_OK){
		fprintf(stderr, "Could not find device.\n");
	}
    } 
    
    if (err != SBUS_OK) {
        fprintf(stderr, "SBUS install error: %d\n\r", err);
        return err;
    }

    while ((err = sbus.read()) != SBUS_FAIL)
    {
        if (err == SBUS_ERR_DESYNC)
        {
        	fprintf(stderr, "SBUS desync\n\r");
        }
    }

    fprintf(stderr, "SBUS error: %d\n\r", err);

    return err;
}
