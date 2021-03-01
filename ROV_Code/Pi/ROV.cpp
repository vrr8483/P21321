#include <cstdio>
#include <ctime>
#include <ctime>
#include <csignal>

#include <chrono>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <errno.h>
#include <iomanip>
#include <fcntl.h>
#include <termios.h>

//#include <asm/termbits.h>
#include <asm/ioctls.h>
#include <sys/ioctl.h>

#include <wiringPi.h>

#include "SBUS.h"
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

//sensor commands are the packets that the raspberry pi
//sends to the arduino to tell it to chenge its internally stored sensor value
//that the receiver polls for. the headers and footers were chosen for
//their ability to be converted to ASCII. Long story.
#define SENSOR_CMD_HEADER (0x53)
#define SENSOR_CMD_FOOTER (0x45)

//The Raspberry pi has two pin addressing modes: Wiring pi (WPI), and Broadcom (BCM).
//BCM is usually better to use.
#define INA_WPI_PIN (3)
#define ENA_WPI_PIN (4)
#define ENB_WPI_PIN (5)
#define INB_WPI_PIN (6)

//EN A and EN B are the H bridge enables for path A and B (active high)
//IN A and IN B are turned on one at a time to indicate the direction of the motor

#define INA_BCM_PIN (22)
#define ENA_BCM_PIN (23)
#define ENB_BCM_PIN (24)
#define INB_BCM_PIN (25)

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

PCA9685 pca{};

int arduino_serial_fd;
std::string arduino_stream_buf = "";

std::fstream log_file;

void arduino_serial_init(){
	//https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/#overview
    //This will be the only arduino plugged into the Pi,
    //which is an assumption, but an accurate one.
	arduino_serial_fd = open("/dev/ttyACM0", O_RDWR);
	if (arduino_serial_fd < 0){
		fprintf(stderr, "Arduino serial port open error %i\n", errno);
		exit(errno);
	}

	termios tty;
	if(tcgetattr(arduino_serial_fd, &tty) != 0) {
		fprintf(stderr, "Error %i from tcgetattr\n", errno);
		exit(errno);
	}
	
	tty.c_cflag &= ~PARENB; // no parity
	tty.c_cflag &= ~CSTOPB; //one stop bit

	tty.c_cflag &= ~CSIZE; // Clear all the size bits
	tty.c_cflag |= CS8; // 8 bits per byte

	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control

	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

	tty.c_lflag &= ~ICANON; //disable canonical mode (line by line processing)

	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl

	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); 
	// Disable any special handling of received bytes
	
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	
	//dont block, return with what is immediately available.
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	cfsetspeed(&tty, B115200);

	if (tcsetattr(arduino_serial_fd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "Error %i from tcsetattr\n", errno);
		exit(errno);
	}
	
	//delete contents of input buffer to avoid reading old data
	usleep(10000); //sleep for 10ms cause apparently this wont work otherwise? Linus pls fix
	tcflush(arduino_serial_fd, TCIFLUSH);
}

void log_file_init(std::string filename){
	log_file.open(filename, std::ios::out);
	if (!log_file){
		fprintf(stderr, "Cannot open log file for writing at %s\n", filename.c_str());
		exit(-1);
	}
}

void log_data(int t, int dist, int currsense){
	
	/*static milliseconds first_log_time = duration_cast<milliseconds>(
			system_clock::now().time_since_epoch()
			);

	milliseconds now = duration_cast<milliseconds>(
			system_clock::now().time_since_epoch()
			); 
	
	uint64_t diff_ms = (now - first_log_time).count();*/

	std::string line = "";
	line += std::to_string(t) + "," + std::to_string(dist) + "," + std::to_string(currsense) + '\n';
	log_file << line;
}

//called when SIGINT is received (Ctrl+C)
void signalHandler(int signum) {
   
	pca.set_pwm(0, 0, 0);
	pca.set_pwm(1, 0, 0);

	digitalWrite(ENA_BCM_PIN, 0);
	digitalWrite(ENB_BCM_PIN, 0);
	digitalWrite(INA_BCM_PIN, 0);
	digitalWrite(INB_BCM_PIN, 0);
	
	close(arduino_serial_fd);

	log_file.close();

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
	//frsky_arduino.ino as of the time of writing)
	//to reverse the endianness of the packet inside arduino.
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
	//milliseconds period = milliseconds(100);
	/*static milliseconds lastms = duration_cast<milliseconds>(
			system_clock::now().time_since_epoch()
			);*/
	/*milliseconds ms = duration_cast<milliseconds>(
			system_clock::now().time_since_epoch()
			);*/

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
        static int value = 0;
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

	digitalWrite(INA_BCM_PIN, A_enabled);
	digitalWrite(INB_BCM_PIN, B_enabled);

	bool drill_on = packet.channels[SD_ON_CHANNEL] > 1000;
	if (drill_on){
		pca.set_pwm(1, 0, 4095);
	} else {
		pca.set_pwm(1, 0, 0);
	}
	
	const int buf_size = 256;
	char readbuf[buf_size];
	int num_read = read(arduino_serial_fd, &readbuf, buf_size);

	if (num_read < 0){
		fprintf(stderr, "Arduino read error: %d\n", num_read);
	} else {
		arduino_stream_buf.append(readbuf, num_read);
		
		size_t index = arduino_stream_buf.find_last_of('\n');
		size_t second_to_last = arduino_stream_buf.find_last_of('\n', index-1); 
		
		if (index > 1 && second_to_last != std::string::npos){
			size_t tab_index_2 = arduino_stream_buf.find_last_of('\t', index-1);
			size_t tab_index_1 = arduino_stream_buf.find_last_of('\t', tab_index_2-1);
			
			std::string first_num_str = arduino_stream_buf.substr(second_to_last+1, tab_index_1 - second_to_last - 1);
			std::string second_num_str = arduino_stream_buf.substr(tab_index_1+1, tab_index_2 - tab_index_1 - 1);
			std::string third_num_str = arduino_stream_buf.substr(tab_index_2+1, index - tab_index_2 - 1);
			printf("First num: %s; second: %s; third: %s\n", first_num_str.c_str(), second_num_str.c_str(), third_num_str.c_str());
			
			int t = 0;
			int actuator_dist = 0;
			int curr_sense = 0;

			try {
				t = std::stoi(first_num_str);
				actuator_dist = std::stoi(second_num_str);
				curr_sense = std::stoi(third_num_str);

				printf("t: %d; D: %d; C: %d\n", t, actuator_dist, curr_sense);

				send_sensor_cmd(0x5900, actuator_dist);
				send_sensor_cmd(0x5958, curr_sense);

				log_data(t, actuator_dist, curr_sense);

				arduino_stream_buf.erase(0, index);
			} catch (...){
				fprintf(stderr, "Could not convert to integers: '%s', '%s'\n", first_num_str.c_str(), second_num_str.c_str());
			}
		}
		if (arduino_stream_buf.length() > buf_size) arduino_stream_buf.erase();
	}

}

int main(int argc, char* argv[])
{
	//use wpi pin numbers
	//wiringPiSetup();
	
	//use BCM pin numbers
	wiringPiSetupGpio();

	pinMode(ENA_BCM_PIN, OUTPUT);
	pinMode(ENB_BCM_PIN, OUTPUT);
	pinMode(INA_BCM_PIN, OUTPUT);
	pinMode(INB_BCM_PIN, OUTPUT);

	digitalWrite(ENA_BCM_PIN, 1);
	digitalWrite(ENB_BCM_PIN, 1);

	pca.set_pwm_freq(1500.0);
	pca.set_pwm(0, 0, 0);

	arduino_serial_init();

    
	std::time_t t = std::time(nullptr);
    	std::tm tm = *std::localtime(&t);

	std::string log_file_path = "Data/";
	std::string log_file_prefix = "test_";

	std::string log_file_suffix = "";
    
	char timestring_buffer[256];
	size_t chars_written = std::strftime(timestring_buffer, 256, "_%m_%d_%Y__%H_%M_%S", &tm);
	if (chars_written == 0){
		fprintf(stderr, "Datetime format failed\n");
		exit(-1);
	}
	log_file_suffix += timestring_buffer;
       	log_file_suffix += ".csv";
	
	std::string log_filename = log_file_path + log_file_prefix + log_file_suffix;

    
	//https://www.geeksforgeeks.org/getopt-function-in-c-to-parse-command-line-arguments/
	int opt;
    
    const int num_usb_ports = 4;
    const std::string port_base = "/dev/ttyUSB";
	std::string arduino_filepath = port_base + std::to_string(0);

	while ((opt = getopt(argc, argv, ":p:n:")) != -1){
		switch(opt){
		case 'n':
			log_filename = log_file_path + log_file_prefix + optarg + log_file_suffix;
			printf("Logging to %s\n", log_filename.c_str());
			break;
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

	log_file_init(log_filename);

    //this call passes the onPacket fxn pointer to the SBUS
    //library for sucessful packet callbacks
    sbus.onPacket(onPacket);

    sbus_err_t err = sbus.install(arduino_filepath.c_str(), true);
    
    if (err == SBUS_ERR_OPEN){
        //loop through /dev/ttyUSB<n> and attempt to find a working USB port
 
        printf("Could not open device at %s.\n"
               "Attempting to find device by searching devices that match %s<n> "
               "(searching indices from 0 to %d)...\n",
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

    std::signal(SIGINT, signalHandler);

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
