//used for catching program interrupt signals
#include <csignal>

//used for log file writing
#include <fstream>

//getopt, read, write, and close
#include <unistd.h>

//localtime, time, strftime
#include <iomanip>

//file open
#include <fcntl.h>

//serial port interface capabilities
#include <termios.h>

//GPIO manipulation (digitalWrite)
#include <wiringPi.h>

//SBUS protocol
//https://github.com/Carbon225/raspberry-sbus
#include "SBUS.h"

//I2C PWM hat controls
//https://github.com/barulicm/PiPCA9685
#include "PCA9685.h"

//uncomment if using timing stuff
//#include <chrono>
//using namespace std::chrono;

#define ACTUATOR_DIST_SENSOR_ID (0x5900)
#define CURR_SENSE_SENSOR_ID (0x5958)

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
#define SC_UP_CHANNEL (7)
#define SC_DOWN_CHANNEL (8)

//Switches:
//SA: leftmost switch on face of controller (three state stable)
//SB: just to the right of SA (three state stable)
//SC: rightmost switch on face of controller (three state stable)
//SD: left switch on top of controller, sticking out like an antannae. (two state stable)
//SE: right switch on top of controller, sticking out like an antannae.
//	(two state springloaded, normally pointing down
//	but this is registered as "up" in the firmware of the controller)

//sensor commands are the packets that the raspberry pi
//sends to the arduino to tell it to chenge its internally stored sensor value
//that the receiver polls for. the headers and footers were chosen for
//their ability to be converted to ASCII. Long story.
#define SENSOR_CMD_HEADER (0x53)
#define SENSOR_CMD_FOOTER (0x45)

//The Raspberry pi has two pin addressing modes: Wiring pi (WPI), and Broadcom (BCM).
//BCM is usually better to use.
#define DRILL_ACT_INA_WPI_PIN (3)
#define DRILL_ACT_ENA_WPI_PIN (4)
#define DRILL_ACT_ENB_WPI_PIN (5)
#define DRILL_ACT_INB_WPI_PIN (6)

//EN A and EN B are the H bridge enables for path A and B (active high)
//IN A and IN B are turned on one at a time to indicate the direction of the motor

#define DRILL_ACT_INA_BCM_PIN (22)
#define DRILL_ACT_ENA_BCM_PIN (23)
#define DRILL_ACT_ENB_BCM_PIN (24)
#define DRILL_ACT_INB_BCM_PIN (25)

#define LEFT_DRIVE_INA_BCM_PIN (4)
#define LEFT_DRIVE_ENA_BCM_PIN (17)
#define LEFT_DRIVE_ENB_BCM_PIN (18)
#define LEFT_DRIVE_INB_BCM_PIN (27)

#define RIGHT_DRIVE_INA_BCM_PIN (13)
#define RIGHT_DRIVE_ENA_BCM_PIN (12)
#define RIGHT_DRIVE_ENB_BCM_PIN (6)
#define RIGHT_DRIVE_INB_BCM_PIN (5)

enum PWM_pins_enum {
	PWM_CHANNEL_ACTUATOR = 0,
	PWM_CHANNEL_DRILL,
	PWM_CHANNEL_LEFT_DRIVE,
	PWM_CHANNEL_RIGHT_DRIVE,
	
	PWM_CHANNEL_4,
	PWM_CHANNEL_5,
	PWM_CHANNEL_6,
	PWM_CHANNEL_7,
	
	PWM_CHANNEL_8,
	PWM_CHANNEL_9,
	PWM_CHANNEL_10,
	PWM_CHANNEL_11,
	
	PWM_CHANNEL_12,
	PWM_CHANNEL_13,
	PWM_CHANNEL_14,
	PWM_CHANNEL_15,
	
	num_PWM_channels
};

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

//This is the object that represents our interface to the SBUS protocol.
//SBUS is used for the FrSky controller to pass controls data from the controller
//to the motor controls systems, normally.
//In this system, the raspberry pi reads from this signal directly
//and decodes it using the SBUS library.
SBUS sbus;

//The PCA object allows us the I2C command interface required to control
//the PWM signals to the multiple motor drivers
PCA9685* pca;

//These two are used for interfacing with the Arduino serial stream,
//which reads the actuator force sensor and distance potentiometer
//and sends it to the raspberry pi, who then sends it off to the FrSky receiver
//so that the transmitter can display those values to the user
//TODO: switch this to a file stream and add CSV file input simulation (Vic)
int arduino_serial_fd;
std::string arduino_stream_buf = "";

std::fstream log_file;

//function declaration for the cleanup function so we can call it from anywhere
void cleanup();

//Initializes the Arduino serial stream on arduino_serial_fd
//on error, it aborts the program.
void arduino_serial_init(){

	//https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/#overview
	//This will be the only arduino plugged into the Pi,
	//which is an assumption, but an accurate one.
	
	arduino_serial_fd = open("/dev/ttyACM0", O_RDWR);
	if (arduino_serial_fd < 0){
		fprintf(stderr, "Arduino serial port open error %i\n", errno);
		cleanup();
		exit(errno);
	}

	termios tty;
	if(tcgetattr(arduino_serial_fd, &tty) != 0) {
		fprintf(stderr, "Error %i from tcgetattr\n", errno);
		cleanup();
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
		cleanup();
		exit(errno);
	}
	
	//delete contents of input buffer to avoid reading old data
	usleep(10000); //sleep for 10ms cause apparently this wont work otherwise? Linus pls fix
	tcflush(arduino_serial_fd, TCIFLUSH);
}

//initializes the log file stream at filename.
//aborts the program on error
void log_file_init(std::string filename){

	log_file.open(filename, std::ios::out);
	if (!log_file){
		fprintf(stderr, "Cannot open log file for writing at %s\n", filename.c_str());
		cleanup();
		exit(-1);
	}
}

//logs a timestamped pair of distance and current sense values to the logfile
void log_data(int t, int dist, int currsense){
	
	//need std::chrono for this
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

//this function is called whenever the program is about to exit.
//It turns off the PWM signals and cleans up the PCA object,
//closes the arduino serial port and log file.
//It also disables all the motor drivers.
//This function should be kept up to date with any hardware additions to
//ensure that if the program isnt running, the motors don't draw any current from the battery.
void cleanup(){
	
	pca->set_all_pwm(0, 0);
	delete pca;

	digitalWrite(DRILL_ACT_ENA_BCM_PIN, 0);
	digitalWrite(DRILL_ACT_ENB_BCM_PIN, 0);
	digitalWrite(DRILL_ACT_INA_BCM_PIN, 0);
	digitalWrite(DRILL_ACT_INB_BCM_PIN, 0);
	
	digitalWrite(LEFT_DRIVE_ENA_BCM_PIN, 0);
	digitalWrite(LEFT_DRIVE_ENB_BCM_PIN, 0);
	digitalWrite(LEFT_DRIVE_INA_BCM_PIN, 0);
	digitalWrite(LEFT_DRIVE_INB_BCM_PIN, 0);
	
	digitalWrite(RIGHT_DRIVE_ENA_BCM_PIN, 0);
	digitalWrite(RIGHT_DRIVE_ENB_BCM_PIN, 0);
	digitalWrite(RIGHT_DRIVE_INA_BCM_PIN, 0);
	digitalWrite(RIGHT_DRIVE_INB_BCM_PIN, 0);
	
	close(arduino_serial_fd);

	log_file.close();
}

//called when SIGINT is received (Ctrl+C)
void exitFxn(int signum) {
 
	cleanup();

	exit(signum);
}

//updates a sensor value on the Arduino with ID id to new_val
int send_sensor_cmd(uint16_t id, uint32_t new_val){
	
	//send command to sensor
	sensor_cmd_type cmd;
	cmd.sensor_id = id;
	cmd.value = new_val;
	
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

//Called by the SBUS library whenever a valid SBUS packet is received.
//This uses the received controller values to write motor driver enable
//values (to control the direction of the motors)
//as well as PWM signal duty cycle to control motor speed.
//It also logs actuator data to a CSV file
void onPacket(sbus_packet_t packet){

	//printf("Callback called\n");
	//printf("now: %ld, lastPrint: %ld\n", now, lastPrint);
	
	//need std::chrono for this
	//milliseconds period = milliseconds(100);
	/*static milliseconds lastms = duration_cast<milliseconds>(
			system_clock::now().time_since_epoch()
			);*/
	/*milliseconds ms = duration_cast<milliseconds>(
			system_clock::now().time_since_epoch()
			);*/

	/*if (ms - lastms > period){
		lastms = ms;
		lastPrint = now;
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
	
	//-------------------------------------------------------------------
	//constants
	
	//max and min PWM % (out of 100) for the actuator to take on
	int max_act_PWM = 25;
	int min_act_PWM = 0;
	
	//max and min PWM % (out of 100) for the wheels to take on
	int max_wheel_PWM = 50;
	int min_wheel_PWM = 0;
	
	//max and min throttle values that the controller will give us.
	//The min has been set to the middle value since this is the resting position of the throttle.
	int max_throttle = 1811;
	int nominal_throttle = 992;
	
	//The PCA9685 uses this value as a maxinum clock count per PWM pulse: 
	//setting 'on' to 0 (the start of the pulse time) 
	//and 'off' to this value 
	//will result in a 100% duty cycle.
	int max_PCA_val = 4095;
	
	//greater than this, and the switch is active (condition satisfied)
	//NOTE: on the controller, for channels meant to be active when a switch is DOWN,
	//you have to modify the scale to -100 (totally inverted) to make it work with this program.
	int FrSky_switch_threshold = 1000;
	
	//-------------------------------------------------------------------
	//actuator PWM
	
	//range: -max to +max PWM for actuator
	int act_PWM = (int)(
		((max_act_PWM - min_act_PWM)*1.0/(max_throttle - nominal_throttle))*
		(packet.channels[RIGHT_THROTTLE_CHANNEL] - nominal_throttle)
		);
	
	if (act_PWM < 0) act_PWM = 0;
	
	//adjust for PCA
	int PCA_PWM = (int)((max_PCA_val*1.0/100)*act_PWM);
	
	pca->set_pwm(PWM_CHANNEL_ACTUATOR, 0, PCA_PWM);
	
	//-------------------------------------------------------------------
	//wheels PWM
	
	int wheel_PWM = (int)(
		((max_wheel_PWM - min_wheel_PWM)*1.0/(max_throttle - nominal_throttle))*
		(packet.channels[LEFT_THROTTLE_CHANNEL] - nominal_throttle)
	);
	
	if (wheel_PWM < 0) wheel_PWM = 0;
	
	PCA_PWM = (int)((max_PCA_val*1.0/100)*wheel_PWM);
	
	pca->set_pwm(PWM_CHANNEL_LEFT_DRIVE, 0, PCA_PWM);
	pca->set_pwm(PWM_CHANNEL_RIGHT_DRIVE, 0, PCA_PWM);

	//-------------------------------------------------------------------
	//enable and direction pins

	//wheels
	bool A_enabled = packet.channels[SA_DOWN_CHANNEL] > FrSky_switch_threshold;
	bool B_enabled = packet.channels[SA_UP_CHANNEL] > FrSky_switch_threshold;

	digitalWrite(LEFT_DRIVE_INA_BCM_PIN, A_enabled);
	digitalWrite(LEFT_DRIVE_INB_BCM_PIN, B_enabled);
	
	//switched! cause they're on opposite sides
	digitalWrite(RIGHT_DRIVE_INB_BCM_PIN, A_enabled);
	digitalWrite(RIGHT_DRIVE_INA_BCM_PIN, B_enabled);
	
	
	//actuator/drill
	A_enabled = packet.channels[SC_DOWN_CHANNEL] > FrSky_switch_threshold;
	B_enabled = packet.channels[SC_UP_CHANNEL] > FrSky_switch_threshold;

	digitalWrite(DRILL_ACT_INA_BCM_PIN, A_enabled);
	digitalWrite(DRILL_ACT_INB_BCM_PIN, B_enabled);

	//-------------------------------------------------------------------
	//drill power
	
	bool drill_on = packet.channels[SD_ON_CHANNEL] > FrSky_switch_threshold;
	if (drill_on){
		pca->set_pwm(PWM_CHANNEL_DRILL, 0, max_PCA_val);
	} else {
		pca->set_pwm(PWM_CHANNEL_DRILL, 0, 0);
	}
	
	//here begins the code for receiving, logging, and sending commands due to:
	//the analog read pins from the Arduino (curr sense and actuator dist)
	//the format for the data coming in is:
	//<t>\t<dist>\t<currsense>\n
	//where /t is a tab character, \n is a carriage return,
	//and <t>, <dist>, and <currsense> are the string representations
	//of the corresponding values.
	const int buf_size = 256;
	char readbuf[buf_size];
	int num_read = read(arduino_serial_fd, &readbuf, buf_size);

	//TODO: add data analysis
	if (num_read < 0){
		fprintf(stderr, "Arduino read error: %d\n", num_read);
	} else {
		//append to global read buffer in case packets got split up
		arduino_stream_buf.append(readbuf, num_read);
		
		//only set after all three values are successfully converted, guaranteed to have valid and related values
		int valid_t = 0;
		int valid_actuator_dist = 0;
		int valid_curr_sense = 0;
		bool valid_values_found = false;
		
		while(true){
			//index of the very LAST carriage return
			//size_t index = arduino_stream_buf.find_last_of('\n');
			//size_t second_to_last = arduino_stream_buf.find_last_of('\n', index-1);
			
			//index of first and second carriage returns
			size_t first_cr_indx = arduino_stream_buf.find_first_of('\n');
			size_t second_cr_indx = arduino_stream_buf.find_last_of('\n', index+1);
			
			//if both carriage returns were found
			if (index != std::string::npos && second_to_last != std::string::npos){

				size_t tab_index_2 = arduino_stream_buf.find_last_of('\t', second_cr_indx-1);
				size_t tab_index_1 = arduino_stream_buf.find_last_of('\t', tab_index_2-1);
				
				//get numbers as strings first
				std::string first_num_str =
					arduino_stream_buf.substr(
							first_cr_indx + 1,
							tab_index_1 - first_cr_indx - 1
							);

				std::string second_num_str =
					arduino_stream_buf.substr(
							tab_index_1 + 1,
							tab_index_2 - tab_index_1 - 1
							);
				
				std::string third_num_str =
					arduino_stream_buf.substr(
							tab_index_2 + 1,
							second_cr_indx - tab_index_2 - 1
							);
				//printf("First num: %s; second: %s; third: %s\n",
				//first_num_str.c_str(), second_num_str.c_str(), third_num_str.c_str());
				
				int t = 0;
				int actuator_dist = 0;
				int curr_sense = 0;

				try {
					//attempt to convert to ints. throws an exception if stoi fails
					t = std::stoi(first_num_str);
					actuator_dist = std::stoi(second_num_str);
					curr_sense = std::stoi(third_num_str);
					
					valid_t = t;
					valid_actuator_dist = actuator_dist;
					valid_curr_sense = curr_sense;
					valid_values_found = true;

					//printf("t: %d; D: %d; C: %d\n", t, actuator_dist, curr_sense);
					
					//write to log file
					log_data(t, actuator_dist, curr_sense);
					
					//erase anything before and including this packet
					arduino_stream_buf.erase(0, first_cr_indx-1);
				} catch (...){
					fprintf(stderr,
						"Could not convert to integers: '%s', '%s'\n",
						first_num_str.c_str(), second_num_str.c_str());
				}
			} else {
				//CRs not found, no valid data in stream right now
				break;
			} //end stream triplet validity check (if/else)
		} //end while loop iterating through arduino stream
		
		if (valid_values_found){
			send_sensor_cmd(ACTUATOR_DIST_SENSOR_ID, valid_actuator_dist);
			send_sensor_cmd(CURR_SENSE_SENSOR_ID, valid_curr_sense);
		}
		
		//garbage collection of global buffer
		if (arduino_stream_buf.length() > buf_size) arduino_stream_buf.erase();
	}

}

int main(int argc, char* argv[]){

	//initialize the GPIO (general purpose in-out) pins for use
	
	//use wpi pin numbers
	//wiringPiSetup();
	
	//use BCM pin numbers
	wiringPiSetupGpio();

	//TODO: Monitor EN pins and handle faults
	//sets all the enable and direction pins to OUTPUT
	pinMode(DRILL_ACT_ENA_BCM_PIN, OUTPUT);
	pinMode(DRILL_ACT_ENB_BCM_PIN, OUTPUT);
	pinMode(DRILL_ACT_INA_BCM_PIN, OUTPUT);
	pinMode(DRILL_ACT_INB_BCM_PIN, OUTPUT);
	
	pinMode(LEFT_DRIVE_ENA_BCM_PIN, OUTPUT);
	pinMode(LEFT_DRIVE_ENB_BCM_PIN, OUTPUT);
	pinMode(LEFT_DRIVE_INA_BCM_PIN, OUTPUT);
	pinMode(LEFT_DRIVE_INB_BCM_PIN, OUTPUT);
	
	pinMode(RIGHT_DRIVE_ENA_BCM_PIN, OUTPUT);
	pinMode(RIGHT_DRIVE_ENB_BCM_PIN, OUTPUT);
	pinMode(RIGHT_DRIVE_INA_BCM_PIN, OUTPUT);
	pinMode(RIGHT_DRIVE_INB_BCM_PIN, OUTPUT);

	//turn on all the enable pins on the motor drivers
	digitalWrite(DRILL_ACT_ENA_BCM_PIN, 1);
	digitalWrite(DRILL_ACT_ENB_BCM_PIN, 1);
	
	digitalWrite(LEFT_DRIVE_ENA_BCM_PIN, 1);
	digitalWrite(LEFT_DRIVE_ENB_BCM_PIN, 1);
	
	digitalWrite(RIGHT_DRIVE_ENA_BCM_PIN, 1);
	digitalWrite(RIGHT_DRIVE_ENB_BCM_PIN, 1);

	//Initializes the PCA object. On failure, the constructor throws an exception.
	//So this statement will catch that exception and abort the program on such an event
	//Since if it fails, its due to a hardware misconfiguration.
	try {
		pca = new PCA9685{};
	} catch (...) {
		fprintf(stderr, "PCA initialization error.\n");
		cleanup();
		exit(-1);
	}

	//1500 Hz is close to the max frequency of the PCA9685
	pca->set_pwm_freq(1500.0);
	//turn off PWM
	pca->set_all_pwm(0, 0);

	arduino_serial_init();

	//get the current datetime as a string and use it to make a new log file
	std::time_t t = std::time(nullptr);
	std::tm tm = *std::localtime(&t);

	std::string log_file_path = "Data/";
	std::string log_file_prefix = "test_";

	std::string log_file_suffix = "";
	
	const int buf_size = 256;
	char timestring_buffer[buf_size];
	size_t chars_written = std::strftime(timestring_buffer, buf_size, "_%m_%d_%Y__%H_%M_%S", &tm);
	if (chars_written == 0){
		fprintf(stderr, "Datetime format failed\n");
		cleanup();
		exit(-1);
	}
	log_file_suffix += timestring_buffer;
	log_file_suffix += ".csv";
	
	std::string log_filename = log_file_path + log_file_prefix + log_file_suffix;

	//getopt uses the command line arguments to set variables in the program
	//https://www.geeksforgeeks.org/getopt-function-in-c-to-parse-command-line-arguments/
	int opt;
	
	const int num_usb_ports = 4;
	const std::string port_base = "/dev/ttyUSB";
	std::string arduino_filepath = port_base + std::to_string(0);

	while ((opt = getopt(argc, argv, ":p:n:")) != -1){
		switch(opt){
		case 'n':
			//-n <name> adds a custom name to the log file
			log_filename = log_file_path + log_file_prefix + optarg + log_file_suffix;
			printf("Logging to %s\n", log_filename.c_str());
			break;
		case 'p':
			//-p <port> makes this program use <port> for the arduino serial stream
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
	//library for successful packet callbacks
	sbus.onPacket(onPacket);

	//Attempt to 'install' the SBUS object, which just consists of opening a serial
	//port to the FrSky receiver. It returns SBUS_OK on success.
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
		cleanup();
		return err;
	}

	//call this function when SIGINT is received (Ctrl+C, kind of a force quit)
	std::signal(SIGINT, exitFxn);
	
	//ALSO call this function when SIGTERM is received (system shutdown)
	std::signal(SIGTERM, exitFxn);

	while ((err = sbus.read()) != SBUS_FAIL) {
		if (err == SBUS_ERR_DESYNC) {
			fprintf(stderr, "SBUS desync\n\r");
		}
	}
	
	fprintf(stderr, "SBUS error: %d\n\r", err);
	
	cleanup();
	
	return err;
}

