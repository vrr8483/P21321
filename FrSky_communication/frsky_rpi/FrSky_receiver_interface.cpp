#include <cstdio>
#include <ctime>
#include <chrono>

#include <unistd.h>
#include <fcntl.h>
#include <asm/termbits.h>
#include <asm/ioctls.h>
#include <sys/ioctl.h>

#include "SBUS.h"

using namespace std::chrono;

#define SENSOR_CMD_HEADER (0x53)
#define SENSOR_CMD_FOOTER (0x45)

struct sensor_cmd_type {
  uint8_t header = SENSOR_CMD_HEADER;
  uint16_t sensor_id;
  uint32_t value;
  uint8_t footer = SENSOR_CMD_FOOTER;

  bool verify_cmd(){
    return header == SENSOR_CMD_HEADER && footer == SENSOR_CMD_FOOTER;
  }
}__attribute__((aligned(1)));

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

void onPacket(sbus_packet_t packet)
{
	//printf("Callback called\n");
    //static time_t lastPrint = time(nullptr);
    //time_t now = time(nullptr);
    //printf("now: %ld, lastPrint: %ld\n", now, lastPrint);
	milliseconds period = milliseconds(100);
	static milliseconds lastms =  duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

	if (ms - lastms > period){
    //if (now > lastPrint)
    //{
    	lastms = ms;
        //lastPrint = now;
        printf("ch1: %u\tch2: %u\tch3: %u\tch4: %u\t"
               "ch5: %u\tch6: %u\tch7: %u\tch8: %u\t"
               "ch9: %u\tch10: %u\tch11: %u\tch12: %u\t"
               "ch13: %u\tch14: %u\tch15: %u\tch16: %u\tch17: %u\tch18: %u%s%s\n\r",
               packet.channels[0], packet.channels[1], packet.channels[2], packet.channels[3],
               packet.channels[4], packet.channels[5], packet.channels[6], packet.channels[7],
               packet.channels[8], packet.channels[9], packet.channels[10], packet.channels[11],
               packet.channels[12], packet.channels[13], packet.channels[14], packet.channels[15],
               packet.ch17, packet.ch18,
               packet.frameLost ? "\tFrame lost" : "",
               packet.failsafe ? "\tFailsafe active" : "");
    	}
    
    //send command to sensor
    sensor_cmd_type cmd;
    cmd.sensor_id = 0x5958;
    cmd.value = value++;

    uint8_t packet_bytes[8];
    packet_bytes[0] = cmd.header;
    packet_bytes[1] = cmd.sensor_id >> 8;   
    packet_bytes[2] = cmd.sensor_id & 0xFF;   
    packet_bytes[3] = (cmd.value >> 24) & 0xFF;
    packet_bytes[4] = (cmd.value >> 16) & 0xFF;
    packet_bytes[5] = (cmd.value >> 8) & 0xFF;
    packet_bytes[6] = cmd.value & 0xFF;
    packet_bytes[7] = cmd.footer;
    
    sensor_cmd_packet_type sensor_packet;
    sensor_packet.cmd = cmd;
    for (uint8_t i = 0; i < 8; ++i){
	printf("%d: %x\t", i, sensor_packet.bytes[i]);
    }
    printf("\n");
    printf("Header: %x\t", sensor_packet.cmd.header);
    printf("ID: %x\t", sensor_packet.cmd.sensor_id);
    printf("Val: %x\t", sensor_packet.cmd.value);
    printf("Footer: %x\t", sensor_packet.cmd.footer);
    printf("\n");
    
    if (write(sbus._fd, packet_bytes, sizeof(sensor_cmd_packet_type)) != sizeof(sensor_cmd_packet_type)){
        printf("Failed to send sensor command.\n");
    }
}

int main()
{
    printf("SBUS blocking receiver example\n\r");

    sbus.onPacket(onPacket);

    sbus_err_t err = sbus.install("/dev/ttyUSB0", true);
    if (err != SBUS_OK)
    {
        fprintf(stderr, "SBUS install error: %d\n\r", err);
        return err;
    }

    while ((err = sbus.read()) != SBUS_FAIL)
    {
        if (err == SBUS_ERR_DESYNC)
        {
        	//fprintf(stderr, "SBUS desync\n\r");
        }
    }

    fprintf(stderr, "SBUS error: %d\n\r", err);

    return err;
}
