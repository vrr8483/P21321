#include <cstdio>
#include <ctime>
#include <chrono>
#include "SBUS.h"

using namespace std::chrono;

SBUS sbus;

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
