#include <stdio.h>
#include "measure.h"
#include "Timers.h"

int main(){
	
	if (setup_radar()){
		fprintf(stderr, "Setup failed.\n");
		return -1;
	}

	int iterations = 100;

	DECLARE_REPEAT_VAR(rv)
	DECLARE_TIMER(timer)

	START_TIMER(timer)

	BEGIN_REPEAT_TIMING(iterations, rv)
	
	double result = measure();
	printf("Result: %f\n", result);
	
	END_REPEAT_TIMING
	
	STOP_TIMER(timer)

	PRINT_RTIMER(timer, iterations)
	
	if (cleanup_radar()){
		fprintf(stderr, "Cleanup failed.\n");
		return -1;
	}
	return 0;
}
