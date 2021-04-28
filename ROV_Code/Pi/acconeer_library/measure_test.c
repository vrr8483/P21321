#include <stdio.h>
#include "measure.h"

int main(){
	
	if (setup_radar()){
		fprintf("Setup failed.\n");
		return -1;
	}
	
	double result = measure();
	printf("Result: %f\n", result);
	
	if (cleanup_radar()){
		fprintf("Cleanup failed.\n");
		return -1;
	}
	return 0;
}
