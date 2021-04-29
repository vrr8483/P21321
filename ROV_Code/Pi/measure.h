#ifndef MEASURE_H
#define MEASURE_H

#ifdef __cplusplus
extern "C" {
#endif

int setup_radar();
double measure();
int cleanup_radar();

#ifdef __cplusplus
}
#endif

#endif
