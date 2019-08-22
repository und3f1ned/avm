
#ifndef USONIC_FUNCS_H
#define USONIC_FUNCS_H
#include <NewPing.h>
#define SONAR_NUM     2 // Number of sensors.
#define MAX_DISTANCE 400 // Maximum distance (in cm) to ping.

typedef struct us_sens_t
{
    float dist;
} us_sensor;

//externs
extern us_sensor *ptr_us_R;
extern us_sensor *ptr_us_L;

extern NewPing us[SONAR_NUM];


#endif
