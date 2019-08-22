

#include "ultrasonic_funcs.h"


us_sensor us_R;
us_sensor *ptr_us_R = &us_R;

us_sensor us_L;
us_sensor *ptr_us_L = &us_L;

NewPing us[SONAR_NUM] = {     // Sensor object array.
  NewPing(36, A8, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(34, A9, MAX_DISTANCE)
};
