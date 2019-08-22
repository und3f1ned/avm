

#ifndef NAV_FUNCS_H
#define NAV_FUNCS_H

#include <arduino.h>
#include "motor_funcs.h"
#include "helpful_defines.h"

#define TWOPI 6.28315
#define PI 3.141593
#define HALFPI 1.570796

// For robot with 13.7 base, 64mm wheels, and encoders of 1920 pulses/revolution (120RPM motor model)
#define R_MTR_CLICKS_PER_CM 95.5 / 2 // pulses/cm
#define L_MTR_CLICKS_PER_CM 95.5 / 2 // pulses/cm
#define WHEEL_BASE 13.7

typedef struct loc_t
{
    float theta;
    float deg_theta;
    float x_pos;
    float y_pos;
    int encoder_interval_cnt_R_mtr; // motor 0
    int encoder_interval_cnt_L_mtr; // motor 1
} location;

typedef struct targ_t
{
    float init_target_distance;
    float target_distance;
    float heading_error;
    float deg_heading_error;
    float target_bearing;
    float deg_target_bearing;
    float x_target;
    float y_target;
} target;

extern location *ptr_loc;
extern target *ptr_targ;

// func prototypes -- available to other files
void calc_encoder_interval_cnts(location *ptr_location);
//int delta_target(target *ptr_target, float waypoint_x, float waypoint_y);
void odometer(location *ptr_location);
void locate_target(target *ptr_target, location *ptr_location);
int determine_rotate_in_place_flg(target *ptr_target);

#endif
