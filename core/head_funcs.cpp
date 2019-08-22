

#include "head_funcs.h"
#include "helpful_defines.h"

// objects
SoftwareServo head_servo_h;
SoftwareServo head_servo_v;

// Global Variables
// servo position can be between 0 and 180, with 90 being neutral
uint8_t head_default_pos = 90;

uint8_t head_right_pos = 105;
uint8_t head_left_pos = 45;

uint8_t head_top_pos = 75;
uint8_t head_bottom_pos = 0;

uint8_t head_centered = 0;

// functions
void init_head()
{
    head_servo_h.attach(A0);
    head_servo_v.attach(A1);
    center_head();
}

void center_head()
{
    head_servo_h.write(75);
    head_servo_v.write(70);
    head_centered = 1;
}

void move_head(uint8_t servo, uint8_t pos)
{   
  if (pos >= 60 && pos <= 120) {
    if (servo == 0) {
      head_servo_h.write(pos);
    }
    if (servo == 1) {
      head_servo_v.write(pos);
    }
    if (servo == 2) {
      head_servo_h.write(pos);
      head_servo_v.write(pos);
    }
  }
}
