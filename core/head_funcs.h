
#ifndef HEAD_FUNCS_H
#define HEAD_FUNCS_H

#include <arduino.h>
#include <SoftwareServo.h>

// externs
extern SoftwareServo claw_servo;
extern SoftwareServo head_servo_h;
extern SoftwareServo head_servo_v;
extern uint8_t head_centered;

//function prototypes
void init_head();
void center_head();
void move_head(uint8_t servo, uint8_t pos);

#endif
