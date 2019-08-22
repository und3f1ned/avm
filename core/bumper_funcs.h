
#ifndef BUMPER_FUNCS_H
#define BUMPER_FUNCS_H

#include <arduino.h>

// externs
extern int bump_R;
extern int bump_L;

// func prototypes
void init_bumper();
uint8_t test_bumpers();

#endif
