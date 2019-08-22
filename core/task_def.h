
#ifndef TASK_DEF_H
#define TASK_DEF_H

#include <arduino.h>
#include <task.h>
#include <log.h>
#include <sysclock.h>

#include "motor_funcs.h"
#include "wpt_funcs.h"
#include "nav_funcs.h"
#include "ultrasonic_funcs.h"
#include "bumper_funcs.h"
#include "head_funcs.h"
#include "helpful_defines.h"
#include "init.h"

#define VERSION "AVM-0.1.3"
#define WAIT(d)        \
    {                  \
        wake_after(d); \
    }

#if (MACHINE == MACH_AVR) /* Mega2560, Mega328 Teensy-LC */
#define PRINTF Serial.println
#define SPRINTF sprintf
#endif

// function prototypes

void cpu_idle(ASIZE ignored);
void head(ASIZE delay);
void ultrasonic_task(ASIZE delay);
void sensors(ASIZE delay);
void move(ASIZE delay);
void stats_task(ASIZE delay);
void signon(ASIZE version);

#endif
