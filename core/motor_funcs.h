
#ifndef MOTOR_FUNCS_H
#define MOTOR_FUNCS_H

#include <arduino.h>
#include <Encoder.h>
#include <PID_v1.h>

// Motor shield setup
#define BRAKEVCC 0
#define CW 1
#define CCW 2
#define BRAKEGND 3

//externs
extern int dirPin[2];

extern int pwmpin[2]; // PWM input

extern Encoder encRight; // Mega INT4 and INT5
extern Encoder encLeft;  // Mega INT0 and INT1

extern double r_setpoint, r_pid_input, r_pid_output;
extern double l_setpoint, l_pid_input, l_pid_output;
extern double setpoint[];
extern double pid_input[];
extern double pid_output[];

extern double kp[];
extern double ki[];
extern double kd[];

extern PID right_mtr_pid;
extern PID left_mtr_pid;

// function prototypes
void init_pids();
void init_motor_driver_shield();
void motorOff(int motor);
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm);
int clip(int val, int min, int max);
int clip_f(float val, float min, float max);
int slew(int from, int to, int rate);

#endif
