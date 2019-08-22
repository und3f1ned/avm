/************************************************************
*    note: 
*      enc => 3292.4 pulse/rev
*      wheel is 100mm => c = pi*dia = 31.4159cm
*      pulse/cm = 3292.4pulse/rev / 31.4259cm/rev = 104.8pulse/cm 
*      if period of measure is 10ms (i.e., 100 periods/sec)
*         speed in cm/sec = cm / 100 period * 104.8 pulse/cm = 1.048 pulse/period
*
************************************************************/

#include "motor_funcs.h"

Encoder encRight(19, 18);
Encoder encLeft(3, 2);

int dirPin[2] = {4, 7};
int pwmpin[2] = {5, 6}; // PWM input

// PID setup
double setpoint[2] = {130, 130};
double pid_input[2];
double pid_output[2];

double kp[2] = {0.6, 0.6};
double ki[2] = {5, 5};
double kd[2] = {0, 0};

PID right_mtr_pid(&pid_input[0], &pid_output[0], &setpoint[0], kp[0], ki[0], kd[0], DIRECT);
PID left_mtr_pid(&pid_input[1], &pid_output[1], &setpoint[1], kp[1], ki[1], kd[1], DIRECT);

//---------- Functions -------------------------------
void init_pids()
{
  right_mtr_pid.SetSampleTime(10);
  right_mtr_pid.SetMode(AUTOMATIC);
  //right_mtr_pid.SetOutputLimits(20,240);   //min, max
  left_mtr_pid.SetSampleTime(10);
  left_mtr_pid.SetMode(AUTOMATIC);
  //left_mtr_pid.SetOutputLimits(20,240);   //min, max
}

void init_motor_driver_shield()
{
  // motor shield pin setup
  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(dirPin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
}

void motorOff(int motor)
{
  analogWrite(pwmpin[motor], 0);
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled
 
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 
 pwm: should be a value between 0 and 255, higher the number, the faster
 it'll go
 */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <= 4)
    {
      if (direct == 1)
        digitalWrite(dirPin[motor], HIGH);

      if ((direct == 0) || (direct == 2))
        digitalWrite(dirPin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}

/* --------------------------------------------------------------- */
/* motor command utilities - taken from work by D. Anderson        */
/* --------------------------------------------------------------- */
/* clip int value to min and max */

int clip(int val, int min, int max)
{
  if (val > max)
    return max;
  if (val < min)
    return min;
  return val;
}

/* clip float value to min and max */

int clip_f(float val, float min, float max)
{
  if (val > max)
    return max;
  if (val < min)
    return min;
  return val;
}

/* --------------------------------------------------------------- */
/* slew rate generator */
/* --------------------------------------------------------------- */

int slew(int from, int to, int rate)
{
  int x;

  if (to > from)
  {
    x = from + rate;
    if (x <= to)
      return (x);
  }
  else if (to < from)
  {
    x = from - rate;
    if (x >= to)
      return (x);
  }
  return to;
}
