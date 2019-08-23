#include <Wire.h>
#include <YetAnotherPcInt.h>
#include <SoftwareServo.h>
#include <PID_v1.h>
#include <stdio.h>
#include <Encoder.h>

#include <task.h>
#include "task_def.h"
#include "init.h"
#include "motor_funcs.h"
#include "ultrasonic_funcs.h"
#include "bumper_funcs.h"
#include "head_funcs.h"
#include "wpt_funcs.h"

LAYER *layers[LAYERS] =
    {&bump, &motion, &ir, &boundary, &sonar, &photo, &xlate, &prowl, &stop};

void printkbuf(char *s)
{
#if ((MACHINE == MACH_AVR) || (MACHINE == MACH_ARM))
  // Serial.println(s);
  PRINTF(s);
#endif
}

#define LED1_ON digitalWrite(13, HIGH)
#define LED1_OFF digitalWrite(13, LOW)

// Define Slave I2C Address
#define SLAVE_ADDR 9

// Define Slave answer size
#define ANSWERSIZE 5

String answer = "Hello";
int commands[3];

int pir_R = 10;
int pir_L = 12;

/* -------------------------------------- */

void receiveEvent(int count)
{
  // Read while data received
  while (0 < Wire.available())
  {
    for (count; count > 0; count--)
    {
      commands[count - 1] = Wire.read();
      Serial3.print(commands[count - 1]);
      if (count != 1)
      {
        Serial3.print(",");
      }
      else
      {
        Serial3.println();
      }
    }
  }
}

/* -------------------------------------- */

void requestEvent()
{
  // Setup byte variable in the correct size
  byte response[ANSWERSIZE];

  // Format answer as array
  for (byte i = 0; i < ANSWERSIZE; i++)
  {
    response[i] = (byte)answer.charAt(i);
  }

  // Send response back to Master
  Wire.write(response, sizeof(response));

  // Print to Serial Monitor
  Serial3.println("Request event");
}

/* -------------------------------------- */

void led1(ASIZE delay)
{
  unsigned long cnt;

  while (1)
  {
    WAIT(delay);
    LED1_ON;
    WAIT(delay);
    LED1_OFF;
  }
}

void init_pir()
{
  // setup pins
  pinMode(pir_R, INPUT_PULLUP);
  pinMode(pir_L, INPUT_PULLUP);

  PcInt::attachInterrupt(pir_R, pir_change, "1", CHANGE);
  PcInt::attachInterrupt(pir_L, pir_change, "2", CHANGE);
}

void pir_change(const char *message, bool pinstate)
{
  LAYER motion;
  motion.flag = pinstate;

  if (pinstate == 0)
  {
    motion.arg = 0;
  }
  else
  {
    motion.arg = message;
    Serial3.print(1);
    Serial3.print(",");
    Serial3.print(0);
    Serial3.print(",");
    Serial3.print(message);
    Serial3.println();
  }
}

/* -------------------------------------- */

// system_init
void system_init(void)
{
#if ((MACHINE == MACH_AVR) || (MACHINE == MACH_ARM))
  /* AVR & ARM Teesy3.1  */
  init_motor_driver_shield();
  sysclock_init();
  init_bumper();
  init_head();
  init_pir();
  init_temp_waypoint(ptr_temp_wpt);
  pinMode(13, OUTPUT);
  Serial.begin(57600);
  Serial3.begin(57600);
  // Initialize I2C communications as Slave
  Wire.begin(SLAVE_ADDR);

  // Function to run when data requested from master
  Wire.onRequest(requestEvent);

  // Function to run when data received from master
  Wire.onReceive(receiveEvent);
#endif
}

/* ----------------------------------------- */
/* main */

#if ((MACHINE == MACH_AVR) || (MACHINE == MACH_ARM)) /* ARM is Teensy3.1 */
/* this is for the Arduino IDE "sketch" set up */
void setup()
#else
int main()
#endif
{
  system_init();
  printv = printkbuf;

  PRINTF("Howdy Console!\n");

  delay(1500); /* hardware delay, sysclock not running yet */

  pid_count = 0;
  current = 0;

  create_task("LED1", led1, 500, MINSTACK);
  create_task("US", ultrasonic_task, 33, MINSTACK * 2);
  create_task("SENSORS", sensors, 20, MINSTACK * 2); //40
  create_task("HEAD", head, 45, MINSTACK);           //45
  create_task("MOVE", move, 10, MINSTACK * 2);       //25 //20
  create_task("IDLE", cpu_idle, 0, MINSTACK);
  //create_task("STATS",stats_task,10000,MINSTACK*4);
  create_task("SIGNON", signon, 1, MINSTACK * 4);

  scheduler();
  PRINTF("Should never get here.");

  while (1)
    ;
#if ((MACHINE != MACH_AVR) && (MACHINE != MACH_ARM))
  return 0;
#endif
}

void loop()
{
  /* nothing to see here, move along */
  asm("nop");
}

/* ----------------------------------------- */
/* EOF */
