

#include "task_def.h"

// global variables
unsigned long idle_cnt;

int us_sem; // semaphore for sensor task (used with the ultrasonics)

void cpu_idle(ASIZE ignored)
{
  unsigned long t;
  unsigned long cnt;

  t = sysclock + 1000;
  while (1)
  {
    idle_cnt = proc_counter;
    proc_counter = 0;
    // WAIT(1000);
    PERIOD(&t, 1000);
  }
}

void head(ASIZE delay)
{
  unsigned long cnt;
  float trip_dist = 5.0;
  uint8_t fail_cntr = 0;
  uint8_t trip_val = 5;
  uint8_t ena = OFF; // <------------------------claw -----------------------<<<<<
  LAYER motion;

  while (1)
  {
    if (motion.flag) {
      if (&motion.arg == "1") {
        move_head(1, 60);
      }
    }

    SoftwareServo::refresh();

    WAIT(delay);
  }
}

void ultrasonic_task(ASIZE delay)
{
  TSIZE t;
  t = sysclock + delay;
  
  while (1)
  {
    PERIOD(&t, delay);
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
      if (i == 0) {
        ptr_us_R->dist = us[0].ping_cm();
      }
      else if (i == 1) {
        ptr_us_L->dist = us[1].ping_cm();
      }
    }
  }
}

void sensors(ASIZE delay)
{
  unsigned long cnt;
  uint8_t fail_cntr_us = 0;
  uint8_t fail_limit_us = 4; //11;  // may need to be adjusted depending on # of bad us readings
  uint8_t fail_cntr_bump = 0;
  uint8_t fail_limit_bump = 5;
  float trip_dist = 30.0; //35.0
  uint8_t us_running = 0; // used to determine if us are returning values
  // enable switches
  uint8_t us_bump_ena = OFF;  // <---------------------------us_bumper---------------------------<<<<<<
  uint8_t twp_ena = ON;       // <---------------------------temp wpt----------------------------<<<<<<
  uint8_t obs_slow_ena = OFF; // <---------------------------slow before obstacle (claw)---------<<<<<<

  uint8_t bumper_result = 0;

  while (1)
  {

    if ((us_running == 0) && (ptr_us_R->dist >= trip_dist) && (ptr_us_L->dist >= trip_dist))
    {
      semaphore_release(&us_sem);
      us_running = 1;
    }

    if (us_running == 1)
    {
      if ((us_bump_ena == ON) && (twp_ena == OFF))
      { // ultrasonic used as bumper if ena

        if ((ptr_us_R->dist < trip_dist) || (ptr_us_L->dist < trip_dist))
        {
          fail_cntr_us++;
          if (fail_cntr_us > fail_limit_us)
          {
            fail_cntr_us = fail_limit_us;
            stop_movement_flg |= (0x02); // set bit to 1
          }
        }
      }

      if ((us_bump_ena == OFF) && (twp_ena == ON))
      { // temp wpt used with ultrasonic

        if ((ptr_us_R->dist < trip_dist) || (ptr_us_L->dist < trip_dist))
        {
          fail_cntr_us++;
          if (fail_cntr_us > fail_limit_us)
          {
            fail_cntr_us = fail_limit_us;
            create_temp_waypoint(ptr_temp_wpt, ptr_loc, 45.0, 70.0);
          }
        }
      }

      // When ultrasonic is not used as bumper or to generate temp waypts. This behavior slows down
      // the robot when approaching an obstacle. It is used in conjunction with the claw.
      if (((ptr_us_R->dist < trip_dist) || (ptr_us_L->dist < trip_dist)) && (obs_slow_ena == 1))
      {
        fail_cntr_us++;
        if (fail_cntr_us > fail_limit_us)
        {
          fail_cntr_us = fail_limit_us;
          slow_movement_flg |= (0x01); // set bit to 1
        }
      }

      if ((ptr_us_R->dist >= trip_dist) && (ptr_us_L->dist >= trip_dist))
      { // reset fail_cntr_us
        fail_cntr_us = 0;
        stop_movement_flg &= ~(0x02); // set bit to 0
      }
    }

    /*// data report for troubleshooting
    Serial.print(millis());
    Serial.print(" , ");
    Serial.print(ptr_loc->x_pos);
    Serial.print(" , ");
    Serial.print(ptr_loc->y_pos);
    Serial.print(" , ");
    Serial.print(ptr_loc->deg_theta);
    Serial.print(" , ");
    Serial.print(ptr_targ->target_distance);
    Serial.print(" , ");
    Serial.print(ptr_us_R->dist);
    Serial.print(" , ");
    Serial.print(ptr_us_L->dist);
    Serial.print(" , ");
    Serial.print(stop_movement_flg);
    Serial.print(" , ");
    Serial.print(slow_movement_flg);
    Serial.print(" , ");
    Serial.print(ptr_temp_wpt->flgs);
    Serial.print(" , ");
    Serial.print(fail_cntr_us);
    Serial.print(" , ");
    Serial.println(us_running);
    */

    //---------- end of ultrasonic sensors ------------------------

    // bumper
    bumper_result = test_bumpers();

    if (bumper_result != 0)
    { // bumper_results: 0=no hit, 1=right_bumper, 2=left bumper, 3=both bumpers
      fail_cntr_bump++;
      if (fail_cntr_bump > fail_limit_bump)
      {
        fail_cntr_bump = fail_limit_bump;
        stop_movement_flg |= (0x04); //set bit to 1
      }
    }
    else
    {
      fail_cntr_bump = 0;
      stop_movement_flg &= ~(0x04); //set bit to 0
    }
    //----------- end of bumper -----------------------

    WAIT(delay);
  }
}

void move(ASIZE delay)
{
  unsigned long cnt;
  //------------------------------Required when using PERIOD instead of WAIT
  TSIZE t;
  t = sysclock + 10;
  //------------------------------
  int segNav = 0;
  int ballistic_turn_flg = 1;
  int ena = ON; // <-----------------turn in place----<<<<
  int rotate_in_place_flg = 0;
  int turn_speed = 110;
  int slew_val = 0;
  int target_spd = 130;
  float sum_deg_heading_error = 0;
  uint8_t start_to_run_task = 0;

  init_pids();

  if (start_to_run_task == 0)
  {
    semaphore_obtain(&us_sem);
    start_to_run_task = 1;
  }

  while (1)
  {
    segNav = delta_target(ptr_targ, ptr_temp_wpt, waypoint_x[wayptnum], waypoint_y[wayptnum], 15.0);

    if (segNav == 1)
    { // no temp_wpt in play
      wayptnum++;
      if (waypoint_x[wayptnum] == LAST_ELEM)
      { // end of route
        stop_movement_flg = 1;
        Serial.println("end");
      }
    }

    if (segNav == 2)
    { // no temp_wpt in play
      waypoint_x[wayptnum] = ptr_temp_wpt->orig_x_target;
      waypoint_y[wayptnum] = ptr_temp_wpt->orig_y_target;

      init_temp_waypoint(ptr_temp_wpt);
    }

    ptr_targ->x_target = waypoint_x[wayptnum];
    ptr_targ->y_target = waypoint_y[wayptnum];

    odometer(ptr_loc);
    locate_target(ptr_targ, ptr_loc);

    if ((ballistic_turn_flg == ON) && (ena == ON))
    {
      rotate_in_place_flg = determine_rotate_in_place_flg(ptr_targ);
    }

    if (stop_movement_flg == 0)
    {
      if (rotate_in_place_flg == ON)
      {
        right_mtr_pid.SetMode(MANUAL); // turn off pid
        left_mtr_pid.SetMode(MANUAL);
        slew_val = 0;
        //sum_deg_heading_error = 0;
        if (ptr_targ->heading_error > 0) // need to turn right   (>)
        {
          motorGo(R_MTR, CW, turn_speed); // ballistic turn
          motorGo(L_MTR, CCW, turn_speed);
        }
        else if (ptr_targ->heading_error < 0) // need to turn left   (<)
        {
          motorGo(R_MTR, CCW, turn_speed);
          motorGo(L_MTR, CW, turn_speed);
        }
      }
      else
      {
        //right_mtr_pid.SetMode(AUTOMATIC);                 // turn on pid
        //left_mtr_pid.SetMode(AUTOMATIC);
        init_pids();
        slew_val = slew(slew_val, target_spd, 2);

        //slow down as approach obstacle (can)
        if (slow_movement_flg != 0x00)
        { // us sensor sees obj = 1, approaching target = 2
          slew_val = slew(slew_val, 60, 2);
        }

        // would like to add integration factor to remove offset ---- still buggy
        //sum_deg_heading_error += ptr_targ->deg_heading_error;
        //sum_deg_heading_error = clip_f (sum_deg_heading_error,-30.0,30.0);
        //setpoint[R_MTR] = (double)slew_val + ((double) ptr_targ->deg_heading_error + 0.1 * sum_deg_heading_error);  // pi controller for rotation
        //setpoint[L_MTR] = (double)slew_val - ((double) ptr_targ->deg_heading_error + 0.1 * sum_deg_heading_error);

        setpoint[R_MTR] = (double)slew_val + ((double)ptr_targ->deg_heading_error); // p controller for rotation
        setpoint[L_MTR] = (double)slew_val - ((double)ptr_targ->deg_heading_error);

        pid_input[R_MTR] = (double)ptr_loc->encoder_interval_cnt_R_mtr;
        pid_input[L_MTR] = (double)ptr_loc->encoder_interval_cnt_L_mtr;

        right_mtr_pid.Compute();
        left_mtr_pid.Compute();

        motorGo(R_MTR, CW, pid_output[R_MTR]);
        motorGo(L_MTR, CW, pid_output[L_MTR]);
      }
    }
    else
    {
      motorOff(R_MTR);
      motorOff(L_MTR);
      Serial.println("MOTOR STOP");
    }

    // data report for troubleshooting
    Serial.print(millis());
    Serial.print(" , ");
    Serial.print(ptr_loc->x_pos);
    Serial.print(" , ");
    Serial.print(ptr_loc->y_pos);
    Serial.print(" , ");
    Serial.print(ptr_loc->deg_theta);
    Serial.print(" , ");
    Serial.print(ptr_targ->deg_heading_error);
    Serial.print(" , ");
    Serial.print(setpoint[R_MTR]);
    Serial.print(" , ");
    Serial.print(setpoint[L_MTR]);
    Serial.print(" , ");
    Serial.print(pid_input[0]);
    Serial.print(" , ");
    Serial.print(pid_input[1]);
    Serial.print(" , ");
    Serial.print(rotate_in_place_flg);
    Serial.print(" , ");
    Serial.print(pid_output[0]);
    Serial.print(" , ");
    Serial.print(pid_output[1]);
    Serial.print(" , ");
    Serial.print(slew_val);
    Serial.print(" , ");
    Serial.print(sum_deg_heading_error);

    Serial.print(" , ");
    Serial.print(ptr_us_R->dist);
    Serial.print(" , ");
    Serial.print(ptr_us_L->dist);

    Serial.print(" , ");
    Serial.print(stop_movement_flg);
    Serial.print(" , ");
    Serial.print(slow_movement_flg);
    Serial.print(" , ");
    Serial.println(ptr_temp_wpt->flgs);

    //WAIT(delay);
    PERIOD(&t, 10); // every 10 millisecs
  }
}

void stats_task(ASIZE delay)
{
  TSIZE t;
  t = sysclock + delay;

  while (1)
  {
    // WAIT(delay);
    PERIOD(&t, delay);
    PRINTF("");
    SPRINTF(sbuf, "# Sysclock\t%ld\tSampleclock\t%ld\tIdleHz\t%ld",
            sysclock, sampleclock, idle_cnt);
    PRINTF(sbuf);
    PRINTF("");
    print_llist(1);
  }
}

/* ----------------------------------------- */
/* Create signon and terminate task */

void signon(ASIZE version)
{
  PRINTF(VERSION);
  wake_after(2000);
  PRINTF("# SIGNON Messages signing off!\n");
  DELAY(1000);
  terminate();
}
