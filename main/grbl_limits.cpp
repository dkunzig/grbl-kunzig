/*
  limits.c - code pertaining to limit-switches and performing the homing cycle
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
	
	2018 -	Bart Dring This file was modifed for use on the ESP32
					CPU. Do not use this with Grbl for atMega328P
  2018-12-29 - Wolfgang Lienbacher renamed file from limits.h to grbl_limits.h 
          fixing ambiguation issues with limit.h in the esp32 Arduino Framework 
          when compiling with VS-Code/PlatformIO.

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

xQueueHandle limit_sw_queue;  // used by limit switch debouncing

// Homing axis search distance multiplier. Computed by this value times the cycle travel.
#ifndef HOMING_AXIS_SEARCH_SCALAR
  #define HOMING_AXIS_SEARCH_SCALAR  1.1 // Must be > 1 to ensure limit switch will be engaged.
#endif
#ifndef HOMING_AXIS_LOCATE_SCALAR
  #define HOMING_AXIS_LOCATE_SCALAR  5.0 // Must be > 1 to ensure limit switch is cleared.
#endif

void IRAM_ATTR isr_limit_switches()
{
	// Ignore limit switches if already in an alarm state or in-process of executing an alarm.
    // When in the alarm state, Grbl should have been reset or will force a reset, so any pending
    // moves in the planner and serial buffers are all cleared and newly sent blocks will be
    // locked out until a homing cycle or a kill lock command. Allows the user to disable the hard
    // limit setting if their limits are constantly triggering after a reset and move their axes.
	printf("this is a test");
	if (  ( sys.state != STATE_ALARM) & (bit_isfalse(sys.state, STATE_HOMING)) ) {
		if (!(sys_rt_exec_alarm)) {
			#ifdef ENABLE_SOFTWARE_DEBOUNCE
				// we will start a task that will recheck the switches after a small delay
				int evt;
				xQueueSendFromISR(limit_sw_queue, &evt, NULL);	
			#else
				#ifdef HARD_LIMIT_FORCE_STATE_CHECK
				  // Check limit pin state.
				  if (limits_get_state()) {
					mc_reset(); // Initiate system kill.
					system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // Indicate hard limit critical event
				  }
				#else
				  mc_reset(); // Initiate system kill.
				  system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // Indicate hard limit critical event
				#endif				
			#endif
		}						
    }
}

//#define HOMING_CYCLE_0 (1<<Z_AXIS)                // REQUIRED: First move Z to clear workspace.
//#define HOMING_CYCLE_1 (1<<X_AXIS)  
//#define HOMING_CYCLE_2 (1<<Y_AXIS)
void limits_go_home(uint8_t cycle_mask)
{
	

	printf("enter Limits_go_home cycle_mask = %x\n",cycle_mask);
	if (cycle_mask == 0)
	{
		HomeMotorZ();
		HomeMotorX();
		HomeMotorY();
		
	}
	else if (cycle_mask == 4)
	{
		HomeMotorZ();
		//MoveMotorA();
	}
	else if (cycle_mask == 2)
	{
		HomeMotorY();
	}
	else if (cycle_mask == 1)
	{
		HomeMotorX();
	}
	BackOff_All_Motors();
}


void HomeMotorX()
{
	gpio_set_level(X_DIRECTION_PIN, 0);
	//printf("Enter HomeMotorX\n");
	while (1)
	{
		if (gpio_get_level(X_LIMIT_PIN) == 1)
		{
			//MotorStepHigh;
			gpio_set_level(X_STEP_PIN, 1);
			esp_delay_us(15);
			//MotorStepLow;
			gpio_set_level(X_STEP_PIN, 0);
			esp_delay_us(400);
		}
		else
			break;
	}

}

void HomeMotorY()
{
	#ifdef Y_DIRECTION_PIN
		gpio_set_level(Y_DIRECTION_PIN, 0);
	#endif
	gpio_set_level(Y_DIRECTION_B_PIN, 0);

	//printf("Enter HomeMotorY\n");
	while (1)
	{
		if (gpio_get_level(Y_LIMIT_PIN) == 1)
		{
			//MotorStepHigh;
			gpio_set_level(Y_STEP_PIN, 1);
			esp_delay_us(15);
			//MotorStepLow;
			gpio_set_level(Y_STEP_PIN, 0);
			esp_delay_us(400);
		}
		if (gpio_get_level(Y_LIMIT_B_PIN) == 1)
		{
			//MotorStepHigh;
			gpio_set_level(Y_STEP_B_PIN, 1);
			esp_delay_us(15);
			//MotorStepLow;
			gpio_set_level(Y_STEP_B_PIN, 0);
			esp_delay_us(400);
		}
		if (gpio_get_level(Y_LIMIT_PIN) == 0 && gpio_get_level(Y_LIMIT_B_PIN) == 0)
			break;
	}
}

void HomeMotorZ()
{
#ifdef Z_STEP_PIN
	gpio_set_level(Z_DIRECTION_PIN, 0);
	//printf("Enter HomeMotorZ\n");
	while (1)
	{
		if (gpio_get_level(Z_LIMIT_PIN) == 1)
		{
			//MotorStepHigh;
			gpio_set_level(Z_STEP_PIN, 1);
			esp_delay_us(15);
			//MotorStepLow;
			gpio_set_level(Z_STEP_PIN, 0);
			esp_delay_us(800);
		}
		else
			break;
	}
#endif
}



void BackOff_All_Motors()
{
	gpio_set_level(Z_DIRECTION_PIN, 1);
#ifdef Y_DIRECTION_PIN
	gpio_set_level(Y_DIRECTION_PIN, 1);
#endif
	gpio_set_level(Y_DIRECTION_B_PIN, 1);
	gpio_set_level(X_DIRECTION_PIN, 1);
	printf("z limit pin = %d\n", gpio_get_level(Z_LIMIT_PIN));
	printf("Y limit pin = %d\n", gpio_get_level(Y_LIMIT_PIN));
	printf("Y limit B pin = %d\n", gpio_get_level(Y_LIMIT_B_PIN));
	printf("X limit pin = %d\n", gpio_get_level(X_LIMIT_PIN));

	while (1)
	{
		
		if (gpio_get_level(Z_LIMIT_PIN) == 0)
		{
		#ifdef Y_DIRECTION_PIN
			gpio_set_level(Y_STEP_PIN, 1);
		#endif	
			esp_delay_us(15);
			//MotorStepLow;
		#ifdef Z_STEP_PIN
			gpio_set_level(Z_STEP_PIN, 0);
		#endif
			esp_delay_us(800);
		}
		if (gpio_get_level(Y_LIMIT_PIN) == 0)
		{
			//printf("y limit pin = %x\n", gpio_get_level(Y_LIMIT_PIN));
			gpio_set_level(Y_STEP_PIN, 1);
			esp_delay_us(15);
			//MotorStepLow;
			gpio_set_level(Y_STEP_PIN, 0);
			esp_delay_us(800);
		}
		if (gpio_get_level(Y_LIMIT_B_PIN) == 0)
		{
			//MotorStepHigh;
			gpio_set_level(Y_STEP_B_PIN, 1);
			esp_delay_us(15);
			//MotorStepLow;
			gpio_set_level(Y_STEP_B_PIN, 0);
			esp_delay_us(800);
		}
		if (gpio_get_level(X_LIMIT_PIN) == 0)
		{
			//MotorStepHigh;
			gpio_set_level(X_STEP_PIN, 1);
			esp_delay_us(15);
			//MotorStepLow;
			gpio_set_level(X_STEP_PIN, 0);
			esp_delay_us(800);
		}
		if (gpio_get_level(Y_LIMIT_PIN) == 1 && gpio_get_level(Y_LIMIT_B_PIN) == 1 && gpio_get_level(Z_LIMIT_PIN) == 1 && gpio_get_level(X_LIMIT_PIN) == 1)
			break;
	}
	int idx;
	for (idx = 0; idx<N_AXIS; idx++) {
		 sys_position[idx] = 0;
    }

}

void MoveMotorA()
{
	gpio_set_level(A_DIRECTION_PIN, 1);
	//printf("Enter MoveMotorA\n");
	int i = 0;
	for(i=0;i < 2000; i++)
	{
		//MotorStepHigh;
		gpio_set_level(A_STEP_PIN, 1);
		esp_delay_us(15);
		//MotorStepLow;
		gpio_set_level(A_STEP_PIN, 0);
		esp_delay_us(800);
	}
	gpio_set_level(A_DIRECTION_PIN,0);
	//printf("Enter MoveMotorA to minus\n");
	for (i = 0; i < 2000; i++)
	{
		//MotorStepHigh;
		gpio_set_level(A_STEP_PIN, 1);
		esp_delay_us(15);
		//MotorStepLow;
		gpio_set_level(A_STEP_PIN, 0);
		esp_delay_us(800);
	}
}

void esp_delay_us(int delay)
{
	int64_t startTime = esp_timer_get_time();
	int64_t endtime;
	do
	{
		endtime = esp_timer_get_time();
	} while (endtime - startTime < delay);
	//printf("delay = %d\n", (int) (endtime - startTime));
	return;
}


// Homes the specified cycle axes, sets the machine position, and performs a pull-off motion after
// completing. Homing is a special motion case, which involves rapid uncontrolled stops to locate
// the trigger point of the limit switches. The rapid stops are handled by a system level axis lock
// mask, which prevents the stepper algorithm from executing step pulses. Homing motions typically
// circumvent the processes for executing motions in normal operation.
// NOTE: Only the abort realtime command can interrupt this process.
// TODO: Move limit pin-specific calls to a general function for portability.
//void limits_go_home(uint8_t cycle_mask)
//{
//	static uint8_t hold_limit_state;
//	static uint8_t hold_ganged_mode;
//	static uint8_t hold_cycle_mask;
//
//  if (sys.abort) { return; } // Block if system reset has been issued.
//  
//  // Initialize plan data struct for homing motion. Spindle and coolant are disabled.
//  plan_line_data_t plan_data;
//  plan_line_data_t *pl_data = &plan_data;
//  memset(pl_data,0,sizeof(plan_line_data_t));
//  pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
//  #ifdef USE_LINE_NUMBERS
//    pl_data->line_number = HOMING_CYCLE_LINE_NUMBER;
//  #endif
//
//  // Initialize variables used for homing computations.
//  uint8_t n_cycle = (2*N_HOMING_LOCATE_CYCLE+1);
//  uint8_t step_pin[N_AXIS];
//  float target[N_AXIS];
//  float max_travel = 0.0;
//  uint8_t idx;
//  for (idx=0; idx<N_AXIS; idx++) {
//    // Initialize step pin masks
//    step_pin[idx] = get_step_pin_mask(idx);
//    #ifdef COREXY
//      if ((idx==A_MOTOR)||(idx==B_MOTOR)) { step_pin[idx] = (get_step_pin_mask(X_AXIS)|get_step_pin_mask(Y_AXIS)); }
//    #endif
//
//    if (bit_istrue(cycle_mask,bit(idx))) {
//      // Set target based on max_travel setting. Ensure homing switches engaged with search scalar.
//      // NOTE: settings.max_travel[] is stored as a negative value.
//      max_travel = MAX(max_travel,(-HOMING_AXIS_SEARCH_SCALAR)*settings.max_travel[idx]);
//    }
//  }
//  for (idx = 0; idx < N_AXIS; idx++) {
//	  printf("step pin[idx] = %x idx = %d\n", step_pin[idx], idx);
//  }
//  // Set search mode with approach at seek rate to quickly engage the specified cycle_mask limit switches.
//  bool approach = true;
//  float homing_rate = settings.homing_seek_rate;
//
//  uint8_t limit_state, axislock, n_active_axis;
//  do {
//
//    system_convert_array_steps_to_mpos(target,sys_position);
//
//    // Initialize and declare variables needed for homing routine.
//    axislock = 0;
//    n_active_axis = 0;
//    for (idx=0; idx<N_AXIS; idx++) {
//      // Set target location for active axes and setup computation for homing rate.
//      if (bit_istrue(cycle_mask,bit(idx))) {
//        n_active_axis++;
//        #ifdef COREXY
//          if (idx == X_AXIS) {
//            int32_t axis_position = system_convert_corexy_to_y_axis_steps(sys_position);
//            sys_position[A_MOTOR] = axis_position;
//            sys_position[B_MOTOR] = -axis_position;
//          } else if (idx == Y_AXIS) {
//            int32_t axis_position = system_convert_corexy_to_x_axis_steps(sys_position);
//            sys_position[A_MOTOR] = sys_position[B_MOTOR] = axis_position;
//          } else {
//            sys_position[Z_AXIS] = 0;
//          }
//        #else
//          sys_position[idx] = 0;
//        #endif
//        // Set target direction based on cycle mask and homing cycle approach state.
//        // NOTE: This happens to compile smaller than any other implementation tried.
//        if (bit_istrue(settings.homing_dir_mask,bit(idx))) {
//          if (approach) { target[idx] = -max_travel; }
//          else { target[idx] = max_travel; }
//        } else {
//          if (approach) { target[idx] = max_travel; }
//          else { target[idx] = -max_travel; }
//        }
//        // Apply axislock to the step port pins active in this cycle.
//        axislock |= step_pin[idx];
//      }
//
//    }
//    homing_rate *= sqrt(n_active_axis); // [sqrt(N_AXIS)] Adjust so individual axes all move at homing rate.
//    sys.homing_axis_lock = axislock;
//	printf("Axislock = %x\n", axislock);
//	if (cycle_mask == 2 && ganged_mode == 2)
//		axislock = 8;
//    // Perform homing cycle. Planner buffer should be empty, as required to initiate the homing cycle.
//    pl_data->feed_rate = homing_rate; // Set current homing rate.
//    plan_buffer_line(target, pl_data); // Bypass mc_line(). Directly plan homing motion.
//
//    sys.step_control = STEP_CONTROL_EXECUTE_SYS_MOTION; // Set to execute homing motion and clear existing flags.
//    st_prep_buffer(); // Prep and fill segment buffer from newly planned block.
//    st_wake_up(); // Initiate motion
//    do {
//		
//      if (approach) {
//        // Check limit state. Lock out cycle axes when they change.
//		
//        limit_state = limits_get_state();
//		if(limit_state != hold_limit_state || ganged_mode != hold_ganged_mode ||  cycle_mask != hold_cycle_mask)
//		{
//			printf("in do loop limit state = %x ganged mode = %x cycle mask = %x\n", (uint8_t)limit_state, ganged_mode,cycle_mask);
//			hold_limit_state = limit_state;
//			hold_ganged_mode = ganged_mode;
//			hold_cycle_mask = cycle_mask;
//		}
//        for (idx=0; idx<N_AXIS; idx++) {
//          if (axislock & step_pin[idx]) {
//            if (limit_state & (1 << idx)) {
//                axislock &= ~(step_pin[idx]);
//				printf("Axislock = %x step_pin[idx] = %x idx = %d\n", axislock, step_pin[idx], idx);
//            }
//          }
//        }
//        sys.homing_axis_lock = axislock;
//      }
//
//      st_prep_buffer(); // Check and prep segment buffer. NOTE: Should take no longer than 200us.
//
//      // Exit routines: No time to run protocol_execute_realtime() in this loop.
//      if (sys_rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET | EXEC_CYCLE_STOP)) {
//        uint8_t rt_exec = sys_rt_exec_state;
//        // Homing failure condition: Reset issued during cycle.
//        if (rt_exec & EXEC_RESET) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_RESET); }
//        // Homing failure condition: Safety door was opened.
//        if (rt_exec & EXEC_SAFETY_DOOR) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_DOOR); }
//        // Homing failure condition: Limit switch still engaged after pull-off motion
//		printf("limits state = %d\n", limits_get_state());
//		printf("cycle_mask = %d\n", cycle_mask);
//        if (!approach && (limits_get_state() & cycle_mask)) 
//		{ 
//			printf("limits state failing..... limit state =   = %d\n", limits_get_state());
//			system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_PULLOFF); 
//		}
//        // Homing failure condition: Limit switch not found during approach.
//        if (approach && (rt_exec & EXEC_CYCLE_STOP)) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_APPROACH); }
//        if (sys_rt_exec_alarm) {
//          mc_reset(); // Stop motors, if they are running.
//          protocol_execute_realtime();
//          return;
//        } else {
//			printf("homing pull off complete\n");
//          // Pull-off motion complete. Disable CYCLE_STOP from executing.
//          system_clear_exec_state_flag(EXEC_CYCLE_STOP);
//          break;
//        }
//      }
//
//    } while (STEP_MASK & axislock);
//
//    st_reset(); // Immediately force kill steppers and reset step segment buffer.
//    delay_ms(settings.homing_debounce_delay); // Delay to allow transient dynamics to dissipate.
//
//    // Reverse direction and reset homing rate for locate cycle(s).
//    approach = !approach;
//
//    // After first cycle, homing enters locating phase. Shorten search to pull-off distance.
//    if (approach) {
//      max_travel = settings.homing_pulloff*HOMING_AXIS_LOCATE_SCALAR;
//      homing_rate = settings.homing_feed_rate;
//    } else {
//      max_travel = settings.homing_pulloff;
//      homing_rate = settings.homing_seek_rate;
//    }
//
//  } while (n_cycle-- > 0);
//
//  // The active cycle axes should now be homed and machine limits have been located. By
//  // default, Grbl defines machine space as all negative, as do most CNCs. Since limit switches
//  // can be on either side of an axes, check and set axes machine zero appropriately. Also,
//  // set up pull-off maneuver from axes limit switches that have been homed. This provides
//  // some initial clearance off the switches and should also help prevent them from falsely
//  // triggering when hard limits are enabled or when more than one axes shares a limit pin.
//  int32_t set_axis_position;
//  // Set machine positions for homed limit switches. Don't update non-homed axes.
//  for (idx=0; idx<N_AXIS; idx++) {
//    // NOTE: settings.max_travel[] is stored as a negative value.
//    if (cycle_mask & bit(idx)) {
//      #ifdef HOMING_FORCE_SET_ORIGIN
//        set_axis_position = 0;
//      #else
//        if ( bit_istrue(settings.homing_dir_mask,bit(idx)) ) {
//          set_axis_position = lround((settings.max_travel[idx]+settings.homing_pulloff)*settings.steps_per_mm[idx]);
//        } else {
//          set_axis_position = lround(-settings.homing_pulloff*settings.steps_per_mm[idx]);
//        }
//      #endif
//
//      #ifdef COREXY
//        if (idx==X_AXIS) {
//          int32_t off_axis_position = system_convert_corexy_to_y_axis_steps(sys_position);
//          sys_position[A_MOTOR] = set_axis_position + off_axis_position;
//          sys_position[B_MOTOR] = set_axis_position - off_axis_position;
//        } else if (idx==Y_AXIS) {
//          int32_t off_axis_position = system_convert_corexy_to_x_axis_steps(sys_position);
//          sys_position[A_MOTOR] = off_axis_position + set_axis_position;
//          sys_position[B_MOTOR] = off_axis_position - set_axis_position;
//        } else {
//          sys_position[idx] = set_axis_position;
//        }
//      #else
//        sys_position[idx] = set_axis_position;
//      #endif
//
//    }
//  }
//  sys.step_control = STEP_CONTROL_NORMAL_OP; // Return step control to normal operation.
//}


void limits_init()
{  
	
  #ifndef DISABLE_LIMIT_PIN_PULL_UP  
		#ifdef X_LIMIT_PIN
			gpio_set_direction(X_LIMIT_PIN, GPIO_MODE_INPUT);
			gpio_set_pull_mode(X_LIMIT_PIN,GPIO_PULLUP_ONLY);
			//pinMode(X_LIMIT_PIN, INPUT_PULLUP);  // input with pullup
		#endif
		#ifdef Y_LIMIT_PIN
			gpio_set_direction(Y_LIMIT_PIN, GPIO_MODE_INPUT);
			gpio_set_pull_mode(Y_LIMIT_PIN,GPIO_PULLUP_ONLY);
			//pinMode(Y_LIMIT_PIN, INPUT_PULLUP);
		#endif
		#ifdef Y_LIMIT_B_PIN
			gpio_set_direction(Y_LIMIT_B_PIN, GPIO_MODE_INPUT);
			gpio_set_pull_mode(Y_LIMIT_B_PIN,GPIO_PULLUP_ONLY);
			//pinMode(Y_LIMIT_B_PIN, INPUT_PULLUP);
		#endif
		#ifdef Z_LIMIT_PIN
			gpio_set_direction(Z_LIMIT_PIN, GPIO_MODE_INPUT);
			gpio_set_pull_mode(Z_LIMIT_PIN,GPIO_PULLUP_ONLY);
			//pinMode(Z_LIMIT_PIN, INPUT_PULLUP);
		#endif
		
	#else
		#ifdef X_LIMIT_PIN
			gpio_set_direction(X_LIMIT_PIN, GPIO_MODE_INPUT);
			//pinMode(X_LIMIT_PIN, INPUT); // input no pullup
		#endif
		#ifdef Y_LIMIT_PIN
			gpio_set_direction(Y_LIMIT_PIN, GPIO_MODE_INPUT);
			//pinMode(Y_LIMIT_PIN, INPUT);
		#endif
		#ifdef Y_LIMIT_B_PIN
			gpio_set_direction(Y_LIMIT_B_PIN, GPIO_MODE_INPUT);
			//pinMode(Y_LIMIT_B_PIN, INPUT);
		#endif
		#ifdef Z_LIMIT_PIN
			gpio_set_direction(Z_LIMIT_PIN, GPIO_MODE_INPUT);
			//pinMode(Z_LIMIT_PIN, INPUT);	
		#endif

  #endif
   
  if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)) {
    // attach interrupt to them
		#ifdef X_LIMIT_PIN
			//attachInterrupt(digitalPinToInterrupt(X_LIMIT_PIN), isr_limit_switches, CHANGE);
		#endif
		#ifdef Y_LIMIT_PIN
			//attachInterrupt(digitalPinToInterrupt(Y_LIMIT_PIN), isr_limit_switches, CHANGE);
		#endif
		#ifdef Y_LIMIT_B_PIN
			//		attachInterrupt(digitalPinToInterrupt(Y_LIMIT_B_PIN), isr_limit_switches, CHANGE);
		#endif
		#ifdef Z_LIMIT_PIN
			//attachInterrupt(digitalPinToInterrupt(Z_LIMIT_PIN), isr_limit_switches, CHANGE);
		#endif
  } else {
    limits_disable();
  }
  
	// setup task used for debouncing
	limit_sw_queue = xQueueCreate(10, sizeof( int ));
	
	xTaskCreate(limitCheckTask, 
				"limitCheckTask", 
				2048, 
				NULL, 
				5, // priority 
				NULL);
								
								
}


// Disables hard limits.
void limits_disable()
{  
  //detachInterrupt(X_LIMIT_BIT);
  //detachInterrupt(Y_LIMIT_BIT);
  //detachInterrupt(Z_LIMIT_BIT);  
}


// Returns limit state as a bit-wise uint8 variable. Each bit indicates an axis limit, where
// triggered is 1 and not triggered is 0. Invert mask is applied. Axes are defined by their
// number in bit position, i.e. Z_AXIS is (1<<2) or bit 2, and Y_AXIS is (1<<1) or bit 1.
uint8_t limits_get_state()
{
	uint8_t limit_state = 0;
	uint8_t pin = 0;
	
	#ifdef X_LIMIT_PIN
		pin |= gpio_get_level(X_LIMIT_PIN);
		//printf("After read X pin = %x\n", pin);
	#endif
	
	#ifdef Y_LIMIT_PIN
		
		pin |= (gpio_get_level(Y_LIMIT_PIN) << Y_AXIS);
		//printf("After read Y pin = %x\n", pin);
	#endif
	#ifdef Y_LIMIT_B_PIN
			pin |= (gpio_get_level(Y_LIMIT_B_PIN) << 3);
			//printf("After read Y_B pin = %x\n", pin);
	#endif

	#ifdef Z_LIMIT_PIN
		pin |= (gpio_get_level(Z_LIMIT_PIN) << Z_AXIS);
		//printf("After read Z pin = %x\n", pin);
	#endif

	#ifdef INVERT_LIMIT_PIN_MASK // not normally used..unless you have both normal and inverted switches
		pin ^= INVERT_LIMIT_PIN_MASK;
	#endif	
  
	if (bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { 
		pin ^= LIMIT_MASK;
	}
  
	if (pin) {
		uint8_t idx;
		for (idx=0; idx<N_AXIS; idx++) {
			if (pin & get_limit_pin_mask(idx)) { limit_state |= (1 << idx); }
		}
	}
	//printf("Returning  pin = %x\n", pin);
	return(limit_state);
}

// Performs a soft limit check. Called from mc_line() only. Assumes the machine has been homed,
// the workspace volume is in all negative space, and the system is in normal operation.
// NOTE: Used by jogging to limit travel within soft-limit volume.
void limits_soft_check(float *target)
{
	if (system_check_travel_limits(target)) {
		sys.soft_limit = true;
		// Force feed hold if cycle is active. All buffered blocks are guaranteed to be within
		// workspace volume so just come to a controlled stop so position is not lost. When complete
		// enter alarm mode.
		if (sys.state == STATE_CYCLE) {
			system_set_exec_state_flag(EXEC_FEED_HOLD);
		do {
			protocol_execute_realtime();
			if (sys.abort) { return; }
		} while ( sys.state != STATE_IDLE );
    }
	
    mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
    system_set_exec_alarm(EXEC_ALARM_SOFT_LIMIT); // Indicate soft limit critical event
    protocol_execute_realtime(); // Execute to enter critical event loop and system abort
    return;
  }
}

// this is the task
void limitCheckTask(void *pvParameters)
{	
	while(true) {
		int evt;
		xQueueReceive(limit_sw_queue, &evt, portMAX_DELAY); // block until receive queue
		vTaskDelay( DEBOUNCE_PERIOD / portTICK_PERIOD_MS ); // delay a while
		
		if (limits_get_state()) {
			mc_reset(); // Initiate system kill.
			system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // Indicate hard limit critical event
		}		
	}
}

// return true if the axis is defined as a squared axis
// Squaring: is used on gantry type axes that have two motors
// Each motor with touch off its own switch to square the axis
bool axis_is_squared(uint8_t axis_mask)
{
	// Note: multi-axis homing returns false because it will not match any of the following
	
	if (axis_mask == (1<<X_AXIS)) {
		#ifdef X_AXIS_SQUARING
		printf("X Axis Squaring = defined\n");
			return true;
		#endif
	}
	
	if (axis_mask == (1<<Y_AXIS)) {
		#ifdef Y_AXIS_SQUARING
		printf("Y Axis Squaring = defined\n");
			return true;
		#endif
	}
	
	if (axis_mask == (1<<Z_AXIS)) {
		#ifdef Z_AXIS_SQUARING
		printf("Z Axis Squaring = defined\n");
			return true;
		#endif
	}
	
	return false;
	
}