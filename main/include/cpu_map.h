/*
  cpu_map.h - Header for system level commands and real-time processes
  Part of Grbl
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC
	
	2018 -	Bart Dring This file was modified for use on the ESP32
					CPU. Do not use this with Grbl for atMega328P
	
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

#ifndef cpu_map_h
//#define cpu_map_h 

  /*
	Not all pins can can work for all functions.
	Check features like pull-ups, pwm, etc before
  re-assigning numbers
	
	(gpio34-39) are inputs only and don't have software pullup/down functions
	You MUST use external pull-ups or noise WILL cause problems.
	
	Unlike the AVR version certain pins are not forced into the same port. 
	Therefore, bit masks are not use the same way and typically should not be 
	changed. They are just preserved right now to make it easy to stay in sync
	with AVR grbl
	
	*/
#ifdef CPU_MAP_LOWRIDER // !!!!!!!!!!!!!!!!! Warning: Untested !!!!!!!!!!!!!!!!! //
	// This is the CPU Map for the Buildlog.net MPCNC controller
	// used in lowrider mode. Low rider has (2) Y and Z and one X motor
	// These will not match the silkscreen or schematic descriptions	
		#define CPU_MAP_NAME "CPU_MAP_LOWRIDER"
	
	
		#define USE_GANGED_AXES // allow two motors on an axis 
	  
		#define X_STEP_PIN      GPIO_NUM_26    	// use Z labeled connector
		#define X_DIRECTION_PIN   GPIO_NUM_32 	// use Z labeled connector
		
		#define Y_STEP_PIN      GPIO_NUM_16
		#define Y_STEP_B_PIN    GPIO_NUM_17 	// ganged motor
		//#define Y_DIRECTION_PIN   GPIO_NUM_15   //put back in
		#define Y_DIRECTION_B_PIN   GPIO_NUM_2 
		//#define Y_AXIS_SQUARING
		//#define Z_STEP_PIN      GPIO_NUM_14	// use X labeled connector put back in
		#define Z_DIRECTION_PIN   GPIO_NUM_25 	// use X labeled connector
		#define A_STEP_PIN      GPIO_NUM_5	// use X labeled connector
		#define A_DIRECTION_PIN   GPIO_NUM_4 	// use X labeled connector
		// OK to comment out to use pin for other features
		//#define STEPPERS_DISABLE_PIN GPIO_NUM_13	 //put back in	
		
				
		//// Note: if you use PWM rather than relay, you could map GPIO_NUM_17 to mist or flood 
		//#define USE_SPINDLE_RELAY
		//
		//#ifdef USE_SPINDLE_RELAY		
		//	#define SPINDLE_PWM_PIN    GPIO_NUM_17
		//#else
		#define SPINDLE_PWM_PIN    GPIO_NUM_23  
		#define SPINDLE_ENABLE_PIN	GPIO_NUM_18
		//#endif
		
		#define SPINDLE_PWM_CHANNEL 0
		// PWM Generator is based on 80,000,000 Hz counter
		// Therefor the freq determines the resolution
		// 80,000,000 / freq = max resolution
		// For 5000 that is 80,000,000 / 5000 = 16000 
		// round down to nearest bit count for SPINDLE_PWM_MAX_VALUE = 13bits (8192)
		#define SPINDLE_PWM_BASE_FREQ 5000 // Hz
		#define SPINDLE_PWM_BIT_PRECISION 8   // be sure to match this with SPINDLE_PWM_MAX_VALUE
		#define SPINDLE_PWM_OFF_VALUE     0
		#define SPINDLE_PWM_MAX_VALUE     255 // (2^SPINDLE_PWM_BIT_PRECISION)
		
		#ifndef SPINDLE_PWM_MIN_VALUE
				#define SPINDLE_PWM_MIN_VALUE   1   // Must be greater than zero.
		#endif
		
		#define SPINDLE_PWM_RANGE         (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)		
		
		// Note: Only uncomment this if USE_SPINDLE_RELAY is commented out.
		// Relay can be used for Spindle or Coolant
		#define COOLANT_FLOOD_PIN 	GPIO_NUM_19
		
		#define X_LIMIT_PIN      	GPIO_NUM_36 
		#define Y_LIMIT_PIN      	GPIO_NUM_39  
		#define Y_LIMIT_B_PIN      	GPIO_NUM_34
		#define Z_LIMIT_PIN     	GPIO_NUM_35
		#define LIMIT_MASK      	0x15
		
	//	#define PROBE_PIN       	GPIO_NUM_35  
		
		// The default value in config.h is wrong for this controller
		#ifdef INVERT_CONTROL_PIN_MASK
			#undef INVERT_CONTROL_PIN_MASK			
		#endif
		
		#define INVERT_CONTROL_PIN_MASK  0x15
		
		// Note: check the #define IGNORE_CONTROL_PINS is the way you want in config.h
		//#define CONTROL_RESET_PIN         GPIO_NUM_34  // needs external pullup
		//#define CONTROL_FEED_HOLD_PIN     GPIO_NUM_36  // needs external pullup 
		//#define CONTROL_CYCLE_START_PIN   GPIO_NUM_39  // needs external pullup    		
		
#endif



	// ================= common to all machines ================================
	
	// These are some ESP32 CPU Settings that the program needs, but are generally not changed
		#define F_TIMERS	80000000    // a reference to the speed of ESP32 timers
		#define F_STEPPER_TIMER 20000000  // frequency of step pulse timer
		#define STEPPER_OFF_TIMER_PRESCALE 8 // gives a frequency of 10MHz
		#define STEPPER_OFF_PERIOD_uSEC  3  // each tick is
		
		#define STEP_PULSE_MIN 2   // uSeconds
		#define STEP_PULSE_MAX 10  // uSeconds
		
		// =============== Don't change or comment these out ======================
		// They are for legacy purposes and will not affect your I/O 
		
		#define X_STEP_BIT    0  // don't change
		#define Y_STEP_BIT    1  // don't change
		#define Z_STEP_BIT    2  // don't change
		#define A_STEP_BIT    3  // don't change
		#define STEP_MASK       B1111 // don't change
		
		#define X_DIRECTION_BIT   0 // don't change
		#define Y_DIRECTION_BIT   1  // don't change
		#define Z_DIRECTION_BIT   2  // don't change
		#define A_DIRECTION_BIT   3  // don't change

		#define X_LIMIT_BIT      	0  // don't change
		#define Y_LIMIT_BIT      	1  // don't change
		#define Z_LIMIT_BIT     	2  // don't change
		#define Y_B_LIMIT_BIT     	3  // don't change
		
		
		#define PROBE_MASK        1 // don't change		
		
		#define CONTROL_MASK      				B1111  	// don't change
		
		// =======================================================================
		
		
#endif
