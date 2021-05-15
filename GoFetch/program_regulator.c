#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <process_image.h>
#include <leds.h>
#include <movements.h>
#include <sensors/proximity.h>

#define SLOWSPEED 		220
#define NORMALSPEED 	500
#define TOLERANCE_FOR_ALIGNEMENT 7
#define MIN_DISTANCE_TO_TARGET 20
#define WAIT_TIME 		100

#define PIover4			323 //steps for a quarter rotation
#define PI				646 //steps for a half rotation
#define TWOPI				1292//steps for a full rotation


#define TEST4
//#define PROJECT



//Definition of static variables for definition of the Field;

static struct Field{
	int xpos_rob;
	int ypos_rob;
	int xpos_target;
	int ypos_target;
	int robot_angle;
} field = {0,0,0,0,0};




void update_field(void);

static THD_WORKING_AREA(waprogramRegulator, 512);
static THD_FUNCTION(programRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


#ifdef PROJECT
	static int robot_angle = 0;
	static uint8_t searching = TRUE;
	static uint8_t aligned   = FALSE;
	static enum process {SearchAndAlign, GoToTarget, AroundTarget, Shoot} process = SearchAndAlign;
#endif

    while(1){

#ifdef PROJECT

    	//constantly searching for if the target is in sight =========
    	searching = !found_lost_target(searching);

    				if(searching){
    					set_body_led(1);
    				} else {
    					set_body_led(0);
    				}

    	//Switch for the different robot modes


    	switch(process){

    	//======================================================================================
    	case SearchAndAlign:
    		reset_step_count();
    		if(searching){
    			rotate(MODE_INFINITE, SLOWSPEED, 0);
			}else if(!aligned){
				aligned = P_align(!searching, get_angle_to_target(), TOLERANCE_FOR_ALIGNEMENT);
			}else{
				process = GoToTarget;
				halt();
				robot_angle += right_motor_get_pos();
			}

    					if(aligned){
    						set_led(LED1,1);
    					} else {
    						set_led(LED1,0);
    					}

    	  break;

    	//======================================================================================
    	case GoToTarget:

//    		if(get_prox(1)<MIN_DISTANCE_TO_TARGET){
//    			halt();
//    			set_front_led(1);
//    		}
    	  break;

    	//======================================================================================
    	case AroundTarget:
    	  break;

    	//======================================================================================
    	case Shoot:
    	  break;
    	}

#endif


#ifdef TEST1
    //test 1
    	static uint8_t test = 7;

    	switch(test){
    	case 0:
    		forwards(MODE_INFINITE, 500, 0);
    		test++;
    		break;
    	case 1:
    		forwards(MODE_INFINITE, 250, 0);
    		test++;
    		break;
    	case 2:
    		test++;
    		break;
    	case 3:
    		halt();
			test++;
			break;
		case 4:
			halt();
			test++;
			break;
		case 5:
			rotate(MODE_INFINITE, 400, 0);
			test++;
			break;
		case 6:
			rotate(MODE_INFINITE, -400, 0);
			test++;
			break;
		case 7:
					set_led(LED1,1);
					set_front_led(0);
					set_led(LED5,0);
			revolve(MODE_INFINITE, 400, 1000, TRIGONOMETRIC, 0);
			test++;
			break;
		case 8:
					set_led(LED1,0);
					set_front_led(1);
					set_led(LED5,0);
			revolve(MODE_INFINITE, -400, 1000, TRIGONOMETRIC, 0);
			test++;
			break;
		case 9:
					set_led(LED1,0);
					set_front_led(0);
					set_led(LED5,1);
			revolve(MODE_INFINITE, 400, 1000, CLOCK, 0);
			test++;
			break;
		case 10:
			halt();
			test = 7;
			break;
		case 11:
			break;
    	}
#endif

#ifdef TEST2
   //Test 2
    	static uint8_t test = 0;
    	static uint8_t changing_mode = 1;

		switch(test){
		case 0:
			if(!get_ongoing_state() && changing_mode){
				reset_step_count();
				changing_mode = FALSE;
			}
			else if(!get_ongoing_state() && !changing_mode){
				test++;
				changing_mode = TRUE; //this line of code is to avoid need of semaphore in the motors library
			}
					set_front_led(1);
			if(forwards(MODE_FINITE, -500, 5000)){
				test++;
				changing_mode = TRUE;
			}
					set_led(LED5,1);
			break;

		case 1:
			if(!get_ongoing_state() && changing_mode){
				reset_step_count();
				changing_mode = FALSE;
			}
			else if(!get_ongoing_state() && !changing_mode){
				test++;
				changing_mode = TRUE; //this line of code is to avoid need of semaphore in the motors library
			}
			if(rotate(MODE_FINITE, 500, 2000)){
				test++;
				changing_mode = TRUE;
			}
						break;
			break;

		case 2:
			if(!get_ongoing_state() && changing_mode){
				reset_step_count();
				changing_mode = FALSE;
			}
			else if(!get_ongoing_state() && !changing_mode){
				test++;
				changing_mode = TRUE; //this line of code is to avoid need of semaphore in the motors library
			}
			if(rotate(MODE_FINITE, -500, 2000)){
				test++;
				changing_mode = TRUE;
			}
						break;
			break;
		}
#endif

#ifdef TEST3
	//Test 3

		static uint8_t test = 0;
		switch(test){
		case 0:
			if(translational_movement(RIGHT, 2000, 400)){
				test++;
			}
			break;
		case 1:
			halt();
			break;
		}
#endif

        chThdSleepMilliseconds(WAIT_TIME);
    }
}

void program_regulator_start(void){
	chThdCreateStatic(waprogramRegulator, sizeof(waprogramRegulator), NORMALPRIO, programRegulator, NULL);
}
