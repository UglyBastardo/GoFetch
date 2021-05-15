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

#define SLOWSPEED 		150
#define NORMALSPEED 	500


//#define TEST3
#define PROJECT

static THD_WORKING_AREA(waprogramRegulator, 512);
static THD_FUNCTION(programRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    static uint8_t rotating = FALSE, do_not_search = 0;
    while(1){
#ifdef PROJECT


    	if(get_searching()){
    		rotating = TRUE;
    		rotate(MODE_INFINITE, 150, 0);
    	}
    	else if(!get_aligned()){
    		P_align(rotating, get_angle_to_target());
    	}
    	else{
    		halt();
    	}


#endif


#ifdef TEST1
    //test 1
    	static uint8_t test = 0;

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
			revolve(MODE_INFINITE, 400, 1000, TRIGONOMETRIC, 0);
			test++;
			break;
		case 8:
			revolve(MODE_INFINITE, -400, 1000, TRIGONOMETRIC, 0);
			test++;
			break;
		case 9:
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

		switch(test){
		case 0:
			forwards(MODE_FINITE, 500, 500);
			test++;
			break;
		case 1:
			rotate(MODE_FINITE, 500, 400);
			test++;
			break;
		case 2:
			revolve(MODE_FINITE, 500, 500, TRIGONOMETRIC, 500);
			test = 0;
			break;
		}
#endif

#ifdef TEST3
	//Test 3

		static uint8_t test = 0;
		switch(test){
		case 0:
			translational_movement(RIGHT, 500, 400);
			test++;
			break;
		case 2:
			halt();
			test--;
			break;
		}
#endif

        chThdSleepMilliseconds(3000);
    }
}

void program_regulator_start(void){
	chThdCreateStatic(waprogramRegulator, sizeof(waprogramRegulator), NORMALPRIO, programRegulator, NULL);
}
