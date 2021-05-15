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
#define TOLERANCE_FOR_ALIGNEMENT 7


//#define TEST3
#define PROJECT

static THD_WORKING_AREA(waprogramRegulator, 512);
static THD_FUNCTION(programRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	static uint8_t searching = TRUE;
	static uint8_t aligned   = FALSE;
	static uint32_t init_pos = 0, final_pos = 0;
	static Angle delta_angle;


    while(1){

#ifdef PROJECT



//    	init_pos = left_motor_get_pos();
		rotate(MODE_INFINITE, SLOWSPEED, 0);
		searching = !found_lost_target(searching);
		if(!searching){
			aligned = P_align(!searching, get_angle_to_target(), 7);
		}

		if(aligned){
			set_led(LED1,1);
		} else {
			set_led(LED1,0);
		}

		if(searching){
			set_body_led(1);
		} else {
			set_body_led(0);
		}

//        final_pos = left_motor_get_pos();
//        delta_angle = final_pos - init_pos;

		//Attention, ici je ne sépare par les modes...
		if(aligned){
			forwards(MODE_INFINITE, SLOWSPEED,0);
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

        chThdSleepMilliseconds(30);
    }
}

void program_regulator_start(void){
	chThdCreateStatic(waprogramRegulator, sizeof(waprogramRegulator), NORMALPRIO, programRegulator, NULL);
}
