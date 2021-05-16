#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <process_image.h>
#include <leds.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <movements.h>
#include <sensors/proximity.h>

#define SLOWSPEED 		160
#define NORMALSPEED 	500
#define FASTSPEED 		800
#define MIN_DISTANCE_TO_TARGET 20
#define WAIT_TIME 		78
#define RADIUS_INCREASE 1000

#define PIover2			323 //steps for a quarter rotation
#define PISteps				646 //steps for a half rotation
#define TWOPI				1292//steps for a full rotation


//#define TEST5
#define PROJECT



//Definition of static variables for definition of the Field;

//static struct Field{
//	int xpos_rob;
//	int ypos_rob;
//	int xpos_target;
//	int ypos_target;
//	int robot_angle;
//} field = {0,0,0,0,0};




void update_field(void);

static THD_WORKING_AREA(waprogramRegulator, 1024);
static THD_FUNCTION(programRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


#ifdef PROJECT
	static int robot_pos[3] = {0,0,0};
	static uint8_t changing_mode = TRUE;
	static uint32_t radius = 0;
	static int alignement_angle = 0;
	static enum process {SearchAndAlign, GoToTarget, AroundTarget, Shoot} process = SearchAndAlign;
	static enum searching_mode{Rotating, Expanding, Revolving} search_mode = Rotating;
#endif

    while(1){

#ifdef PROJECT



    	switch(process){

    	//======================================================================================
    	case SearchAndAlign:


    		//rotating
    		if(get_searching() &&  search_mode==Rotating){
    			if(verify_done_moving(MODE_FINITE, SLOWSPEED, TWOPI, rotate)){
    				search_mode = Expanding;
    				changing_mode = TRUE;
    			}
    		}

    		//expanding
    		else if(get_searching()&& search_mode==Expanding){
    			if(translational_movement(RIGHT, RADIUS_INCREASE, NORMALSPEED)){
    				search_mode = Revolving;
    				changing_mode = TRUE;
    				radius += RADIUS_INCREASE;
    				robot_pos[X] += calculate_new_x(robot_pos[X], robot_pos[ANGLE], RADIUS_INCREASE);
    				robot_pos[Y] += calculate_new_y(robot_pos[Y], robot_pos[ANGLE], RADIUS_INCREASE);
    			}
    		}

    		//Revolving
    		else if (get_searching() && search_mode==Revolving){
    			if(!get_ongoing_state() && changing_mode){
					reset_step_count();
					changing_mode = FALSE;
				}
				else if(!get_ongoing_state() && !changing_mode){
					changing_mode = TRUE;
					search_mode = Expanding;
				}
				if(revolve(MODE_FINITE, FASTSPEED, radius, TRIGONOMETRIC, 2*3.1416*radius)){
					changing_mode = TRUE;
    				search_mode = Expanding;
    				set_front_led(1);
				}
    		}

    		//In case a process was interrupted, we can get back in.
    		else if (get_searching() && !changing_mode)	changing_mode = TRUE;


    		//calculating if process has been interrupted
    		else if(!get_aligned() && changing_mode){
				halt();
				changing_mode = FALSE;
				switch(search_mode){


				case(Rotating):
					robot_pos[ANGLE] = calculate_new_angle(left_motor_get_pos(),right_motor_get_pos(), robot_pos[ANGLE]);
				break;


				case(Expanding):
					switch(get_state()){
					case FIRSTSTATE:
						robot_pos[ANGLE] = calculate_new_angle(left_motor_get_pos(),right_motor_get_pos(), robot_pos[ANGLE]);
					break;
					case SECONDSTATE:
						robot_pos[ANGLE] = calculate_new_angle(0, get_temp_angle(), robot_pos[ANGLE]);
						robot_pos[X]	 = calculate_new_x(robot_pos[X], robot_pos[ANGLE], right_motor_get_pos());
						robot_pos[Y]	 = calculate_new_y(robot_pos[Y], robot_pos[ANGLE], right_motor_get_pos());
					break;
					case THIRDSTATE:
						robot_pos[ANGLE] = calculate_new_angle(0, get_temp_angle() + right_motor_get_pos(), robot_pos[ANGLE]);
						robot_pos[X]	 = calculate_new_x(robot_pos[X], robot_pos[ANGLE]+get_temp_angle(), abs(get_temp_ypos()));
						robot_pos[Y]	 = calculate_new_y(robot_pos[Y], robot_pos[ANGLE]+get_temp_angle(), abs(get_temp_ypos()));
					break;
					}
				break;


				case(Revolving):
					robot_pos[ANGLE] 	 = calculate_new_angle(left_motor_get_pos(), right_motor_get_pos(), robot_pos[ANGLE]);
//					robot_pos[X]		 = calculate_new_x(robot_pos[X], (PISteps-robot_pos[ANGLE])/2,
//PAS FINI
				break;
				default: break;
				}

				changing_mode=FALSE;
				reset_step_count();
    		}

    		//Aligning
    		else if(!get_aligned()){
				P_align(!get_searching(), get_angle_to_target(), TOLERANCE_FOR_ALIGNEMENT);
			}

    		//Calculating if aligning was interrupted
    		else if(get_aligned() && !changing_mode){
				robot_pos[ANGLE] = calculate_new_angle(left_motor_get_pos(), right_motor_get_pos(), robot_pos[ANGLE]);
				changing_mode = TRUE;
			}

    		//Changing mode
    		else{
				process = GoToTarget;
				halt();
				robot_pos[ANGLE] += right_motor_get_pos();
			}


    	  break;

    	//======================================================================================
//    	case GoToTarget:
//
////    		if(get_prox(1)<MIN_DISTANCE_TO_TARGET){
////    			halt();
////    			set_front_led(1);
////    		}
//    	  break;
//
//    	//======================================================================================
//    	case AroundTarget:
//    	  break;
//
//    	//======================================================================================
//    	case Shoot:
//    	  break;
//    	}
    	default:
    		if(!get_aligned() || get_searching()) process = SearchAndAlign;
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

#ifdef TEST4

		static uint8_t test = 0;
		switch(test){
		case 0:
			test = go_to_pos(field.xpos_rob,field.ypos_rob,field.robot_angle, -1000, -1000, 0, 350);
			break;
		case 1:
			halt();
			break;
		}
#endif

#ifdef TEST5

		static uint8_t test = 0;

		static uint8_t aligned = 0;
		static uint8_t searching = 1;

//		static uint16_t robot_radius = 140, target_radius = 80;
		static uint32_t MAX_DISTANCE = 80, MIN_DISTANCE = 60;
//		static uint16_t radius = 0;
		static uint8_t changing_mode;
//		static uint8_t test2 = 0;


		searching = !found_lost_target(searching);

						if(searching){
							set_body_led(1);
						} else {
							set_body_led(0);
						}
		switch(test){
		case 0:
			if(searching){
				rotate(MODE_INFINITE, SLOWSPEED, 0);
			} else {
				halt();
				test++;
			}
		  break;
		case 1:
			if(searching){
				test = 0;
				rotate(MODE_INFINITE, SLOWSPEED, 0);
			}
			else if(!aligned){
				aligned = P_align(!aligned, get_angle_to_target(), TOLERANCE_FOR_ALIGNEMENT);
			}
			else{
				halt();
				test++;
			}
		  break;
		case 2:
			if(searching){
				test = 0;
				rotate(MODE_INFINITE, SLOWSPEED, 0);
			} else if (!aligned) {
				test = 1;
			} else {
				forwards(MODE_INFINITE, NORMALSPEED, 0);
				if (VL53L0X_get_dist_mm()<2*MAX_DISTANCE){
					test = 3;
				}
			}
		  break;
		case 3:
			if (VL53L0X_get_dist_mm()>MAX_DISTANCE){
				set_front_led(1);
				set_body_led(0);
				forwards(MODE_INFINITE, NORMALSPEED, 0);
			} else if (VL53L0X_get_dist_mm()<MIN_DISTANCE){
				set_front_led(1);
				set_body_led(0);
				forwards(MODE_INFINITE, -SLOWSPEED, 0);
			} else {
				set_front_led(0);
				set_body_led(1);
				test = 4;
				halt();
			}
		  break;
		case 4:
			if(verify_done_moving(MODE_FINITE, NORMALSPEED, PIover2, rotate)){
				test = 5;
			}
		  break;
		case 5:
			if(!get_ongoing_state() && changing_mode){
				reset_step_count();
				changing_mode = FALSE;
			}
			else if(!get_ongoing_state() && !changing_mode){
				test = 6;
				changing_mode = TRUE; //this line of code is to avoid need of semaphore in the motors library
			}
					set_front_led(1);
			if(revolve(MODE_FINITE, NORMALSPEED, 700, CLOCK, 3.1416*700)){
				test = 6;
				changing_mode = TRUE;
			}
		  break;
		case 6:
			if(verify_done_moving(MODE_FINITE, -NORMALSPEED, PIover2, rotate)){
				test = 7;
			}
		  break;
		case 7:
			if(verify_done_moving(MODE_FINITE, NORMALSPEED, 3000, forwards)){
				test = 8;
			}
		  break;
		case 8:
			rotate(MODE_INFINITE, NORMALSPEED*2, 0);
		  break;
		}
#endif


#ifdef TESTVLOX

		static uint8_t test2 = 0;
		static uint16_t robot_radius = 140, target_radius = 80;
		static uint32_t MAX_DISTANCE = 80, MIN_DISTANCE = 60;
		static uint16_t radius = 0;
		static uint8_t changing_mode;

		switch(test2){
		case 0:
		if (VL53L0X_get_dist_mm()>MAX_DISTANCE){
			set_front_led(1);
			set_body_led(0);
			forwards(MODE_INFINITE, NORMALSPEED, 0);
		} else if (VL53L0X_get_dist_mm()<MIN_DISTANCE){
			set_front_led(1);
			set_body_led(0);
			forwards(MODE_INFINITE, -SLOWSPEED, 0);
		} else {
			set_front_led(0);
			set_body_led(1);
			test2 = 1;
			halt();
		}
		break;

		case 1:
			radius = VL53L0X_get_dist_mm()*8 + target_radius+robot_radius;
			test2 =2;
		  break;
		case 2:
			if(verify_done_moving(MODE_FINITE, NORMALSPEED, PIover2, rotate)){
				test2 = 3;
			}
		  break;
		case 3:
			if(!get_ongoing_state() && changing_mode){
				reset_step_count();
				changing_mode = FALSE;
			}
			else if(!get_ongoing_state() && !changing_mode){
				test2++;
				changing_mode = TRUE; //this line of code is to avoid need of semaphore in the motors library
			}
					set_front_led(1);
			if(revolve(MODE_FINITE, NORMALSPEED, radius, CLOCK, 60*radius)){
				test2++;
				changing_mode = TRUE;
			}
		  break;
		case 4:
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
