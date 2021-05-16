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

#define SLOWSPEED 		250
#define NORMALSPEED 	500
#define FASTSPEED 		800
#define MIN_ACCEPTABLE_DISTANCE 50
#define MAX_ACCEPTABLE_DISTANCE	55
#define WAIT_TIME 		75
#define RADIUS_INCREASE 1000
#define RADIUS_ROBOT 	205 //unity: steps (0.13mm)
#define RADIUS_TARGET 	100

#define PIover2			323 //steps for a quarter rotation
#define PISteps				646 //steps for a half rotation
#define TWOPI				1292//steps for a full rotation


#define TEST5
//#define PROJECT



//Definition of static variables for definition of the Field;

//static struct Field{
//	int xpos_rob;
//	int ypos_rob;
//	int xpos_target;
//	int ypos_target;
//	int robot_angle;
//} field = {0,0,0,0,0};




void update_field(void);

static THD_WORKING_AREA(waprogramRegulator, 2048);
static THD_FUNCTION(programRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;



#ifdef PROJECT
	int robot_pos[3] = {0,0,0};
	uint8_t changing_mode = TRUE, aligning = FALSE;
	uint32_t radius = 0;
	int alignement_angle;
	uint32_t remaining_steps = 0;
	int temp;
	static enum process {SearchAndAlign, GoToTarget, AroundTarget, Shoot} process = SearchAndAlign;
	static enum searching_mode{Rotating, Expanding, Revolving} search_mode = Rotating;
#endif

    while(1){

#ifdef PROJECT



    	switch(process){

    	//======================================================================================
    	//ALL THE SEARCH AND ALIGN FUNCTIONALITIES ARE IMPLEMENTED HERE
    	case SearchAndAlign:

    		if(get_searching()){
    			set_front_led(0);

				//if search starts while the robot was aligning, update the angle
				if (aligning)	{
					changing_mode = FALSE;
					temp = (right_motor_get_pos()>0)? -1:1;
					if(verify_done_moving(MODE_FINITE, temp*SLOWSPEED, right_motor_get_pos(), rotate)){
						changing_mode = TRUE;
					}
				}


				//rotating
				else if(search_mode==Rotating){
					changing_mode = FALSE;
					if(verify_done_moving(MODE_FINITE, SLOWSPEED, TWOPI, rotate)){
						search_mode = Expanding;
						changing_mode = TRUE;
					}
				}

				//expanding
				else if(search_mode==Expanding){
					changing_mode = FALSE;
					if(translational_movement(RIGHT, RADIUS_INCREASE, SLOWSPEED)){
						search_mode = Revolving;
						changing_mode = TRUE;
						radius += RADIUS_INCREASE;
//						robot_pos[X] += calculate_new_x(robot_pos[X], robot_pos[ANGLE], RADIUS_INCREASE);
//						robot_pos[Y] += calculate_new_y(robot_pos[Y], robot_pos[ANGLE], RADIUS_INCREASE);
					}
				}

				//Revolving
				else if (search_mode==Revolving){
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

				//catches the function if it stops working
				else if(aligning){
					aligning = FALSE;
				}



    		} else {



				//calculations if movement has been interrupted by alignement
				if(!get_aligned() && !aligning && !changing_mode){
//					chSysLock();
					halt();
					changing_mode = FALSE;
//					switch(search_mode){
//
//
//					case(Rotating):
//						robot_pos[ANGLE] = calculate_new_angle(left_motor_get_pos(),right_motor_get_pos(), robot_pos[ANGLE]);
//					break;
//
//
//					case(Expanding):
//						switch(get_state()){
//						case FIRSTSTATE:
//							robot_pos[ANGLE] = calculate_new_angle(left_motor_get_pos(),right_motor_get_pos(), robot_pos[ANGLE]);
//						break;
//						case SECONDSTATE:
//							robot_pos[ANGLE] = calculate_new_angle(0, get_temp_angle(), robot_pos[ANGLE]);
//							robot_pos[X]	 = calculate_new_x(robot_pos[X], robot_pos[ANGLE], right_motor_get_pos());
//							robot_pos[Y]	 = calculate_new_y(robot_pos[Y], robot_pos[ANGLE], right_motor_get_pos());
//						break;
//						case THIRDSTATE:
//							robot_pos[ANGLE] = calculate_new_angle(0, get_temp_angle() + right_motor_get_pos(), robot_pos[ANGLE]);
//							robot_pos[X]	 = calculate_new_x(robot_pos[X], robot_pos[ANGLE]+get_temp_angle(), abs(get_temp_ypos()));
//							robot_pos[Y]	 = calculate_new_y(robot_pos[Y], robot_pos[ANGLE]+get_temp_angle(), abs(get_temp_ypos()));
//						break;
//						}
//					break;
//
//
//					case(Revolving):
//						robot_pos[ANGLE] 	 = calculate_new_angle(left_motor_get_pos(), right_motor_get_pos(), robot_pos[ANGLE]);
//	//					robot_pos[X]		 = calculate_new_x(robot_pos[X], (PISteps-robot_pos[ANGLE])/2,
//	//PAS FINI
//					break;
//
//					default: break;
//
//					}
					aligning = TRUE;
					reset_step_count();
//					chSysUnlock();
				}

				//Aligning
				else if(!get_aligned()){
					aligning = TRUE;
					P_align(!get_searching(), get_angle_to_target(), TOLERANCE_FOR_ALIGNEMENT_ROBOT);
				}

				//Calculating if aligning was finished
				else{
//					robot_pos[ANGLE] = calculate_new_angle(left_motor_get_pos(), right_motor_get_pos(), robot_pos[ANGLE]);
					changing_mode = TRUE;
					aligning = FALSE;
					process = GoToTarget;
					halt();
				}
    		}


    	break;


    	//======================================================================================
    	case GoToTarget:

    		if(!get_aligned() || get_searching()) {
				halt();
				process = SearchAndAlign;
				search_mode = Rotating;
			}

			else {

				//if the robot is far enough, it can go fast
				if(VL53L0X_get_dist_mm()>MIN_ACCEPTABLE_DISTANCE+MAX_ACCEPTABLE_DISTANCE){
					forwards(MODE_INFINITE, FASTSPEED, 0);
					set_front_led(0);
				}

				//if robot is getting near it goes to a slow speed
				if(VL53L0X_get_dist_mm()>MAX_ACCEPTABLE_DISTANCE){
					forwards(MODE_INFINITE, SLOWSPEED, 0);
					set_front_led(1);
				}

				//if the robot is too near it goes back
				else if (VL53L0X_get_dist_mm()<MIN_ACCEPTABLE_DISTANCE){
					forwards(MODE_INFINITE, -SLOWSPEED, 0);
					set_front_led(1);
				}

				//if the robot is just in the right range
				else {
					halt();
					set_front_led(0);
				}
			}

    	  break;
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
		static uint8_t changing_mode = 0;
		static int distance = 0;
		static int radius;

		switch(test){
		case 0:
			if(get_searching()){
				rotate(MODE_INFINITE, SLOWSPEED, 0);
			} else {
				halt();
				test++;
			}
		  break;
		case 1:
			if(get_searching()){
				test = 0;
				rotate(MODE_INFINITE, SLOWSPEED, 0);
			}
			else if(!get_aligned()){
				P_align(!get_aligned(), get_angle_to_target(), TOLERANCE_FOR_ALIGNEMENT_ROBOT);
			}
			else{
				halt();
				test++;
				changing_mode = TRUE;
			}
		  break;
		case 2:
			if(changing_mode){
				reset_step_count();
			}

			if(get_searching()){
				test = 0;
			} else if (!get_aligned()) {
				test = 1;
			} else if(VL53L0X_get_dist_mm()>MAX_ACCEPTABLE_DISTANCE+MIN_ACCEPTABLE_DISTANCE){
				set_front_led(1);
				set_body_led(0);
				forwards(MODE_INFINITE, NORMALSPEED, 0);
			} else if(VL53L0X_get_dist_mm()>MAX_ACCEPTABLE_DISTANCE){
				set_front_led(1);
				set_body_led(0);
				forwards(MODE_INFINITE, NORMALSPEED, 0);
			} else if (VL53L0X_get_dist_mm()<MIN_ACCEPTABLE_DISTANCE){
				set_front_led(1);
				set_body_led(0);
				forwards(MODE_INFINITE, -SLOWSPEED, 0);
			} else {
				changing_mode=TRUE;
				set_front_led(0);
				set_body_led(1);
				test++;
				halt();
				distance+=right_motor_get_pos();
			}
		  break;
		case 3:
			if(verify_done_moving(MODE_FINITE, NORMALSPEED, PIover2, rotate)){
				test++;
			}
		  break;
		case 4:
			if(!get_ongoing_state() && changing_mode){
				reset_step_count();
				changing_mode = FALSE;
				radius = VL53L0X_get_dist_mm();
				radius = radius * 1000/12.9;
				radius += RADIUS_ROBOT + RADIUS_TARGET;
			}
			else if(!get_ongoing_state() && !changing_mode){
				test++;
				changing_mode = TRUE; //this line of code is to avoid need of semaphore in the motors library
			}
					set_front_led(1);
			if(revolve(MODE_FINITE, NORMALSPEED, radius, CLOCK, 3.1416*radius)){
				test++;
				changing_mode = TRUE;
				distance = 2*radius + distance;
			}
		  break;
		case 5:
			if(verify_done_moving(MODE_FINITE, -NORMALSPEED, PIover2, rotate)){
				test++;
			}
		  break;
		case 6:
			if(verify_done_moving(MODE_FINITE, NORMALSPEED, distance-RADIUS_ROBOT-RADIUS_TARGET, forwards)){
				test++;
			}
		  break;
		case 7:
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
