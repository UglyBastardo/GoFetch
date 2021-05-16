#include "ch.h"
#include "hal.h"
#include <math.h>

#include <main.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>
#include <process_image.h>
#include <big_brain.h>
#include <pi_regulator.h>
#include <leds.h>

//enum Process_Mode {DoNothing, RotateAndSearch, Align, Revolve, Forward, Shoot, Victory};
static enum Process_Mode mode = RotateAndSearch; //

#define STEPS_PER_ANGULAR_UNIT 	1
#define MIN_DISTANCE 		  	60 		//mm
#define MAX_DISTANCE			140		//mm
#define FULLROTATION 			1293 	//steps
#define OBJECT_RADIUS		 	12 		//mm
#define SMALL_DISTANCE			100 //steps

//===============================================================================
//dayan
#define MAX_DETECTION_DISTANCE 500 //mm
#define CHECK_TRESHOLD 3

//regarde s'il d�tecte un objet
uint8_t target_detected(void);

//est-ce qu'on a besoin que �a soit le m�me type ou pas?
uint32_t calculate_distance(int32_t distance_moved_, int16_t angle);

Angle calculate_revolution(int32_t distance_moved_, int16_t angle);


//calcul l'angle � atteindre et aligne le robot
void align_robot(void) {

}

uint32_t calculate_distance(int32_t distance_moved_, int16_t angle){

	//m�me code & calcul que dans angle, moyen d'optimiser?
	int32_t distance_moved = distance_moved_+VL53L0X_get_dist_mm()*MM_TO_STEP;
	int16_t radius = get_radius();
	double alpha = angle * STEPS_TO_RAD; //get_angle()
	uint32_t dist = (uint32_t)sqrt(distance_moved*distance_moved + radius*radius - 2*distance_moved*radius*(-alpha + (alpha*alpha*alpha)/6));
//	uint32_t dist = sqrt(distance_moved*distance_moved + radius*radius);
	dist += VL53L0X_get_dist_mm()*MM_TO_STEP;

//	if(dist<150){
//		set_front_led(1);
//	}
	return dist;
}

uint8_t target_detected(void){
//	if (VL53L0X_get_dist_mm()<MAX_DETECTION_DISTANCE){
//		return 1;
//	}
//	return 0;

//	return line_detected();
//	static uint8_t check=0; //pas s�r du nom
//	if (VL53L0X_get_dist_mm()<MAX_DETECTION_DISTANCE && check<=CHECK_TRESHOLD){
//		check++;
//		return 0; //Object detected
//	} else if (VL53L0X_get_dist_mm()<MAX_DETECTION_DISTANCE){
////		set_front_led(1);
////		check++;
//		return 1;
//	} else {
//		check = 0;
//		return 0;
//	}

	return target_found();
}

//==================================================================================
static Angle robot_angle;
static Position robot_pos = {0, 0};
static Position target_pos  = {0, 0};


//static Bool ongoing = false;
static uint16_t distance_to_target = ZERO;

//===============================================================================================================

/**
* @brief   would be used by other modules to set the new positions of the target relative to the robot
*
* @param
*/

void set_distance_to_target(uint16_t* new_distance_to_target){
	distance_to_target = *new_distance_to_target;
}

//===============================================================================================================

/**
* @brief   Would be used to update the angular position of the robot and reset the angular counting of the steps to be done
*			***This function probably does not work unless we correct the way steps are counted:
*					Maybe use a substraction from initial steps given to the motor if steps were given and from 0 if no steps were given
* @param
*/

void update_robot_angle(void){
	robot_angle += (Angle) (left_motor_get_pos() % FULLROTATION);
	left_motor_set_pos(ZERO);
	right_motor_set_pos(ZERO);
}

//===============================================================================================================
/**
* @brief   would be used to update the robots position based on the number of steps done and the angle at which it has done them
*			This function has not been written yet!!!
* @param
*/

void update_robot_pos(void){
	robot_pos = robot_pos;
}

//===============================================================================================================
/**
* @brief   would be used to update the robots position based on the distance to the target and its angular position
*			This function has not been written yet!!!
*
* @param
*/
void update_target_pos(void){
	target_pos = target_pos;
}

//===============================================================================================================
/**
* @brief   Would be used to calculate the angle of revolution needed based on positions of robot and position of target
*
* @param
*/

Angle calculate_revolution(int32_t distance_moved_, int16_t angle){

//	double distance_moved = (double)distance_moved_;
//	distance_moved += (double)(VL53L0X_get_dist_mm()*MM_TO_STEP);
//	double radius = (double)get_radius();
	int32_t distance_moved = distance_moved_ + VL53L0X_get_dist_mm()*MM_TO_STEP;
	int16_t radius = get_radius();

	double alpha = (double)angle; //get_angle()
	alpha *= STEPS_TO_RAD;

	double dist = distance_moved*distance_moved + radius*radius - 2*distance_moved*radius*(-alpha + (alpha*alpha*alpha)/6);
	dist = sqrt(dist);
//	double dist = sqrt(distance_moved*distance_moved + radius*radius);
	double x = (dist*dist + distance_moved*distance_moved - radius*radius)/(2*distance_moved*dist);

	//developpement Taylor PI - arcos(x)
	double rotation_angle = PI/2 + x + x*x*x/6;
//	double rotation_angle = PI - acos(distance_moved/dist);
//	dist = distance_moved/dist;
//	double rotation_angle = PI;
//	double rotation_angle = 0;
	rotation_angle *= MILIRAD_TO_RAD;
//	Angle angle_to_align = rotation_angle * MILIRAD_TO_RAD;
	Angle angle_to_align = ceil(rotation_angle);
	//to Complete
//	if (angle_to_align==0){
//			set_body_led(1);
//	}
	return angle_to_align;
}

//===============================================================================================================
/**
* @brief   would be used calculate the shooting speed for optimum results
*
* @param
*/
uint16_t calculate_speed(void){
	return 0;
}

//===============================================================================================================
/**
* @brief   Not sure this would be useful, but tells the user if the robot has hit the target or not
* 			The alternative would be to simply assign a distance the Robot should do and used "forward_steps" or smth like that
*
* @param
*/
//uint8_t is_hit(void){
//	/*
//	 * To Complete: becomes true once robot_pos = target_pos
//	 */
//}



//static BSEMAPHORE_DECL(BigBrain_sem, TRUE);


static THD_WORKING_AREA(waBigBrain, 1024);
static THD_FUNCTION(BigBrain, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    static int16_t angle = 0;
    static int32_t distance = 0;

    while(mode!=Victory)
    {
      switch(mode)
      {
      //===============================================================================================================
        case DoNothing:
//        	if(target_found()){
//        		set_body_led(1);
//        		set_p_regulator();
//        	} else {
//        		set_body_led(0);
//        		set_front_led(0);
//        		motor_stop();
//        	}
          continue;

      //===============================================================================================================
        case RotateAndSearch:
        	motor_search_ball();
        	set_front_led(1);
        	if(target_detected()==1)
        	{
        		motor_stop();
        		set_front_led(0);
        		mode = Align;
        	} /*else {
        		//turn(_RIGHT, NORMAL_SPEED);
        	}*/

          continue;

      //===============================================================================================================
        case Align:

        	//First, the robot detects and aligns itself. If The target is no longer detected, it searches again.
        	if(target_detected()==1 && finished_moving()){
        		//gets angle in steps from the P-controller
        		angle = get_angle();
        		//rotate_angle(get_angle_to_target(), NORMAL_SPEED );
//        		rotate_angle(get_angle_to_target());
        		//mode = FinishedAligning;
        		mode = Forward;
        	} else if (target_detected()==1) {
        		set_p_regulator();
//        		mode = RotateAndSearch;
        	} else {
        		mode = RotateAndSearch;
        	}

        	/*
        	//Then, the robot decides what next move to do: go towards target if it is far enough and revolve around target if it is close enough
        	if(distance_to_target<MIN_DISTANCE || distance_to_target>MAX_DISTANCE){
        		mode = Forward;
        	} else {
        		mode = Revolve;
        	}

        	//This last part updates the position and angular position of the robot
        	update_robot_angle();
			*/
          continue;

        //j'ai pas trouv� plus beau comme mani�re de faire
        case FinishedAligning:
        	if (finished_moving()==1){
        		mode = Forward;
        	}
        	continue;
      //===============================================================================================================
        case Revolve:

        	//This part uses the distance to the target and positions to decide on the sequence to follow to get behind it.
        	if(target_detected()==1){
        		get_around(calculate_revolution(distance, angle), VL53L0X_get_dist_mm()*MM_TO_STEP);
        		mode = ReAlign;
        	} else {
        		mode = RotateAndSearch;
        	}

//        	motor_stop();
//        	set_body_led(1);
          continue;

      //===============================================================================================================
        case Forward:
        	/*while(VL53L0X_get_dist_mm()>MAX_DISTANCE){
        		//risque de bug non?, monopolise les threads et si d�tecte pas objet -> boucle infini jusqu'� mur
        		motors_set_ongoing(TRUE);
        		forward(_FORWARD, NORMAL_SPEED);
        	}
        	while(VL53L0X_get_dist_mm()<MIN_DISTANCE)
        	{
        		motors_set_ongoing(TRUE);
        		forward(_BACKWARD, NORMAL_SPEED);
        	}
        	motors_set_ongoing(FALSE);
        	*/
//        	if (target_detected()==1){
				if (VL53L0X_get_dist_mm()>MAX_DISTANCE){
					//set_front_led(1);
					forward(_FORWARD, SLOW_SPEED);
				} else if (VL53L0X_get_dist_mm()<MIN_DISTANCE){
					//set_front_led(0);
					forward(_BACKWARD, SLOW_SPEED);
				} else {
					//mode = Revolve;
					distance = motor_stop();
					mode = Revolve;
				}
//        	} else {
//        		mode = RotateAndSearch;
//        	}

          continue;

      //===============================================================================================================
        //revoir le nom?
        case ReAlign:
        	/*if(target_detected()) {
        		motors_set_ongoing(TRUE);
        		forward(_FORWARD, calculate_speed());
        	} else {
        		mode = RotateAndSearch;
        	}

        	while(!is_hit()){
        		;//Je sais que cette merde a peu de sens et je compte bien la changer
        	}
        	motors_set_ongoing(FALSE);
        	mode = RotateAndSearch;
        	*/
//        	set_front_led(1);
        	//mauvais code danger finished moving one time read
        	if(target_detected() && finished_moving()){
//        		forward_nb_steps(calculate_distance());
////        		forward_nb_steps(500);
//        		set_body_led(1);
//        		mode = DoNothing;
        		set_p_regulator();
        		mode = Shoot;

        	//in case the robot doesn't see the piece because too close
        	} else if (finished_moving()){
        		forward_nb_steps(-SMALL_DISTANCE);
        	}

          continue;
      //===============================================================================================================

        case Shoot:
        	if (finished_moving()){
        		forward_nb_steps(calculate_distance(distance, angle));
        		set_body_led(1);
        		mode = DoNothing;
        	}
        case Victory:
        	//scream();
          continue;
      }
      chThdSleepMilliseconds(100);
    }
}

void big_brain_start(void){
	chThdCreateStatic(waBigBrain, sizeof(waBigBrain), NORMALPRIO, BigBrain, NULL);
}
