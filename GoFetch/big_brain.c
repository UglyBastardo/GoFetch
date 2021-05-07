#include "ch.h"
#include "hal.h"
#include <math.h>

#include <main.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>
#include <process_image.h>
#include <big_brain.h>
#include <pi_regulator.h>

//enum Process_Mode {DoNothing, RotateAndSearch, Align, Revolve, Forward, Shoot, Victory};
static enum Process_Mode mode = RotateAndSearch; // /!\

#define STEPS_PER_ANGULAR_UNIT 	1
#define MIN_DISTANCE 		  	15 		//mm
#define MAX_DISTANCE			80		//mm
#define FULLROTATION 			1293 	//steps
#define OBJECT_RADIUS		 	12 		//mm

//===============================================================================
//dayan
#define MAX_DETECTION_DISTANCE 500 //mm

//regarde s'il d�tecte un objet
uint8_t target_detected(void);

//calcul l'angle � atteindre et aligne le robot
void align_robot(void) {

}

uint8_t target_detected(void){
	static uint8_t check=0; //pas s�r du nom
	if (VL53L0X_get_dist_mm()<MAX_DETECTION_DISTANCE && check==0){
		check++;
		return 0; //Object detected
	} else if (VL53L0X_get_dist_mm()<MAX_DETECTION_DISTANCE && check==1){
		set_front_led(1);
		check++;
		return 0;
	} else if (VL53L0X_get_dist_mm()<MAX_DETECTION_DISTANCE && check==2){
		set_body_led(1);
		check = 0;
		return 1;
	} else {
		check = 0;
		return 0;
	}
}

//==================================================================================
static Angle robot_angle;
static Position robot_pos = {0, 0};
static Position target_pos  = {0, 0};


static Bool ongoing = false;
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

Angle calculate_revolution(void){
	Angle angle_to_align = 0;

	//to Complete

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
uint8_t is_hit(void){
	/*
	 * To Complete: becomes true once robot_pos = target_pos
	 */
}



static BSEMAPHORE_DECL(BigBrain_sem, TRUE);


static THD_WORKING_AREA(waBigBrain, 1024);
static THD_FUNCTION(BigBrain, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    while(mode!=Victory)
    {
      switch(mode)
      {
      //===============================================================================================================
        case DoNothing:
          continue;

      //===============================================================================================================
        case RotateAndSearch:
        	motor_search_ball();
        	if(target_detected()==1)
        	{
        		motor_stop();
        		mode = Align;
        	} else {
        		//turn(_RIGHT, NORMAL_SPEED);
        	}

          continue;

      //===============================================================================================================
        case Align:

        	//First, the robot detects and aligns itself. If The target is no longer detected, it searches again.
        	if(target_detected()==1){
        		//rotate_angle(get_angle_to_target(), NORMAL_SPEED );
        		rotate_angle(get_angle_to_target());
        		mode = FinishedAligning;
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
        	/*if(target_detected()){
        		revolve_around(calculate_revolution(), distance_to_target);
        	} else {
        		mode = RotateAndSearch;
        	}
        	*/
        	set_body_led(1);
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
        	if (VL53L0X_get_dist_mm()>MAX_DISTANCE){
        		set_front_led(1);
        		forward(_FORWARD, SLOW_SPEED);
        	} else if (VL53L0X_get_dist_mm()<MIN_DISTANCE){
        		set_front_led(0);
        		forward(_BACKWARD, SLOW_SPEED);
        	} else {
        		//mode = Revolve;
        		motor_stop();
        		mode = Forward;
        	}

          continue;

      //===============================================================================================================
        case Shoot:
        	if(target_detected()) {
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
          continue;
      //===============================================================================================================
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
