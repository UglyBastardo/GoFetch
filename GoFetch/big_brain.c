#include "ch.h"
#include "hal.h"
#include <math.h>

#include <main.h>
#include <motors.h>
#include <process_image.h>
#include <big_brain.h>
#include <leds.h>
#include <motor_process.h>
#include <tof_process.h>

enum Process_Mode {DoNothing, RotateAndSearch, Align, Revolve, Forward, ReAlign, Shoot, Victory};

#define MIN_DISTANCE 		60 		//mm
#define MAX_DISTANCE		140		//mm
#define SMALL_DISTANCE		100 	//steps
#define RADIUS_OBJECT		15 		//steps

/**
* @brief   calculate the distance needed for the robot to travel back to the center
*
* @param	distance_moved_ 	distance moved towards the objet (from the searching circle) (in step)
* 			angle 				rotation of the robot to be aligned with the object (in step)
*
* @return distance (in step)
*/
uint32_t calculate_distance(int16_t distance_moved_, int16_t angle);

/**
* @brief   calculate the angle needed for the robot get behind the object and facing the center
*
* @param	distance_moved_ 	distance moved towards the objet (from the searching circle) (in step)
* 			angle 				rotation of the robot to be aligned with the object (in step)
*
* @return angle (in mRad)
*/
Angle calculate_revolution(int16_t distance_moved_, int16_t angle);



uint32_t calculate_distance(int16_t distance_moved_, int16_t angle){

	//distance including the distance between the object and the robot
	int16_t distance_moved = distance_moved_+get_tof_distance()*MM_TO_STEP;
	int16_t radius = get_radius();
	double alpha = angle * STEPS_TO_RAD;

	//calculation of the distance with trigonometry (Taylor development of sin)
	uint32_t dist = distance_moved*distance_moved + radius*radius - 2*distance_moved*radius*(-alpha + (alpha*alpha*alpha)/6);
	dist=sqrt(dist);

	//distance between the robot and the object + between the object and the center
	dist += get_tof_distance()*MM_TO_STEP;

	return dist;
}

Angle calculate_revolution(int16_t distance_moved_, int16_t angle){

	//distance including the distance between the object and the robot
	int16_t distance_moved = distance_moved_ + get_tof_distance()*MM_TO_STEP;
	int16_t radius = get_radius();

	double alpha = (double)angle;
	alpha *= STEPS_TO_RAD;

	//calculation of the distance between object and center with trigonometry (Taylor development of sin)
	double dist = distance_moved*distance_moved + radius*radius - 2*distance_moved*radius*(-alpha + (alpha*alpha*alpha)/6);
	dist = sqrt(dist);

	//developpement Taylor PI - arcos(x)
	double x = (dist*dist + distance_moved*distance_moved - radius*radius)/(2*distance_moved*dist);
//	double rotation_angle = PI/2 + x + x*x*x/6;
	double rotation_angle = PI - acos(x);

	//converts angle to MILIRAD
	rotation_angle *= RAD_TO_MILIRAD;

	//converts double to int
	Angle angle_to_align = ceil(rotation_angle);

	return angle_to_align;
}


//main thread
static THD_WORKING_AREA(waBigBrain, 1024);
static THD_FUNCTION(BigBrain, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    static int16_t angle = 0;
    static int16_t distance = 0;

    //chose the actions to do next
    static enum Process_Mode mode = RotateAndSearch;

    while(1)
    {
      switch(mode)
      {
      //===============================================================================================================
        case DoNothing:
          continue;

      //===============================================================================================================
        case RotateAndSearch:
        	motor_search_ball();
        	set_front_led(1);
        	if(target_found()){
        		motor_stop();
        		set_front_led(0);
        		mode = Align;
        	}
          continue;

      //===============================================================================================================
        case Align:

        	// is true once the p_regulator has finished
        	if(target_found() && finished_moving()){
        		//gets angle in steps from the P-controller
        		angle = get_angle();
        		mode = Forward;
        	} else if (target_found()==1) {
        		//lauch the P regulator to align the robot and object
        		set_p_regulator();
        	} else {
        		//object lost -> begin searching again
        		mode = RotateAndSearch;
        	}
          continue;

	  //===============================================================================================================
		case Forward:

			//use the ToF detector to be at a good distance from the object
			if (get_tof_distance()>MAX_DISTANCE){
				forward(_FORWARD, SLOW_SPEED);
			} else if (get_tof_distance()<MIN_DISTANCE){
				forward(_BACKWARD, SLOW_SPEED);
			} else {
				//stop the motor and get the distance traveled towards the object
				distance = motor_stop();
				mode = Revolve;
			}
		  continue;

      //===============================================================================================================
        case Revolve:

        	//This part uses the distance to the target and positions to decide on the sequence to follow to get behind it.
        	if(target_found()==1){
        		get_around(calculate_revolution(distance, angle), get_tof_distance()*MM_TO_STEP);
        		mode = ReAlign;
        	} else {
        		mode = RotateAndSearch;
        	}
          continue;

	  //===============================================================================================================
		//revoir le nom?
		case ReAlign:
			//robot has finished revolvig around, now aligning again to push the object
			if(target_found() && finished_moving()){
				set_p_regulator();
				mode = Shoot;
			} else if (finished_moving()){
				//in case the robot doesn't see the object because too close
				forward_nb_steps(-SMALL_DISTANCE);
			}
		  continue;

      //===============================================================================================================

	  //push the object towards the center
        case Shoot:
        	//check if the P controller has finished -> starts moving
        	if (finished_moving()){
        		forward_nb_steps(calculate_distance(distance, angle)-RADIUS_OBJECT);
        		mode = Victory;
        	}
        	continue;

        case Victory:
        	if (finished_moving()){
        		//small dance
        		set_body_led(1);
        		rotate_angle(8*PI*RAD_TO_MILIRAD);
        		mode = DoNothing;
        	}
        	//scream();
          continue;
      }
      chThdSleepMilliseconds(100);
    }
}

void big_brain_start(void){
	chThdCreateStatic(waBigBrain, sizeof(waBigBrain), NORMALPRIO, BigBrain, NULL);
}
