#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <leds.h>
#include <motor_process.h>
#include <process_image.h>

//distance between the wheels
#define DIST_WHEELS 411 //unity: steps (0.129mm)
#define INCREASE_RADIUS 2500
//parameters for the p controller
#define KP 2
#define ROT_ERR_THRESHOLD 2

enum motor_mode{Stop, TurnAround, IncreaseRadius, DoNothing_, CurrentlyMoving, FinishedMoving, Forward_, GetAround, PRegulator};

//radius of the searching circle
static int16_t position_radius = 0; //unity step
//mode of the thread
static enum motor_mode currentState = DoNothing_;


/**
* @brief   reset to 0 the step counter in motor.c for both motors
*
*/
void reset_number_step(void);

/**
* @brief   increase the searching radius by a certain distance (INCREASE_RADIUS)
*
*/
void increase_radius(void);

/**
* @brief   robot makes a full 360° turn around a point (given by position_radius)
*
*/
void turn_around(void);

/**
* @brief   turn the robot in a certain direction
*
* @param	dir		1 = RIGHT, -1 = LEFT
*/
void turn(Direction dir);

/**
* @brief   p_regulator to align the robot and the object
*
*/
void p_regulator(void);

/**
* @brief   stop the motor without changing the currentState
*
*/
void motor_halt(void);

static THD_WORKING_AREA(waMotorRegulator, 256);
static THD_FUNCTION(MotorRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();

        switch (currentState){
			case Stop:
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				continue;

			case TurnAround:
				//check if IncreaseRadius has finished to start turning
				if (get_ongoing_state()!=1){
				turn_around();
				}
				continue;

			case IncreaseRadius:
				//check if TurnAround has finished to start increase the radius
				if (get_ongoing_state()!=1){
					increase_radius();
				}
				continue;

			case DoNothing_: //faut laisser ça?
				continue;

			case CurrentlyMoving:
				//check if the motor is still moving
				if (get_ongoing_state()!=1){
					currentState=FinishedMoving;
				}
				continue;

			case GetAround:
				//get around the object then face the center
				if (get_ongoing_state()!=1){
					get_around(0,0);
				}
				continue;

			case PRegulator:
				p_regulator();
				continue;

			default:
				;
        }
        //20Hz
        chThdSleepUntilWindowed(time, time + MS2ST(50)); //avant 10
    }
}

//turn clock-wise
void rotate_angle(uint16_t angle_to_complete) {

	currentState = DoNothing_; //au cas où il était en stop

	reset_number_step();

	left_motor_set_speed_step(NORMAL_SPEED, DIST_WHEELS/2*angle_to_complete/RAD_TO_MILIRAD);
	right_motor_set_speed_step(-NORMAL_SPEED, -DIST_WHEELS/2*angle_to_complete/RAD_TO_MILIRAD);

	currentState = CurrentlyMoving;
}

//turn the robot in the given direction (HALT(0) RIGHT(1) or LEFT(-1))
void turn(Direction dir){
	reset_number_step();
	left_motor_set_speed_step(dir*NORMAL_SPEED, dir*QUARTER_TURN);
	right_motor_set_speed_step(-dir*NORMAL_SPEED, -dir*QUARTER_TURN);
}



void increase_radius(void){
	//turn right, move forward and then turn left to increase the radius
	static uint8_t state=0;
	switch(state){
		case 0:
			turn(_RIGHT);
			state++;
			break;
		case 1:
			forward_nb_steps(INCREASE_RADIUS);
			position_radius+=INCREASE_RADIUS;
			state++;
			break;
		case 2:
			turn(_LEFT);
			state=0;
			currentState=TurnAround;
			break;
		default:
			state=0;
	}
}

void turn_around(void){

	//makes a full turn around the current radius
	revolve_around(2*RAD_TO_MILIRAD*PI,position_radius);

	currentState = IncreaseRadius;
}

void revolve_around(Angle angle_to_revolve, uint16_t radius_of_revolution){

	int16_t speed_left = -SLOW_SPEED;
	int16_t speed_right = SLOW_SPEED;

	if (radius_of_revolution!=0){
		speed_left = ((radius_of_revolution - DIST_WHEELS/2)*NORMAL_SPEED)/(radius_of_revolution);
		speed_right = ((radius_of_revolution+ DIST_WHEELS/2)*NORMAL_SPEED)/(radius_of_revolution);
	}

	int32_t left_distance = (radius_of_revolution-DIST_WHEELS/2)*angle_to_revolve/RAD_TO_MILIRAD;
	int32_t right_distance = (radius_of_revolution+DIST_WHEELS/2)*angle_to_revolve/RAD_TO_MILIRAD;

	reset_number_step();

	left_motor_set_speed_step(speed_left, left_distance);
	right_motor_set_speed_step(speed_right, right_distance);
}

void reset_number_step(void){
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}

void forward_nb_steps(int32_t steps_to_complete){

	reset_number_step();

	//avoid changing state when IncreaseRadius calls this function
	if (currentState!=IncreaseRadius){
		currentState = CurrentlyMoving;
	}

	if (steps_to_complete<0){
		//move backward
		left_motor_set_speed_step(-NORMAL_SPEED, steps_to_complete);
		right_motor_set_speed_step(-NORMAL_SPEED, steps_to_complete);
	} else {
		//move forward
		left_motor_set_speed_step(NORMAL_SPEED, steps_to_complete);
		right_motor_set_speed_step(NORMAL_SPEED, steps_to_complete);
	}
}

void motor_regulator_start(void){
	chThdCreateStatic(waMotorRegulator, sizeof(waMotorRegulator), NORMALPRIO, MotorRegulator, NULL);
}

void motor_search_ball(void){

	//check if it is already in search mode
	if (currentState != TurnAround && currentState != IncreaseRadius){
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		//start searching
		currentState = TurnAround;
	}
}

int16_t motor_stop(void){
	//to calculate the position according to the state it was in
	enum motor_mode ancientState = currentState;
	currentState = Stop;
	switch (ancientState){
		case Forward_:
			return left_motor_get_pos();
			break;
		default:
			return 0;
	}
}


uint8_t finished_moving(void){
	//one time read
	if (currentState == FinishedMoving){
		currentState = DoNothing_;
		return 1;
	}
	return 0;
}

void forward(Direction dir, uint16_t speed){

	if (currentState != Forward_){
		currentState = Forward_;
		//so we can count the steps moved by the robot
		reset_number_step();
	}

	if(dir==-1 || dir==1){
		left_motor_set_speed(speed*dir);
		right_motor_set_speed(speed*dir);
	} else {
		//in case false argument given (f.ex. 10)
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
}

int16_t get_radius(void){
	return position_radius;
}

void get_around(Angle angle_to_revolve, uint16_t radius_of_revolution){
	static uint8_t ar_state = 0;
	static Angle ar_angle = 0;
	static uint16_t ar_radius = 0;

	if (currentState != GetAround){
		//first time the function is called -> stocks the values
		currentState = GetAround;
		ar_radius = radius_of_revolution;
		ar_angle = angle_to_revolve;
		ar_state=0;
	}
	//turn right -> revolve -> turn left to be aligned with the object and facing the center
	switch(ar_state){
		case 0:
			turn(_RIGHT);
			ar_state++;
			break;
		case 1:
			revolve_around(ar_angle, ar_radius);
			ar_state++;
			break;
		case 2:
			turn(_LEFT);
			ar_state++;
			break;
		case 3:
			currentState=FinishedMoving;
			break;
		default:
			;
	}
}

void set_p_regulator(void){

	if (currentState!=PRegulator && currentState != FinishedMoving){
		//count the number of step to have the angle
		reset_number_step();
		currentState = PRegulator;
	}
}

void p_regulator(void){
	int err = get_angle_to_target();

	if (err<ROT_ERR_THRESHOLD && err>-ROT_ERR_THRESHOLD){
		//aligned enough
		//stops the motor without changing the currentState
		motor_halt();
		currentState = FinishedMoving;
		set_front_led(1);
		return;
	}

	//p controller
	right_motor_set_speed(err*KP);
	left_motor_set_speed(-err*KP);
}

void motor_halt(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

int16_t get_angle(void){
	return left_motor_get_pos();

}
