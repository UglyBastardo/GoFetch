#include <math.h>

#include <leds.h>
#include <motors.h>
#include <movements.h>

//================================================================================
/*	Begin of the Complicated Movement Functions
 *  These need to be put into a thread
 *
 *
 */
//================================================================================
#define KP 		1
#define PIover2 323 //steps


uint8_t P_align(uint8_t ongoing, int error, int tolerance){
	if(ongoing && (error>tolerance || error<-tolerance)){
		rotate(MODE_INFINITE, KP*error, 0);
		return FALSE;
	} else {
		halt();
		return TRUE;
	}
}

void translational_movement(int side, int delta_radius, int execution_speed){
/*
static enum mode {Rotate1Start,Rotate1Ongoing, ForwardsStart, ForwardsOngoing , Rotate2Start, Rotate2Ongoing, Done} mode;
//tracking is done using motor going forward first

	while(1);
	switch(mode){
	case Rotate1Start:
		rotate(MODE_FINITE, side*execution_speed, PIover2);
		mode=Rotate1Ongoing;
		break;
	case Rotate1Ongoing:
		if(left_motor_get_pos()>)
	}
	rotate(MODE_FINITE, side*execution_speed, PIover2);
	forwards(MODE_FINITE, execution_speed, delta_radius);
	rotate(MODE_FINITE, -side*execution_speed, PIover2);
*/
}







//================================================================================
/*	Begin of the Simple Movement Functions
 *
 *	!!!!!!
 *	revolution calculation is not functional
 */
//================================================================================
#define RADIUS_ROBOT 192 //unity: steps (0.13mm)

void forwards(uint8_t mode, int speed, uint32_t nbSteps){

	switch(mode){
	  case MODE_INFINITE:
		  left_motor_set_speed(speed);
		  right_motor_set_speed(speed);
		  set_led(LED3, 0);
		break;

	  case MODE_FINITE:
		  left_motor_set_speed_step(speed, nbSteps);
		  right_motor_set_speed_step(speed, nbSteps);
		break;

	  default:
		  left_motor_set_speed(0);
		  right_motor_set_speed(0);
		break;
	}

}

void rotate(uint8_t mode, int speed, uint32_t nbSteps){

	switch(mode){
	  case MODE_INFINITE:
		  left_motor_set_speed(-speed);
		  right_motor_set_speed(speed);
	    break;

	  case MODE_FINITE:
		  left_motor_set_speed_step(-speed, nbSteps);
		  right_motor_set_speed_step(speed, nbSteps);
		break;

	  default:
		  left_motor_set_speed(0);
		  right_motor_set_speed(0);
		break;
	}

}

void revolve(uint8_t mode, int speed, int radius, uint8_t direction, uint32_t nbSteps){

	int ratio = speed/radius;
	int inside_ratio = (radius - RADIUS_ROBOT)/radius;
	int outside_ratio = (radius+ RADIUS_ROBOT)/radius;

	switch(mode){
	  case MODE_INFINITE:

		  if(direction==TRIGONOMETRIC){
			  left_motor_set_speed((radius - RADIUS_ROBOT)*ratio);
			  right_motor_set_speed((radius+ RADIUS_ROBOT)*ratio);
		  }
		  else /*direction = clock by default*/{
			  left_motor_set_speed((radius + RADIUS_ROBOT)*ratio);
			  right_motor_set_speed((radius- RADIUS_ROBOT)*ratio);
		  }
		break;

	  case MODE_FINITE:

		  if(direction==TRIGONOMETRIC){
			  left_motor_set_speed_step(inside_ratio*speed, nbSteps*inside_ratio);
			  right_motor_set_speed_step(outside_ratio*speed, nbSteps*outside_ratio);
		  }
		  else /*direction = clock by default*/{
			  left_motor_set_speed_step(outside_ratio*speed, nbSteps*outside_ratio);
			  right_motor_set_speed_step(inside_ratio*speed, nbSteps*inside_ratio);		  }
		break;

	  default:
		  left_motor_set_speed(0);
		  right_motor_set_speed(0);
		break;
	}
}

//Halt the robot
void halt(void){
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void reset_step_count(void){
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}
