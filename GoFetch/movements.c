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

uint8_t translational_movement(int side, int delta_radius, int execution_speed){

	static enum mode {Rotate1, Forwards , Rotate2} mode = Rotate1;
	static uint8_t done = FALSE, changing_mode = TRUE;
	//tracking is done using motor going forward first
	switch(mode){

	case Rotate1:
		if(!get_ongoing_state() && changing_mode){
			reset_step_count();
			changing_mode = FALSE;
		}
		else if(!get_ongoing_state() && !changing_mode){
			mode = Forwards;
			changing_mode = TRUE; //this line of code is to avoid need of semaphore in the motors library
		}
				set_front_led(1);
		if(rotate(MODE_FINITE, side*execution_speed, PIover2)){
			mode = Forwards;
			changing_mode = TRUE;
		}
		break;

	case Forwards:
		if(!get_ongoing_state() && changing_mode){
			reset_step_count();
			changing_mode = FALSE;
		}
		else if(!get_ongoing_state() && !changing_mode){
			mode = Rotate2;
			changing_mode = TRUE; //this line of code is to avoid need of semaphore in the motors library
		}
				set_front_led(1);
		if(forwards(MODE_FINITE, execution_speed, delta_radius)){
			mode = Rotate2;
			changing_mode = TRUE;
		}
		break;

	case Rotate2:
		if(!get_ongoing_state() && changing_mode){
			reset_step_count();
			changing_mode = FALSE;
		}
		else if(!get_ongoing_state() && !changing_mode){
			done = TRUE;
			changing_mode = TRUE; //this line of code is to avoid need of semaphore in the motors library
		}
				set_front_led(1);
		if(rotate(MODE_FINITE, side*execution_speed, PIover2)){
			done = TRUE;
			changing_mode = TRUE;
		}
		break;

	}

	return done;
}




//================================================================================
/*	Begin of the Simple Movement Functions
 *
 *	!!!!!!
 *	revolution calculation is not functional
 */
//================================================================================
#define RADIUS_ROBOT 192 //unity: steps (0.13mm)

uint8_t forwards(uint8_t mode, int speed, uint32_t nbSteps){


	switch(mode){
	  case MODE_INFINITE:
		  left_motor_set_speed(speed);
		  right_motor_set_speed(speed);
		break;

	  case MODE_FINITE:
		  if(!get_ongoing_state()){
			  left_motor_set_speed_step(speed, nbSteps);
			  right_motor_set_speed_step(speed, nbSteps);
		  }
		break;

	  default:
		  left_motor_set_speed(0);
		  right_motor_set_speed(0);
		break;
	}


	return !get_ongoing_state();

}

uint8_t rotate(uint8_t mode, int speed, uint32_t nbSteps){

	switch(mode){
	  case MODE_INFINITE:
		  left_motor_set_speed(-speed);
		  right_motor_set_speed(speed);
	    break;

	  case MODE_FINITE:
		  if(!get_ongoing_state()){
			left_motor_set_speed_step(-speed, nbSteps);
			right_motor_set_speed_step(speed, nbSteps);
		  }
		break;

	  default:
		  left_motor_set_speed(0);
		  right_motor_set_speed(0);
		break;
	}

	return !get_ongoing_state();

}

uint8_t revolve(uint8_t mode, int speed, int radius, uint8_t direction, uint32_t nbSteps){

	switch(mode){
	  case MODE_INFINITE:
		  if(direction==TRIGONOMETRIC){
			  left_motor_set_speed((radius - RADIUS_ROBOT)*speed/radius);
			  right_motor_set_speed((radius+ RADIUS_ROBOT)*speed/radius);
		  }
		  else /*direction = clock by default*/{
			  left_motor_set_speed((radius + RADIUS_ROBOT)*speed/radius);
			  right_motor_set_speed((radius- RADIUS_ROBOT)*speed/radius);
		  }
		break;

	  case MODE_FINITE:

		  if(!get_ongoing_state()){
			  if(direction==TRIGONOMETRIC){
				  left_motor_set_speed_step((radius - RADIUS_ROBOT)*speed/radius, nbSteps*(radius - RADIUS_ROBOT)/radius);
				  right_motor_set_speed_step((radius + RADIUS_ROBOT)*speed/radius, nbSteps*(radius+ RADIUS_ROBOT)/radius);
			  }
			  else /*direction = clock by default*/{
				  left_motor_set_speed_step((radius + RADIUS_ROBOT)*speed/radius, nbSteps*(radius+ RADIUS_ROBOT)/radius);
				  right_motor_set_speed_step((radius - RADIUS_ROBOT)*speed/radius, nbSteps*(radius - RADIUS_ROBOT)/radius);
			  }
		  }
		break;

	  default:
		  left_motor_set_speed(0);
		  right_motor_set_speed(0);
		break;
	}

	return !get_ongoing_state();
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
