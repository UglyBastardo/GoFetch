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
#define KP 				2
#define PIover2Steps 	323 //steps
#define PISteps			646 //steps
#define PI				3.1416 //no unit


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
	static uint8_t done = FALSE;
	//tracking is done using motor going forward first
	switch(mode){

	case Rotate1:
		if(verify_done_moving(MODE_FINITE, -side*execution_speed, PIover2Steps, rotate)){
			mode = Forwards;
		}
		break;

	case Forwards:
		if(verify_done_moving(MODE_FINITE, execution_speed, delta_radius, forwards)){
			mode = Rotate2;
		}
		break;

	case Rotate2:
		if(verify_done_moving(MODE_FINITE, side*execution_speed, PIover2Steps, rotate)){
			done = TRUE;
		}
		break;

	}

	return done;
}

uint8_t go_to_pos(double xinit, double yinit, double angle_init, double xfinal, double yfinal, double angle_final, int execution_speed){
	double dx = xfinal-xinit;
	double dy = yfinal-yinit;
	static uint8_t done = FALSE;

	double angle_to_cover = atan2(dy, dx)-angle_init;  //angle needed to turn to be looking at destination
	double distance_to_cover = sqrt(dx*dx +dy*dy);	   //distance to destination

	static enum mode {Rotate1, Forwards , Rotate2} mode = Rotate1;


	switch(mode){

	case Rotate1:
	//Very dangerous coding... we shall see the results
		if(angle_to_cover>0){
			if(verify_done_moving(MODE_FINITE, execution_speed, (int)(angle_to_cover*PISteps/PI), rotate)){
				mode = Forwards;
			}
		} else if (angle_to_cover <0){
			if(verify_done_moving(MODE_FINITE, -execution_speed, (int)(-angle_to_cover*PISteps/PI), rotate)){
				mode = Forwards;
			}
		} else {
			halt();
			mode = Forwards;
		}

	break;

	case Forwards:
		if(verify_done_moving(MODE_FINITE, execution_speed, (int)distance_to_cover, forwards)){
			mode = Rotate2;
		}

	break;

	case Rotate2:

		set_body_led(1);
		angle_to_cover = angle_final-angle_to_cover;

		if(angle_to_cover){
			done = verify_done_moving(MODE_FINITE, execution_speed, (int)((angle_to_cover)*PISteps/PI), rotate);
		} else if (angle_to_cover <0){
			done = verify_done_moving(MODE_FINITE, -execution_speed, (int)((-angle_to_cover)*PISteps/PI), rotate);
		} else {
			halt();
		}
	  break;
	}
	return done;
}


//================================================================================
/*	Begin of the Simple Movement Functions
 *
 *
 *
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

//================================================================================
/*	Begin of the complicated internal non movement functions
 *
 *
 *
 */
//================================================================================
uint8_t verify_done_moving(uint8_t mode, int speed, uint32_t nbSteps, uint8_t (*f)(uint8_t, int, uint32_t)){
	static uint8_t changing_mode = TRUE;
	if(changing_mode) set_led(LED1, 1); else set_led(LED1, 0);
	if(!get_ongoing_state() && changing_mode){
		reset_step_count();
		changing_mode = FALSE;
	}
	else if(!get_ongoing_state() && !changing_mode){
		changing_mode = TRUE; //this line of code is to avoid need of semaphore in the motors library
		return TRUE;
	}
	if((*f)(mode, speed, nbSteps)){
		changing_mode = TRUE;
		return TRUE;
	}

	return FALSE;
}


