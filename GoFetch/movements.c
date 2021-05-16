#include <math.h>
#include <stdlib.h>

#include <leds.h>
#include <motors.h>
#include <movements.h>


#define MAX_STEPS_PER_MVT 		3

static int temp_pos[3] = {0,0,0};
static uint8_t state = FIRSTSTATE;
//================================================================================
/*	Getter Function definitions
 *
 *
 */
//================================================================================
int get_temp_xpos(void){
	return temp_pos[X];
}
int get_temp_ypos(void){
	return temp_pos[Y];
}
int get_temp_angle(void){
	return temp_pos[ANGLE];
}
int get_state(void){
	return state;
}
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
#define TWOPI 			6.28


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

	static enum mode {Begin, Rotate1, Forwards , Rotate2} mode = Begin;
	static uint8_t done = FALSE;
//	static int temp[3];
//	static int* new_pos_ptr = 0; //12 bytes for 4 ints
	//tracking is done using motor going forward first
//	new_pos_ptr = malloc(12)
	switch(mode){

	case Begin:
		done = FALSE;
		mode = Rotate1;
		break;
	case Rotate1:
		if(verify_done_moving(MODE_FINITE, side*execution_speed, PIover2Steps*3, rotate)){
			mode = Forwards;
//			new_pos_ptr = delta_pos(ROTATE, left_motor_get_pos(), right_motor_get_pos(), 0);
//			temp_pos[X] = 0;
//			temp_pos[Y] = 0;
			temp_pos[ANGLE] = -side*PIover2Steps;
			state = SECONDSTATE;
		}
		break;

	case Forwards:
		if(verify_done_moving(MODE_FINITE, execution_speed, delta_radius, forwards)){
			mode = Rotate2;
//			new_pos = delta_pos(FORWARDS, left_motor_get_pos(), right_motor_get_pos(), 0);
//			temp_pos = calculate_position(old_pos[0], old_pos[1], old_pos[2], new_pos[0], new_pos[1], new_pos[2])
			temp_pos[Y] = delta_radius;
			state = THIRDSTATE;
		}
		break;

	case Rotate2:
		if(verify_done_moving(MODE_FINITE, -side*execution_speed, PIover2Steps*3, rotate)){
			done = TRUE;
//			new_pos = delta_pos(ROTATE, left_motor_get_pos(), right_motor_get_pos(), 0);
//			temp_pos = calculate_position(old_pos[0], old_pos[1], old_pos[2], new_pos[0], new_pos[1], new_pos[2])
//			temp_pos[ANGLE] = 0;
			state = REINIT;
			mode = Begin;
		}
		break;

	}

	return done;
}

uint8_t go_to_pos(double xinit, double yinit, double angle_init, double xfinal, double yfinal, double angle_final, int execution_speed){
	double dx = xfinal-xinit;
	double dy = yfinal-yinit;
	uint8_t done = FALSE;
	int temp[3];

	double angle_to_cover = atan2(dy, dx)-angle_init;  //angle needed to turn to be looking at destination
	double distance_to_cover = sqrt(dx*dx +dy*dy);	   //distance to destination

	static enum mode {Begin, Rotate1, Forwards , Rotate2} mode = Begin;


	switch(mode){
	case Begin:
		mode = Rotate1;
		done = FALSE;
	break;

	case Rotate1:
	//Very dangerous coding... we shall see the results
		if(angle_to_cover>0){
			if(verify_done_moving(MODE_FINITE, execution_speed, (int)(angle_to_cover*PISteps/PI), rotate)){
				mode = Forwards;
				temp_pos[X] = 0;
				temp_pos[Y] = 0;
				temp_pos[ANGLE] = calculate_new_angle(left_motor_get_pos(), right_motor_get_pos(), 0);
			}
		} else if (angle_to_cover <0){
			if(verify_done_moving(MODE_FINITE, -execution_speed, (int)(-angle_to_cover*PISteps/PI), rotate)){
				mode = Forwards;
				temp_pos[X] = 0;
				temp_pos[Y] = 0;
				temp_pos[ANGLE] = calculate_new_angle(left_motor_get_pos(), right_motor_get_pos(), 0);
			}
		} else {
			halt();
			mode = Forwards;
			temp_pos[X] = 0;
			temp_pos[Y] = 0;
			temp_pos[ANGLE] = calculate_new_angle(left_motor_get_pos(), right_motor_get_pos(), 0);
		}

	break;

	case Forwards:
		if(verify_done_moving(MODE_FINITE, execution_speed, (int)distance_to_cover, forwards)){
			mode = Rotate2;
			temp[X] = calculate_new_x(temp_pos[X], temp_pos[ANGLE], (int)distance_to_cover);
			temp_pos[Y] = calculate_new_y(temp_pos[Y], temp_pos[ANGLE], (int)distance_to_cover);
			temp_pos[X] = temp[0];
		}

	break;

	case Rotate2:

		set_body_led(1);
		angle_to_cover = angle_final-angle_to_cover;

		if(angle_to_cover>0){
			if(verify_done_moving(MODE_FINITE, execution_speed, (int)(angle_to_cover*PISteps/PI), rotate)){
				done = TRUE;
				temp_pos[X] = 0;
				temp_pos[Y] = 0;
				temp_pos[ANGLE] = calculate_new_angle(left_motor_get_pos(), right_motor_get_pos(), 0);
			}
		} else if (angle_to_cover <0){
			if(verify_done_moving(MODE_FINITE, -execution_speed, (int)(-angle_to_cover*PISteps/PI), rotate)){
				done = TRUE;
				temp_pos[X] = 0;
				temp_pos[Y] = 0;
				temp_pos[ANGLE] = calculate_new_angle(left_motor_get_pos(), right_motor_get_pos(), 0);
			}
		} else {
			halt();
			done = TRUE;
			temp_pos[X] = 0;
			temp_pos[Y] = 0;
			temp_pos[ANGLE] = calculate_new_angle(left_motor_get_pos(), right_motor_get_pos(), 0);
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
#define RADIUS_ROBOT 205 //unity: steps (0.13mm)

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

//returns the variation in position and in angle taking a position (x0,y0,alpha0) = (0,0,0)
int *delta_pos(uint8_t mvtType, int nbSteps_done_right_motor, int nbSteps_done_left_motor, uint32_t radius) /*int for accounting for direction*/{
	//{xpos,ypos,alphapos}
	static int new_pos[3] = {0,0,0};
	static int temp;
	switch(mvtType){
	case FORWARDS:
		new_pos[0] = nbSteps_done_right_motor;
		new_pos[1] = 0;
		new_pos[2] = 0;
	break;

	case ROTATE:
		nbSteps_done_right_motor %= PISteps; //Checked: modulo should work on negative numbers too
		if(nbSteps_done_right_motor >PISteps){
			nbSteps_done_right_motor -= PISteps;
		}

		new_pos[0] = 0;
		new_pos[1] = 0;
		new_pos[2] = nbSteps_done_right_motor;
	break;

	case REVOLVE:
		temp = (nbSteps_done_right_motor-nbSteps_done_left_motor)/2; //calculates as though it were a rotation
		temp %= PISteps;
		if(temp>PISteps){
					temp -= PISteps;
		}

		if(nbSteps_done_right_motor > nbSteps_done_left_motor){
			new_pos[X] = (int) radius * sin(temp*PI/PISteps);
			new_pos[Y] = (int) radius * cos(temp*PI/PISteps);
			new_pos[ANGLE] = temp;
		} else {
			new_pos[X] = (int) -radius * sin(temp*PI/PISteps);
			new_pos[Y] = (int) -radius * cos(temp*PI/PISteps);
			new_pos[ANGLE] = temp;
		}
	break;
	}

	return new_pos;
}

int *calculate_position(double xinit, double yinit, double angle_init, double xprime, double yprime, double angle_prime){
	static int new_pos[3] = {0, 0, 0};
	new_pos[ANGLE] = angle_prime+angle_init;
	new_pos[X] = xinit + sqrt(xprime*xprime+yprime*yprime)*cos(new_pos[ANGLE]);
	new_pos[Y] = yinit + sqrt(xprime*xprime+yprime*yprime)*sin(new_pos[ANGLE]);
	return new_pos;
}


int calculate_distance(int dx, int dy){
	return (int)sqrt(dx*dx+dy*dy);
}

int calculate_new_x(int xinit, int angle, uint32_t distance_Steps){
	return xinit + distance_Steps*cos(angle*PI/PISteps);
}

int calculate_new_y(int yinit, int angle, uint32_t distance_Steps){
	return yinit + distance_Steps*sin(angle*PI/PISteps);
}

int calculate_new_angle(int32_t left_rotation, int32_t right_rotation, int old_angle)/*unit is steps*/{
	int new_angle = right_rotation-left_rotation;
	new_angle/=2;
	new_angle += old_angle;

	new_angle %= PISteps; //Checked: modulo should work on negative numbers too
	if(new_angle >PISteps){
		new_angle -= PISteps;
	}
	return new_angle;
}

//int calculate_revolve(uint8_t coordinate, uint8_t direction, int angle, uint32_t radius){
//
//	int coord = 0;
//
//	switch(coordinate){
//	case X:
//
//		if(direction == TRIGONOMETRIC)
//			coord = radius*sin(angle*PI/PISteps-PI/2);
//		if(direction == CLOCK)
//			coord = -radius*sin(angle*PI/PISteps-PI/2);
//		return coord;
//
//	break;
//
//	case Y:
//
//		coord = radius-radius*cos(angle*PI/PISteps-PI/2);
//		return coord;
//
//	break;
//
//	default:
//		;
//	break;
//	}
//
//	return 0;
//}




