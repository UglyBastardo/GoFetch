#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <big_brain.h>
#include <leds.h>
//#include <proximity.h>

#define INFINITE_ROTATION   1
#define FINITE_ROTATION 	2
#define FIND_MODE			1
#define LOSE_MODE 			0
#define KP 					3
#define MIN_ANGLE 			5
#define FRAMES_FOR_DETECTION 7
//La partie d'Eric! Je vais faire à ma manière et tu peux changer si tu trouves que c'est pas adapté:

/*
void rotate_angle(Angle angle_to_complete, Angular_speed angular_speed){
	//Set Motor Speeds
	left_motor_set_steps_to_complete(angle_to_complete);
	right_motor_set_steps_to_complete(angle_to_complete);
	left_motor_set_speed(angular_speed);
	right_motor_set_speed(angular_speed);
}
*/


uint8_t found_lost_target_eric(uint8_t mode);
uint8_t P_control(int error); //returns true if the robot is aligned
void rotate_eric(uint8_t mode, Angle rotation_angle, Speed speed);
void rotate_angle_eric(Speed speed, Angle angle_to_complete);
void halt_robot_eric(void);

void forward_nb_steps(uint32_t steps_to_complete);

/* --> dans ma partie
void revolve_around(Angle angle_to_revolve, uint16_t radius_of_revolution){
	angle_to_revolve = 0;
	radius_of_revolution = 0;
}
*/




//constante calculer une fois exp�rimentalement de mani�re pr�cise

//==========================================================
//test partie Dayan

//distance entre les roues
#define DIST_WHEELS 384.6 //unity: steps (0.13mm)
#define NBR_STEP_FULLTURN 1000
#define PI 3.14159265
#define INCREASE_RADIUS 1500 //arbitraire

/*
//besoin de 16 bits?
static int16_t speed_left = 200;
static int16_t speed_right = 200;
*/

//static Process_mode mode = DoNothing;
//robot position in cylindrical coordinates
static int16_t position_radius = DIST_WHEELS/2; //unity steps
static int16_t position_angle = 0; // unity mRad

static enum motor_mode currentState = DoNothing_;


//void turn_around();

void reset_number_step(void);
void increase_radius(void);
void turn_around(void);

/*
void set_mode(Process_mode mode_){
	mode = mode_;
}
*/

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    /*static int32_t motor_left_position=0;
    static int32_t motor_right_position=0;

    //initalise values
    motor_left_position = left_motor_get_pos();
    motor_right_position = right_motor_get_pos();
	*/
    //int16_t speed = 400;
    static uint8_t searching = TRUE;
    static uint8_t aligned   = FALSE;
    static uint32_t init_pos = 0; final_pos = 0;
    static Angle delta_angle;


    while(1){
//    	init_pos = left_motor_get_pos();
    	rotate_eric(searching, 0, 200);
        searching = !found_lost_target_eric(searching);
        if(!searching){
        	aligned = P_control(get_angle_to_target());
        }

        if(aligned){
        	set_led(LED1,1);
        } else {
        	set_led(LED1,0);
        }

        if(searching){
        	set_body_led(1);
        } else {
        	set_body_led(0);
        }

//        final_pos = left_motor_get_pos();
//        delta_angle = final_pos - init_pos;

        //Attention, ici je ne sépare par les modes...
        if(aligned){
        	forward(NORMAL_SPEED);
        }
//
//        switch (currentState){
//			case Stop:
//				left_motor_set_speed(0);
//				right_motor_set_speed(0);
//				continue;
//
//			case TurnAround:
//				if (get_ongoing_state()!=1){
//				turn_around();//�
//				currentState = IncreaseRadius;
//				}
//				continue;
//
//			case IncreaseRadius:
//				if (get_ongoing_state()!=1){
//					increase_radius();
//				}
//				//forward_nb_steps(1);
//				//turn(_LEFT);
//				//currentState = TurnAround;
//				continue;
//
//			case DoNothing_:
//				continue;
//
//			case CurrentlyMoving:
//				if (get_ongoing_state()!=1){
//					currentState=FinishedMoving;
//				} //faut faire un case FinishedMoving?
//				continue;
//				//left_motor_set_speed(0);
//				//right_motor_set_speed(0);
//        }
//        /*
//		*	To complete
//		*/
//
//        //applies the speed from the PI regulator
//
//        /*
//		right_motor_set_speed(speed_left);
//		left_motor_set_speed(speed_right);
//
//
//		motor_left_position = left_motor_get_pos();
//		motor_right_position = right_motor_get_pos();
//
//		if (motor_left_position >= 1293){
//			speed_left=0;
//			speed_right=0;
//		}
//		*/
//        //100Hz
//        chThdSleepUntilWindowed(time, time + MS2ST(50)); //avant 10
        chThdSleepMilliseconds(20);
    }
}

//turn clock-wise
void rotate_angle(uint16_t angle_to_complete) {

	currentState = DoNothing_; //au cas o� il �tait en stop

	left_motor_set_speed_step(NORMAL_SPEED, DIST_WHEELS/2*angle_to_complete/MILIRAD_TO_RAD);//magic number
	right_motor_set_speed_step(-NORMAL_SPEED, -DIST_WHEELS/2*angle_to_complete/MILIRAD_TO_RAD);

	currentState = CurrentlyMoving;
}

//turn the robot in the given direction (HALT(0) RIGHT(1) or LEFT(-1))
void turn(Direction dir){

	/*
	if(dir==-1 || dir==1){
		right_motor_set_speed(-speed*dir);
		left_motor_set_speed(speed*dir);
	} else {
		right_motor_set_speed(ZERO);
		left_motor_set_speed(ZERO);
	}*/
	reset_number_step();
	left_motor_set_speed_step(dir*NORMAL_SPEED, dir*DIST_WHEELS*PI/4);//magic number
	right_motor_set_speed_step(-dir*NORMAL_SPEED, -dir*DIST_WHEELS*PI/4);
	/*while (get_ongoing_state() == 1){
			chThdSleepMilliseconds(10);
	}*/

}



void increase_radius(void){
	static int8_t state=0;
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

//void turn_eric(Speed speed){
//	left_motor_set_speed_step(dir*NORMAL_SPEED, dir*DIST_WHEELS*PI/4);//magic number
//	right_motor_set_speed_step(-dir*NORMAL_SPEED, -dir*DIST_WHEELS*PI/4);
//}
//
//void align_with_target(){
//	speed = get_angle_to_target() * Kp;
//}


void turn_around(void){
	revolve_around(2*PI,position_radius); //passer angle en mRad ou m�me microRad?
	/*
	//perte en pr�cision vu que c'est que des int (r�flechir si probl�matique)
	int16_t speed_left = ((position_radius - DIST_WHEELS/2)*NORMAL_SPEED)/(position_radius);
	int16_t speed_right = ((position_radius+ DIST_WHEELS/2)*NORMAL_SPEED)/(position_radius);

	//reset nbr step
	//fonction convertisseur, choisir unit�
	int32_t left_distance = 2*(position_radius-DIST_WHEELS/2)*PI;
	int32_t right_distance = 2*(position_radius+DIST_WHEELS/2)*PI;
	reset_number_step();
	left_motor_set_speed_step(speed_left, left_distance);
	right_motor_set_speed_step(speed_right, right_distance);
	 while (get_ongoing_state() == 1){
		chThdSleepMilliseconds(10);
	}
	turn(_RIGHT);
	turn(_LEFT);
	*/

}

void revolve_around(Angle angle_to_revolve, uint16_t radius_of_revolution){

	int16_t speed_left = ((radius_of_revolution - DIST_WHEELS/2)*NORMAL_SPEED)/(radius_of_revolution);
	int16_t speed_right = ((radius_of_revolution+ DIST_WHEELS/2)*NORMAL_SPEED)/(radius_of_revolution);

	int32_t left_distance = (radius_of_revolution-DIST_WHEELS/2)*angle_to_revolve;
	int32_t right_distance = (radius_of_revolution+DIST_WHEELS/2)*angle_to_revolve;

	reset_number_step();

	left_motor_set_speed_step(speed_left, left_distance);
	right_motor_set_speed_step(speed_right, right_distance);
}

void reset_number_step(void){
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}

void forward_nb_steps(uint32_t steps_to_complete){
	left_motor_set_speed_step(NORMAL_SPEED, steps_to_complete);//cr�er pour aussi aller en arri�re?
	right_motor_set_speed_step(NORMAL_SPEED, steps_to_complete);
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

void motor_search_ball(void){
	//check if it is already in search mode
	//set_body_led(1);
	if (currentState != TurnAround && currentState != IncreaseRadius){
		//set ongoing = 0
		//peut-�tre faire un ongoing droite et gauche
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		currentState = TurnAround;
	}
}

void motor_stop(void){
	//to calculate the position according to the state it was in
	enum motor_mode ancientState = currentState;
	currentState = Stop; // ===============================================================================0
	/*switch (ancientState){
		case TurnAround:
			position_angle = right_motor_get_pos()*1000/(position_radius+DIST_WHEELS/2);
			break;
			//faut mettre un default?
	}
	*/
}


uint8_t finished_moving(void){
	if (currentState == FinishedMoving){
		currentState = DoNothing;
		return 1;
	}
	return 0;
}

void forward(int speed){
	//left_motor_set_steps_to_complete(MAXSTEPS);
	//right_motor_set_steps_to_complete(MAXSTEPS);

	left_motor_set_speed(speed);
	right_motor_set_speed(speed);
//	currentState = DoNothing_;
//
//	if(dir==-1 || dir==1){
//		left_motor_set_speed(speed*dir);
//		right_motor_set_speed(speed*dir);
//	} else {
//		left_motor_set_speed(ZERO);
//		right_motor_set_speed(ZERO);
//	}
}


//implementation of eric functions =========================================================================
uint8_t found_lost_target_eric(uint8_t mode){

	//counts the number of times the object was detected
	static uint8_t detection_counter;
	if(target_detected_camera() == mode){
		detection_counter++;
	} else {
		detection_counter = 0;
	}

	//returns true if the number of detections was sufficient for the object to be detected
	//Also returns true when the object has not been proven to be absent
	if(detection_counter>FRAMES_FOR_DETECTION){
		detection_counter = 0;
		//returns true if the mode is find and false if the mode is not
		return (TRUE==mode);
	} else {
		//returns true if the mode is find and false otherwise
		return (FALSE==mode);
	}
}

uint8_t P_control(int error){
	if(error > MIN_ANGLE || error < -MIN_ANGLE){
		rotate_eric(INFINITE_ROTATION, 0, KP*error);
		return FALSE;
	}
	else{
		halt_robot_eric();
		return TRUE;
	}
}


//mode = 1 simply rotates with speed;
//mode = 2 rotates a certain angle with speed;
//mode = 0 Halts the robot
void rotate_eric(uint8_t mode, Angle rotation_angle, Speed speed){
	switch(mode){
	  case INFINITE_ROTATION:
		    right_motor_set_speed(speed);
		  	left_motor_set_speed(-speed);
	    break;

	  case FINITE_ROTATION:
			rotate_angle_eric((int)speed, rotation_angle);
		break;
	}
}

//the angle given here uses a 1293 degrees as a full rotation (nb step necessary)
void rotate_angle_eric(Speed speed, Angle angle_to_complete){
	left_motor_set_speed_step((int)-speed, (uint32_t)angle_to_complete);
	right_motor_set_speed_step((int)speed, (uint32_t)angle_to_complete);
}

//Halt the robot
void halt_robot_eric(void){
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}
