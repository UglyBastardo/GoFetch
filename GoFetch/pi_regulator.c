#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <leds.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <big_brain.h>

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


//void forward_nb_steps(uint32_t steps_to_complete);

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
#define INCREASE_RADIUS 1500 //arbitraire
#define KP 2
#define ROT_ERR_THRESHOLD 2

/*
//besoin de 16 bits?
static int16_t speed_left = 200;
static int16_t speed_right = 200;
*/

//static Process_mode mode = DoNothing;
//robot position in cylindrical coordinates
static int16_t position_radius = DIST_WHEELS/2; //unity steps
//static int16_t position_angle = 0; // unity mRad

//pas s�r de �a (mettre en 32?)
static int16_t distance = 0;

enum motor_mode{Stop, TurnAround, IncreaseRadius, DoNothing_, CurrentlyMoving, FinishedMoving, Forward_, GetAround, PRegulator};
static enum motor_mode currentState = DoNothing_;


//void turn_around();

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
* @brief   robot makes a full 360� turn around a point (given by position_radius)
*
*/
void turn_around(void);

void p_regulator(void);

void motor_halt(void);
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

    while(1){
        time = chVTGetSystemTime();

        switch (currentState){
			case Stop:
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				continue;

			case TurnAround:
				if (get_ongoing_state()!=1){
				turn_around();//�
				//currentState = IncreaseRadius;
				}
				continue;

			case IncreaseRadius:
				if (get_ongoing_state()!=1){
					increase_radius();
				}
				//forward_nb_steps(1);
				//turn(_LEFT);
				//currentState = TurnAround;
				continue;

			case DoNothing_:
				continue;

			case CurrentlyMoving:
				if (get_ongoing_state()!=1){
					currentState=FinishedMoving;
				} //faut faire un case FinishedMoving?
				continue;
				//left_motor_set_speed(0);
				//right_motor_set_speed(0);
			case GetAround:
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
        /*
		*	To complete
		*/
        
        //applies the speed from the PI regulator

        /*
		right_motor_set_speed(speed_left);
		left_motor_set_speed(speed_right);


		motor_left_position = left_motor_get_pos();
		motor_right_position = right_motor_get_pos();

		if (motor_left_position >= 1293){
			speed_left=0;
			speed_right=0;
		}
		*/
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(50)); //avant 10
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
	left_motor_set_speed_step(dir*NORMAL_SPEED, dir*QUARTER_TURN);//magic number
	right_motor_set_speed_step(-dir*NORMAL_SPEED, -dir*QUARTER_TURN);
	/*while (get_ongoing_state() == 1){
			chThdSleepMilliseconds(10);
	}*/

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

/*void search_ball(){
	//while var global set par big_brain.c?
	int8_t angle=position_angle; //set par rapport � l'angle actuel
	while (position_angle<=angle+360){
		turn_around();
		chThdSleepMilliseconds(100);
	};
	turn(right);
	//move forward certain distance r=+distance
	//turn right and keep on turning

}
*/

void turn_around(void){
	revolve_around(2*MILIRAD_TO_RAD*PI,position_radius); //passer angle en mRad ou m�me microRad?

	currentState = IncreaseRadius;
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

//	currentState = DoNothing_;

	int16_t speed_left = -SLOW_SPEED;
	int16_t speed_right = SLOW_SPEED;

	if (radius_of_revolution!=0){
		speed_left = ((radius_of_revolution - DIST_WHEELS/2)*NORMAL_SPEED)/(radius_of_revolution);
		speed_right = ((radius_of_revolution+ DIST_WHEELS/2)*NORMAL_SPEED)/(radius_of_revolution);
	}
//	int16_t speed_left = ((radius_of_revolution - DIST_WHEELS/2)*NORMAL_SPEED)/(radius_of_revolution);
//	int16_t speed_right = ((radius_of_revolution+ DIST_WHEELS/2)*NORMAL_SPEED)/(radius_of_revolution);

	int32_t left_distance = (radius_of_revolution-DIST_WHEELS/2)*angle_to_revolve/MILIRAD_TO_RAD;
	int32_t right_distance = (radius_of_revolution+DIST_WHEELS/2)*angle_to_revolve/MILIRAD_TO_RAD;

	reset_number_step();

	left_motor_set_speed_step(speed_left, left_distance);
	right_motor_set_speed_step(speed_right, right_distance);

//	currentState = CurrentlyMoving;
}

void reset_number_step(void){
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}

void forward_nb_steps(int32_t steps_to_complete){

	reset_number_step();

	if (currentState!=IncreaseRadius){
		currentState = CurrentlyMoving;
	}

	if (steps_to_complete<0){
		left_motor_set_speed_step(-NORMAL_SPEED, steps_to_complete);//cr�er pour aussi aller en arri�re?
		right_motor_set_speed_step(-NORMAL_SPEED, steps_to_complete);
	} else {
		left_motor_set_speed_step(NORMAL_SPEED, steps_to_complete);//cr�er pour aussi aller en arri�re?
		right_motor_set_speed_step(NORMAL_SPEED, steps_to_complete);
	}
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
	switch (ancientState){
		case Forward:
			distance = left_motor_get_pos();
			break;
			//faut mettre un default?
		default:
			;
	}
}


uint8_t finished_moving(void){
	if (currentState == FinishedMoving){
		currentState = DoNothing;
		return 1;
	}
	return 0;
}

void forward(Direction dir, uint16_t speed){
	//left_motor_set_steps_to_complete(MAXSTEPS);
	//right_motor_set_steps_to_complete(MAXSTEPS);

	if (currentState != Forward_){
		currentState = Forward_;
		reset_number_step();
	}

	if(dir==-1 || dir==1){
		left_motor_set_speed(speed*dir);
		right_motor_set_speed(speed*dir);
	} else {
		left_motor_set_speed(ZERO);
		right_motor_set_speed(ZERO);
	}
}

int16_t get_distance(void){
	return distance;
}

int16_t get_radius(void){
	return position_radius;
}

void get_around(Angle angle_to_revolve, uint16_t radius_of_revolution){
	static uint8_t ar_state = 0;
	static Angle ar_angle = 0;
	static uint16_t ar_radius = 0;
	if (currentState != GetAround){
		currentState = GetAround;
		ar_radius = radius_of_revolution;
		ar_angle = angle_to_revolve;
		ar_state=0;
	}
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
		reset_number_step();
		currentState = PRegulator;
	}
}

void p_regulator(void){
	int err = get_angle_to_target();

	if (err<ROT_ERR_THRESHOLD && err>-ROT_ERR_THRESHOLD){
		motor_halt();
		currentState = FinishedMoving;
		set_front_led(1);
		return;
	}

	right_motor_set_speed(err*KP);
	left_motor_set_speed(-err*KP);
}

//utile?
void motor_halt(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}
