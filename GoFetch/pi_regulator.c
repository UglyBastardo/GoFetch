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

//La partie d'Eric! Je vais faire à ma manière et tu peux changer si tu trouves que c'est pas adapté:

void rotate_angle(Angle angle_to_complete, Angular_speed angular_speed){
	//Set Motor Speeds
	left_motor_set_steps_to_complete(angle_to_complete);
	right_motor_set_steps_to_complete(angle_to_complete);
	left_motor_set_speed(angular_speed);
	right_motor_set_speed(angular_speed);
}

void forward(Direction dir, uint16_t speed){
	left_motor_set_steps_to_complete(MAXSTEPS);
	right_motor_set_steps_to_complete(MAXSTEPS);

	if(dir==-1 || dir==1){
		left_motor_set_speed(speed*dir);
		right_motor_set_speed(speed*dir);
	} else {
		left_motor_set_speed(ZERO);
		right_motor_set_speed(ZERO);
	}
}

void forward_nb_steps(uint32_t steps_to_complete);

void revolve_around(Angle angle_to_revolve, uint16_t radius_of_revolution){
	angle_to_revolve = 0;
	radius_of_revolution = 0;
}





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
static int16_t position_angle = 0;

static enum motor_mode currentState = TurnAround;


//void turn_around();

void reset_number_step();
void increase_radius();
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
				//left_motor_set_speed(0);
				//right_motor_set_speed(0);
				continue;

			case TurnAround:
				if (get_ongoing_state()!=1){
				turn_around();//�
				currentState = IncreaseRadius;
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

			default:
				left_motor_set_speed(0);
				right_motor_set_speed(0);
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
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
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

void increase_radius(){
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

void turn_around(){
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
	/*while (get_ongoing_state() == 1){
		chThdSleepMilliseconds(10);
	}
	turn(_RIGHT);
	turn(_LEFT);
	*/
}

void reset_number_step(){
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

