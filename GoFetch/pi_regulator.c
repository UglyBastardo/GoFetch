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

//La partie d'Eric! Je vais faire Ã  ma maniÃ¨re et tu peux changer si tu trouves que c'est pas adaptÃ©:

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





//constante calculer une fois expï¿½rimentalement de maniï¿½re prï¿½cise

//==========================================================
//test partie Dayan


//comme ï¿½a qu'on dï¿½clare variable const?
//time for the robot to turn 90ï¿½
static const int16_t TIME_TURN = 1000;
//distance entre les roues
static const int8_t DIST_WHEELS = 53; //mm
#define PI = 3.14159265;


//besoin de 16 bits?
static int16_t speed_left = 200;
static int16_t speed_right = 200;

//static Process_mode mode = DoNothing;
//robot position in cylindrical coordinates
static int16_t position_radius = 240;
static int16_t position_angle = 0;

static motor_mode currentState = DoNothing;


void turn_around();

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

    static int32_t motor_left_position=0;
    static int32_t motor_right_position=0;

    //initalise values
    motor_left_position = left_motor_get_pos();
    motor_right_position = right_motor_get_pos();

    //int16_t speed = 400;
    left_motor_set_speed_step(NORMAL_SPEED, motor_left_position+1293);
    right_motor_set_speed_step(NORMAL_SPEED, motor_right_position+1293);

    while(1){
        time = chVTGetSystemTime();

        switch (currentState){
        case DoNothing:
        	left_motor_set_speed(0);
        	right_motor_set_speed(0);
        	break;

        case TurnAround:
        	turn_around();//à
        	currentState = IncreaseRadius;
        	break;

        case IncreaseRadius:
        	turn(RIGHT);
        	forward_nb_steps(1);
        	turn(LEFT);
        	currentState = TurnAround;
        	break;
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
	left_motor_set_speed_step(NORMAL_SPEED, motor_left_position+100*dir);//magic number
	right_motor_set_speed_step(NORMAL_SPEED, motor_right_position-100*dir);
	while (get_ongoing_state == 1){
			chThdSleepMilliseconds(10);
	}

}


void search_ball(){
	//while var global set par big_brain.c?
	int8_t angle=position_angle; //set par rapport à l'angle actuel
	while (position_angle<=angle+360){ //var à déclarer
		turn_around();
		chThdSleepMilliseconds(100);
	};
	turn(right);
	//move forward certain distance r=+distance
	//turn right and keep on turning

}


void turn_around(){
	//perte en prï¿½cision vu que c'est que des int (rï¿½flechir si problï¿½matique)
	int16_t speed_left_ = (position_radius*NORMAL_SPEED)/(position_radius + DIST_WHEELS);
	int16_t speed_right = NORMAL_SPEED;
	//reset nbr step
	//fonction convertisseur, choisir unité
	left_motor_set_speed_step(speed_left, 2*PI*position_radius-DIST_WHEELS/2));//magic number
	right_motor_set_speed_step(speed_right, 2*PI*position_radius+DIST_WHEELS/2);
	while (get_ongoing_state == 1){
		chThdSleepMilliseconds(10);
	}

}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), HIGHPRIO, PiRegulator, NULL);
}

