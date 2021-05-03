#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

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


/*

//comme ï¿½a qu'on dï¿½clare variable const?
//time for the robot to turn 90ï¿½
static const int16_t TIME_TURN = 1000;
//distance entre les roues
static const int8_t DIST_WHEELS = 53; //mm



//besoin de 16 bits?
static int16_t speed_left = 0;
static int16_t speed_right = 0;



//robot position in cylindrical coordinates
static int16_t position_radius = 240;
static int16_t position_angle = 0;

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    //int16_t speed = 400;

    while(1){
        time = chVTGetSystemTime();

        /*
		*	To complete
		/
        
        //applies the speed from the PI regulator
		right_motor_set_speed(speed_left);
		left_motor_set_speed(speed_right);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}
*/

//turn the robot in the given direction (HALT(0) RIGHT(1) or LEFT(-1))
void turn(Direction dir, uint16_t speed){

	if(dir==-1 || dir==1){
		right_motor_set_speed(-speed*dir);
		left_motor_set_speed(speed*dir);
	} else {
		right_motor_set_speed(ZERO);
		left_motor_set_speed(ZERO);
	}
}

void search_ball(){
	//while var global set par big_brain.c?
	int8_t angle=0; //set par rapport à l'angle actuel
	while (current_angle!=angle){ //var à déclarer
		turn_around();
		chThdSleepMilliseconds(1000);
	};
	turn(right);
	//move forward certain distance r=+distance
	//turn right and keep on turning

}
/*
void turn_around(){
	//perte en prï¿½cision vu que c'est que des int (rï¿½flechir si problï¿½matique)
	speed_left = (position_radius*NORMAL_SPEED)/(position_radius + DIST_WHEELS);
	speed_right = NORMAL_SPEED;
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
*/
