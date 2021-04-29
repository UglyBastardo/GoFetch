#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

//constante calculer une fois expérimentalement de manière précise


//comme ça qu'on déclare variable const?
//time for the robot to turn 90°
static const int16_t TIME_TURN = 1000;
//distance entre les roues
static const int8_t DIST_WHEELS = 80; //mm (valeur random actuellement ^^')

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
		*/
        
        //applies the speed from the PI regulator
		right_motor_set_speed(speed_left);
		left_motor_set_speed(speed_right);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}


//turn the robot in the given direction (RIGHT or LEFT)
void turn(bool direction){

	if (direction) {
		speed_left = NORMAL_SPEED;
		speed_right = -NORMAL_SPEED;
	} else {
		speed_left = -NORMAL_SPEED;
		speed_right = NORMAL_SPEED;
	}

	chThdSleepMilliseconds(TIME_TURN);
	speed_left = 0;
	speed_right = 0;
}

void turn_around(){
	//perte en précision vu que c'est que des int (réflechir si problématique)
	speed_left = (position_radius*NORMAL_SPEED)/(position_radius + DIST_WHEELS);
	speed_right = NORMAL_SPEED;
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
