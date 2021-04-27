#include "ch.h"
#include "hal.h"
#include <math.h>

#include <main.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>
#include <process_image.h>
#include <big_brain.h>

typedef angle_256deg uint8_t;
typedef angular_speed uint8_t;

#define STEPS_PER_ANGULAR_UNIT 3;

static angle_256deg robot_angular_position;

static BSEMAPHORE_DECL(BigBrain_sem, TRUE);

void rotate_robot(angle_256deg* angle_ptr, angular_speed ang_speed){
	//modify value in static angle while turning wheels
	//use while function determined by boolean "is_detected"
	for(angle256deg i = &angle_ptr; i>0; i--){



		if(!target_detected())
			break;
	}
}

static THD_WORKING_AREA(waMotorHandling, 256);
static THD_FUNCTION(MotorHandling, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;



}

static THD_WORKING_AREA(waBigBrain, 256);
static THD_FUNCTION(BigBrain, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


}

