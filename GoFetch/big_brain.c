#include "ch.h"
#include "hal.h"
#include <math.h>

#include <main.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>
#include <process_image.h>
#include <big_brain.h>

typedef angle_1293deg uint16_t;
typedef angular_speed uint16_t;

#define STEPS_PER_ANGULAR_UNIT 1;

static angle_1293deg robot_angular_position;

static BSEMAPHORE_DECL(BigBrain_sem, TRUE);


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

