#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <i2c_bus.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <process_image.h>
#include <big_brain.h>
#include <motor_process.h>
#include <tof_process.h>

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //start the i2c communication for the ToF sensor
    i2c_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//starts the ToF sensor
	VL53L0X_start();
	//inits the motors
	motors_init();

	//stars the threads for the pi regulator and the processing of the image
	motor_regulator_start();
	process_image_start();
	tof_thread_start();

	//inits the core thread
	big_brain_start();

    while (1) {
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
