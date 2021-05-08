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

#include <pi_regulator.h>
#include <process_image.h>
#include <big_brain.h>

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //start the i2c communication for the ToF sensor
    i2c_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//starts the ToF sensor
	VL53L0X_start();
	//inits the motors
	motors_init();
	//inits the core thread
	big_brain_start();

	//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	//process_image_start();

	//test
	//turn_around();
	//turn(1);
    /* Infinite loop. */
	static uint8_t a = 0;
	//forward(_BACKWARD, SLOW_SPEED);
    while (1) {
    	//waits 1 second
    	/*if (VL53L0X_get_dist_mm()<500){
    		set_body_led(1);
    	} else {
    		set_body_led(0);
    	}*/
    	/*if (a==0){
    		forward(_BACKWARD, SLOW_SPEED);
    		//left_motor_set_speed(SLOW_SPEED);
    		//right_motor_set_speed(SLOW_SPEED);
    		a++;
    	} else {
    		motor_stop();
    		a=0;
    	}*/

        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
