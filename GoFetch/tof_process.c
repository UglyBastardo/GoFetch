#include "ch.h"
#include "hal.h"

#include <tof_process.h>
#include <sensors/VL53L0X/VL53L0X.h>

#define TOF_NBR_VALUE_TAKEN 3
#define MAX_DETECT_DISTANCE 300

static uint16_t distance_robot_obj = 0;

static THD_WORKING_AREA(waToF, 256);

static THD_FUNCTION(ToF, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	tof_mean_calculator();
    	chThdSleepMilliseconds(100);
    }
}


void tof_mean_calculator(void){

	static int8_t i=-1;
	static uint16_t tof_values[TOF_NBR_VALUE_TAKEN];

	if (i<0){
		for(uint8_t j=0; j<TOF_NBR_VALUE_TAKEN; j++){
			//initalisation of the array
			tof_values[j]=VL53L0X_get_dist_mm();
		}
		i=0;
	}

	uint16_t new_value = VL53L0X_get_dist_mm();

	if (new_value>MAX_DETECT_DISTANCE){
		//avoid too high value
		new_value = MAX_DETECT_DISTANCE;
	}

	if (i<TOF_NBR_VALUE_TAKEN-2){
		tof_values[i]=VL53L0X_get_dist_mm();
		i++;
	} else {
		//we're at the end of the array
		tof_values[i]=VL53L0X_get_dist_mm();
		i=0;
	}

	uint16_t mean = 0;

	for(uint8_t j=0; j<TOF_NBR_VALUE_TAKEN; j++){
				//initalisation of the array
		mean+=tof_values[j];
	}

	mean/=TOF_NBR_VALUE_TAKEN;

	distance_robot_obj = mean;
}

uint16_t get_tof_distance(void){
	return distance_robot_obj;
}

void tof_thread_start(void){
	chThdCreateStatic(waToF, sizeof(waToF), NORMALPRIO+2, ToF, NULL);
}
