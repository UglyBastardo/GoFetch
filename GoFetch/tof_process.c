#include "ch.h"
#include "hal.h"

#include <tof_process.h>
#include <sensors/VL53L0X/VL53L0X.h>

#define TOF_NBR_VALUE_TAKEN 3
#define MAX_DETECT_DISTANCE 300
#define KTOF 3

static uint16_t distance_robot_obj = 0;

/**
* @brief   calculate the mean of the last values measured and check if the new is
* 		   significantly bigger -> may be a false measure -> give the last measure
*
*/
void tof_mean_calculator(void);


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

	uint16_t mean = 0;

	for(uint8_t j=0; j<TOF_NBR_VALUE_TAKEN; j++){
				//initalisation of the array
		mean+=tof_values[j];
	}

	mean/=TOF_NBR_VALUE_TAKEN;

	if (i<TOF_NBR_VALUE_TAKEN-2){
		tof_values[i]=VL53L0X_get_dist_mm();
		i++;
	} else {
		//we're at the end of the array
		tof_values[i]=VL53L0X_get_dist_mm();
		i=0;
	}

	//value significantly bigger -> maybe error from the captor
	if(new_value>KTOF*mean && new_value > MAX_DETECT_DISTANCE){
		//keep the last value measured
		if(i==0){
			distance_robot_obj = tof_values[TOF_NBR_VALUE_TAKEN-1];
		} else {
			distance_robot_obj = tof_values[i-1];
		}
	} else {
		//give the new value
		distance_robot_obj = new_value;
	}
}

uint16_t get_tof_distance(void){
	return distance_robot_obj;
}

void tof_thread_start(void){
	chThdCreateStatic(waToF, sizeof(waToF), NORMALPRIO+2, ToF, NULL);
}
