#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <leds.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

#define WIDTH_SLOPE 		10
#define MIN_WIDTH_PIXELS	30
#define MIN_HALFWIDTH_PX	15
#define MIN_OFFSET 			2
#define NOISE_LEVEL			6
#define MEDIAN_OFFSET		NOISE_LEVEL
#define MAX_PX_VALUE		32
#define THRESHOLD			20
#define OFFSET				14			//Offset if the number to substract from max_px_value to find the threshhold value
#define PXTOCM				2000

#define RED					0
#define GREEN 				1
#define BLUE 				2

#define BRIGHTNESS_DEFAULT	0
#define CONTRAST_DEFAULT	64
#define RED_GAIN_DEFAULT 	0x5E
#define GREEN_GAIN_DEFAULT 	0x40
#define BLUE_GAIN_DEFAULT 	0x5D
#define INTEGRAL_DEFAULT 	0x80
#define FRACTIONAL_DEFAULT 	0x00

#define LINE_TO_READ_BEGIN  200
#define NB_LINES_TO_READ 	2

#define FRAMES_FOR_DETECTION 3

//static Variables for the file
static uint8_t target_not_found = 0, searching = TRUE, aligned = FALSE;
static int target_position = 0; //angular position given in pixels with
static uint16_t target_width = 0;
static uint16_t target_distance = 0;



//=================================================================
/*
 * internal Functions
 */
//=================================================================

void calibrate_camera(uint8_t brightness, uint8_t contrast, uint8_t awb, uint8_t ae, uint8_t r_gain, uint8_t g_gain, uint8_t b_gain, uint16_t e_integral, uint8_t e_fractional){
	po8030_set_brightness(brightness);
	po8030_set_contrast(contrast);
	po8030_set_awb(awb);
	po8030_set_ae(ae);
	po8030_set_rgb_gain(r_gain,g_gain, b_gain);
	po8030_set_exposure(e_integral, e_fractional);
}
void extract_data(uint8_t* data, uint16_t size,	uint8_t *pc){
	//Extracts red bits from buffer
	for(uint16_t i = 0; i<size;i++){
		data[i]= *pc>>0b11;
		pc += 0b10;
	}
}

void update_target_detection(uint8_t *buffer){


	uint16_t i = 0, begin = 0, end = 0;
	uint8_t stop = 0, wrong_target = 0;
	target_not_found = 0;
	uint32_t mean = 0;


	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_target = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
//			if(buffer[i] < MAX_PX_VALUE-OFFSET-MIN_OFFSET && buffer[i+WIDTH_SLOPE] > MAX_PX_VALUE-OFFSET)
			if(buffer[i] > THRESHOLD)
			{
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;

		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
//		    	if(buffer[i] < MAX_PX_VALUE-OFFSET-MIN_OFFSET && buffer[i-WIDTH_SLOPE] > MAX_PX_VALUE-OFFSET)
				if(buffer[i] < THRESHOLD-MIN_OFFSET)
		    	{
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }

		    //if an end was not found
		    if(begin && !end){
		    	end = i;
		    }


//		    if (i > IMAGE_BUFFER_SIZE || !end)
//		    {
//		        target_not_found = 1;
//		    }
		}
		else//if no begin was found
		{
		    target_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!target_not_found && (end-begin) < MIN_WIDTH_PIXELS){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_target = 1;
		}
	}while(wrong_target);

	if(target_not_found){
		begin = 0;
		end = 0;
	}else{
		target_width = end - begin;
		target_distance = PXTOCM/target_width;
		target_position = (begin + end - IMAGE_BUFFER_SIZE)/2; //gives the target position relativ to the center of the image.
	}

}

uint8_t target_detected_camera(void){
	return (target_not_found == 0);
}

//=================================================================
/*
 * Threads
 */
//=================================================================

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line to read (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, LINE_TO_READ_BEGIN, IMAGE_BUFFER_SIZE, NB_LINES_TO_READ, SUBSAMPLING_X1, SUBSAMPLING_X1);
	calibrate_camera(BRIGHTNESS_DEFAULT, CONTRAST_DEFAULT+0x30, FALSE, FALSE, RED_GAIN_DEFAULT, GREEN_GAIN_DEFAULT, BLUE_GAIN_DEFAULT, INTEGRAL_DEFAULT+0x8F, FRACTIONAL_DEFAULT);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }

}

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

//	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts red pixels
		extract_data(image, IMAGE_BUFFER_SIZE, img_buff_ptr);

		//search for a target in the image and gets its position in pixels
		update_target_detection(image);

		//update searching
		searching = !found_lost_target(searching);

		if(searching){
			set_body_led(1);
		} else {
			set_body_led(0);
		}

		//update aligned
		aligned = (!searching && target_position<TOLERANCE_FOR_ALIGNEMENT && target_position>-TOLERANCE_FOR_ALIGNEMENT);

		if(aligned){
			set_led(LED1, 1);
		} else {
			set_led(LED1, 0);
		}


//		send_to_computer = !send_to_computer;
		if(target_not_found) {
			set_led(LED5,0);
		} else {
			set_led(LED5,1);
		}


		if(target_position>0){
			set_led(LED3, 1);
			set_led(LED7, 0);
		} else {
			set_led(LED3, 0);
			set_led(LED7, 1);
		}
//
//		if(send_to_computer){
//			//sends to the computer the image
//			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
//		}
//		//invert the bool
    }
}

//=================================================================
/*
 * Public Functions
 */
//=================================================================


uint8_t get_searching(void){
	return searching;
}

uint8_t get_aligned(void){
	return aligned;
}

uint8_t get_distance_camera(void){
	return target_distance;
}


//mode=1=searching mode=0=not_searching
uint8_t found_lost_target(uint8_t mode){

	//counts the number of times the object was detected
	static uint8_t detection_counter = 0;

	if(mode == target_detected_camera()){
		detection_counter++;
	} else {
		detection_counter = 0;
	}

	//returns true if the number of detections was sufficient for the object to be detected
	//Also returns true when the object has not been proven to be absent
	if(detection_counter>FRAMES_FOR_DETECTION){
		detection_counter = 0;
		//returns true if the mode is find and false if the mode is not
		return mode;
	} else {
		//returns true if the mode is find and false otherwise
		return !mode;
	}
}

int get_angle_to_target(void){
	return -target_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO+1, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO+1, CaptureImage, NULL);
}


