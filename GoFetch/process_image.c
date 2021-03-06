#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <leds.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

static int target_position = 0; //angular position given in pixels with

//mode 0: target found, mode 1: target not found
static uint8_t mode = 1;

#define WIDTH_SLOPE 		8
#define MIN_WIDTH_PIXELS	30
#define MIN_HALFWIDTH_PX	15
#define MIN_OFFSET 			3
#define MAX_PX_VALUE		32
#define OFFSET				11			//Offset if the number to substract from max_px_value to find the threshhold value

#define BRIGHTNESS_DEFAULT	0
#define CONTRAST_DEFAULT	64
#define RED_GAIN_DEFAULT 	0x5E
#define GREEN_GAIN_DEFAULT 	0x40
#define BLUE_GAIN_DEFAULT 	0x5D
#define INTEGRAL_DEFAULT 	0x80
#define FRACTIONAL_DEFAULT 	0x00

#define LINE_TO_READ_BEGIN  200
#define NB_LINES_TO_READ 	2

#define FRAMES_FOR_DETECTION 4
#define MIN_TOLERANCE_FOR_ALIGNEMENT 5


// some functions have been taken from TP4 and modified for the purpose of our project,
// so the names of the variables and functions may be pretty close

//=================================================================
/*
 * internal Functions
 */
//=================================================================

/**
* @brief   update if the object has been seen /////
*
*/
void found_lost_target(uint8_t target_not_found);

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
	uint8_t stop = 0;
	uint8_t mean = 0; // attention si WIDTH grand --> 16_t
	static uint8_t target_not_found = 0; //searching = TRUE, aligned = FALSE;
	target_not_found = 0;

	//search for a begin

	//is the object cut by the camera on the left side?

	for(uint8_t j = 0; j < WIDTH_SLOPE; j++){
		mean+=buffer[j];
	}


	if (mean>(MAX_PX_VALUE-OFFSET-MIN_OFFSET)*WIDTH_SLOPE){
		//the object is on the left side
		stop = 1;
		begin = 1;
		i=1;
	}

	while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
	{
		//the slope must at least be WIDTH_SLOPE wide and is compared
		//to the mean of the image
		if(buffer[i] < MAX_PX_VALUE-OFFSET-MIN_OFFSET && buffer[i+WIDTH_SLOPE] > MAX_PX_VALUE-OFFSET)
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

		while(stop == 0 && i < IMAGE_BUFFER_SIZE-WIDTH_SLOPE)
		{
			if(buffer[i+WIDTH_SLOPE] < MAX_PX_VALUE-OFFSET-MIN_OFFSET && buffer[i] > MAX_PX_VALUE-OFFSET)
			{
				end = i;
				stop = 1;
			}
			i++;
		}

		//is the object cut on the right side?

		mean = 0;

		for(uint16_t j = (IMAGE_BUFFER_SIZE - WIDTH_SLOPE - 1); j < IMAGE_BUFFER_SIZE - 1; j++){
			mean+=buffer[j];
		}


		if (mean>(MAX_PX_VALUE-OFFSET)*WIDTH_SLOPE && !end){
			//the object is on the left side
			end = IMAGE_BUFFER_SIZE - 1;
		}

		if (!end)
		{
			target_not_found = 1;
		}
	}
	else//if no begin was found
	{
		target_not_found = 1;
	}

	//if a line too small has been detected, continues the search
	if(!target_not_found && (end-begin) < MIN_WIDTH_PIXELS){
		target_not_found = 1;
	}

	found_lost_target(target_not_found);

	if(target_not_found){
		begin = 0;
		end = 0;
	}else{
		target_position = (begin + end - IMAGE_BUFFER_SIZE)/2; //gives the target position relativ to the center of the image.
	}

}

void found_lost_target(uint8_t target_not_found){

	//counts the number of times the object was detected
	static uint8_t detection_counter = 0;


	//mode 0: target found, mode 1: target not found
	if(mode != target_not_found){
		detection_counter++;
	} else {
		detection_counter = 0;
	}

	//go true if the number of detections was sufficient for the object to be detected
	//also stay true when the object has not been proven to be absent
	if(detection_counter>FRAMES_FOR_DETECTION){
		//inverse mode
		detection_counter = 0;
		mode = (mode==FALSE);
	}
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
    }
}

//=================================================================
/*
 * Public Functions
 */
//=================================================================



uint8_t target_found(void){
	return (mode==FALSE);
//	return 0;
}

int get_angle_to_target(void){
	return -target_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO+10, CaptureImage, NULL);
}


