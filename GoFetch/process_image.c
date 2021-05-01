#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>


static uint16_t angle_to_target = 0;
static uint8_t is_detected = 0;

#define MIN_WIDTH_PIXELS		30

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

}

void extract_data(uint8_t* data, uint16_t size,	uint8_t *pc){

	/*
	 * pc is the position counter for the data.
	 * data is the data to process
	 * size is the size of the given data
	 */


	/*
	 *   To Complete (find ball in bits)
	 */
	for(uint16_t i = 0; i<size;i++){
		data[i]= *pc>>0b11;
		pc += 0b10;
	}
}
void find_target_center(uint8_t* data, uint16_t size){

	/*
	*   To Complete (find position of beginning and end of ball pos and return the )
	*/

}
void process_detection(uint8_t* data, uint16_t size){



	/*
	 *   To Complete (find ball in bits)
	 */

	if(is_detected)
	{
		find_target_center(data, size);
	}
}

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//extract the useful bits from each pixel on the processed line
		extract_data(image, IMAGE_BUFFER_SIZE,img_buff_ptr);

		//Process the extracted bits in order to detect the presence of the target and, if so,
		//get a value for the angular position of the ball compared to the robots orientation
		process_detection(image, IMAGE_BUFFER_SIZE);

		//SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
    }
}

uint16_t get_angle_to_target(void){
	return angle_to_target;
}

uint8_t target_detected(void){
	return is_detected;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
