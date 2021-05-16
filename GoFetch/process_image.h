#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H


/**
* @brief  get the angle between the middle of the image and the middle of the object
*
* @return angle in px
*
*/
int get_angle_to_target(void);

/**
* @brief  start the threads to capture and process the image
*
*/
void process_image_start(void);


/**
* @brief  know if the object is found or not
*
* @return 1 if found, 0 if the object is not found
*
*/
uint8_t target_found(void);

#endif /* PROCESS_IMAGE_H */
