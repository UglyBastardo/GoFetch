#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

int get_angle_to_target(void);
uint8_t found_lost_target(uint8_t mode);
uint8_t target_detected_camera(void);
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
