#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define TOLERANCE_FOR_ALIGNEMENT 2

uint8_t get_searching(void);
uint8_t get_aligned(void);
uint8_t get_distance_camera(void);
uint8_t found_lost_target(uint8_t mode);
int get_angle_to_target(void);
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
