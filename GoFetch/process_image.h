#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

int get_angle_to_target(void);
void process_image_start(void);


//return 1 if found, else 0
uint8_t target_found(void);

//uint8_t get_searching(void);
//uint8_t get_aligned(void);

#endif /* PROCESS_IMAGE_H */
