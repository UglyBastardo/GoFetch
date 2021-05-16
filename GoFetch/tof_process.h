#ifndef TOF_PROCESS_H_
#define TOF_PROCESS_H_

/**
* @brief   start the ToF thread
*
*/
void tof_thread_start(void);

/**
* @brief   get the ToF distance, if one measure is significantly bigger then the one before,
* 		   -> may be a false one, so it gives the measure before that
*
* @return the distance in mm
*
*/
uint16_t get_tof_distance(void);

#endif /* TOF_PROCESS_H_ */
