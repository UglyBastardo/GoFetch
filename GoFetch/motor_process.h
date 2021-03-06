#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

#define NORMAL_SPEED 500
#define SLOW_SPEED 250
#define _RIGHT 1
#define _LEFT  -1
#define _FORWARD 1
#define _BACKWARD -1
#define RAD_TO_MILIRAD 1000 //je le mets o??
#define MM_TO_STEP 7.75
#define PI 3.1415926
#define FULL_TURN 1290
#define QUARTER_TURN FULL_TURN/4
#define STEPS_TO_RAD 2*PI/FULL_TURN

/**
* @brief start the motor thread
*
*/
void motor_regulator_start(void);

/**
* @brief   Uses the motors to get the robot to rotate a certain angle (clockwise)
*
* @param	angle_to_complete		angle in mRad
*/
void rotate_angle(uint16_t angle_to_complete);

/**
* @brief   turn around a certain point (anti-clockwise)
*
* @param 	angle_to_revolve		rotation angle in mRad
* 			radisus_of_revolution 	radius of the circle
*/
void revolve_around(Angle angle_to_revolve, uint16_t radius_of_revolution);

/**
* @brief   robot moves at a certain speed (forward or backward)
*
* @param 	dir		direction (_FORWARD: 1, _BACKWARD: -1)
* 			speed 	speed of the robot (in step/s)
*/
void forward(Direction dir, uint16_t speed);

/**
* @brief   the robot starts moving in circle to find to find the ball
*
*/
void motor_search_ball(void);

/**
* @brief   stops the motor and return a value depending on the previous state
*
* @return 	if the mode was forward: return the distance traveled (in steps)
*/
int16_t motor_stop(void);

/**
* @brief   get the radius of the current searching circle
*
* @return radius of the circle (in step)
*/
int16_t get_radius(void);

/**
* @brief   know if the motor has finished moving (one time read)
*
* @return 1 if the robot has finished moving
* 		  0 if the robot is still moving, or the result has already been read
*/
uint8_t finished_moving(void);

/**
* @brief   move forwards a certain number of steps
*
* @param steps_to_complete		distance to travel (in step)
*/
void forward_nb_steps(int32_t steps_to_complete);

/**
* @brief   turn around a certain point (facing the point at the beginning and at the end)
*
* @param angle_to_revolve		angle traveled (in mRad)
* 		 radius_of_revolution	distance between the robot and the point (in step)
*/
void get_around(Angle angle_to_revolve, uint16_t radius_of_revolution);

/**
* @brief   start a P_regulator to align the robot and the object (communicate with process_image.h)
*
*/
void set_p_regulator(void);

/**
* @brief   get the angle traveled to align the robot and the object
*
* @return the angle (+: clockwise)(in step)
*/
int16_t get_angle(void);

#endif /* PI_REGULATOR_H */
