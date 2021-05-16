#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//pas sï¿½r que cette valeur ï¿½ besoin d'ï¿½tre dans le .h

#define NORMAL_SPEED 500
#define SLOW_SPEED 250
#define _RIGHT 1
#define _LEFT  -1
#define _FORWARD 1
#define _BACKWARD -1
#define MAXSTEPS 20000
#define MILIRAD_TO_RAD 1000 //je le mets où?
#define MM_TO_STEP 7.69
#define PI 3.1415926
#define FULL_TURN 1213
#define QUARTER_TURN FULL_TURN/4

//start the PI regulator thread
void pi_regulator_start(void);

//pas forcémeent besoin de le mettre là?

//peut-etre creer son propre enum

/**
* @brief
*
* @param
*/
//void set_mode(Process_mode mode_);

/**
* @brief   Uses the motors to get the robot to rotate a certain anmount of steps (using the new functions in motors.h (demande moi au cas oÃ¹)
*
* @param
*/

//void rotate_angle(Angle angle_to_complete, Angular_speed angular_speed);
void rotate_angle(uint16_t angle_to_complete); //uint16_t?
//turn in the given direction

/**
* @brief   function simply makes the robot turn without counting steps (maybe not useful)
*
* @param
*/
void turn(Direction dir); //pas forcï¿½ment besoin de mettre dans le .h peut-ï¿½tre


/**
* @brief   Most complicated function: Makes the robot revolve all the while making sure it is trackable: I'm sure we could use your own function from before Dayan
*
* @param
*/


/**
* @brief   turn around a certain point
*
* @param 	angle_to_revolve		rotation angle in mRad
* 			radisus_of_revolution 	radius of the circle
*/
void revolve_around(Angle angle_to_revolve, uint16_t radius_of_revolution);

/**
* @brief   robot moves at a certain speed (forward or backward)
*
* @param 	dir		direction (FORWARD: 1, BACKWARD: -1)
* 			speed 	speed of the robot (in step/s)
*/
void forward(Direction dir, uint16_t speed);

/**
* @brief   the robot starts moving in circle to find to find the ball
*
*/
void motor_search_ball(void);

/**
* @brief   stops the motor
*
*/
void motor_stop(void);

/**
* @brief   get the distance moved by the robot (after the call of the function forward)
*
* @return distance (in step)
*/
int16_t get_distance(void);

/**
* @brief   get the radius of the current searching circle
*
* @return radius of the circle (in step)
*/
int16_t get_radius(void);


//one-time read: 1:done moving 0:still moving
//vraiment si utile le finishedmoving?

/**
* @brief   know if the motor has finished moving (one time read)
*
* @return 1 if the robot has finished moving
* 		  0 if the robot is still moving, or the result has already been read
*
*/
uint8_t finished_moving(void);

void forward_nb_steps(uint32_t steps_to_complete);

void get_around(Angle angle_to_revolve, uint16_t radius_of_revolution);

void set_p_regulator(void);

#endif /* PI_REGULATOR_H */
