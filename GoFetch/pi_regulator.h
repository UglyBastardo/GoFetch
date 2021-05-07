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

//start the PI regulator thread
void pi_regulator_start(void);

//pas forcémeent besoin de le mettre là?
enum motor_mode{Stop, TurnAround, IncreaseRadius, DoNothing_, CurrentlyMoving, FinishedMoving};

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
//turn around the target specified by distance to target and angle to align
void revolve_around(Angle angle_to_revolve, uint16_t radius_of_revolution);

void forward(Direction dir, uint16_t speed);

void motor_search_ball(void);

void motor_stop(void);

//one-time read: 1:done moving 0:still moving
//vraiment si utile le finishedmoving?
uint8_t finished_moving();

#endif /* PI_REGULATOR_H */
