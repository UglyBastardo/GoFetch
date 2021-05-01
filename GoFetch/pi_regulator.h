#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//pas s�r que cette valeur � besoin d'�tre dans le .h
#define NORMAL_SPEED 500
#define _RIGHT 1
#define _LEFT  -1
#define _FORWARD 1
#define _BACKWARD -1
#define MAXSTEPS 20000

//start the PI regulator thread
void pi_regulator_start(void);


/**
* @brief   Uses the motors to get the robot to rotate a certain anmount of steps (using the new functions in motors.h (demande moi au cas où)
*
* @param
*/
void rotate_angle(Angle angle_to_complete, Angular_speed angular_speed);
//turn in the given direction

/**
* @brief   function simply makes the robot turn without counting steps (maybe not useful)
*
* @param
*/
void turn(Direction dir, uint16_t speed); //pas forc�ment besoin de mettre dans le .h peut-�tre


/**
* @brief   Most complicated function: Makes the robot revolve all the while making sure it is trackable: I'm sure we could use your own function from before Dayan
*
* @param
*/
//turn around the target specified by distance to target and angle to align
void revolve_around(Angle angle_to_revolve, uint16_t radius_of_revolution);
//turn around anti-clockwise
//void turn_around(void);

void forward(Direction dir, uint16_t speed);

#endif /* PI_REGULATOR_H */
