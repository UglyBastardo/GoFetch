#ifndef SIMPLE_MOVEMENTS_H
#define SIMPLE_MOVEMENTS_H

#define HALT 			0
#define MODE_INFINITE 	1
#define MODE_FINITE 	2

#define TRIGONOMETRIC	0
#define CLOCK		 	1

#define RIGHT 			1
#define LEFT 			-1

//================================================================================
/*	Some functions to make life easier
 *
 *
 */
//================================================================================
uint8_t verify_done_moving(uint8_t mode, int speed, uint32_t nbSteps, uint8_t (*f)(uint8_t, int, uint32_t));



//================================================================================
/*	Begin of the Complicated Movement Functions
 *
 *
 */
//================================================================================
//make the motor rotate with speed proportional to error as long as mode ongoing is set;
uint8_t P_align(uint8_t ongoing, int error, int tolerance);

//changes the radius of the robots revolution.
//Goes to the RIGHT=1 or to the LEFT=0
uint8_t translational_movement(int side, int delta_radius, int execution_speed);

//is a slower function for bring the robot whereever we want it to be
uint8_t go_to_pos(double xinit, double yinit, double angle_init, double xfinal, double yfinal, double angle_final, int execution_speed);

//================================================================================
/*	Begin of the Simple Movement Functions
 *
 *
 */
//================================================================================
//mode can be 0=HALT, 1=forwards independant of steps=MODE_INFINITE, 2=forwards dependant of steps=MODE_FINITE
uint8_t forwards(uint8_t mode, int speed, uint32_t nbSteps);

//mode can be 0=HALT, 1=rotate independant of steps=MODE_INFINITE, 2=rotate dependant of steps=MODE_FINITE
//rotation is in trigonometric way if speed is positive
uint8_t rotate(uint8_t mode, int speed, uint32_t nbSteps);

//mode can be 0=HALT, 1=revolve independant of steps=MODE_INFINITE, 2=revolve dependant of steps=MODE_FINITE
//radius is given in steps
//direction is either 0 = TRIGONOMETRIC, or 1 = CLOCK
uint8_t revolve(uint8_t mode, int speed, int radius, uint8_t direction, uint32_t nbSteps);

//This functions simply stops the robot from moving
void halt(void);

//This function simply resets the motor step count for ease of calculation and low risk of overflow
void reset_step_count(void);

#endif /* PI_REGULATOR_H */
