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
/*	Begin of the Complicated Movement Functions
 *
 *
 */
//================================================================================
//make the motor rotate with speed proportional to error as long as mode ongoing is set;
void P_align(uint8_t ongoing, int error);

//changes the radius of the robots revolution.
//Goes to the RIGHT=1 or to the LEFT=0
void translational_movement(int side, int delta_radius, int execution_speed);

//================================================================================
/*	Begin of the Simple Movement Functions
 *
 *
 */
//================================================================================
//mode can be 0=HALT, 1=forwards independant of steps=MODE_INFINITE, 2=forwards dependant of steps=MODE_FINITE
void forwards(uint8_t mode, int speed, uint32_t nbSteps);

//mode can be 0=HALT, 1=rotate independant of steps=MODE_INFINITE, 2=rotate dependant of steps=MODE_FINITE
//rotation is in trigonometric way if speed is positive
void rotate(uint8_t mode, int speed, uint32_t nbSteps);

//mode can be 0=HALT, 1=revolve independant of steps=MODE_INFINITE, 2=revolve dependant of steps=MODE_FINITE
//radius is given in steps
//direction is either 0 = TRIGONOMETRIC, or 1 = CLOCK
void revolve(uint8_t mode, int speed, int radius, uint8_t direction, uint32_t nbSteps);

//This functions simply stops the robot from moving
void halt(void);

//This function simply resets the motor step count for ease of calculation and low risk of overflow
void reset_step_count(void);

#endif /* PI_REGULATOR_H */
