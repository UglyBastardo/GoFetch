#ifndef SIMPLE_MOVEMENTS_H
#define SIMPLE_MOVEMENTS_H

#define HALT 			0
#define MODE_INFINITE 	1
#define MODE_FINITE 	2

#define TRIGONOMETRIC	0
#define CLOCK		 	1

#define RIGHT 			1
#define LEFT 			-1

#define FORWARDS 	0
#define ROTATE		1
#define REVOLVE		2

#define FIRSTSTATE  0
#define SECONDSTATE 1
#define THIRDSTATE  2
#define REINIT 		FIRSTSTATE

#define X			0
#define Y 			1
#define ANGLE 		2

//================================================================================
/*	Some functions to make life easier
 *	Mostly calculations
 *
 */
//================================================================================

//Function to make sure the MODE_FINITE does not stop or reinitialise while counting
uint8_t verify_done_moving(uint8_t mode, int speed, uint32_t nbSteps, uint8_t (*f)(uint8_t, int, uint32_t));

//Function to calculate positions relative to the (x,y,alpha) = (0,0,0) point
int *delta_pos(uint8_t mvtType, int nbSteps_done_right_motor, int nbSteps_done_left_motor, uint32_t radius);

//Function to calculate position using displacement (prime) and initial positions (init)
int *calculate_position(double xinit, double yinit, double angle_init, double xprime, double yprime, double angle_prime);

int calculate_new_x(int xinit, int angle, uint32_t distance_Steps);
int calculate_new_y(int yinit, int angle, uint32_t distance_Steps);
int calculate_new_angle(int32_t left_rotation, int32_t right_rotation, int old_angle);/*unit is steps*/
int calculate_distance(int dx, int dy);
int get_state(void);

//================================================================================
/*	Getter Functions
 *
 */
//================================================================================

int get_temp_xpos(void);
int get_temp_ypos(void);
int get_temp_angle(void);


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
