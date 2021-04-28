#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//pas s�r que cette valeur � besoin d'�tre dans le .h
#define NORMAL_SPEED 500

#define RIGHT 1
#define LEFT 0

//start the PI regulator thread
void pi_regulator_start(void);

//turn in the given direction
void turn(bool direction); //pas forc�ment besoin de mettre dans le .h peut-�tre

//turn around anti-clockwise
void turn_around();

#endif /* PI_REGULATOR_H */
