#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//pas sûr que cette valeur à besoin d'être dans le .h
#define NORMAL_SPEED 500

#define RIGHT 1
#define LEFT 0

//start the PI regulator thread
void pi_regulator_start(void);

//turn in the given direction
void turn(bool direction); //pas forcèment besoin de mettre dans le .h peut-être

//turn around anti-clockwise
void turn_around();

#endif /* PI_REGULATOR_H */
