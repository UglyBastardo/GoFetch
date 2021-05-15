#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//pas s�r que cette valeur � besoin d'�tre dans le .h

#define NORMAL_SPEED 500
#define SLOW_SPEED 250
#define _RIGHT 1
#define _LEFT  -1
#define _FORWARD 1
#define _BACKWARD -1
#define MAXSTEPS 20000
#define MILIRAD_TO_RAD 1000 //je le mets o�?

//start the PI regulator thread
void program_regulator_start(void);

#endif /* PI_REGULATOR_H */
