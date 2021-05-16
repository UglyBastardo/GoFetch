#ifndef BIG_BRAIN_H
#define BIG_BRAIN_H

#endif /* BIG_BRAIN_H */


enum Process_Mode {DoNothing, RotateAndSearch, Align, FinishedAligning, Revolve, Forward, ReAlign, Shoot, Victory};

void 	big_brain_start(void);

void 	set_distance_to_target(uint16_t* new_distance_to_target);
