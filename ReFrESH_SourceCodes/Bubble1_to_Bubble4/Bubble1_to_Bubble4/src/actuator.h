/* --- actuator.h       Version 1.0
   --- This file provides the related function prototype of actuator(tbot, hexmanipulator, hexrotor, etc.).
   ---
   --- Copyright 2015, Collaborative Robotics Lab.
   ---
   --- 02/10/15 YC	Initial coding.
   ---
   --- */

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

char actuator_on(processT *p_ptr);
char actuator_cycle(processT *p_ptr);
char actuator_init(processT *p_ptr, void *vptr);

uint8_t actuatorState;

#endif /* ACTUATOR_H_ */
