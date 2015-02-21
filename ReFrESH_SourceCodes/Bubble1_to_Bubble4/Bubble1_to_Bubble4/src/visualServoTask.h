/* --- visualServoTask.h       Version 1.0
   --- This file provides the related function prototype.
   ---
   --- Copyright 2015, Collaborative Robotics Lab.
   ---
   --- 02/10/15 YC	Initial coding.
   ---
   --- */


#ifndef VISUALSERVOTASK_H_
#define VISUALSERVOTASK_H_

#include "pbort.h"

typedef struct{
	uint8_t	state;			/* flag indicating which state the TASK is in */
} task_localT;

char visualServoTask_on(processT *p_ptr);
char visualServoTask_cycle(processT *p_ptr);
char visualServoTask_init(processT *p_ptr, void *vptr);

extern processT *processG;

#endif /* VISUALSERVOTASK_H_ */
