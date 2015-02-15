/* --- trajGen.h       Version 1.0
   --- This file provides the related function prototype.
   ---
   --- Copyright 2015, Collaborative Robotics Lab.
   ---
   --- 02/10/15 YC	Initial coding.
   ---
   --- */
   
#ifndef		TRAJGen_H_
#define		TRAJGen_H_

char trajGen_on(processT *p_ptr);
char trajGen_set(processT *p_ptr, int16_t type, int16_t arg, void *vptr);
char trajGen_cycle(processT *p_ptr);
char trajGen_init(processT *p_ptr, void*vptr);

uint8_t trajState;


#endif	/* MENU_H_ */
