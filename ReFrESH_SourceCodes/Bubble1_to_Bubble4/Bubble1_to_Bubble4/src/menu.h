/* --- menu.h       Version 1.0
   --- This file provides the related function prototype.
   ---
   --- Copyright 2015, Collaborative Robotics Lab.
   ---
   --- 02/10/15 YC	Initial coding.
   ---
   --- */

#ifndef MENU_H_
#define MENU_H_

char menu_on(void *vptr);
char menu_cycle(processT *p_ptr);
char menu_init(processT *p_ptr, void *vptr);

#endif /* MENU_H_ */
