/* --- SSD.h       Version 1.0
   --- This file provides the related function prototype of SSD.
   ---
   --- Copyright 2015, Collaborative Robotics Lab.
   ---
   --- 02/10/15 YC	Initial coding.
   ---
   --- */

#ifndef SSD_H_
#define SSD_H_

char SSD_on(processT *p_ptr);
char SSD_set(processT *p_ptr, int16_t type, int16_t arg, void *vptr);
char SSD_get(processT *p_ptr, int16_t type, int16_t arg, void *vptr);
char SSD_cycle(processT *p_ptr);
char SSD_init(processT *p_ptr, void*vptr);

uint8_t ssdState;


#endif /* SSD_H_ */
