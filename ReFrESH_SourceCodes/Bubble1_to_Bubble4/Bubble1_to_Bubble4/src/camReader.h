/* --- camReader.h       Version 1.0
   --- This file provides the related function prototype of frame grabber.
   ---
   --- Copyright 2015, Collaborative Robotics Lab.
   ---
   --- 02/10/15 YC	Initial coding.
   ---
   --- */

#ifndef CAMREADER_H_
#define CAMREADER_H_

char camReader_on(processT *p_ptr);
char camReader_set(processT *p_ptr, int16_t type, int16_t arg, void *vptr);
char camReader_cycle(processT *p_ptr);
char camReader_init(processT *p_ptr, void *vptr);

#endif /* CAMREADER_H_ */
