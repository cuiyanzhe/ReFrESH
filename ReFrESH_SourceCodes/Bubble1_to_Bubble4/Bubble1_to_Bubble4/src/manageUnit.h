/*
 * manageUnit.h
 *
 *  Created on: Feb 15, 2015
 *      Author: YanzheCui
 */

#ifndef MANAGEUNIT_H_
#define MANAGEUNIT_H_

char manageUnit_on(processT *p_ptr);
char manageUnit_cycle(processT *p_ptr);
char manageUnit_init(processT *p_ptr, void*vptr);

typedef struct{
	uint8_t	state;			/* flag indicating which state the TASK is in */
} manageUnit_localT;

#endif /* MANAGEUNIT_H_ */
