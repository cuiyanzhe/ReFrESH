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
	uint8_t	state;
	uint8_t taskFlag;		/* flag indicating if a TASK should be turned off */
	uint8_t nodeNum;		/* management unit is running on which node */
	uint8_t numOfNodes;		/* how many nodes available in the system */
	uint16_t panID;
	uint16_t destAddr;
	uint16_t srcAddr;
} manageUnit_localT;

#endif /* MANAGEUNIT_H_ */
