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

//uint8_t actuatorState;

typedef struct{
/* this part is for Executor (EX) */
	char				*compName ;
	uint8_t				inVarNum;
	uint8_t				outVarNum;
	char				*inVarType[10];
	char				*outVarType[10];
	uint8_t				*inPtr;
	uint8_t				*outPtr;
	uint8_t				nodeNum;
//	uint8_t				runFreq;

	/* this part is for Evaluator (EV), it includes 1) functional evaluated value
	 * and 2) non-functional evaluated value. These values are compared with the
	 * status of the node. */
	uint8_t				funcPerfValue;		/* functional performance value of each component */
	uint8_t				power;
	int					commuRSSI;

	/* TODO:
	 * parameters should be added for ES
	 */
	uint8_t				*estInPtr;
	uint8_t				*estOutPtr;

	uint8_t				actuatorState;
	uint8_t				type;
}actuator_localT;

#endif /* ACTUATOR_H_ */
