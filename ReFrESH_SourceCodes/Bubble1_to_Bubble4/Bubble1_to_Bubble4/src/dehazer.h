/* --- dehazer.h       Version 1.0
   --- This file provides the related function prototype of dehazer.
   ---
   --- Copyright 2015, Collaborative Robotics Lab.
   ---
   --- 02/10/15 YC	Initial coding.
   ---
   --- */

#ifndef DEHAZER_H_
#define DEHAZER_H_

char dehazer_on(processT *p_ptr);
char dehazer_set(processT *p_ptr, int16_t type, int16_t arg, void *vptr);
char dehazer_get(processT *p_ptr, int16_t type, int16_t arg, void *vptr);
char dehazer_cycle(processT *p_ptr);
char dehazer_init(processT *p_ptr, void *vptr);
char dehazer_eval(processT *p_ptr, int type, int arg, void *vptr);
char dehazer_est(processT *p_ptr);

//uint8_t cameraState;   ////////////////////////////////////fault

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
	int					power;
	int					commuRSSI;

	/* TODO:
	 * parameters should be added for ES
	 */
	uint8_t				*estInPtr;
	uint8_t				*estOutPtr;

	uint8_t				dehazerState;
	uint8_t				type;
}dehazer_localT;

#endif /* DEHAZER_H_ */
