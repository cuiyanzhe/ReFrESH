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
char camReader_get(processT *p_ptr, int16_t type, int16_t arg, void *vptr);
char camReader_cycle(processT *p_ptr);
char camReader_init(processT *p_ptr, void *vptr);
char camReader_eval(processT *p_ptr, int type, int arg, void *vptr);
char camReader_est(processT *p_ptr);

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

	uint8_t				cameraState;
	uint8_t				type;
}camReader_localT;

#endif /* CAMREADER_H_ */
