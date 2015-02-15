/* --- compStructure.h       Version 1.0
   --- This file provides the general structure for a PBO.
   ---
   --- Copyright 2015, Collaborative Robotics Lab.
   ---
   --- 02/10/15 YC	Initial coding.
   ---
   --- */

#ifndef COMPSTRUCTURE_H_
#define COMPSTRUCTURE_H_

//typedef char img;
//typedef int position;
//typedef int err;
#define actuate 3
#define compute 4
#define sensing 5
#define DB_HEAD 6


/* --- Provide information of an Extended-PBO (E-PBO) */
typedef struct _EXTENDED_PBO{
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

}PBOextendT;



typedef struct _COMPONENT_LIST{
	char			*compName;
	PBOextendT		*next;
}compListT;


#endif /* COMPSTRUCTURE_H_ */
