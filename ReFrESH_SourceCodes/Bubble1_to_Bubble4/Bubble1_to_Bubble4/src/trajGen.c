/* --- trajGen.c       Version 1.0
   --- this module generates the trajectory through the data obtained from SSD output ports.
   ---
   --- Copyright 2015, Collaborative Robotics Lab .
   ---
   --- 02/10/15 YC	Initial coding. V1.0
   --- */

/* ********************************************************************************************************************************************* */
/*      Include Files						                                              														 */
/* ********************************************************************************************************************************************* */

#include <stdio.h>
#include <stdlib.h>		/* for malloc() */
#include "pbort.h"
#include "serial.h"
//#include "compStructure.h"
#include "trajGen.h"


/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */

#define DEBUG 0
#define EVAL_TEST	0

#define TRAJGEN_DEBUG 0
#define TASK_DEBUG 1

#define IMG_HEIGHT 10
#define IMG_WIDTH  10
#define TEMP_HEIGHT 3
#define TEMP_WIDTH  3

#define COMPUTATION  0
#define SENSOR		 1
#define ACTUATOR	 2

/* ********************************************************************************************************************************************* */
/*      global variable						                                              														 */
/* ********************************************************************************************************************************************* */


/* ******************************************************************** */
/*       trajGen_on                Start up the module.                 */
/* ******************************************************************** */

char trajGen_on(processT *p_ptr)
{
	return I_OK;
}


/* ******************************************************************** */
/*       trajGen_set              Set connection among components.      */
/* ******************************************************************** */

char trajGen_set(processT *p_ptr, int16_t type, int16_t arg, void *vptr)
{
	trajGen_localT  *pTrajGenLocal = (trajGen_localT *)p_ptr->local;
	
	switch(type){
		case DATA_IN:
			pTrajGenLocal->inPtr = (uint8_t *)vptr;
			return I_OK;
		break;

		case DATA_OUT:
			pTrajGenLocal->outPtr = (uint8_t *)vptr;
			return I_OK;
		break;

		case EST_DATA_IN:
			pTrajGenLocal->estInPtr = (uint8_t *)vptr;
			return I_OK;
		break;

		case EST_DATA_OUT:
			pTrajGenLocal->estOutPtr = (uint8_t *)vptr;
			return I_OK;
		break;

		/* ---TODO:
		 * This part is for EVLAUATOR, uncomment this part after test each single component
		 */
#if EVAL_TEST
		case FR_EVAL:
			local->FR_Eval = *((uint16_t *)vptr);
			return I_OK;
		break;
		case FR_EST: 
			local->FR_Est = *((uint16_t *)vptr);
			return I_OK;
		break;
		case NFR_EVAL:  
			local->NFR_Eval = *((uint16_t *)vptr);
			return I_OK;
		break;
		case NFR_EST: 
			local->NFR_Est = *((uint16_t *)vptr);
			return I_OK;
		break;
#endif
	}
	
	return I_OK;
}


/* ******************************************************************** */
/*       trajGen_get                      Get parameter from local.     */
/* ******************************************************************** */

char trajGen_get(processT *p_ptr, int16_t type, int16_t arg, void *vptr)
{
	trajGen_localT  *pTrajGenLocal = (trajGen_localT *)p_ptr->local;

	switch(type){
		case DATA_IN:
			*((uint8_t *)vptr) = *(pTrajGenLocal->inPtr);
			return I_OK;
		break;

		case DATA_OUT:
			*((uint16_t *)vptr) = *(pTrajGenLocal->outPtr);
			return I_OK;
		break;

		case IN_SIZE:
			*((size_t *)vptr) = sizeof(pTrajGenLocal->inPtr);
			return I_OK;
		break;

		case OUT_SIZE:
			*((size_t *)vptr) = sizeof(pTrajGenLocal->outPtr);
			return I_OK;
		break;

//		case EST_INPUT:
//			*((uint8_t *)vptr) = *(pTrajGenLocal->estInPtr);
//			return I_OK;
//		break;
//
//		case EST_OUTPUT:
//			*((uint16_t *)vptr) = *(pTrajGenLocal->estOutPtr);
//			return I_OK;
//		break;

		case TYPE:
			*((uint8_t *)vptr) = pTrajGenLocal->type;
			return I_OK;
		break;

		case LOCAL_STATE:
			*((uint8_t *)vptr) = pTrajGenLocal->trajState;
			return I_OK;
		break;

		case NODE_NUM:
			*((uint8_t *)vptr) = pTrajGenLocal->nodeNum;
			return I_OK;
		break;
		/* ---TODO:
		 * This part is for EVLAUATOR, uncomment this part after test each single component
		 */
#if EVAL_TEST
		case FR_EVAL:
			*((uint16_t *)vptr) = pSSDLocal->FR_Eval;
			return I_OK;
		break;
		case FR_EST:
			*((uint16_t *)vptr) = pSSDLocal->FR_Est;
			return I_OK;
		break;
		case NFR_EVAL:
			*((uint16_t *)vptr) = pSSDLocal->NFR_Eval;
			return I_OK;
		break;
		case NFR_EST:
			*((uint16_t *)vptr) = pSSDLocal->NFR_Est;
			return I_OK;
		break;
#endif
	}

	return I_OK;
}


void trajGen_function(trajGen_localT *l_ptr, uint8_t *inType, uint8_t *outType)
{
	uint8_t imgCenter = IMG_WIDTH / 2;

		/* ---TODO:
		 * Implement a real trajectory generator
		 */
	#if TRAJGEN_DEBUG
		uint8_t xLoc = 3;
		xil_printf("The X Loc is: %d.\r\n", xLoc);

		if(xLoc < imgCenter){
			xil_printf("Turn left!\r\n");
		}else if(xLoc > imgCenter){
			xil_printf("Turn right!\r\n");
		}else{
			xil_printf("Stay there!\r\n");
		}
	#endif

	#if TASK_DEBUG
		uint8_t xLoc = inType[0];
		xil_printf("(trajGen.c)The X Loc got from SSD module is: %d.\r\n", xLoc);

		if(xLoc < imgCenter){
			xil_printf("(trajGen.c)Turn left!\r\n");
		}else if(xLoc > imgCenter){
			xil_printf("(trajGen.c)Turn right!\r\n");
		}else{
			xil_printf("(trajGen.c)Stay there!\r\n");
		}
	#endif

}

/* ******************************************************************** */
/*       SSD_cycle              Trajectory generation.                  */
/* ******************************************************************** */

char trajGen_cycle(processT *p_ptr)
{
	trajGen_localT  *pTrajGenLocal = (trajGen_localT *)p_ptr->local;
	
//	uint8_t imgCenter = IMG_WIDTH / 2;
//
//	/* ---TODO:
//	 * Implement a real trajectory generator
//	 */
//#if TRAJGEN_DEBUG
//	uint8_t xLoc = 3;
//	xil_printf("The X Loc is: %d.\r\n", xLoc);
//
//	if(xLoc < imgCenter){
//		xil_printf("Turn left!\r\n");
//	}else if(xLoc > imgCenter){
//		xil_printf("Turn right!\r\n");
//	}else{
//		xil_printf("Stay there!\r\n");
//	}
//#endif
//
//#if TASK_DEBUG
//	uint8_t xLoc = pTrajGenLocal->inPtr[0];
//	xil_printf("(trajGen.c)The X Loc got from SSD module is: %d.\r\n", xLoc);
//
//	if(xLoc < imgCenter){
//		xil_printf("(trajGen.c)Turn left!\r\n");
//	}else if(xLoc > imgCenter){
//		xil_printf("(trajGen.c)Turn right!\r\n");
//	}else{
//		xil_printf("(trajGen.c)Stay there!\r\n");
//	}
//#endif

	trajGen_function(pTrajGenLocal, pTrajGenLocal->inPtr, 0);

	pTrajGenLocal->trajState = 2;

	return I_OK;
}


/* ******************************************************************** */
/*       trajGen_eval                obtain required parameters.        */
/* ******************************************************************** */

char trajGen_eval(processT *p_ptr, int type, int arg, void *vptr)
{
	trajGen_localT  *pTrajGenLocal = (trajGen_localT *)p_ptr->local;

	switch(type){
		case POWER:
			*((uint8_t *)vptr) = pTrajGenLocal->power;
			return I_OK;
		break;

		case LINK_RSSI:
			*((int *)vptr) = pTrajGenLocal->commuRSSI;
			return I_OK;
		break;

		case FUNC_PERF:
			*((uint8_t *)vptr) = pTrajGenLocal->funcPerfValue;
		break;
	}

	return I_OK;
}


/* ******************************************************************** */
/*       trajGen_est                 obtain required parameters.        */
/* ******************************************************************** */

char trajGen_est(processT *p_ptr)
{
	trajGen_localT  *pTrajGenLocal = (trajGen_localT *)p_ptr->local;

	trajGen_function(pTrajGenLocal, pTrajGenLocal->estInPtr, 0);

	return I_OK;
}


/* ******************************************************************** */
/*        trajGen_init             Initiate module information.         */
/* ******************************************************************** */

char trajGen_init(processT *p_ptr, void*vptr)
{	
	p_ptr->on_fptr = trajGen_on;
	p_ptr->cycle_fptr = trajGen_cycle;
	p_ptr->off_fptr = NULL;
	p_ptr->set_fptr = trajGen_set;
	p_ptr->get_fptr = trajGen_get;
	p_ptr->eval_fptr = trajGen_eval;
	p_ptr->est_fptr = trajGen_est;
	
	/* Allocate the local structure for the module - optional
	 * This struct will be freed automatically when SBS_KILL is implemented */
	if((p_ptr->local = (pointer)malloc(sizeof(trajGen_localT))) == NULL){
			return I_ERROR;
	}
	
	/* define a pointer points to local structure */
	trajGen_localT  *pTrajGenLocal = (trajGen_localT *)p_ptr->local;

	pTrajGenLocal->trajState = 0;
	pTrajGenLocal->type = COMPUTATION;
	pTrajGenLocal->power = 25;	/* required power of trajGen */
	pTrajGenLocal->commuRSSI = 5; /* minimum RSSI */
	pTrajGenLocal->nodeNum = 1; /* trajGen on NODE 1 */

	return I_OK;
}
