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
#include "compStructure.h"


/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */

#define DEBUG 0
#define EVAL_TEST	0

#define IMG_HEIGHT 10
#define IMG_WIDTH  10
#define TEMP_HEIGHT 3
#define TEMP_WIDTH  3

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

/* **********************  traj_set() ***************************************/

char trajGen_set(processT *p_ptr, int16_t type, int16_t arg, void *vptr)
{
	PBOextendT  *pTrajGenLocal = (PBOextendT *)p_ptr->local;
	
	switch(type){
		case DATA_IN:
			pTrajGenLocal->inPtr = (uint8_t *)vptr;
			return I_OK;
		break;

		case DATA_OUT:
			pTrajGenLocal->outPtr = (int8_t *)vptr;
			return I_OK;
		break;

		case EST_INPUT:
			pTrajGenLocal->estInPtr = (uint8_t *)vptr;
			return I_OK;
		break;

		case EST_OUTPUT:
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

/* **********************  traj_cycle() ***************************************/
char trajGen_cycle(processT *p_ptr)
{
	PBOextendT  *pTrajGenLocal = (PBOextendT *)p_ptr->local;
	
	uint8_t xLoc = pTrajGenLocal->inPtr[0];
	uint8_t imgCenter = IMG_WIDTH / 2;
  
	xil_printf("The X Loc got from SSD module is: %d.\r\n", xLoc);

	if(xLoc < imgCenter){
		xil_printf("Turn left!\r\n");
	}else if(xLoc > imgCenter){
		xil_printf("Turn right!\r\n");
	}else{
		xil_printf("Stay there!\r\n");
	}

	return I_OK;
}

/* **********************  traj_init() ***************************************/
char trajGen_init(processT *p_ptr, void*vptr)
{	
	p_ptr->on_fptr = trajGen_on;
	p_ptr->cycle_fptr = trajGen_cycle;
	p_ptr->off_fptr = NULL;
	p_ptr->set_fptr = trajGen_set;
	
	/* Allocate the local structure for the module - optional
	 * This struct will be freed automatically when SBS_KILL is implemented */
	if((p_ptr->local = (pointer)malloc(sizeof(PBOextendT))) == NULL){
			return I_ERROR;
	}
	
	return I_OK;
}
