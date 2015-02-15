/* --- SSD.c       Version 1.0
   --- this module implements the function of finding a target by using SUM OF SQUARED DIFFERENCE(SSD).
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
#include "SSD.h"
#include "compStructure.h"


/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */

#define DEBUG 0
#define EVAL_TEST	0

#define SSD_DEBUG 0
#define TASK_DEBUG 1

#define IMG_HEIGHT 10
#define IMG_WIDTH  10
#define TEMP_HEIGHT 3
#define TEMP_WIDTH  3


/* ********************************************************************************************************************************************* */
/*      global variable						                                              														 */
/* ********************************************************************************************************************************************* */

uint8_t imgBufferTestG[IMG_HEIGHT * IMG_WIDTH] = {
		55, 55, 55, 255, 255, 255, 255, 255, 255, 255,
		55, 55, 55, 255, 255, 255, 255, 255, 255, 255,
		55, 55, 55, 255, 255, 255, 255, 255, 255, 255,
		255, 255, 255, 65, 65, 65, 255, 255, 255, 255,
		255, 255, 255, 65, 65, 65, 255, 255, 255, 255,
		255, 255, 255, 65, 65, 65, 255, 255, 255, 255,
		255, 255, 255, 255, 255, 255, 75, 75, 75, 255,
		255, 255, 255, 255, 255, 255, 75, 75, 75, 255,
		255, 255, 255, 255, 255, 255, 75, 75, 75, 255,
		255, 255, 255, 255, 255, 255, 255, 255, 255, 255
};

uint8_t template1G[TEMP_HEIGHT * TEMP_WIDTH] = {
		53, 55, 55,
		55, 55, 55,
		55, 55, 55
};

uint8_t template2G[TEMP_HEIGHT * TEMP_WIDTH] = {
		65, 65, 65,
		65, 65, 65,
		65, 65, 65
};

uint8_t template3G[TEMP_HEIGHT * TEMP_WIDTH] = {
		75, 75, 75,
		75, 75, 75,
		75, 75, 75
};

//uint8_t *inValG;
/* ******************************************************************** */
/*       SSD_on                Start up the module.                    */
/* ******************************************************************** */

char SSD_on(processT *p_ptr)
{
	return I_OK;
}


/* ******************************************************************** */
/*       SSD_set                      Set SSD component's ports.        */
/* ******************************************************************** */

char SSD_set(processT *p_ptr, int16_t type, int16_t arg, void *vptr)
{
	PBOextendT  *pSSDLocal = (PBOextendT *)p_ptr->local;

	switch(type){
		case DATA_IN:
			pSSDLocal->inPtr = (uint8_t *)vptr;
			return I_OK;
		break;

		case DATA_OUT:
			pSSDLocal->outPtr = (uint8_t *)vptr;
			return I_OK;
		break;

		case EST_INPUT:
			pSSDLocal->estInPtr = (uint8_t *)vptr;
			return I_OK;
		break;

		case EST_OUTPUT:
			pSSDLocal->estOutPtr = (uint8_t *)vptr;
			return I_OK;
		break;

		/* ---TODO:
		 * This part is for EVLAUATOR, uncomment this part after test each single component
		 */
#if EVAL_TEST
		case TEMP_COUNT:
			pSSDLocal->temp_count = *((uint8_t *)vptr);
		break;

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
/*       SSD_get                      Get SSD component's content.      */
/* ******************************************************************** */

char SSD_get(processT *p_ptr, int16_t type, int16_t arg, void *vptr)
{
	PBOextendT  *pSSDLocal = (PBOextendT *)p_ptr->local;

	switch(type){
		case DATA_IN:
			*((uint8_t *)vptr) = *(pSSDLocal->inPtr);
			return I_OK;
		break;

		case DATA_OUT:
			*((uint16_t *)vptr) = *(pSSDLocal->outPtr);
			return I_OK;
		break;

		case IN_SIZE:
			*((size_t *)vptr) = sizeof(pSSDLocal->inPtr);
			return I_OK;
		break;

		case OUT_SIZE:
			*((size_t *)vptr) = sizeof(pSSDLocal->outPtr);
			return I_OK;
		break;

		case EST_INPUT:
			*((uint8_t *)vptr) = *(pSSDLocal->estInPtr);
			return I_OK;
		break;

		case EST_OUTPUT:
			*((uint16_t *)vptr) = *(pSSDLocal->estOutPtr);
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


/* ******************************************************************** */
/*       SSD_cycle              Get the position of target and error.   */
/* ******************************************************************** */

char SSD_cycle(processT *p_ptr)
{
	PBOextendT  *pSSDLocal = (PBOextendT *)p_ptr->local;

	short 	tmpShort;
	char	tmpChar;
	uint8_t		i, j, k, l;
	uint8_t		pixel_error = 0;
	uint8_t		targetXLoc=0;
	uint8_t		targetYLoc=0;
	uint8_t		errorImg = 50;       //max positive value for a long type

	for(i=0; i<=(IMG_HEIGHT-TEMP_HEIGHT); i++){
		for(j=0; j<=(IMG_WIDTH-TEMP_WIDTH); j++){
			for(k=0; k<TEMP_HEIGHT; k++){
				for(l=0; l<TEMP_WIDTH; l++){
#if TASK_DEBUG
					pixel_error = pixel_error + ((int)pSSDLocal->inPtr[(i + k) * IMG_WIDTH + j + l]-(int)template3G[k * TEMP_WIDTH + l])*
							((int)pSSDLocal->inPtr[(i + k) * IMG_WIDTH + j + l] - (int)template3G[k * TEMP_WIDTH + l]);
#endif

#if SSD_DEBUG
					pixel_error = pixel_error + ((int)imgBufferTestG[(i + k) * IMG_WIDTH + j + l]-(int)template2G[k * TEMP_WIDTH + l])*
							((int)imgBufferTestG[(i + k) * IMG_WIDTH + j + l] - (int)template2G[k * TEMP_WIDTH + l]);
#endif
				}
			}

			if(pixel_error < errorImg){
				errorImg = pixel_error;
				targetXLoc = j;
				targetYLoc = i;
			}
			pixel_error = 0;
		}
	}

//	xil_printf("Smallest Error: %d, X Loc: %d, Y Loc: %d\r\n", errorImg, targetXLoc, targetYLoc);

#if TASK_DEBUG
	pSSDLocal->outPtr[0] = targetXLoc;
	pSSDLocal->outPtr[1] = targetYLoc;
	xil_printf("(SSD.c)SSD output ports: X Loc: %d, Y Loc: %d\r\n", pSSDLocal->outPtr[0], pSSDLocal->outPtr[1]);
#endif

	ssdState = 2;

	return I_OK;

}


/* ******************************************************************** */
/*        SSD_init                 Initiate module information.         */
/* ******************************************************************** */

char SSD_init(processT *p_ptr, void*vptr)
{
	p_ptr->on_fptr = SSD_on;
	p_ptr->cycle_fptr = SSD_cycle;
	p_ptr->off_fptr = NULL;

#if DEBUG
	xil_printf("(SSD_init)Debug the affection of SET function\r\n");
#else
	p_ptr->set_fptr = SSD_set;
	p_ptr->get_fptr = SSD_get;


	/* Allocate the local structure for the module - optional
	 * This struct will be freed automatically when SBS_KILL is implemented */
	if((p_ptr->local = (pointer)malloc(sizeof(PBOextendT))) == NULL){
			return I_ERROR;
	}

	/* define a pointer points to local structure */
//	PBOextendT  *pSSDLocal = (PBOextendT *)p_ptr->local;
//	pSSDLocal->inPtr = &inValG;


#endif

	ssdState = 0;

	return I_OK;
}


