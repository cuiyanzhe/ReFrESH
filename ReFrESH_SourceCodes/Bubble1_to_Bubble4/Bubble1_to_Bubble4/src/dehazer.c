/* --- dehazer.c       Version 1.0
   --- this module filters the image (currently, just using another good image to simulate dehazing process).
   ---
   --- Copyright 2015, Collaborative Robotics Lab .
   ---
   --- 02/22/15 YC	Initial coding. V1.0
   --- */

/* ********************************************************************************************************************************************* */
/*      Include Files						                                              														 */
/* ********************************************************************************************************************************************* */

#include <string.h>
#include <stdlib.h>		/* for malloc() */
#include "pbort.h"
#include "serial.h"
#include "dehazer.h"

/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */
#define CAMERA_DEBUG 0
#define TASK_DEBUG 1

#define IMG_HEIGHT 10
#define IMG_WIDTH  10

#define COMPUTATION  0
#define SENSOR		 1
#define ACTUATOR	 2

/* ********************************************************************************************************************************************* */
/*      global variable						                                              														 */
/* ********************************************************************************************************************************************* */
uint8_t imgBufferDehazedG[IMG_HEIGHT * IMG_WIDTH] = {
		55, 55, 55, 255, 255, 255, 255, 255, 255, 255,
		55, 55, 55, 255, 255, 255, 255, 255, 255, 255,
		55, 55, 55, 255, 255, 255, 255, 255, 255, 255,
		255, 255, 255, 65, 65, 65, 255, 255, 255, 255,
		255, 255, 255, 65, 65, 65, 255, 255, 255, 255,
		255, 255, 255, 65, 65, 65, 255, 255, 255, 255,
		255, 255, 255, 255, 255, 255, 75, 75, 75, 255,
		255, 255, 255, 255, 255, 255, 75, 75, 75, 255,
		255, 255, 255, 255, 255, 255, 75, 75, 75, 255,
		255, 255, 255, 255, 255, 255, 255, 255, 255, 255};


/* ********************************************************************************************************************************************* */
/*      local function							                                              													 */
/* ********************************************************************************************************************************************* */




/* ******************************************************************** */
/*       dehazer_on                Start up the module.               */
/* ******************************************************************** */

char dehazer_on(processT *p_ptr)
{
	return I_OK;
}

/* ******************************************************************** */
/*       dehazer_set                Set camera reader component.      */
/* ******************************************************************** */

char dehazer_set(processT *p_ptr, int16_t type, int16_t arg, void *vptr)
{
	dehazer_localT  *pDehazerLocal = (dehazer_localT *)p_ptr->local;

	switch(type){
		case DATA_IN:
			pDehazerLocal->inPtr = (uint8_t *)vptr;
			return I_OK;
		break;

		case DATA_OUT:
			pDehazerLocal->outPtr = (uint8_t *)vptr;
			return I_OK;
		break;

		case EST_DATA_IN:
			pDehazerLocal->estInPtr = (uint8_t *)vptr;
			return I_OK;
		break;

		case EST_DATA_OUT:
			pDehazerLocal->estOutPtr = (uint8_t *)vptr;
			return I_OK;
		break;
	}

	return I_OK;
}


/* ******************************************************************** */
/*       dehazer_get                Get parameter from local.         */
/* ******************************************************************** */

char dehazer_get(processT *p_ptr, int16_t type, int16_t arg, void *vptr)
{
	dehazer_localT  *pDehazerLocal = (dehazer_localT *)p_ptr->local;

	switch(type){
		case DATA_IN:
			*((uint8_t *)vptr) = *(pDehazerLocal->inPtr);
			return I_OK;
		break;

		case DATA_OUT:
			*((uint16_t *)vptr) = *(pDehazerLocal->outPtr);
			return I_OK;
		break;

		case IN_SIZE:
			*((size_t *)vptr) = sizeof(pDehazerLocal->inPtr);
			return I_OK;
		break;

		case OUT_SIZE:
			*((size_t *)vptr) = sizeof(pDehazerLocal->outPtr);
			return I_OK;
		break;

		case EST_DATA_IN:
			*((uint8_t *)vptr) = *(pDehazerLocal->inPtr);
			return I_OK;
		break;

		case EST_DATA_OUT:
			*((uint8_t *)vptr) = *(pDehazerLocal->outPtr);
			return I_OK;
		break;

		case TYPE:
			*((uint8_t *)vptr) = pDehazerLocal->type;
			return I_OK;
		break;

		case LOCAL_STATE:
			*((uint8_t *)vptr) = pDehazerLocal->dehazerState;
			return I_OK;
		break;

		case NODE_NUM:
			*((uint8_t *)vptr) = pDehazerLocal->nodeNum;
			return I_OK;
		break;
	}

	return I_OK;
}


void dehazer_function(dehazer_localT *l_ptr, uint8_t *inType, uint8_t *outType)
{
	uint16_t pixelCnt;
//	uint8_t *pImg = imgBufferDehazedG[0];

	/* TODO:
	 * simulated dehazing process */
	for(pixelCnt = 0; pixelCnt < 100; pixelCnt++){
		if(inType[pixelCnt] != imgBufferDehazedG[pixelCnt]){
			outType[pixelCnt] = imgBufferDehazedG[pixelCnt];
		}else{
			outType[pixelCnt] = inType[pixelCnt];
		}
	}

	/* test if the output of outPtr buffer is correct */
	uint8_t i = 0;
	xil_printf("(dehazer.c)Print out pixels info.\r\n");
	for(i = 0; i < 100; i++){
		xil_printf("%d, ", outType[i]);
		if(i % 10 == 9){
			xil_printf("\r\n");
		}
	}
}


/* ******************************************************************** */
/*       dehazer_cycle              Get the frame.                      */
/* ******************************************************************** */

char dehazer_cycle(processT *p_ptr)
{
	dehazer_localT  *pDehazerLocal = (dehazer_localT *)p_ptr->local;

//	uint16_t pixelCnt;
////	uint8_t *pImg = imgBufferDehazedG[0];
//
//	/* TODO:
//	 * simulated dehazing process */
//	for(pixelCnt = 0; pixelCnt < 100; pixelCnt++){
//		if(pDehazerLocal->inPtr[pixelCnt] != imgBufferDehazedG[pixelCnt]){
//			pDehazerLocal->outPtr[pixelCnt] = imgBufferDehazedG[pixelCnt];
//		}else{
//			pDehazerLocal->outPtr[pixelCnt] = pDehazerLocal->inPtr[pixelCnt];
//		}
//	}
//
//	/* test if the output of outPtr buffer is correct */
//	uint8_t i = 0;
//	xil_printf("(dehazer.c)Print out pixels info.\r\n");
//	for(i = 0; i < 100; i++){
//		xil_printf("%d, ", pDehazerLocal->outPtr[i]);
//		if(i % 10 == 9){
//			xil_printf("\r\n");
//		}
//	}

	dehazer_function(pDehazerLocal, pDehazerLocal->inPtr, pDehazerLocal->outPtr);

	pDehazerLocal->dehazerState = 2;

	return I_OK;
}


/* ******************************************************************** */
/*       dehazer_eval              Obtain required parameters.        */
/* ******************************************************************** */

char dehazer_eval(processT *p_ptr, int type, int arg, void *vptr)
{
	dehazer_localT  *pDehazerLocal = (dehazer_localT *)p_ptr->local;

	switch(type){
		case POWER:
			*((uint8_t *)vptr) = pDehazerLocal->power;
			return I_OK;
		break;

		case LINK_RSSI:
			*((int *)vptr) = pDehazerLocal->commuRSSI;
			return I_OK;
		break;
	}

	return I_OK;
}


/* ******************************************************************** */
/*       dehazer_est              Obtain estimated value.               */
/* ******************************************************************** */

char dehazer_est(processT *p_ptr)
{
	dehazer_localT  *pDehazerLocal = (dehazer_localT *)p_ptr->local;

	dehazer_function(pDehazerLocal, pDehazerLocal->estInPtr, pDehazerLocal->estOutPtr);

	return I_OK;
}


/* ******************************************************************** */
/*        dehazer_init           Initiate module information.         */
/* ******************************************************************** */

char dehazer_init(processT *p_ptr, void *vptr)
{
	p_ptr->on_fptr = dehazer_on;
	p_ptr->cycle_fptr = dehazer_cycle;
	p_ptr->off_fptr = NULL;
	p_ptr->set_fptr = dehazer_set;
	p_ptr->get_fptr = dehazer_get;
	p_ptr->eval_fptr = dehazer_eval;
	p_ptr->est_fptr = dehazer_est;

//	xil_printf("init get fptr = %X\r\n",camReader_get);

	/* Allocate the local structure for the module - optional
	 * This struct will be freed automatically when SBS_KILL is implemented */
	if((p_ptr->local = (pointer)malloc(sizeof(dehazer_localT))) == NULL){
		return I_ERROR;
	}

	/* define a pointer points to local structure */
	dehazer_localT  *pDehazerLocal = (dehazer_localT *)p_ptr->local;

	/* malloc a space for the outPtr buffer that would be used by sbsSet*/
//	pCamReaderLocal->outPtr = (uint8_t*)malloc(IMG_HEIGHT * IMG_WIDTH);

	pDehazerLocal->dehazerState = 0;

	pDehazerLocal->type = COMPUTATION;
	pDehazerLocal->power = -20;	/* required power of camReader */
	pDehazerLocal->commuRSSI = 10; /* minimum RSSI if camReader is on the different node as SSD */
	pDehazerLocal->nodeNum = 1;	/* camReader on NODE 1 */

	return I_OK;
}
