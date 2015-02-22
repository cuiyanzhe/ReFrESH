/* --- camReader.c       Version 1.0
   --- this module provides the function of frame grabber of a camera and set both func & non-func characteristics.
   ---
   --- Copyright 2015, Collaborative Robotics Lab .
   ---
   --- 02/10/15 YC	Initial coding. V1.0
   --- */

/* ********************************************************************************************************************************************* */
/*      Include Files						                                              														 */
/* ********************************************************************************************************************************************* */

#include <string.h>
#include <stdlib.h>		/* for malloc() */
#include "pbort.h"
//#include "compStructure.h"
#include "serial.h"
#include "camReader.h"

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
uint8_t imgBufferTestG[IMG_HEIGHT][IMG_WIDTH] = {
		{56, 55, 55, 255, 255, 255, 255, 255, 255, 255},
		{55, 56, 55, 255, 255, 255, 255, 255, 255, 255},
		{55, 55, 56, 255, 255, 255, 255, 255, 255, 255},
		{255, 255, 255, 65, 65, 65, 255, 255, 255, 255},
		{255, 255, 255, 65, 65, 65, 255, 255, 255, 255},
		{255, 255, 255, 65, 65, 65, 255, 255, 255, 255},
		{255, 255, 255, 255, 255, 255, 75, 75, 75, 255},
		{255, 255, 255, 255, 255, 255, 75, 75, 75, 255},
		{255, 255, 255, 255, 255, 255, 75, 75, 75, 255},
		{255, 255, 255, 255, 255, 255, 255, 255, 255, 255}
};

uint8_t imgBufferBadTestG[IMG_HEIGHT][IMG_WIDTH] = {
		{55, 65, 65, 255, 255, 255, 255, 255, 255, 255},
		{55, 60, 60, 255, 255, 255, 255, 255, 255, 255},
		{60, 65, 65, 255, 255, 255, 255, 255, 255, 255},
		{255, 255, 255, 65, 65, 65, 255, 255, 255, 255},
		{255, 255, 255, 65, 65, 65, 255, 255, 255, 255},
		{255, 255, 255, 65, 65, 65, 255, 255, 255, 255},
		{255, 255, 255, 255, 255, 255, 75, 75, 75, 255},
		{255, 255, 255, 255, 255, 255, 75, 75, 75, 255},
		{255, 255, 255, 255, 255, 255, 75, 75, 75, 255},
		{255, 255, 255, 255, 255, 255, 255, 255, 255, 255}
};


/* ********************************************************************************************************************************************* */
/*      local function							                                              													 */
/* ********************************************************************************************************************************************* */




/* ******************************************************************** */
/*       camReader_on                Start up the module.               */
/* ******************************************************************** */

char camReader_on(processT *p_ptr)
{
	return I_OK;
}

/* ******************************************************************** */
/*       camReader_set                Set camera reader component.      */
/* ******************************************************************** */

char camReader_set(processT *p_ptr, int16_t type, int16_t arg, void *vptr)
{
	camReader_localT  *pCamReaderLocal = (camReader_localT *)p_ptr->local;

	switch(type){
		case DATA_IN:
			pCamReaderLocal->inPtr = (uint8_t *)vptr;
			return I_OK;
		break;

		case DATA_OUT:
			pCamReaderLocal->outPtr = (uint8_t *)vptr;
			return I_OK;
		break;

		case EST_DATA_OUT:
			pCamReaderLocal->estOutPtr = (uint8_t *)vptr;
			return I_OK;

//		case EST_INPUT:
//			pCamReaderLocal->estInPtr = (uint8_t *)vptr;
//			return I_OK;
//		break;
//
//		case EST_OUTPUT:
//			pCamReaderLocal->estOutPtr = (uint8_t *)vptr;
//			return I_OK;
//		break;

/*		case FR_EVAL:
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
		break;*/
	}

	return I_OK;
}


/* ******************************************************************** */
/*       camReader_get                Get parameter from local.         */
/* ******************************************************************** */

char camReader_get(processT *p_ptr, int16_t type, int16_t arg, void *vptr)
{
	camReader_localT  *pCamReaderLocal = (camReader_localT *)p_ptr->local;

	switch(type){
		case DATA_IN:
			*((uint8_t *)vptr) = *(pCamReaderLocal->inPtr);
			return I_OK;
		break;

		case DATA_OUT:
			*((uint16_t *)vptr) = *(pCamReaderLocal->outPtr);
			return I_OK;
		break;

		case IN_SIZE:
			*((size_t *)vptr) = sizeof(pCamReaderLocal->inPtr);
			return I_OK;
		break;

		case OUT_SIZE:
			*((size_t *)vptr) = sizeof(pCamReaderLocal->outPtr);
			return I_OK;
		break;

//		case EST_INPUT:
//			*((uint8_t *)vptr) = *(pCamReaderLocal->estInPtr);
//			return I_OK;
//		break;
//
//		case EST_OUTPUT:
//			*((uint16_t *)vptr) = *(pCamReaderLocal->estOutPtr);
//			return I_OK;
//		break;

		case TYPE:
			*((uint8_t *)vptr) = pCamReaderLocal->type;
			return I_OK;
		break;

		case LOCAL_STATE:
			*((uint8_t *)vptr) = pCamReaderLocal->cameraState;
			return I_OK;
		break;

		case NODE_NUM:
			*((uint8_t *)vptr) = pCamReaderLocal->nodeNum;
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

void camReader_function(camReader_localT *l_ptr, uint8_t *inType, uint8_t *outType)
{
	/* --- TODO:
		 * make a camReader subfunction that return the value to a 2D array, named imgBuffer;
		 * for(i = 0; i < IMG_SIZE; i++){
		 *    imgBuffer[i++] = camReader;
		 * }
		 */
		uint16_t pixelCnt = 0;
		uint8_t *pImg = imgBufferTestG[0];

	//	xil_printf("TP1\r\n");

	#if CAMERA_DEBUG
		uint8_t testImgBuf[IMG_HEIGHT * IMG_WIDTH];
		while(pixelCnt < IMG_HEIGHT * IMG_WIDTH){
			testImgBuf[pixelCnt] = pImg[pixelCnt];
			pixelCnt++;
		}

		uint8_t i = 0;
		for(i = 0; i < 100; i++){
			xil_printf("%d, ", testImgBuf[i]);
			if(i % 10 == 9){
				xil_printf("\r\n");
			}
		}
	#endif

	#if TASK_DEBUG
		while(pixelCnt < IMG_HEIGHT * IMG_WIDTH){
			outType[pixelCnt] = pImg[pixelCnt];
			pixelCnt++;
		}
		/* test if the output of outPtr buffer is correct */
		uint8_t i = 0;
		xil_printf("(camReader.c)Print out pixels info.\r\n");
		for(i = 0; i < 100; i++){
			xil_printf("%d, ", outType[i]);
			if(i % 10 == 9){
				xil_printf("\r\n");
			}
		}
	#endif
}

/* ******************************************************************** */
/*       camReader_cycle              Get the frame.                    */
/* ******************************************************************** */

char camReader_cycle(processT *p_ptr)
{
	camReader_localT  *pCamReaderLocal = (camReader_localT *)p_ptr->local;

//	/* --- TODO:
//	 * make a camReader subfunction that return the value to a 2D array, named imgBuffer;
//	 * for(i = 0; i < IMG_SIZE; i++){
//	 *    imgBuffer[i++] = camReader;
//	 * }
//	 */
//	uint16_t pixelCnt = 0;
//	uint8_t *pImg = imgBufferTestG[0];
//
////	xil_printf("TP1\r\n");
//
//#if CAMERA_DEBUG
//	uint8_t testImgBuf[IMG_HEIGHT * IMG_WIDTH];
//	while(pixelCnt < IMG_HEIGHT * IMG_WIDTH){
//		testImgBuf[pixelCnt] = pImg[pixelCnt];
//		pixelCnt++;
//	}
//
//	uint8_t i = 0;
//	for(i = 0; i < 100; i++){
//		xil_printf("%d, ", testImgBuf[i]);
//		if(i % 10 == 9){
//			xil_printf("\r\n");
//		}
//	}
//#endif
//
//#if TASK_DEBUG
//	while(pixelCnt < IMG_HEIGHT * IMG_WIDTH){
//		pCamReaderLocal->outPtr[pixelCnt] = pImg[pixelCnt];
//		pixelCnt++;
//	}
//	/* test if the output of outPtr buffer is correct */
//	uint8_t i = 0;
//	xil_printf("(camReader.c)Print out pixels info.\r\n");
//	for(i = 0; i < 100; i++){
//		xil_printf("%d, ", pCamReaderLocal->outPtr[i]);
//		if(i % 10 == 9){
//			xil_printf("\r\n");
//		}
//	}
//#endif

	camReader_function(pCamReaderLocal, 0, pCamReaderLocal->outPtr);

	pCamReaderLocal->cameraState = 2;

	return I_OK;
}


/* ******************************************************************** */
/*       camReader_eval              Obtain required parameters.        */
/* ******************************************************************** */

char camReader_eval(processT *p_ptr, int type, int arg, void *vptr)
{
	camReader_localT  *pCamReaderLocal = (camReader_localT *)p_ptr->local;

	switch(type){
		case POWER:
			*((uint8_t *)vptr) = pCamReaderLocal->power;
			return I_OK;
		break;

		case LINK_RSSI:
			*((int *)vptr) = pCamReaderLocal->commuRSSI;
			return I_OK;
		break;
	}

	return I_OK;
}


/* ******************************************************************** */
/*       SSD_est              Obtain estimated value.               */
/* ******************************************************************** */

char camReader_est(processT *p_ptr)
{
	camReader_localT  *pCamReaderLocal = (camReader_localT *)p_ptr->local;

	camReader_function(pCamReaderLocal, 0, pCamReaderLocal->estOutPtr);

	return I_OK;
}


/* ******************************************************************** */
/*        camReader_init           Initiate module information.         */
/* ******************************************************************** */

char camReader_init(processT *p_ptr, void *vptr)
{
	p_ptr->on_fptr = camReader_on;
	p_ptr->cycle_fptr = camReader_cycle;
	p_ptr->off_fptr = NULL;
	p_ptr->set_fptr = camReader_set;
	p_ptr->get_fptr = camReader_get;
	p_ptr->eval_fptr = camReader_eval;
	p_ptr->est_fptr = camReader_est;

//	xil_printf("init get fptr = %X\r\n",camReader_get);

	/* Allocate the local structure for the module - optional
	 * This struct will be freed automatically when SBS_KILL is implemented */
	if((p_ptr->local = (pointer)malloc(sizeof(camReader_localT))) == NULL){
		return I_ERROR;
	}

	/* define a pointer points to local structure */
	camReader_localT  *pCamReaderLocal = (camReader_localT *)p_ptr->local;

	/* malloc a space for the outPtr buffer that would be used by sbsSet*/
//	pCamReaderLocal->outPtr = (uint8_t*)malloc(IMG_HEIGHT * IMG_WIDTH);

	pCamReaderLocal->cameraState = 0;

	pCamReaderLocal->type = SENSOR;
	pCamReaderLocal->power = 10;	/* required power of camReader */
	pCamReaderLocal->commuRSSI = 10; /* minimum RSSI if camReader is on the different node as SSD */
	pCamReaderLocal->nodeNum = 1;	/* camReader on NODE 1 */

	return I_OK;
}
