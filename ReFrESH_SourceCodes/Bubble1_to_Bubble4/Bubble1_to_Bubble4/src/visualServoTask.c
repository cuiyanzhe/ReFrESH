/* --- visualServoTask.c       Version 1.0
   --- this module construct a FSM to execute a visual servoing task.
   ---
   --- Copyright 2015, Collaborative Robotics Lab .
   ---
   --- 02/10/15 YC	Initial coding. V1.0  TODO: HAS A BUG, WHEN TURNED OFF AND TURNED ON A TASK, FSM HAS PROBLEM
   --- */

/* ********************************************************************************************************************************************* */
/*      Include Files						                                              														 */
/* ********************************************************************************************************************************************* */

#include <stdio.h>
#include <stdlib.h>		/* for malloc() */
#include "pbort.h"
#include "serial.h"
#include "visualServoTask.h"
#include "refresh_main.h"
#include "camReader.h"
#include "SSD.h"
#include "trajGen.h"
#include "actuator.h"

#include "SSD_new.h"


/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */

#define DEBUG 0
#define EVAL_TEST	0

#define TRAJGEN_DEBUG 0
#define TASK_DEBUG 1

#define IMG_HEIGHT 10
#define IMG_WIDTH  10


/* ********************************************************************************************************************************************* */
/*      global variable						                                              														 */
/* ********************************************************************************************************************************************* */

int waitCnt;
//uint8_t template_cnt;
uint8_t imgBufferG[IMG_HEIGHT * IMG_WIDTH];
uint8_t tempPosBufferG[2];	/* X & Y location in image frame, 2 bytes */

uint8_t camReaderStateG;
uint8_t ssdStateG;
uint8_t trajGenStateG;
uint8_t actuatorStateG;


/* ******************************************************************** */
/*       visualServoTask_on             Start up the module.            */
/* ******************************************************************** */

char visualServoTask_on(processT *p_ptr)
{
//	task_localT  *local = (task_localT *)p_ptr->local;
//
//	local->state = 0;

	return I_OK;
}


/* ******************************************************************** */
/*       visualServoTask_cycle       FSM to execute a task.             */
/* ******************************************************************** */

char visualServoTask_cycle(processT *p_ptr)
{
	task_localT  *local = (task_localT *)p_ptr->local;

//	sbsSet(camReaderIDG, DATA_OUT, 0, (void *)imgBufferG);
//	sbsSet(ssdIDG, DATA_IN, 0, (void *)imgBufferG);
//	sbsSet(ssdIDG, DATA_OUT, 0, (void *)tempPosBufferG);

	switch(local->state){
		case 0:	/* Acquire an image from camera */
			sbsGet(camReaderIDG, LOCAL_STATE, 0, (void *)&camReaderStateG);

			if(camReaderStateG == 0){
//				sbsSet(camReaderIDG, DATA_OUT, 0, (void *)imgBufferG);
				sbsControl(camReaderIDG, SBS_ON);

				camReaderStateG = 1;
			}else if(camReaderStateG == 2){
				camReaderStateG = 0;
				local->state = 1;
			}

//			sbsGet(camReaderIDG, LOCAL_STATE, 0, (void *)&camReaderStateG);
//
////			sbsSet(camReaderIDG, DATA_OUT, 0, (void *)imgBufferG);
//			sbsControl(camReaderIDG, SBS_ON);


//			sbsGet(camReaderIDG, LOCAL_STATE, 0, (void *)&camReaderStateG);
//
//			if(camReaderStateG == 0){
//				sbsSet(camReaderIDG, DATA_OUT, 0, (void *)imgBufferG);
//				sbsControl(camReaderIDG, SBS_ON);
//				local->state = 0;
//			}else if(camReaderStateG == 2){
//				local->state = 1;
//			}

#if DEBUG
			waitCnt = 0; /* initialize stop and stare state counter */

			sbsControl(camReaderIDG, SBS_ON);
			sbsControl(ssdIDG, SBS_ON);

//			uint8_t *imgBuffer = (uint8_t*)malloc(IMG_HEIGHT * IMG_WIDTH);
//			uint8_t *tempPosBuffer = (uint8_t*)malloc(2);	/* X & Y location in image frame, 2 bytes */

			sbsSet(camReaderIDG, DATA_OUT, 0, (void *)imgBufferG);
			sbsSet(ssdIDG, DATA_IN, 0, (void *)imgBufferG);
			sbsSet(ssdIDG, DATA_OUT, 0, (void *)tempPosBufferG);

			sbsControl(camReaderIDG, SBS_OFF);
			sbsControl(ssdIDG, SBS_OFF);
#endif
//			sbsEvaluator(camReaderIDG);  /* why?? */

			local->state = 1;
		break;

		case 1: /* Target Detect */
			/*if(!sbsControl(trajGenIDG, SBS_ON)){
				sbsControl(trajGenIDG, SBS_ON);
			}*/

//			sbsGet(ssdIDG, LOCAL_STATE, 0, (void *)&ssdStateG);
//
////			sbsSet(ssdIDG, DATA_IN, 0, (void *)imgBufferG);
////			sbsSet(ssdIDG, DATA_OUT, 0, (void *)tempPosBufferG);
//
//			sbsControl(ssdIDG, SBS_ON);


			sbsGet(ssdIDG, LOCAL_STATE, 0, (void *)&ssdStateG);

			if(ssdStateG == 0){
//				sbsSet(ssdIDG, DATA_IN, 0, (void *)imgBufferG);
//				sbsSet(ssdIDG, DATA_OUT, 0, (void *)tempPosBufferG);

				sbsControl(ssdIDG, SBS_ON);


				ssdStateG = 1;
			}else if(ssdStateG == 2){
				ssdStateG = 0;
				local->state = 2;
			}
#if DEBUG
			sbsControl(trajGenIDG, SBS_ON);
			sbsSet(trajGenIDG, DATA_IN, 0, (void *)tempPosBufferG);


			sbsControl(trajGenIDG, SBS_OFF);
#endif
			local->state = 2;
		break;

		case 2: /* Trajectory Generation */
//			sbsGet(trajGenIDG, LOCAL_STATE, 0, (void *)&trajGenStateG);
//
//			sbsSet(trajGenIDG, DATA_IN, 0, (void *)tempPosBufferG);
//
//			sbsControl(trajGenIDG, SBS_ON);
//
//			local->state = 3;

			sbsGet(trajGenIDG, LOCAL_STATE, 0, (void *)&trajGenStateG);

			if(trajGenStateG == 0){
//				sbsSet(trajGenIDG, DATA_IN, 0, (void *)tempPosBufferG);

				sbsControl(trajGenIDG, SBS_ON);

				trajGenStateG = 1;
			}else if(trajGenStateG == 2){
				trajGenStateG = 0;
				local->state = 2;
			}
			local->state = 3;

		case 3: /* Move to Target */
//			sbsGet(actuatorIDG, LOCAL_STATE, 0, (void *)&actuatorStateG);
//
//			sbsControl(actuatorIDG, SBS_ON);

			sbsGet(actuatorIDG, LOCAL_STATE, 0, (void *)&actuatorStateG);

			if(actuatorStateG == 0){
				sbsControl(actuatorIDG, SBS_ON);

				actuatorStateG = 1;
			}else if(actuatorStateG == 2){
				actuatorStateG = 0;
				local->state = 4;
			}

			local->state = 4;
		break;

		case 4: /* Stay and Stare */
			for(waitCnt = 0; waitCnt < 5; waitCnt++){
				xil_printf("Wait %d seconds\r\n", waitCnt);
			}
//			sbsControl(manageUnitIDG, SBS_ON);
			local->state = 5;
		break;

		case 5: /* Turn on management unit */
			sbsControl(manageUnitIDG, SBS_ON);
//			xil_printf("(task.c)camReaderIDG = %X\r\n", camReaderIDG);
//			xil_printf("(task.c)ssdIDG = %X\r\n", ssdIDG);
//			xil_printf("(task.c)trajGenIDG = %X\r\n", trajGenIDG);
//			xil_printf("(task.c)actuatorIDG = %X\r\n", actuatorIDG);
//			xil_printf("(task.c)menuIDG = %X\r\n", menuIDG);
//			xil_printf("(task.c)visualServoTaskIDG = %X\r\n", visualServoTaskIDG);
//			xil_printf("(task.c)manageUnitIDG = %X\r\n", manageUnitIDG);
			local->state = 0;
		break;
	}

	return I_OK;
}


/* ******************************************************************** */
/*        visualServoTask_init          Initiate module information.    */
/* ******************************************************************** */

char visualServoTask_init(processT *p_ptr, void *vptr)
{
	p_ptr->on_fptr = visualServoTask_on;
	p_ptr->cycle_fptr = visualServoTask_cycle;
	p_ptr->off_fptr = NULL;

//	p_ptr->eval_fptr = NULL;

	/* Allocate the local structure for the module - optional
	 * This struct will be freed automatically when SBS_KILL is implemented */
	if((p_ptr->local = (pointer)malloc(sizeof(task_localT))) == NULL){
		return I_ERROR;
	}

	task_localT  *local = (task_localT *)p_ptr->local;

	local->state = 0;

	return I_OK;
}
