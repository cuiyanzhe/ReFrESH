/* --- visualServoTask.c       Version 1.0
   --- this module construct a FSM to execute a visual servoing task.
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
#include "visualServoTask.h"
#include "refresh_main.h"


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

/* ******************************************************************** */
/*       visualServoTask_on             Start up the module.            */
/* ******************************************************************** */

char visualServoTask_on(processT *p_ptr)
{
	task_localT  *local = (task_localT *)p_ptr->local;

	local->state = 0;
//	template_cnt = 0;

	return I_OK;
}

/* ******************************************************************** */
/*       visualServoTask_cycle       FSM to execute a task.             */
/* ******************************************************************** */

char visualServoTask_cycle(processT *p_ptr)
{
	task_localT  *local = (task_localT *)p_ptr->local;

	switch(local->state){
		case 0:	/* Target Search: camReaderIDG + ssdIDG */
			/*if(!sbsControl(camReaderIDG, SBS_ON)){
				sbsControl(camReaderIDG, SBS_ON);
			}
			if(!sbsControl(ssdIDG, SBS_ON)){
				sbsControl(ssdIDG, SBS_ON);
			}*/

			waitCnt = 0; /* initialize stop and stare state counter */

			sbsControl(camReaderIDG, SBS_ON);
			sbsControl(ssdIDG, SBS_ON);

//			uint8_t *imgBuffer = (uint8_t*)malloc(IMG_HEIGHT * IMG_WIDTH);
//			uint8_t *tempPosBuffer = (uint8_t*)malloc(2);	/* X & Y location in image frame, 2 bytes */

			sbsSet(camReaderIDG, DATA_OUT, 0, (void *)imgBufferG);
			sbsSet(ssdIDG, DATA_IN, 0, (void *)imgBufferG);
			sbsSet(ssdIDG, DATA_OUT, 0, (void *)tempPosBufferG);
#if DEBUG
			sbsControl(camReaderIDG, SBS_OFF);
			sbsControl(ssdIDG, SBS_OFF);
#endif
			local->state = 1;
		break;

		case 1: /* Trajectory Generation */
			/*if(!sbsControl(trajGenIDG, SBS_ON)){
				sbsControl(trajGenIDG, SBS_ON);
			}*/

			sbsControl(trajGenIDG, SBS_ON);
			sbsSet(trajGenIDG, DATA_IN, 0, (void *)tempPosBufferG);

#if DEBUG
			sbsControl(trajGenIDG, SBS_OFF);
#endif
			local->state = 2;
		break;

		case 2: /* Move to Target */
			/* --- TODO:
			 * create a servo component
			 */
			xil_printf("Move 10 degree!\r\n");
			local->state = 3;
		break;

		case 3: /* Stay and Stare */
			for(waitCnt = 0; waitCnt < 5; waitCnt++){
				xil_printf("Wait %d seconds\r\n", waitCnt);
			}

			local->state = 0;
		break;
	}

	return I_OK;
}


/* ******************************************************************** */
/*        SSD_init                 Initiate module information.         */
/* ******************************************************************** */

char visualServoTask_init(processT *p_ptr, void *vptr)
{
	p_ptr->on_fptr = visualServoTask_on;
	p_ptr->cycle_fptr = visualServoTask_cycle;
	p_ptr->off_fptr = NULL;

	/* Allocate the local structure for the module - optional
	 * This struct will be freed automatically when SBS_KILL is implemented */
	if((p_ptr->local = (pointer)malloc(sizeof(task_localT))) == NULL){
		return I_ERROR;
	}

	return I_OK;
}