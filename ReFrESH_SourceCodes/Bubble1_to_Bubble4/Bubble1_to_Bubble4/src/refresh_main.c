/* --- REFRESH_MAIN.C       Version 1.0
   --- This module initializes and runs PBORT modules.
   ---
   --- Copyright 2015, Collaborative Robotics Lab at Purdue.
   ---
   --- 02/10/15 YC	Initial coding. V1.0
   ---
   --- */

/* ********************************************************************************************************************************************* */
/*      Include Files						                                              														 */
/* ********************************************************************************************************************************************* */
#include <stdlib.h>					/* for malloc() */
#include <stdint.h>
#include <stdio.h>
#include "pbort.h"
#include "serial.h"
#include "refresh_main.h"
#include "camReader.h"
#include "SSD.h"
#include "trajGen.h"
#include "menu.h"
#include "visualServoTask.h"
#include "actuator.h"

/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */
#define TEST	0
#define DEBUG	0
#define DEBUG_PRINT		0
#define MENU_DEBUG 1

/* ********************************************************************************************************************************************* */
/*      Global variables                                             														 					 */
/* ********************************************************************************************************************************************* */
processT *camReaderIDG;
processT *ssdIDG;
processT *trajGenIDG;
processT *menuIDG;
processT *visualServoTaskIDG;
processT *actuatorIDG;

#define IMG_HEIGHT 10
#define IMG_WIDTH  10

/* ********************************************************************************************************************************************* */
/*      Function prototype                                              																		 */
/* ********************************************************************************************************************************************* */



/* ********************************************************************************************************************************************* */
/*     main function                                              																		 	 */
/* ********************************************************************************************************************************************* */
int main()
{
	short camMissed; /* to monitor the miss cycles of camera component */
	short ssdMissed; /* to monitor the miss cycles of SSD component */

	xil_printf("Start, please check menu to test...\r\n");

#if TEST
	uint8_t imgBuffer[10][10] = {
			{255, 55, 255, 255, 255, 255, 255, 255, 255, 255},
			{75, 85, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 65, 65, 65, 255, 255, 255, 255},
			{255, 255, 255, 65, 65, 65, 255, 255, 255, 255},
			{255, 255, 255, 65, 65, 65, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
			{255, 255, 255, 255, 255, 255, 255, 255, 255, 111}
	};

	uint8_t *pTest = imgBuffer[0];
	xil_printf("%d\r\n", pTest[0]);
	pTest++;
	xil_printf("%d\r\n", pTest[0]);
#endif

	sched_init(1000);

	/* Spawn components */
	visualServoTaskIDG = sbsSpawn(visualServoTask_init, 20, 0, 0);

	camReaderIDG = sbsSpawn(camReader_init, 5, 0, 0);
	ssdIDG = sbsSpawn(SSD_init, 5, 0, 0);
	trajGenIDG = sbsSpawn(trajGen_init, 5, 0, 0);
	actuatorIDG = sbsSpawn(actuator_init, 5, 0, 0);


#if MENU_DEBUG
	menuIDG = sbsSpawn(menu_init, 5, 0, 0);
	sbsControl(menuIDG, SBS_ON);
#endif

#if DEBUG
	/* Turn on components */
	sbsControl(camReaderIDG, SBS_ON);
	sbsControl(ssdIDG, SBS_ON);
	sbsControl(trajGenIDG, SBS_ON);

	/* Connect components manually */
	uint8_t *imgBuffer = (uint8_t*)malloc(IMG_HEIGHT * IMG_WIDTH);
	uint8_t *tempPosBuffer = (uint8_t*)malloc(2);	/* X & Y location in image frame, 2 bytes */

	sbsSet(camReaderIDG, DATA_OUT, 0, (void *)imgBuffer);
	sbsSet(ssdIDG, DATA_IN, 0, (void *)imgBuffer);
	sbsSet(ssdIDG, DATA_OUT, 0, (void *)tempPosBuffer);
	sbsSet(trajGenIDG, DATA_IN, 0, (void *)tempPosBuffer);
#endif

	while(1)
	{
		sched();
#if DEBUG_PRINT
		sbsGet(camReaderIDG, SBS_MISSED, 0, (void *)&camMissed);
		sbsGet(ssdIDG, SBS_MISSED, 0, (void *)&ssdMissed);
		xil_printf("Camera missed cycles: %d; SSD missed cycles: %d\r\n", camMissed, ssdMissed);
#endif
	}

    return 0;
}

/* ********************************************************************************************************************************************* */
/*     Local functions                                              																		 	 */
/* ********************************************************************************************************************************************* */
/* --- user_test_code() is for test a small piece of code outside of PBORT.
 * ---
 */

