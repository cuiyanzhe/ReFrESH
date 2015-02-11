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
#include "menu.h"
#include "main.h"
#include "camReader.h"

/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */
#define TEST	0
#define DEBUG	1

/* ********************************************************************************************************************************************* */
/*      Global variables                                             														 					 */
/* ********************************************************************************************************************************************* */
processT *camReaderIDG;


/* ********************************************************************************************************************************************* */
/*      Function prototype                                              																		 */
/* ********************************************************************************************************************************************* */



/* ********************************************************************************************************************************************* */
/*     main function                                              																		 	 */
/* ********************************************************************************************************************************************* */
int main()
{
	xil_printf("Start...\r\n");

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
	camReaderIDG = sbsSpawn(camReader_init, 1, 0, 0);
	sbsControl(camReaderIDG, SBS_ON);

	while(1)
	{
		sched();
	}

    return 0;
}

/* ********************************************************************************************************************************************* */
/*     Local functions                                              																		 	 */
/* ********************************************************************************************************************************************* */
/* --- user_test_code() is for test a small piece of code outside of PBORT.
 * ---
 */

