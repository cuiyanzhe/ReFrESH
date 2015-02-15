/* --- refresh_main.h       Version 1.0
   --- header for BasicCC2520.c
   ---
   --- Copyright 2001, Mark V Automation Corp.
   ---
   --- 12/07/14 GYJ	Initial coding. V1.0
   ---
   --- */

#ifndef REFRESH_MAIN_H_
#define REFRESH_MAIN_H_

/* ********************************************************************************************************************************************* */
/*      Include Files						                                              														 */
/* ********************************************************************************************************************************************* */
#include "pbort.h"

/* ********************************************************************************************************************************************* */
/*      Global variables                                             														 					 */
/* ********************************************************************************************************************************************* */

extern processT	*menuIDG;
extern processT	*camReaderIDG;
extern processT *ssdIDG;
extern processT *trajGenIDG;
extern processT *visualServoTaskIDG;
extern processT *actuatorIDG;



////////////////////////////////////////////////////////////////////////////////////

enum {NO_TASK, BLINK, PD, TRAJQ};


#endif /* REFRESH_MAIN_H_ */
