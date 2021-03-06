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
extern processT	*camReader1IDG;
extern processT *ssdIDG;

extern processT *trajGenIDG;
extern processT *visualServoTaskIDG;
extern processT *actuatorIDG;
extern processT *manageUnitIDG;

extern processT *ssdnewIDG;
extern processT *dehazerIDG;


////////////////////////////////////////////////////////////////////////////////////

enum {NO_TASK, BLINK, PD, TRAJQ};


#endif /* REFRESH_MAIN_H_ */
