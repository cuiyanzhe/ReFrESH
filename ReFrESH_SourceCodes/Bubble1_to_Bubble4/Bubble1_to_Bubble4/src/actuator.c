/* --- actuator.c       Version 1.0
   --- this module implements the movement of actuator(s).
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
#include "actuator.h"
#include "compStructure.h"

/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */



/* ********************************************************************************************************************************************* */
/*      global variable						                                              														 */
/* ********************************************************************************************************************************************* */


/* ******************************************************************** */
/*       actuator_on                Start up the module.                    */
/* ******************************************************************** */

char actuator_on(processT *p_ptr)
{
	return I_OK;
}


/* ******************************************************************** */
/*       actuator_set            Set actuator component's ports.        */
/* ******************************************************************** */

char actuator_cycle(processT *p_ptr)
{
/*---TODO:
	heading_to_yaw();
	Inverse_Yaw();

	//InverseKinematics();
	//Set_Position();
	Set_Servo();
*/
	xil_printf("(actuator.c)Move 10 degree (just for debug)!\r\n");

	actuatorState = 2;

	return I_OK;
}


/* ******************************************************************** */
/*        actuator_init                 Initiate module information.    */
/* ******************************************************************** */

char actuator_init(processT *p_ptr, void *vptr)
{
	p_ptr->on_fptr = actuator_on;
	p_ptr->cycle_fptr = actuator_cycle;
	p_ptr->off_fptr = NULL;

	actuatorState = 0;

	return I_OK;
}
