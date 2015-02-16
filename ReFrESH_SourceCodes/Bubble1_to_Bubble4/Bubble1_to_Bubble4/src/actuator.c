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
//#include "compStructure.h"

/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */
#define COMPUTATION  0
#define SENSOR		 1
#define ACTUATOR	 2


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
/*       actuator_get                  Get parameter from local         */
/* ******************************************************************** */

char actuator_get(processT *p_ptr, int16_t type, int16_t arg, void *vptr)
{
	actuator_localT  *pActuatorLocal = (actuator_localT *)p_ptr->local;

	switch(type){
		case DATA_IN:
			*((uint8_t *)vptr) = *(pActuatorLocal->inPtr);
			return I_OK;
		break;

		case DATA_OUT:
			*((uint16_t *)vptr) = *(pActuatorLocal->outPtr);
			return I_OK;
		break;

		case IN_SIZE:
			*((size_t *)vptr) = sizeof(pActuatorLocal->inPtr);
			return I_OK;
		break;

		case OUT_SIZE:
			*((size_t *)vptr) = sizeof(pActuatorLocal->outPtr);
			return I_OK;
		break;

		case EST_INPUT:
			*((uint8_t *)vptr) = *(pActuatorLocal->estInPtr);
			return I_OK;
		break;

		case EST_OUTPUT:
			*((uint16_t *)vptr) = *(pActuatorLocal->estOutPtr);
			return I_OK;
		break;

		case TYPE:
			*((uint8_t *)vptr) = pActuatorLocal->type;
			return I_OK;
		break;

		case LOCAL_STATE:
			*((uint8_t *)vptr) = pActuatorLocal->actuatorState;
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
	actuator_localT  *pActuatorLocal = (actuator_localT *)p_ptr->local;
	xil_printf("(actuator.c)Move 10 degree (just for debug)!\r\n");

	pActuatorLocal->actuatorState = 2;

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
	p_ptr->get_fptr = actuator_get;

	/* Allocate the local structure for the module - optional
	 * This struct will be freed automatically when SBS_KILL is implemented */
	if((p_ptr->local = (pointer)malloc(sizeof(actuator_localT))) == NULL){
			return I_ERROR;
	}

	/* define a pointer points to local structure */
	actuator_localT  *pActuatorLocal = (actuator_localT *)p_ptr->local;

	pActuatorLocal->actuatorState = 0;
	pActuatorLocal->type = ACTUATOR;

	return I_OK;
}
