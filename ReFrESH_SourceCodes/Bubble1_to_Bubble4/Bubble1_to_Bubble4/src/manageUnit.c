/* --- manageUnit.c       Version 1.0
   --- this module provides: (1) monitor func & nonfunc performance;
   --- 						 (2) make a decision to distinguish func or nonfunc problem
   ---
   --- Copyright 2015, Collaborative Robotics Lab .
   ---
   --- 02/15/15 YC	Initial coding. V1.0
   --- */

/* ********************************************************************************************************************************************* */
/*      Include Files						                                              														 */
/* ********************************************************************************************************************************************* */

#include <stdio.h>
#include <stdlib.h>		/* for malloc() */
#include "pbort.h"
#include "serial.h"
//#include "compStructure.h"
#include "camReader.h"
#include "SSD.h"
#include "trajGen.h"
#include "actuator.h"
#include "refresh_main.h"
#include "manageUnit.h"


/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */

#define COMPUTATION  0
#define SENSOR		 1
#define ACTUATOR	 2

/* ******************************************************************** */
/*       manageUnit_on             start up the module.                 */
/* ******************************************************************** */

char manageUnit_on(processT *p_ptr)
{
	return I_OK;
}


/* ******************************************************************** */
/*       manageUnit_cycle               run the module.                 */
/* ******************************************************************** */

char manageUnit_cycle(processT *p_ptr)
{
	manageUnit_localT  *local = (manageUnit_localT *)p_ptr->local;

	uint8_t	requirePowerUsage[10] = {0};
	uint8_t node1LeftPower = 60;
	uint8_t node2LeftPower = 100;

	uint8_t node1PowerThreshold = 50;
	uint8_t node2PowerThreshold = 100;

	uint8_t	powerUsageNode1 = 0;
	uint8_t	powerUsageNode2 = 0;
	uint8_t	powerUsageNodeTotal = 0;

	char 	nonFuncPower, nonFuncLink;
	char	nonFunc, func;

	int		linkRSSI = 10;
	int		linkRSSIthreshold = 4;

	char 	taskFlag = 1;
	processT *temp = NULL;

	uint8_t	funcPerf[4] = {0};

#if DEBUG
	switch(local->state){
    	case 0: /* Performance monitor */
    		/* equation will use to determine if the performance is decreased */
			/*--- TODO:
			 * A method to acquire node power from a sensor: uint8_t nonFuncResourseAcquire(uint8_t nodeNum, uint8_t type);
			 * type includes POWER, LINK...
			 * BUT HERE, I JUST DEFINE IT AS A PAREMETER
			 */

    		/* gather together all required nonFunc values of all running components */
    		/*---TODO:
    		 * In this process, it should distinguish if the component is running on the same node as ManageUnit, if not, call sbsZigbeeOSparse().
    		 * BUT HERE, AUSSME ALL THE COMPOENNTS RUN ON THE SAME NODE.
    		 */
    		requirePowerUsage[camReaderIDG->pid] = sbsEvaluator(camReaderIDG, POWER);
    		requirePowerUsage[ssdIDG->pid] = sbsEvaluator(ssdIDG, POWER);
    		requirePowerUsage[trajGenIDG->pid] = sbsEvaluator(trajGenIDG, POWER);
    		requirePowerUsage[actuatorIDG->pid] = sbsEvaluator(actuatorIDG, POWER);

    		/* sum all the required power usage */
			powerUsageNodeTotal = requirePowerUsage[camReaderIDG->pid] + requirePowerUsage[ssdIDG->pid] + requirePowerUsage[trajGenIDG->pid] + requirePowerUsage[actuatorIDG->pid];

			/* binarize nonfunc performance */
			if((powerUsageNodeTotal > node1LeftPower) || (node1LeftPower < node1PowerThreshold)){
				nonFuncPower = 0;
			}else{
				nonFuncPower = 1;
			}

			/* test link performance through RSSI value, can get from "rfRxInfoG.rssi" in cc2520.c */
			if(linkRSSI < linkRSSIthreshold){
				nonFuncLink = 1;
			}else{
				nonFuncLink = 0;
			}

			/* nonFunc equals AND all the nonFuncXXXX binary value */
			nonFunc = nonFuncPower && nonFuncLink;

			/* gather together all required func values of all running components (binary values obtained from EVALUATOR) */
//			funcPerf[0] = sbsEvaluator(camReaderIDG, FUNC);
//			funcPerf[2] = sbsEvaluator(trajGenIDG, FUNC);
//			funcPerf[3] = sbsEvaluator(actuatorIDG, FUNC);
			funcPerf[0] = sbsEvaluator(ssdIDG, FUNC);


			/* binarize func performance */
			if(funcPerf[0] > 90){
				func = 1;
			}else{
				func = 0;
			}

			if(nonFunc == 1 && func == 1){
				taskFlag = 1;  /* TODO: continue run task */
				local->state = 0; /* keep monitoring */
			}else{
				taskFlag = 0;  /* TODO: stop the task */
				local->state = 1;
			}
		break;

    	case 1:	/* Performance analysis, determine if func or nonfunc causes the problem */
			if(nonFunc == 0 && func == 1){
				local->state = 2; /* go to state 2, non-functional analysis */
			}else if((nonFunc == 1 && func == 0) || (nonFunc == 0 && func == 0)){
				local->state = 5; /* go to state 5, functional problem causes the performance degradation */
			}
    	break;

    	case 2:	/* Non-functional analysis, determine if the current configuration is OK */
			/* Assume only one nonfunc resource has a problem */
    		if(nonFuncPower == 0){
    			/* find a component that need maximum required power
    			 *---TODO:
    			 *   Create an API to obtain the maximum nonFunc consumption: processT findMax(uint8_t *ptr);
    			 */
    			temp = requirePowerUsage[camReaderIDG->pid] > requirePowerUsage[ssdIDG->pid] ? camReaderIDG : ssdIDG;
    			temp = requirePowerUsage[temp->pid] > requirePowerUsage[trajGenIDG->pid] ? temp : trajGenIDG;
    			temp = requirePowerUsage[temp->pid] > requirePowerUsage[actuatorIDG->pid] ? temp : actuatorIDG;

    			if(temp->local-> == COMPUTATION){
    				local->state = 3; /* go to state 3 to generate node configurations */
    			}else{
    				local->state = 6; /* go to state 6 means current cfg is not OK, should do calibration analysis */
    			}
    		}
		break;

    	case 3:	/* Generate Node Configurations */
			/* Fix SENSOR and ACTUATOR on original node, just generate based on COMPUTATION */


    		local->state = 4; /* go to state 4 to re-connect component and run */
		break;

    	case 4:	/* Re-connect component and run */


			local->state = 0; /* go to state 0 to monitor the new cfg */
		break;

    	case 5:	/* TODO: Identify source of degradation */
			xil_printf("This is a TODO.\r\n");
		break;

    	case 6:	/* TODO: Calibration analysis */
			xil_printf("This is a TODO.\r\n");
		break;

	}
#endif
	return I_OK;
}


/* ******************************************************************** */
/*        manageUnit_init                 Initiate module information.         */
/* ******************************************************************** */

char manageUnit_init(processT *p_ptr, void*vptr)
{
	p_ptr->on_fptr = manageUnit_on;
	p_ptr->cycle_fptr = manageUnit_cycle;
	p_ptr->off_fptr = NULL;

	/* Allocate the local structure for the module - optional
	 * This struct will be freed automatically when SBS_KILL is implemented */
	if((p_ptr->local = (pointer)malloc(sizeof(manageUnit_localT))) == NULL){
			return I_ERROR;
	}

	/* define a pointer points to local structure */
	manageUnit_localT  *local = (manageUnit_localT *)p_ptr->local;

	local->state = 0;

	return I_OK;
}

