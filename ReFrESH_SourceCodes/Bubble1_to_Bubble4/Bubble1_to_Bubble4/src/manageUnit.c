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
#include "dehazer.h"
//#include "SSD_new.h"


/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */

#define COMPUTATION  0
#define SENSOR		 1
#define ACTUATOR	 2

#define IMG_HEIGHT 10
#define IMG_WIDTH  10

/* ********************************************************************************************************************************************* */
/*      global variable						                                              														 */
/* ********************************************************************************************************************************************* */


uint8_t	requirePowerUsageG[10] = {0};
uint8_t nodeLeftPowerG[10] = {70, 100, 100};  /* change [0] to 70 to test power causes degradation case; 90 for normal case */

uint8_t nodePowerThresholdG[10] = {70, 50, 60};

uint8_t	powerUsageNodeG[10] = {0};
uint8_t	powerUsageNodeTotalG = 0;

char 	nonFuncPowerG, nonFuncLinkG;
char	nonFuncG, funcG;

int		linkRSSIG = 0;
int		linkRSSIthresholdG = 15;

processT *maxProcessT = NULL;
uint8_t  maxProcessTypeG;

uint8_t	funcPerfG[10] = {0};
uint8_t funcPerfTotalG = 0;

uint8_t	camReaderNodeG;
uint8_t	ssdNodeG;
uint8_t	trajGenNodeG;
uint8_t	actuatorNodeG;

uint8_t i;

uint8_t imgBufferNewG[IMG_HEIGHT * IMG_WIDTH];
uint8_t imgBufferESTG[IMG_HEIGHT * IMG_WIDTH];
uint8_t imgBufferDehazerESTG[IMG_HEIGHT * IMG_WIDTH];
uint8_t tempPosBufferNewG[2];	/* X & Y location in image frame, 2 bytes */
uint8_t tempPosBufferESTG[2];	/* X & Y location in image frame, 2 bytes */

procListT *tempHeaderG;
//procListT *temp;
//procListT *onModules;
//procListT *onQueue;

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

	procListT *temp;
	procListT *tempTemp;
	procListT *onQueue;
	procListT *onQueueTest;
	procListT *tempTempHeader;

	onQueueTest = sbsOnModuleList();
//	onQueue = onQueueTest;

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
    		if(sbsGet(camReaderIDG, NODE_NUM, 0, (void *)&camReaderNodeG) == local->nodeNum){ /* on same node */
    			sbsEvaluator(camReaderIDG, POWER, 0, (void *)&requirePowerUsageG[camReaderIDG->pid]);
    		}else{
    			/*---TODO: use zigbee to acquire corresponding info
    			 *
    			 */
    		}

    		sbsEvaluator(ssdIDG, POWER, 0, (void *)&requirePowerUsageG[ssdIDG->pid]);
    		sbsEvaluator(trajGenIDG, POWER, 0, (void *)&requirePowerUsageG[trajGenIDG->pid]);
    		sbsEvaluator(actuatorIDG, POWER, 0, (void *)&requirePowerUsageG[actuatorIDG->pid]);

//    		xil_printf("Required power of camReader component %X is: %d\r\n", camReaderIDG->pid, requirePowerUsage[camReaderIDG->pid]);
//    		xil_printf("Required power of SSD component %X is: %d\r\n", ssdIDG->pid, requirePowerUsage[ssdIDG->pid]);
//    		xil_printf("Required power of trajGen component %X is: %d\r\n", trajGenIDG->pid, requirePowerUsage[trajGenIDG->pid]);
//    		xil_printf("Required power of actuator component %X is: %d\r\n", actuatorIDG->pid, requirePowerUsage[actuatorIDG->pid]);

    		/* sum all the required power usage */
			powerUsageNodeTotalG = requirePowerUsageG[camReaderIDG->pid] + requirePowerUsageG[ssdIDG->pid] + requirePowerUsageG[trajGenIDG->pid] + requirePowerUsageG[actuatorIDG->pid];
			xil_printf("Total power usage is: %d\r\n", powerUsageNodeTotalG);

			/* binarize nonfunc performance */
			if((powerUsageNodeTotalG > nodeLeftPowerG[0]) || (nodeLeftPowerG[0] < nodePowerThresholdG[0])){
				nonFuncPowerG = 0;
			}else{
				nonFuncPowerG = 1;
			}

			sbsEvaluator(camReaderIDG, LINK_RSSI, 0, (void *)&linkRSSIG);
			xil_printf("RSSI is: %d\r\n", linkRSSIG);
			/* test link performance through RSSI value, can get from "rfRxInfoG.rssi" in cc2520.c */
			if(linkRSSIG < linkRSSIthresholdG){
				nonFuncLinkG = 1;
			}else{
				nonFuncLinkG = 0;
			}

			/* nonFunc equals AND all the nonFuncXXXX binary value */
			nonFuncG = nonFuncPowerG && nonFuncLinkG;
			xil_printf("nonFunc final value is: %d\r\n", nonFuncG);

			/* gather together all required func values of all running components (binary values obtained from EVALUATOR) */
//			funcPerf[0] = sbsEvaluator(camReaderIDG, FUNC);
//			funcPerf[2] = sbsEvaluator(trajGenIDG, FUNC);
//			funcPerf[3] = sbsEvaluator(actuatorIDG, FUNC);
			sbsEvaluator(ssdIDG, FUNC_PERF, 0, (void *)&funcPerfG[ssdIDG->pid]);
//			xil_printf("The realtime functional performance of SSD component %X is: %d\r\n", ssdIDG->pid, funcPerf[ssdIDG->pid]);

			/* multiplication of all normalized functional performance */
			funcPerfTotalG = funcPerfG[ssdIDG->pid];
			xil_printf("Total functional performance is: %d\r\n", funcPerfTotalG);

			/* binarize func performance */
			if(funcPerfTotalG > 90){
				funcG = 1;
			}else{
				funcG = 0;
			}
			xil_printf("Func final value is: %d\r\n", funcG);

			if(nonFuncG == 1 && funcG == 1){
				local->state = 0; /* keep monitoring */
			}else{ /* turn off task and every related PBO, for testing */
//				sbsControl(camReaderIDG, SBS_OFF);
//				sbsControl(ssdIDG, SBS_OFF);
//				sbsControl(trajGenIDG, SBS_OFF);
//				sbsControl(actuatorIDG, SBS_OFF);
//				sbsControl(visualServoTaskIDG, SBS_OFF);
				local->state = 1;
			}
		break;

    	case 1: /* Performance analysis, determine if func or nonfunc causes the problem */
    		xil_printf("You're in performance analysis state (func or nonfunc) \r\n");
    		if(nonFuncG == 0 && funcG == 1){
				local->state = 2; /* go to state 2, non-functional analysis */
			}else if((nonFuncG == 1 && funcG == 0) || (nonFuncG == 0 && funcG == 0)){
				local->state = 5; /* go to state 5, functional problem causes the performance degradation */
			}
    	break;

    	case 2: /* Non-functional analysis, determine if the current configuration is OK */
			xil_printf("You're in non-functional analysis state (cfg is OK or isn't OK) \r\n");
			/* Assume only one nonfunc resource has a problem */
			if(nonFuncPowerG == 0){
				/* find a component that need maximum required power
				 *---TODO:
				 *   Create an API to obtain the maximum nonFunc consumption: processT findMax(uint8_t *ptr);
				 */
				maxProcessT = requirePowerUsageG[camReaderIDG->pid] > requirePowerUsageG[ssdIDG->pid] ? camReaderIDG : ssdIDG;
				maxProcessT = requirePowerUsageG[maxProcessT->pid] > requirePowerUsageG[trajGenIDG->pid] ? maxProcessT : trajGenIDG;
				maxProcessT = requirePowerUsageG[maxProcessT->pid] > requirePowerUsageG[actuatorIDG->pid] ? maxProcessT : actuatorIDG;

				xil_printf("PID of camReader is: %X\r\n", camReaderIDG->pid);
				xil_printf("PID of SSD is: %X\r\n", ssdIDG->pid);
				xil_printf("PID of trajGen is: %X\r\n", trajGenIDG->pid);
				xil_printf("PID of actuator is: %X\r\n", actuatorIDG->pid);
				xil_printf("The pid of the component which has the maximum power consumption is: %d\r\n", maxProcessT->pid);

				/* get the type of temp */
				sbsGet(maxProcessT, TYPE, 0, (void *)&maxProcessTypeG);
				xil_printf("The type of processT which has maximum power consumption is: %d (0: COMP; 1: SEN; 2: ACT)\r\n", maxProcessTypeG);
				if(maxProcessTypeG == COMPUTATION){
					local->state = 3; /* go to state 3 to generate node configurations */
				}else{
					local->state = 6; /* go to state 6 means current cfg is not OK, should do calibration analysis */
				}
			}
		break;

    	case 3: /* Generate Node Configurations */
    		xil_printf("You're in the generate node configuration state (just re-locate COMPUTATION component(s)) \r\n");

    		/* firstly determine if move the node which has maximum power consumption to another node is fine */
    		powerUsageNodeTotalG -= requirePowerUsageG[maxProcessT->pid];
    		xil_printf("Total power after move: %d\r\n", powerUsageNodeTotalG);
    		if(powerUsageNodeTotalG < nodeLeftPowerG[0] && powerUsageNodeTotalG < nodePowerThresholdG[0]){ /* just move maximum power consumption component is fine */
    			for(i = 1; i < local->numOfNodes; i++){  /* determine all other nodes, if has a satisfied one, choose this one, jump out */
    				if(requirePowerUsageG[maxProcessT->pid] < nodeLeftPowerG[i] && powerUsageNodeTotalG < nodePowerThresholdG[i]){
//    					xil_printf("test cases\r\n");
    					break;
    				}
    			}
    		}else{ /*---TODO: need an method to sort the COMPUTATION and save in a buffer in decreasing order and try to move one by one*/
    			xil_printf("This is TODO\r\n");
    		}

			local->state = 4; /* go to state 4 to re-connect component and run */
		break;

    	case 4:	/* Re-connect component and run */
    		xil_printf("You're in the re-connect and run state \r\n");

    		/* Obtain a new on queue list except menu, task and manageUnit*/
    		/* initialize a procList to build a new process list */
    		tempHeaderG = (procListT *)malloc(sizeof(procListT));
    		tempHeaderG->process = NULL;
    		tempHeaderG->nextProc = NULL;
    		temp = tempHeaderG;
    		tempTempHeader = tempHeaderG;

    		onQueue = onQueueTest;

    		while(onQueue != NULL){
				if((onQueue->process->pid == manageUnitIDG->pid) || (onQueue->process->pid == visualServoTaskIDG->pid) || (onQueue->process->pid == menuIDG->pid)){
					onQueue = (procListT *)onQueue->nextProc;
				}else{
//					onModules = (procListT *)malloc(sizeof(procListT));
//					onModules->process = onQueue->process;
//					temp->nextProc = onModules;
//					temp->process = onModules->process;
//					onQueue = (procListT *)onQueue->nextProc;
					temp->nextProc = (procListT *) malloc(sizeof(procListT));
					temp = temp->nextProc;
					temp->process = onQueue->process;
					onQueue = (procListT *)onQueue->nextProc;

				}
			}
			temp->nextProc = NULL;

			tempTempHeader = (procListT *)tempTempHeader->nextProc; /* jump from the header null node */

			temp = tempTempHeader;
			tempTemp = temp;

			while(tempTempHeader != NULL){
				xil_printf("test onQueueG: %X\r\n", tempTempHeader->process);
				tempTempHeader = (procListT *)tempTempHeader->nextProc;
			}

			free(tempHeaderG);

			while(tempTemp != NULL){
				tempTemp->nextProc = temp->nextProc;
				tempTemp = temp->nextProc;
				free(temp);
				temp = tempTemp;
			}

    		/* Management Unit re-connect components */
//			sbsSet(camReaderIDG, DATA_OUT, 0, (void *)imgBufferNewG);
//			sbsSet(ssdnewIDG, DATA_IN, 0, (void *)imgBufferNewG);
//			sbsSet(ssdnewIDG, DATA_OUT, 0, (void *)tempPosBufferNewG);
//			sbsSet(trajGenIDG, DATA_IN, 0, (void *)tempPosBufferNewG);


    		/*---TODO:
    		 * NEED TO FIGURE OUT WHERE TO PUT sbsSet to connect all components
    		 * should have a series of sbsSet, and sbsControl(XXX, SBS_ON)
    		 * API:
    		 */

    		/* This is just a try, NEED TO THINK HOW TO BUILD A FSM */
//    		sbsSet(camReaderIDG, DATA_OUT, 0, (void *)imgBufferNewG);
//    		sbsSet(ssdIDG, DATA_IN, 0, (void *)imgBufferNewG);
//    		sbsSet(ssdIDG, DATA_OUT, 0, (void *)tempPosBufferNewG);

//    		sbsControl(camReaderIDG, SBS_ON);
//    		sbsControl(ssdIDG, SBS_ON);
//    		sbsControl(trajGenIDG, SBS_ON);
//    		sbsControl(actuatorIDG, SBS_ON);

//    		sbsControl(visualServoTaskIDG, SBS_ON);

			xil_printf("-----------------BEGIN:Test Estimator-----------------\r\n");

			sbsSet(camReaderIDG, EST_DATA_OUT, 0, (void *)imgBufferESTG);
			sbsSet(dehazerIDG, EST_DATA_IN, 0, (void *)imgBufferESTG);
			sbsSet(dehazerIDG, EST_DATA_OUT, 0, (void *)imgBufferDehazerESTG);
			sbsSet(ssdIDG, EST_DATA_IN, 0, (void *)imgBufferDehazerESTG);
			sbsSet(ssdIDG, EST_DATA_OUT, 0, (void *)tempPosBufferESTG);
			sbsSet(trajGenIDG, EST_DATA_IN, 0, (void *)tempPosBufferESTG);

			sbsEstimator(camReaderIDG);
			sbsEstimator(dehazerIDG);
			sbsEstimator(ssdIDG);
			sbsEstimator(trajGenIDG);

			xil_printf("------------------END:Test Estimator------------------\r\n");

			local->state = 5; /* SHOULD BE 0, 5 IS JUST FOR DEBUG. go to state 0 to monitor the new cfg */
		break;

		case 5:	/* TODO: Identify source of degradation */
			xil_printf("This is a TODO.\r\n");
		break;

		case 6:	/* TODO: Calibration analysis */
			xil_printf("This is a TODO.\r\n");
		break;
	}

	return I_OK;
}


/* ******************************************************************** */
/*        manageUnit_init             Initiate module information.      */
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
	local->taskFlag = 1; /* task can be ran */
	local->nodeNum = 1; /* management unit is running on NODE 1 */

	local->numOfNodes = 2; /* there are two nodes in the system */

	local->panID = 0xAAAA;
	local->srcAddr = 0x11AA;
	local->destAddr = 0x11BB;

	/* Management Unit connect components firstly */
	sbsSet(camReaderIDG, DATA_OUT, 0, (void *)imgBufferNewG);
	sbsSet(ssdIDG, DATA_IN, 0, (void *)imgBufferNewG);
	sbsSet(ssdIDG, DATA_OUT, 0, (void *)tempPosBufferNewG);
	sbsSet(trajGenIDG, DATA_IN, 0, (void *)tempPosBufferNewG);

	return I_OK;
}

