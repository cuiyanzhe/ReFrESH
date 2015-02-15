/* --- PBORT_RECONODE.C
   --- This file contains the scheduler routines for the RecoNode processor with Virtex 4 and PPC.
   ---
   --- These are only demo routines and are not complete.
   --- You are free to use these routines, or any
   --- part thereof, AT YOUR OWN RISK, provided this entire comment
   --- block and the included copyright notice remain intact.
   ---
   --- Initial coding by Richard Voyles, Trident Robotics.
   --- 09/14/02 RMV	Fixed local bug in SBS_KILL routine.
   ---
   --- Copyright 1996, 2002 Mark V Automation Corp.
   ---
   --- 09/09/09 Modified for PPC405 on Xilinx Virtex4
   --- 01/05/10 Modified by Kang Li
   --- 09/30/13 Modified for PPC440 on Xilinx Virtex5 by Cui
   --- 09/11/14 RMV Fixed bugs in API -- made all module methods xxx(processT *...)
   --- */

#define PBORT_SRC
//#define DEBUG

#include "stdio.h"
#include <stdlib.h>
#include <string.h>
#include "xparameters.h"
#include "xtime_l.h"
//#include <xexception_l.h>
#include "xexception_l.h"
//#include <xstatus.h>
//#include "intr.h"
//#include "xtmrctr_l.h"
//#include "vtypes.h"
#include "pbort.h"
#include "serial.h"
#include "compStructure.h"
//#include "database.h"
//#include "cc2520.h"
#define UARTLITE_BASEADDRESS XPAR_UARTLITE_0_BASEADDR

//#include "cc2520.h"

#define MAX_READY       32
const unsigned char myNodeG = 1;

/* --- Input frequency to the Virtex 4 Timer */
//#define TIMER_INPUT_FREQ        100000000
/* --- Input frequency to the Virtex 5 Timer */
#define TIMER_INPUT_FREQ        125000000

/* --- Global variables (all globals end in "G") */
long		timerTicksG;            /* number of timer interrupts since init */
//float		clockPeriodG;           /* period, in seconds, of interrupts */
int		heartBeatFreqG;             /* actual frequency, in hertz */
//short   DOStimerCntG;           /* number of new timer ticks per DOS tick */
short   installFlagG = 0;       /* flag indicating handler installed */
processT   *readyQueueG[MAX_READY]; /* list of functions ready to execute */
procListT       *spawnQueueG;   /* list of functions spawned */
procListT       *onQueueG;      /* list of functions running */
short   readyHeadG, readyTailG;
unsigned long   pidG;           /* counting process ID */
long			LED_valueG;

// These are hacks to prove out the downloading function
uint8_t		codeBufG[256];		// Temporary code buffer -- can be larger than 256 bytes, but only 256 bytes to FLASH at a time
uint8_t		codeIndexG;			// Index into the codeBufG array
uint8_t		codeStatusG;		// Download status indicator
uint16_t	codeAddrG;			// Current pointer into a reserved block of FLASH program space
uint16_t	codeInitAddrG;
uint16_t	codeOrigAddrG;

// These are resident in FLASH memory - 4 pages of empty space for location of new code modules
uint8_t	fillerCodeG[0x70] = "01234567890123456789ABCDEF";
uint8_t	newCodeG[1024] = "987654321098765432109876543210FEDCBA9876543210";
uint8_t	*baseCodeG;

// The top version implements the blinkRed module for testing purposes.
//uint8_t			progCodeG[256] = {0x19, 0x9A, 0x1C, 0x9A, 0x81, 0xE0, 0x90, 0xE0, 0x08, 0x95, 0x1C, 0x9B, 0x02, 0xC0, 0x1C, 0x98, 0x01, 0xC0, 0x1C, 0x9A, 0x81, 0xE0, 0x90, 0xE0, 0x08, 0x95};
uint8_t			progCodeG[256] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x98, 0x01, 0xC0, 0x1C, 0x9A, 0x81, 0xE0, 0x11, 0x22, 0x33, 0x44};




uint8_t		strG[150];			// string var -- doesn't need to be global

//Global buffers for eval and est for decider, JTL 10/28/14
uint16_t	evBufferG[10] = {0,0,0,0,0,0,0,0,0,0};				//assume max 10 spawned components for now
uint16_t	estFBufferG[10] = {0,0,0,0,0,0,0,0,0,0};			//assume max 10 spawned components for now
uint16_t	estNFBufferG[10] = {0,0,0,0,0,0,0,0,0,0};			//assume max 10 spawned components for now
uint8_t		estOutBufferG = 0;				//assume data out type is uint8_t for now


// These are the Intel HEX record types that are defined. (Values above 6 are custom to PBO/RT)
#define		DATA_REC	0
#define		EOF_REC		1
#define		OFFSET_REC	7
#define		INITPTR_REC	8
#define		DATA_IN_REC 9	/*****Added 9-0D by JTL 10/23/14 for internode comm*******/
#define		EVRQ_REC 0x0A
#define		EVRS_REC 0x0B
#define		ESRQ_REC 0x0C
#define		ESRS_REC 0x0D
#define 	REQUEST_COMMAND 0
#define 	RESPONSE_CONTAINED 1


// These are the Intel HEX record types that are defined. (Values above 6 are custom to PBO/RT)
#define		DATA_REC	0
#define		EOF_REC		1
#define		OFFSET_REC	7
#define		INITPTR_REC	8


#define CUI_DEBUG 1    /* YC: debug the SBS_ON, 02/15/2015 */

/* ************ Function Prototypes ************** */

/* ************ Interrupt Code ******************* */

/* --- This function is the interrupt handler for timer 0 (interrupt 8).
   --- Working code can be inserted here or a call to another
   --- subroutine. */
void timer_handler(void * baseaddr_p)
{
  procListT     *queue;
  short         i, inQueue;

  timerTicksG++;
  /*Xuint32 baseaddr = (Xuint32) XPAR_LED_0_BASEADDR;
// Xuint32 Reg32Value;

  if (timerTicksG++ & 0x01)
  {
  	LED_valueG |= 0x08;
  	LED_mWriteSlaveReg0(baseaddr, 0, LED_valueG);
//    Reg32Value = LED_mReadSlaveReg0(baseaddr, 0);
  }//printf("   - read %d from register 0 word 0\n\r", Reg32Value);;
  else {

  	LED_valueG &= 0xFFF7;
    LED_mWriteSlaveReg0(baseaddr, 0, LED_valueG);
//    Reg32Value = LED_mReadSlaveReg0(baseaddr, 0);
  }*/

  queue = onQueueG;
  /* Look for processes in the on queue that are ready to execute */
  /* Are there entries in the on queue? (first element not NULL?) */
  if (onQueueG->process != NULL){
    /* If a process is ready, put it in the ready queue */
    if (queue->process->nextReady <= timerTicksG){

      /* first check to see if it is in the ready queue already */
      inQueue = 0;
      if (readyHeadG != readyTailG){
//        i = readyHeadG + 1;     /* skip anything executing now */
	i = readyHeadG;     /* Start with current process */
	while (i != readyTailG){
	  if (readyQueueG[i]->pid == queue->process->pid){
	    inQueue = 1;
	    queue->process->missed++;
	    break;
	  } /* endif */
	  if (++i >= MAX_READY)
	    i = 0;
	} /* endwhile */
      } /* endif */

      /* If it is not in the queue already, put it in now. */
      if (inQueue == 0)
        readyQueueG[readyTailG] = queue->process;
      /* indicate the next time it will be ready */
      queue->process->nextReady += queue->process->nPeriod;
      if (++readyTailG >= MAX_READY)
        readyTailG = 0;
    } /* endif */

    /* now search through the entire on queue */
    while (queue->nextProc != NULL){
      queue = queue->nextProc;
      if (queue->process->nextReady <= timerTicksG){
        /* put it in the ready queue */

        /* first check to see if it is in the ready queue already */
        inQueue = 0;
        if (readyHeadG != readyTailG){
//          i = readyHeadG + 1;
          i = readyHeadG;
          while (i != readyTailG){
            if (readyQueueG[i]->pid == queue->process->pid){
              inQueue = 1;
              queue->process->missed++;
              break;
            } /* endif */
            if (++i >= MAX_READY)
              i = 0;
          } /* endwhile */
        } /* endif */

        /* If it is not in the queue already, put it in now. */
        if (inQueue == 0)
          readyQueueG[readyTailG] = queue->process;
        /* indicate the next time it will be ready */
        queue->process->nextReady += queue->process->nPeriod;
        if (++readyTailG >= MAX_READY)
          readyTailG = 0;
      } /* endif */
    } /* endwhile */

    /* sort the queue? */

  } /* endif */

  /* The interrupt handler must clear the TSR(PIS) field before exiting */
  /* PIT interrupts are persistent */

  XTime_DECClearInterrupt();
}

/* --- This function sets the frequency of the timer 0 interrupts.
   --- The int value FREQ is in hertz. The actual interrupt frequency
   --- in hertz is returned. */
int interrupt_rate(int freq)
{
//   int_id_t id = XPAR_XPS_INTC_0_XPS_TIMER_1_INTERRUPT_INTR;
   long   timerCnt;

  timerCnt = (long)(TIMER_INPUT_FREQ / freq);

#ifdef DEBUG
  print ("CLOCK: Configuring extra timer to generate interrupts..\r\n");
#endif

  XTime_DECSetInterval( timerCnt );

  XTime_DECEnableAutoReload();

  /* Enable pit interrupt */
  XTime_DECEnableInterrupt() ;


#if 0
    XTmrCtr_mWriteReg (XPAR_XPS_TIMER_1_BASEADDR, TIMER_COUNTER_ID,
                       XTC_TLR_OFFSET, timerCnt);

    // reset the timers, and clear interrupts
    XTmrCtr_mSetControlStatusReg (XPAR_XPS_TIMER_1_BASEADDR, TIMER_COUNTER_ID,
				  XTC_CSR_INT_OCCURED_MASK | XTC_CSR_LOAD_MASK );

    // start the timer
    XTmrCtr_mSetControlStatusReg (XPAR_XPS_TIMER_1_BASEADDR, 0,
                                  XTC_CSR_ENABLE_TMR_MASK | XTC_CSR_ENABLE_INT_MASK |
                                  XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_DOWN_COUNT_MASK );

#endif

  /* --- Set the global variables that keep track of interrupt rate. */
  heartBeatFreqG = TIMER_INPUT_FREQ / (float)timerCnt;
//  clockPeriodG = (float)timerCnt / TIMER_INPUT_FREQ;

//  enable_interrupt (id);

  return heartBeatFreqG;
}

/* --- This function installs the new timer 0 interrupt handler. It
   --- grabs the old timer handler for restoration upon quitting the
   --- program and for execution at 18.2 Hz. */
void install_handler()
{
//    int_id_t id = XPAR_XPS_INTC_0_XPS_TIMER_1_INTERRUPT_INTR;
//    XStatus status;

  /* Do not erase the old timer handler if initing again. */
  if (installFlagG == 0){
    installFlagG = 1;           /* set flag to indicate handler installed */
  } /* endif */
  timerTicksG = 0;

  /* Initialize exception handling */

  XExc_Init();

   /* Register PIT interrupt handler */
  XExc_RegisterHandler(XEXC_ID_DEC_INT, (XExceptionHandler)timer_handler, (void *)0);

//    if ((status = register_int_handler(id, extra_timer_int_handler, NULL)) != XST_SUCCESS) {
//	xil_printf ("CLOCK: Unable to register handler. Error code: %d.\r\n", status);

#ifdef DEBUG
  xil_print ("CLOCK: Successfully registered a handler for extra timer interrupts.\r\n");
#endif

  /* Enable PPC non-critical interrupts */

  XExc_mEnableExceptions(XEXC_NON_CRITICAL);

}

/* --- This function provides an orderly shutdown by restoring the original
   --- timer interrupt handler. It should also reset the system time from
   --- the time-of-day clock. */
void shutdown()
{
  interrupt_rate(18.2);
  installFlagG = 0;
}

/* --- This function initializes the scheduler. */
int sched_init(int freq)
{
  /* Initialize the ready queue */
  readyHeadG = readyTailG = 0;
  readyQueueG[0] = NULL;
  /* Initialize the spawn queue (a linked list) */
  spawnQueueG = (procListT *) malloc(sizeof(procListT));
  spawnQueueG->process = NULL;
  spawnQueueG->nextProc = NULL;
  /* Initialize the ready queue (a linked list) */
  onQueueG = (procListT *) malloc(sizeof(procListT));
  onQueueG->process = NULL;
  onQueueG->nextProc = NULL;
  pidG = 1;
  if (onQueueG->process != NULL)
	  xil_printf("problem\n");
  install_handler();
  return interrupt_rate(freq);
}

/* --- sbsSpawn creates a "child process" in the sense that it allows
   --- a function to be executed periodically.
   --- Spawning is just an initialization step. The "process" must
   --- be turned on to execute. */
processT *sbsSpawn(charfnc_ptr f_ptr, float freq, short crit, void *vptr)
{
  procListT     *queue;
  unsigned long per;

  queue = spawnQueueG;

  /* make sure this is not the first process to spawn */
  if (queue->process != NULL){

#ifdef DEBUG
	  xil_printf("not first spawning \n");
#endif

    /* find the end of the linked list */
    while (queue->nextProc != NULL)
      queue = (procListT *) queue->nextProc;

    /* end of list found so allocate a new list element */
    if ((queue->nextProc = (pointer) malloc(sizeof(procListT))) == NULL){
    	xil_printf("Error: malloc failed during spawn (1)\r\n");
    	return NULL;
    } /* endif */
    queue = queue->nextProc;
  } /* endif */

  /* allocate storage for the process structure */
  if ((queue->process = (processT *)malloc(sizeof(processT))) == NULL){
	xil_printf("Error: malloc failed during spawn (2)\r\n");
    return NULL;
  } /* endif */
  /* Initialize the process structure */
  queue->process->pid = pidG++;
  queue->process->status = SBS_OFF;
  queue->process->criticality = crit;
  per = (unsigned long)(heartBeatFreqG / freq);
  if (per < 1){
    per = 1;
    xil_printf("WARNING: requested frequency faster than clock\r\n");
    //XUartLite_SendByte(XPAR_RS232_UART_2_BASEADDR,0x99);
  } /* endif */

#ifdef DEBUG
  xil_printf("per %ld \n",per);
#endif

  queue->process->nPeriod = per;
  queue->process->nextReady = 0;
  queue->process->freq = heartBeatFreqG / per;
  queue->process->missed = 0;
  /* the init process allocates this structure as needed but it is
     deallocated automatically */
  queue->process->local = NULL;

  /* null the function pointers just to be safe. */
  queue->process->on_fptr = NULL;
  queue->process->cycle_fptr = NULL;
  queue->process->off_fptr = NULL;
  queue->process->kill_fptr = NULL;
  queue->process->set_fptr = NULL;
  queue->process->get_fptr = NULL;
  queue->process->eval_fptr = NULL;    /* YC adds to support EVALUATOR 02/15/2015 */

  queue->nextProc = NULL;

  /* Call the init routine, which sets up the function pointers. */
  f_ptr(queue->process, vptr);

  return queue->process;
}

int sbsControl(processT *p_ptr, short cmd)
{
	procListT     *queue;
	procListT     *spawned, *prevSpawn;
	procListT     *temp;
	int           i, tmp;

	queue = onQueueG;
	spawned = spawnQueueG;
	prevSpawn = NULL;

	/* Make sure it has been spawned */
	if (spawned->process == NULL){
		xil_printf("ERROR: nothing spawned \r\n");
		return I_ERROR;
	} /* endif */
	if ((p_ptr == NULL) || (p_ptr->pid >= pidG)){
		xil_printf("ERROR: process ID invalid \r\n");
		return I_ERROR;
	} /* endif */

	switch (cmd){
		case SBS_ON:
			/* make sure it's off */
			if (p_ptr->status != SBS_OFF){
				//sprintf(message, "ERROR: process not off \r\n");
				//UART_SendString(message);
				return I_ERROR;
			} /* endif */

#if CUI_DEBUG
			/* Execute the "ON" routine */
			if (p_ptr->on_fptr != NULL){
				if ((tmp = p_ptr->on_fptr(p_ptr->local)) != I_OK){
					/* If SBS_OFF was returned, call the off routine */
			    	if ((tmp == SBS_OFF) && (p_ptr->off_fptr != NULL))
			    		p_ptr->off_fptr(p_ptr->local);

			    	return tmp;
				}
			} /* endif */
#else
			if ((p_ptr->on_fptr != NULL) && ((tmp = p_ptr->on_fptr(p_ptr)) != I_OK)){
				/* If SBS_OFF was returned, call the off routine */
				//XUartLite_SendByte(XPAR_RS232_UART_2_BASEADDR,0x50);
				if ((tmp == SBS_OFF) && (p_ptr->off_fptr != NULL))
					p_ptr->off_fptr(p_ptr);
				return tmp;
			} /* endif */
#endif

			/* get the process ready to run */
			p_ptr->status = SBS_ON;
			p_ptr->nextReady = timerTicksG;
			//p_ptr->nextReady = timerTicksG + p_ptr->nPeriod;			//Changed by JTL 9/7/14 to equate to older version PBORT
			//XUartLite_SendByte(XPAR_RS232_UART_2_BASEADDR,0x52);
		   // xil_printf("Process on!\r\n");

			/* if it's the first process in the on queue, put it there. */
			if (queue->process == NULL){
				//XUartLite_SendByte(XPAR_RS232_UART_2_BASEADDR,0x53);
#ifdef DEBUG
				xil_printf("first process turned on \n");
#endif

				queue->nextProc = NULL;
				queue->process = p_ptr;

			} else {    /* not first process, so add element to list */
				/* find the end of the linked list */
				//XUartLite_SendByte(XPAR_RS232_UART_2_BASEADDR,0x54);
#ifdef DEBUG
				xil_printf("not first on\n");
#endif

				while (queue->nextProc != NULL){
					queue = (procListT *) queue->nextProc;
					//XUartLite_SendByte(XPAR_RS232_UART_2_BASEADDR,0x55);
				}
				/* end of list found so allocate a new list element */
				if ((temp = (procListT *) malloc(sizeof(procListT))) == NULL)
					return I_ERROR;
				//XUartLite_SendByte(XPAR_RS232_UART_2_BASEADDR,0x56);
				temp->nextProc = NULL;
				temp->process = p_ptr;
				queue->nextProc = (pointer) temp;
				//XUartLite_SendByte(XPAR_RS232_UART_2_BASEADDR,0x57);
			} /* endif */
			break;

  case SBS_OFF:
    /* make sure it's on */
    if (p_ptr->status != SBS_ON){
      //xil_printf("ERROR: process not on!\r\n");
      return I_ERROR;
    } /* endif */

    /* get the process ready to turn off */
    p_ptr->status = SBS_OFFING;

    /* Search for the process in the on queue */
    /* check the first element in the queue */
    if (queue->process->pid == p_ptr->pid){
      /* check if it's the only element in the queue */
      if (queue->nextProc == NULL){
        queue->process = NULL;
      } else {
        /* there are other elements so reset onQueueG and free the
           first element */
        onQueueG = (procListT *) queue->nextProc;
        free(queue);
      } /* endif */
    } else {
      /* it's not the first element in the queue, so search */
      temp = queue;     /* remember the previous element in list */
      queue = (procListT *)queue->nextProc;
      while (queue->process->pid != p_ptr->pid){
        temp = queue;   /* remember the previous element in list */
        if (queue->nextProc != NULL){
          queue = (procListT *)queue->nextProc;
        } else {
        	xil_printf("ERROR: process not found in on queue!\r\n");
          return I_ERROR;
        } /* endif */
      } /* endwhile */

      /* found process in on queue so take it out. */
      temp->nextProc = queue->nextProc;
      free(queue);
    } /* endif */

    /* Execute the "OFF" routine */
    if (p_ptr->off_fptr != NULL)
      p_ptr->off_fptr(p_ptr);

    p_ptr->status = SBS_OFF;

    xil_printf("Process off!\r\n");


    /* what about the ready queue? */

    break;
  case SBS_KILL:
    /* Find the process in the spawned queue */
    while (spawned->process->pid != p_ptr->pid){
      prevSpawn = spawned;
      if (spawned->nextProc != NULL){
        spawned = (procListT *)spawned->nextProc;
      } else {
    	  xil_printf("ERROR: process must be spawned first!\r\n");
        return I_ERROR;
      } /* endif */
    } /* endwhile */

    /* make sure it's off */
    if (spawned->process->status != SBS_OFF){
      sbsControl(p_ptr, SBS_OFF);
    } /* endif */

    /* make sure it's not in the ready queue */
    for (i=0; i<MAX_READY; i++)
      if (readyQueueG[i]->pid == spawned->process->pid)
        readyQueueG[i] = NULL;

    xil_printf("Misses: %d \r\n",spawned->process->missed);

    /* Execute the "KILL" routine */
    if (p_ptr->kill_fptr != NULL)
      p_ptr->kill_fptr(p_ptr);

    /* free the local structure */
    if (p_ptr->local != NULL)
      free(p_ptr->local);

    /* Take it out of the spawned queue */
    /* check if it's the first element of the spawn queue */
    if (prevSpawn == NULL){
      /* see if it's the only element in the spawn queue */
      if (spawned->nextProc == NULL){
        free(spawned->process);
        spawned->process = NULL;
      } else {
        free(spawned->process);
        spawnQueueG = (procListT *)spawned->nextProc;
        free(spawned);
      } /* endif */
    } else {
      free(spawned->process);
      prevSpawn->nextProc = spawned->nextProc;
      free(spawned);
    } /* endif */

    break;
  default:
	  xil_printf("ERROR: Bad sbsControl command!\r\n");
    return I_ERROR;
  } /* endswitch */
  return I_OK;
}

int sbsGet(processT *p_ptr, int type, int arg, void *vptr)
{
  switch(type){
    case SBS_FREQ:
      *((float *)vptr) = p_ptr->freq;
      break;
    case SBS_NTICKS:
      *((long *)vptr) = p_ptr->nPeriod;
      break;
    case SBS_MISSED:
      *((short *)vptr) = p_ptr->missed;
      break;
    case SBS_NAME:
      strcpy((char *)vptr, p_ptr->modName);
      break;
    default:
      if (p_ptr->get_fptr != NULL)
	return (p_ptr->get_fptr(p_ptr, type, arg, vptr));
      else
	return I_ERROR;
  } /* endswitch */

  return I_OK;
}

int sbsSet(processT *p_ptr, int type, int arg, void *vptr)
{
  unsigned long per;

  switch(type){
    case SBS_FREQ:
      per = (unsigned long)(heartBeatFreqG / *((float *)vptr));
      if (per < 1){
	per = 1;\
	xil_printf("WARNING: requested frequency faster than clock!\r\n");
      } /* endif */

      p_ptr->nPeriod = per;
      p_ptr->freq = heartBeatFreqG / per;
      break;
    case SBS_NTICKS:
      p_ptr->nPeriod = *((unsigned long *)vptr);
      p_ptr->freq = heartBeatFreqG / p_ptr->nPeriod;
      break;
    case SBS_MISSED:
      p_ptr->missed = 0;
      break;
    default:
      if (p_ptr->set_fptr != NULL){
#ifdef DEBUG
    	  xil_printf("userSet ptr:%08X\n",(long)p_ptr->set_fptr);
#endif
	return (p_ptr->set_fptr(p_ptr, type, arg, vptr));
      }else
	return I_ERROR;
  } /* endswitch */

  return I_OK;
}

/* --- sbsSpawn creates a "child process" in the sense that it allows
   --- a function to be executed periodically.
   --- Spawning is just an initialization step. The "process" must
   --- be turned on to execute. */
/*void sbsBuildModList(int num_of_nodes)
{
	int i,n;
	char Tx_Message_node1[] = "!OS0E0201";
	char *message;
	for(i =1;i<num_of_nodes;i++)
	{
		if(i== myNodeG)
		{
		n = getDatabasesize();
		}
		else
		{
			//Zigbee_Transmit(Tx_Message_node1);
			//Zigbee_Receive();


		}
	}
	 XUartLite_SendByte(UARTLITE_BASEADDRESS, n);
}
*/

/* --- sbsModPtr returns the processT pointer to the specified
   --- module if the name exists. NULL is returned if the name
   --- is not found. */
processT *sbsModPtr(char *mname)
{
  procListT	*pList = spawnQueueG;

  while ((pList != NULL) && (pList->process != NULL)){
    if (strcmp(pList->process->modName, mname) == 0)
      return pList->process;
    pList = pList->nextProc;
  } /* endwhile */

  return NULL;
}

/* --- sbsListMods prints a list of module names. */
int sbsListMods()
{
  procListT	*pList = spawnQueueG;

  while ((pList != NULL) && (pList->process != NULL)){
    if (pList->process->modName == NULL)
    {
    	xil_printf("Error: no modname assigned!\r\n");
    }
    else
    {
    	xil_printf("%s\r\n",pList->process->modName);
    }
    pList = pList->nextProc;
  } /* endwhile */

  return I_OK;
}

/********Parse OS Command************/					//Added by JTL 10/23/14 to support internode communication using the same method as code migration
void parseOSCmd(uint8_t *str)
{
  uint16_t			len, rec, i, dataReturnSize,cmdID,rech;
  char				slen[5], *dummy;
  char 				*srcNodeStr = "SN";
  char      		*header = "!OS";
  char 				*buf = malloc(6);
  char 				*response = "%%";
  unsigned char 	compID;
  uint8_t			dataIn,srcNode;
  procListT     	*queue;
  uint16_t			evPerf = 0;
  uint16_t			FesPerf = 0;
  uint16_t			NFesPerf = 0;
  uint8_t			estOut = 0;
  char 				*EVRSRec = "0B";
  char 				*ESRSRec = "0D";

  // After the '!OS'and the node destination get 'record-type'
  slen[0] = str[0];
  slen[1] = str[1];
  slen[2] = str[2];
  slen[3] = str[3];
  slen[4] = 0;


  rec = (uint16_t)strtol(slen,&dummy,16);	// Record type
  rech = rec>>8;
  rec = (rec&0x00FF);
 // XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,(rech));
  //XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,(rec));
  //XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,(rec>>8));


  // Next get cmd id
  slen[0] = str[4];
  slen[1] = str[5];
  slen[2] = str[6];
  slen[3] = str[7];
  slen[4] = 0;

  cmdID = (uint16_t)strtol(slen,&dummy,16);	//command ID
 // XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,(cmdID&0x00FF));
 // XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,(cmdID>>8));
  //get the source node number
  // Next get cmd id
    slen[0] = str[8];
    slen[1] = str[9];
    slen[2] = 0;

    srcNode = (uint8_t)strtol(slen,&dummy,16);	//command ID
   // XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,srcNode);

    slen[0] = str[10];
	slen[1] = str[11];
	slen[2] = 0;

	len = (uint8_t)strtol(slen,&dummy,16);	//command ID
	//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,len);

#if 1  // Process the string as appropriate for the record type

    if(rech == REQUEST_COMMAND)
    {
      	switch(rec){
		case DATA_IN_REC:			// Copy data to the Comp_ID input port
		slen[0] = str[12];
		slen[1] = str[13];
		slen[2] = 0;

		compID = (unsigned long)strtol(slen,&dummy,16);		//component ID

		for(i=0; i<(2*len); i++)
		{
			slen[i] = str[14+i];
		}
		slen[i+1] = 0;

		dataIn = (uint8_t)strtol(slen,&dummy,16);

		queue = spawnQueueG;
		for(i=1; i<pidG; i++)
		{
			if(queue->process->pid == compID)
			{
				sbsSet(queue->process, PORT_IN, 0, (void *)&dataIn);
				break;
			}
			queue = (procListT *)queue->nextProc;
		}
		break;
		case EVRQ_REC:		// Request for evaluator output from Comp_ID
			slen[0] = str[12];
			slen[1] = str[13];
			slen[2] = 0;
			//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,0x1F);
			compID = (unsigned char)strtol(slen,&dummy,16);		//component ID

			queue = spawnQueueG;
			for(i=1; i<pidG; i++)
			{
				if(queue->process->pid == compID)
				{
					sbsGet(queue->process, FR_EVAL, 0, (void *)&evPerf);
					break;
				}
				queue = (procListT *)queue->nextProc;
			}
			//Now we've gotten the evaluation data and need to send the response back to the source node.
			//optionally we can add more compIDs in the data section to tell the node to send evaluation data for multiple components
			/*for(i=0; i<(2*len); i++)
			{
				slen[i] = str[6+i];
			}
			slen[i+1] = 0;*/

			srcNodeStr[0] = str[8];
			srcNodeStr[1] = str[9];

			dataReturnSize = sizeof(evPerf);
			i =0;
			while(i<3)
			{
				response[i] = header[i];
				i++;
			}
			sprintf (buf, "%u",srcNode );
			//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,buf[0]);
			i = 0;
			while(i<2)
			{
				response[i+3] = buf[1-i];
				i++;
			}

			//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,buf[0]);


			response[5] = '0';
			response[6] = '1';
			response[7] = '0';
			response[8] = 'B';
			cmdID = 0xABCD;

			sprintf (buf, "%X",cmdID);
			if(cmdID<=0x000F)
			{XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,0x41);
				response[9]= '0';
				response[10]= '0';
				response[11]= '0';
				response[12]= buf[0];
			}
			if ((cmdID<=0x00FF)&&(cmdID>0x000F))
			{
				response[9]= '0';
				response[10]= '0';
				response[11]= buf[0];
				response[12]= buf[1];
			}
			if ((cmdID>0x00FF)&&(cmdID<=0x0FFF))
			{
				response[9]= '0';
				response[10]= buf[0];
				response[11]= buf[1];
				response[12]= buf[2];
			}

			if ((cmdID>0x0FFF)&&(cmdID<=0xFFFF))
			{
				response[9]= buf[0];
				response[10]= buf[1];
				response[11]= buf[2];
				response[12]= buf[3];
			}
			//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,response[9]);
			//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,response[10]);

			//i = 0;
			//response[9] = cmdID;
			//i++;
			sprintf (buf, "%X",myNodeG);
			response[13] = buf[0];
			response[14] = buf[1];
		   // XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,response[13]);
			//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,response[14]);

			sprintf (buf, "%u",dataReturnSize);
			response[15] = buf[0];
			response[16] = buf[1];
			//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,response[15]);
			//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,response[16]);
				sprintf (buf, "%X",evPerf);
				if(evPerf<=0x000F)
				{XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,0x41);
					response[17]= '0';
					response[18]= '0';
					response[19]= '0';
					response[20]= buf[0];
				}
				if ((evPerf<=0x00FF)&&(evPerf>0x000F))
				{
					response[17]= '0';
					response[18]= '0';
					response[19]= buf[0];
					response[20]= buf[1];
				}
				if ((evPerf>0x00FF)&&(evPerf<=0x0FFF))
				{
					response[17]= '0';
					response[18]= buf[0];
					response[19]= buf[1];
					response[20]= buf[2];
				}

				if ((evPerf>0x0FFF)&&(evPerf<=0xFFFF))
				{
					response[17]= buf[0];
					response[18]= buf[1];
					response[19]= buf[2];
					response[20]= buf[3];
				}


			XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,response[17]);
			//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,response[18]);
			//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,response[19]);
			//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,response[20]);
			free(buf);

			//xil_printf("!%s%02X%s%02X%04X\r\n",srcNodeStr, dataReturnSize, EVRSRec, compID, evPerf);		//If the nodes are connected through comm1, use this
			//xil_printf2("!%s%02X%s%02X%04X\r\n",srcNodeStr, dataReturnSize, EVRSRec, compID, evPerf);		//If the nodes are connected through comm2, use this

			//char *command1 = "!010109011E";
			//xil_printf2(command1);


		break;
		//This case is not tested yet.
		/*case EVRS_REC:		//Response to evaluator request
			slen[0] = str[12];
			slen[1] = str[13];
			slen[2] = 0;

			compID = (unsigned char)strtol(slen,&dummy,16);		//component ID

			slen[0] = str[14];
			slen[1] = str[15];
			slen[2] = str[16];
			slen[3] = str[17];
			slen[4] = 0;

			evPerf = (uint16_t)strtol(slen,&dummy,16);

			evBufferG[compID] = evPerf;

			//xil_printf("Eval Performance Received: %d\r\n", evBufferG[compID]);

		break;*/
		case ESRQ_REC:		// Request for estimator output from Comp_ID
			slen[0] = str[12];
			slen[1] = str[13];
			slen[2] = 0;

			compID = (unsigned char)strtol(slen,&dummy,16);		//component ID

			srcNodeStr[0] = str[8];
			srcNodeStr[1] = str[9];

			//assuming one byte of input data for now
			slen[0] = str[14];
			slen[1] = str[15];
			slen[2] = 0;

			dataIn = (uint8_t)strtol(slen,&dummy,16);

			queue = spawnQueueG;
			for(i=1; i<pidG; i++)
			{
				if(queue->process->pid == compID)
				{
					sbsSet(queue->process, EST_DATA_IN, 0, (void *)&dataIn);
					sbsControl(queue->process, SBS_EST);
					sbsGet(queue->process, FR_EST, 0, (void *)&FesPerf);
					sbsGet(queue->process, NFR_EST, 0, (void *)&NFesPerf);
					sbsGet(queue->process, EST_OUTPUT, 0, (void *)&estOut);
					break;
				}
				queue = (procListT *)queue->nextProc;
			}
			//Now we've gotten the evaluation data and need to send the response back to the source node.
			//optionally we can add more compIDs in the data section to tell the node to send estimation data for multiple components
			/*for(i=0; i<(2*len); i++)
			{
				slen[i] = str[6+i];
			}
			slen[i+1] = 0;*/

			dataReturnSize = (sizeof(FesPerf)+sizeof(NFesPerf)+sizeof(estOut));

			xil_printf("!%s%02X%s%02X%04X%04X%02X\r\n",srcNodeStr, dataReturnSize, ESRSRec, compID, FesPerf, NFesPerf, estOut);		//If the nodes are connected through comm1, use this
			//xil_printf2("!%s%02X%s%02X%04X%04X%02X\r\n",srcNodeStr, dataReturnSize, ESRSRec, compID, FesPerf, NFesPerf, estOut);		//If the nodes are connected through comm2, use this

			//char *command1 = "!010109011E";
			//xil_printf2(command1);


		break;
		//has not been tested yet
		/*case ESRS_REC:		//Response to estimator request
			slen[0] = str[12];
			slen[1] = str[13];
			slen[2] = 0;

			compID = (unsigned char)strtol(slen,&dummy,16);		//component ID

			slen[0] = str[14];
			slen[1] = str[15];
			slen[2] = str[16];
			slen[3] = str[17];
			slen[4] = 0;

			FesPerf = (uint16_t)strtol(slen,&dummy,16);

			slen[0] = str[18];
			slen[1] = str[19];
			slen[2] = str[20];
			slen[3] = str[21];
			slen[4] = 0;

			NFesPerf = (uint16_t)strtol(slen,&dummy,16);

			slen[0] = str[22];
			slen[1] = str[23];
			slen[2] = 0;

			estOut = (uint8_t)strtol(slen,&dummy,16);

			estFBufferG[compID] = FesPerf;
			estNFBufferG[compID] = NFesPerf;
			estOutBufferG = estOut;

			//xil_printf("EstF Performance Received: %d\r\n", estFBufferG[compID]);
			//xil_printf("EstNF Performance Received: %d\r\n", estNFBufferG[compID]);
			//xil_printf("Est Output Received: %d\r\n", estOutBufferG);
		break;*/
		default:
			//error no matching type
		break;
  }
    }
    else if(rech == RESPONSE_CONTAINED)
    {
    	switch(rec){
    	//This case is not tested yet.
		case EVRS_REC:		//Response to evaluator request
			slen[0] = str[12];
			slen[1] = str[13];
			slen[2] = 0;

			compID = (unsigned char)strtol(slen,&dummy,16);		//component ID

			slen[0] = str[14];
			slen[1] = str[15];
			slen[2] = str[16];
			slen[3] = str[17];
			slen[4] = 0;

			evPerf = (uint16_t)strtol(slen,&dummy,16);

			evBufferG[compID] = evPerf;

			//xil_printf("Eval Performance Received: %d\r\n", evBufferG[compID]);
			break;
			//has not been tested yet
			case ESRS_REC:		//Response to estimator request
				slen[0] = str[12];
				slen[1] = str[13];
				slen[2] = 0;

				compID = (unsigned char)strtol(slen,&dummy,16);		//component ID

				slen[0] = str[14];
				slen[1] = str[15];
				slen[2] = str[16];
				slen[3] = str[17];
				slen[4] = 0;

				FesPerf = (uint16_t)strtol(slen,&dummy,16);

				slen[0] = str[18];
				slen[1] = str[19];
				slen[2] = str[20];
				slen[3] = str[21];
				slen[4] = 0;

				NFesPerf = (uint16_t)strtol(slen,&dummy,16);

				slen[0] = str[22];
				slen[1] = str[23];
				slen[2] = 0;

				estOut = (uint8_t)strtol(slen,&dummy,16);

				estFBufferG[compID] = FesPerf;
				estNFBufferG[compID] = NFesPerf;
				estOutBufferG = estOut;

				//xil_printf("EstF Performance Received: %d\r\n", estFBufferG[compID]);
				//xil_printf("EstNF Performance Received: %d\r\n", estNFBufferG[compID]);
				//xil_printf("Est Output Received: %d\r\n", estOutBufferG);
			break;
    	}
    }



#endif
}

/* ******************************************************************** */
/*      parseDLline                                                     */
/* ******************************************************************** */

/* --- parseDLline()
   --- This fxn parses a download line from an Intel HEX file and loads it into appropriate global buffers.
   --- Only the Data Record (00) and EOF Record (01) are implemented, plus two new records for relocation:
   --- The Offset Record offsets specific pointer assignments in memory.
   --- The InitPtr Record provides a pointer to a function in the code.
   --- */
void parseDLline(uint8_t *str)
{
  uint8_t	len, rec, i;
  char		slen[5], *dummy;
  uint8_t	n, pstr[60];
  uint16_t	addr, adjAddr;
  int16_t	off;

  // After the ':' char is 1 byte that encodes the number of data bytes
  slen[0] = str[0];
  slen[1] = str[1];
  slen[2] = 0;

  len = (uint8_t)strtol(slen,&dummy,16);		// length of data

  // After the length is the address (2 bytes)
  slen[0] = str[2];
  slen[1] = str[3];
  slen[2] = str[4];
  slen[3] = str[5];
  slen[4] = 0;

  addr = (uint16_t)strtol(slen,&dummy,16);		// raw address (as found in compiled code)

  // If this is the first data record, store the initial address
  // This assumes the records are contiguous
  if (codeIndexG == 0)
  	codeOrigAddrG = addr;

  // After the address is a 1-byte record type
  slen[0] = str[6];
  slen[1] = str[7];
  slen[2] = 0;

  rec = (uint8_t)strtol(slen,&dummy,16);		// Record type

  // Process the string as appropriate for the record type
  switch(rec){
  	case DATA_REC:			// Store data records in the temporary code buffer in SRAM
		if (codeStatusG >=2)
			serXmit1((unsigned char *)"REC1 ",5);	// ERROR - out of order records

		// Grab one byte at a time
		slen[2] = 0;
		for(i=8; i< 8+(len<<1); i++){
			slen[0] = str[i++];
			slen[1] = str[i];
			codeBufG[codeIndexG++] = (uint8_t)strtol(slen,&dummy,16);
		}
		//should use checksum here
		codeStatusG = 1;		// Indicate we are downloading code
	break;
  	case OFFSET_REC:		// Offset the fxn pointers in the code (calls also need to be updated - this won't work for that)
		if (codeStatusG >=3)
			serXmit1((unsigned char *)"OFF1 ",5);		// ERROR - out of order

		// The record consists of an address (in the original) and the offset from the initial address
		slen[0] = str[8];
		slen[1] = str[9];
		slen[2] = str[10];
		slen[3] = str[11];
		slen[4] = 0;

		addr = (uint16_t)strtol(slen,&dummy,16);

		n=sprintf((char *)pstr,"Addr 0x%X: %X \n\r",addr, addr-codeOrigAddrG);
		serXmit1((unsigned char *)pstr,n);

		// Now get the offset
		slen[0] = str[12];
		slen[1] = str[13];
		slen[2] = str[14];
		slen[3] = str[15];
		slen[4] = 0;

		off = (uint16_t)strtol(slen,&dummy,16);

		n=sprintf((char *)pstr,"Off %d: %02X%02X \n\r",off, codeBufG[addr-codeOrigAddrG+1], codeBufG[addr-codeOrigAddrG]);
		serXmit1((unsigned char *)pstr,n);

		// Adjust byte address (from map file) to word address for the ATmega
		// codeAddrG holds the destination address for the code block
		adjAddr = (codeAddrG + off) >> 1;

		//Adjust the loaded addresses in the code buffer
		// 16-bit loads are spread into the lower nibbles of 4 consecutive bytes
		codeBufG[addr-codeOrigAddrG] &= 0xF0;
		codeBufG[addr-codeOrigAddrG] |= adjAddr & 0x0F;
		codeBufG[addr-codeOrigAddrG+1] &= 0xF0;
		codeBufG[addr-codeOrigAddrG+1] |= (adjAddr>>4) & 0x0F;
		codeBufG[addr-codeOrigAddrG+2] &= 0xF0;
		codeBufG[addr-codeOrigAddrG+2] |= (adjAddr>>8) & 0x0F;
		codeBufG[addr-codeOrigAddrG+3] &= 0xF0;
		codeBufG[addr-codeOrigAddrG+3] |= (adjAddr>>12) & 0x0F;

		codeStatusG = 2;
	break;
  	case INITPTR_REC:			// Send a function pointer (byte address in original map file)
		if (codeStatusG >=3)
			serXmit1((unsigned char *)"INIT1 ",6);		// ERROR - out of order

		// Get the byte address
		slen[0] = str[8];
		slen[1] = str[9];
		slen[2] = str[10];
		slen[3] = str[11];
		slen[4] = 0;

		addr = (uint16_t)strtol(slen,&dummy,16);

		n=sprintf((char *)pstr,"Addr 0x%X: %X \n\r",addr, addr-codeOrigAddrG);
		serXmit1((unsigned char *)pstr,n);


		// Adjust byte address to word address for the ATmega
		codeInitAddrG = (addr - codeOrigAddrG + codeAddrG) >> 1;

		codeStatusG = 3;
	break;

	// May need additional record types for relocating code

  	case EOF_REC:			// This can be used as a flag to Spawn the module when complete
		codeStatusG = 5;	// Not implemented that way yet
	break;
  }
}

#if !DEBUG
/* TODO:
 * --- This function extends traditional PBO by adding an EVALUATOR.
 * --- NEED TO FIGURE OUT: API sbsEvaluator(processT *p_ptr, void * funcPerfBuffer; void *nonFuncPerfBuffer)?????
 */
int sbsEvaluator(processT *p_ptr)
{
//	procListT     *queue;
	procListT     *spawned; //, *prevSpawn;
//	procListT     *temp;
//	int           i, tmp;

//	queue = onQueueG;
	spawned = spawnQueueG;
//	prevSpawn = NULL;

	/* Make sure it has been spawned */
	if (spawned->process == NULL){
		xil_printf("ERROR: the corresponding executor (PBO) is not spawned \r\n");
		return I_ERROR;
	}
	if ((p_ptr == NULL) || (p_ptr->pid >= pidG)){
		xil_printf("ERROR: the corresponding executor's (PBO) process ID is invalid \r\n");
		return I_ERROR;
	}

	/* Make sure it's on */
	if (p_ptr->status != SBS_ON){
		xil_printf("ERROR: the corresponding executor (PBO) is not ON \r\n");
		return I_ERROR;
	}

	/* Make sure EVALUATOR is defined */
	if (p_ptr->eval_fptr == NULL){
		xil_printf("ERROR: no evaluator defined \r\n");
		return I_ERROR;
	}

	p_ptr->eval_fptr(p_ptr);

	return I_OK;
}
#endif


void sched()
{
	//XUartLite_SendByte(UARTLITE_BASEADDRESS, 0x41);
	uint8_t	n,m;
	/* check if anything is on the ready queue */
	if (readyHeadG != readyTailG){
		/* Make sure it's a real pointer */
		if (readyQueueG[readyHeadG] == NULL){
			if (++readyHeadG >= MAX_READY)
				readyHeadG = 0;
		}
		else {
			/* execute the next function on the ready queue by passing the ProcessT* to the cycle fxn */
			if ((readyQueueG[readyHeadG])->cycle_fptr(readyQueueG[readyHeadG]) == SBS_OFF)
				sbsControl(readyQueueG[readyHeadG], SBS_OFF);
			if (++readyHeadG >= MAX_READY)
				readyHeadG = 0;
		} /* endif */
	} /* endif */

	/*n = serGetDLline1(strG);	// Check each serial char for the ':' indicator

	if(n >0){					// This is just diagnostics
		//serXmit1((unsigned char *)"N = something\r\n",16);
		serXmit1((unsigned char *)strG,n);
	  	parseDLline(strG);
	}*/
	//XUartLite_SendByte(UARTLITE_BASEADDRESS, 0xAA);
//	m = serGetOSCmd1(strG);		// Check each serial char for the '!' indicator
	//XUartLite_SendByte(UARTLITE_BASEADDRESS, 0xAA);
	//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,m);
//		if(m > 0)
//		{
//			//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,0x38);
//			//serXmit1((unsigned char *)strG,m);
//		  	parseOSCmd(strG);
//		}

}
