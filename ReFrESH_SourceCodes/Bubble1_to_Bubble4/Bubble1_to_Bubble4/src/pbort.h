/* --- PBORT.H
   ---
   --- Include file for PBO/RT scheduler.
   ---
   --- Copyright 1996 Trident Robotics and Research, Inc.
   ---
   --- 01/20/96 RMV     Initial coding
	--- 02/04/10 RMV		Modified for RecoNode
   ---
   --- */

#ifndef	SCHED_H
#define  SCHED_H

#include "stdint.h"
//#include "BasicRF.h"
//#include "cc2520.h"

#define         SBS_ON          100
#define         SBS_OFF         101
#define         SBS_KILL        102
#define         SBS_OFFING      103
#define			SBS_EST			104
#define 		I_OK			1
#define 		I_ERROR			-1
#define 		NUM_OF_TASKS		30


/* Set and get options */
#define		SBS_FREQ	200
#define		SBS_NTICKS	201
#define		SBS_MISSED	202
#define		SBS_NAME	203

//#define		TRACK_EST		204
//#define		TRACK_EVAL		205
//#define		DATA_IN			206
//#define		DATA_OUT		207
//#define		EST_INPUT		208
//#define		EST_OUTPUT		209
//#define		IN_SIZE		  	210
//#define		OUT_SIZE		211
//#define		TEMP_COUNT		212

#define		TRACK_EST		204
#define		TRACK_EVAL		205
#define		DATA_IN			206
#define		DATA_OUT		207
#define		EST_INPUT		208
#define		EST_OUTPUT		209
#define		IN_SIZE		  	210
#define		OUT_SIZE		211
#define		TEMP_COUNT		212
#define		PORT_IN			213
#define		FR_EVAL			214
#define		FR_EST			215
#define		NFR_EST			216
#define		EST_DATA_IN		217
#define		TYPE			218
#define		LOCAL_STATE		219

//typedef void interrupt (*hndlr_ptr)(void);
typedef void (*voidfnc_ptr)();
typedef int (*intfnc_ptr)();
typedef char (*charfnc_ptr)();
typedef char (*setfnc_ptr)(void*, int16_t, int16_t, void*);
//typedef char (*setfnc_ptr)(processT *, int16_t, int16_t, void*);
typedef void *pointer;

typedef struct{ charfnc_ptr     on_fptr;
                charfnc_ptr     cycle_fptr;
                charfnc_ptr     off_fptr;
                charfnc_ptr		eval_fptr;		/* ??????? YC added to support EVALUATOR 02/15/2015 */
                charfnc_ptr     kill_fptr;
//	            charfnc_ptr     get_fptr;
//	            charfnc_ptr     set_fptr;
                setfnc_ptr 		get_fptr;
                setfnc_ptr 		set_fptr;
//                charfnc_ptr		eval_fptr;	/* ??????? YC added to support EVALUATOR 02/15/2015 */
		        unsigned long   pid;
                short           status;
                short           criticality;
                unsigned long   nPeriod;
                unsigned long   nextReady;
                float           freq;
                short           missed;
		        pointer         local;
		        char			modName[16];
		    }   processT;

typedef struct{ processT        *process;
                pointer         nextProc;
              } procListT;

#ifndef SCHED_SRC
#define SCHED_SRC

extern int		heartBeatFreqG;
//extern float    clockPeriodG;
extern long		timerTicksG;
extern const unsigned char myNodeG;
extern processT *sbsSpawn(charfnc_ptr f_ptr, float freq, short crit, void *vptr);

extern int sbsControl(processT *p_ptr, short cmd);

extern int sbsSet(processT *p_ptr, int type, int arg, void *vptr);

extern int sbsGet(processT *p_ptr, int type, int arg, void *vptr);

extern void sbsBuildModList(int num_of_nodes);

/* --- Returns a pointer to the processT structure for a given mod name */
extern processT *sbsModPtr(char *mname);

/* --- sbsListMods prints a list of module names. */
extern int sbsListMods();

/* YC adds this for EVALUATOR 02/15/2015 */
extern int sbsEvaluator(processT *p_ptr);

extern procListT *spawnQueueG;

extern void sched();

extern int sched_init(int freq);

/* --- This function re-installs the original timer interrupt handler. */
extern void shutdown();

extern void parseDLline(uint8_t *str);

//Hacks for testing Migrate functionality
extern uint8_t		codeBufG[256];
extern uint8_t		codeIndexG;
extern uint8_t		codeStatusG;
extern uint16_t		codeAddrG;
extern uint16_t		codeInitAddrG;

extern uint8_t		fillerCodeG[0x70];
extern uint8_t		newCodeG[1024];
extern uint8_t		*baseCodeG;
extern uint8_t		progCodeG[256];


#endif
#endif
