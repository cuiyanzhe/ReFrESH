/* --- menu.c       Version 1.0
   --- this module is a console to build a task.
   ---
   --- Copyright 2015, Collaborative Robotics Lab .
   ---
   --- 02/10/15 YC	Initial coding. V1.0
   --- */

/* ********************************************************************************************************************************************* */
/*      Include Files						                                              														 */
/* ********************************************************************************************************************************************* */

#include <string.h>
#include <stdlib.h>
#include "pbort.h"
#include "serial.h"

/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */

#define DEBUG 0

#define TARGET_SERACH		0x30	/* 0 */
#define TRAJ_GEN			0x31	/* 1 */
#define MOVE_TO_TARGET		0x32	/* 2 */
#define WAIT_STABLE			0x33	/* 3 */
#define CAMERA_READER		0x61	/* a */
#define SSD					0x62	/* b */
#define TRAJ_GEN_COMP		0x63	/* c */
#define SERVO_ROTATE		0x64	/* d */


/* ******************************************************************** */
/*       menu_on                Start up the module.                    */
/* ******************************************************************** */
char menu_on(processT *p_ptr)
{
	return I_OK;
}


/* ******************************************************************** */
/*      Menu handler            Handle task build and  Debug            */
/* ******************************************************************** */

/* This is the menu handler via the serial port */

/* --- 0	TARGET_SEARCH		*/
/* --- 1	TRAJ_GEN			*/
/* --- 2	MOVE_TO_TARGET		*/
/* --- 3	WAIT_STABLE			*/
/* --- a	CAMERA_READER		*/
/* --- b	SSD(TARGET MATCH)	*/
/* --- c	TRAJ_GENERATION		*/
/* --- d	SERVO_ROTATE		*/

char menu_cycle(processT *p_ptr)
{
	unsigned char str[10];
	uint8_t n;

	/* look for incoming serial commands */
	if (serRecvChk1() != 0){
		str[0] = 0;
		n = serGetChar1(str);
		switch(str[0]){

#if DEBUG
			case CAMERA_READER:
				sbsControl(camIDG,SBS_ON);
			break;

			case TARGET_SERACH:/* read camera*/
				if(cam_st == 0){
					sbsControl(camIDG,SBS_ON);
					cam_st = 1;
				}
				else if(cam_st  == 2){
					cam_st = 0;
					local->state = 1;
				}
			break;

			case TRAJ_GEN: /*check for target*/
				if(ssd_st == 0){
					sbsControl(ssdIDG,SBS_ON);
					ssd_st = 1;
				}
				else if (ssd_st == 2){
					ssd_st = 0;
					template_cnt++;
					if (template_cnt >= 3)
						template_cnt = 0;
					sbsSet(ssdIDG, TEMP_COUNT, 0, (void *)&template_cnt);
					local->state = 2;
				}
			break;

			case MOVE_TO_TARGET: /*move to target*/
				if(traj_st == 0 && hex_st == 0){
					sbsControl(trajIDG,SBS_ON);
					sbsControl(hexmanipulatorIDG, SBS_ON);
					traj_st = 1;
					hex_st = 1;
				}
				else if (traj_st == 2 && hex_st == 2){
					traj_st = 0;
					hex_st = 0;
					local->state = 3;
				}
			break;

			case WAIT_STABLE: /*sit and stare state*/
				i++;
				if(i>=2)
				{	i = 0;
					local->state = 0;
				}
			break;

			default:
				xil_printf("Unknown Command \r\n");
				break;
#endif
		}
	}
}

/* ******************************************************************** */
/*        menu_init             Initiate module information.            */
/* ******************************************************************** */

char menu_init(processT *p_ptr, void *vptr)
{
	p_ptr->on_fptr = menu_on;
	p_ptr->cycle_fptr = menu_cycle;
	p_ptr->off_fptr = NULL;

	return I_OK;
}




