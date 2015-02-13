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
#include "refresh_main.h"

/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */

#define DEBUG 1

#define SBS_SET_TEST_ON			0x30	/* 0 */
#define SBS_SET_TEST_OFF		0x39	/* 9 */

#define TARGET_SERACH			0x31	/* 1 */
#define TRAJ_GEN				0x32	/* 2 */
#define MOVE_TO_TARGET			0x33	/* 3 */
#define WAIT_STABLE				0x34	/* 4 */

#define CAMERA_READER_ON		0x61	/* a */
#define CAMERA_READER_OFF		0x41	/* A */
#define SSD_ON					0x62	/* b */
#define SSD_OFF					0x42	/* B */
#define TRAJ_GEN_ON				0x63	/* c */
#define TRAJ_GEN_OFF			0x43	/* C */
#define SERVO_ON				0x64	/* d */
#define SERVO_OFF				0x44	/* D */
#define VS_TASK_ON				0x74	/* t */
#define VS_TASK_OFF				0x54	/* T */

#define IMG_HEIGHT 10
#define IMG_WIDTH  10

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

/* --- 0	SBS_SET_TEST_ON		*/
/* --- 9	SBS_SET_TEST_OFF		*/
/* --- 1	TARGET_SEARCH		*/
/* --- 2	TRAJ_GEN			*/
/* --- 3	MOVE_TO_TARGET		*/
/* --- 4	WAIT_STABLE			*/
/* --- a	CAMERA_READER_ON		*/
/* --- A	CAMERA_READER_OFF		*/
/* --- b	SSD_ON	*/
/* --- B	SSD_OFF	*/
/* --- c	TRAJ_GEN_ON		*/
/* --- C	TRAJ_GEN_OFF		*/
/* --- d	SERVO_ROTATE		*/

/* --- T	VS_TASK_ON		*/

char menu_cycle(processT *p_ptr)
{
	unsigned char str[10];
	uint8_t n;

	/* look for incoming serial commands */
	if (serRecvChk1() != 0){
		str[0] = 0;
		n = serGetChar1(str);
		switch(str[0]){
			/* in order to test individual component, be sure change from TASK_DEBUG to XXX_DEBUG mode
			 * or you can run SBS_SET_TEST_ON firstly. Otherwise, the pointers are not initialized */
			case CAMERA_READER_ON:
				xil_printf("Turn on camReader component to test!\r\n");
				sbsControl(camReaderIDG,SBS_ON);
			break;

			case CAMERA_READER_OFF:
				sbsControl(camReaderIDG,SBS_OFF);
				xil_printf("Turn off camReader component!\r\n");
			break;

			case  SSD_ON:
				xil_printf("Turn on SSD component to test!\r\n");
				sbsControl(ssdIDG,SBS_ON);
			break;

			case  SSD_OFF:
				sbsControl(ssdIDG,SBS_OFF);
				xil_printf("Turn off SSD component!\r\n");
			break;

			case  TRAJ_GEN_ON:
				xil_printf("Turn on trajGen component to test!\r\n");
				sbsControl(trajGenIDG,SBS_ON);
			break;

			case  TRAJ_GEN_OFF:
				sbsControl(trajGenIDG,SBS_OFF);
				xil_printf("Turn off trajGen component!\r\n");
			break;

			case SBS_SET_TEST_ON:
				/* Turn off components just in case
				 * Don't forget to flip xxx_DEBUG and TASK_DEBUG from 0 to 1 and 1 to 0 in each 'component'.c */
				sbsControl(camReaderIDG, SBS_OFF);
				sbsControl(ssdIDG, SBS_OFF);
				sbsControl(trajGenIDG, SBS_OFF);

				/* Turn on components */
				sbsControl(camReaderIDG, SBS_ON);
				sbsControl(ssdIDG, SBS_ON);
				sbsControl(trajGenIDG, SBS_ON);

				/* Connect components manually */
				uint8_t *imgBuffer = (uint8_t*)malloc(IMG_HEIGHT * IMG_WIDTH);
				uint8_t *tempPosBuffer = (uint8_t*)malloc(2);	/* X & Y location in image frame, 2 bytes */

				sbsSet(camReaderIDG, DATA_OUT, 0, (void *)imgBuffer);
				sbsSet(ssdIDG, DATA_IN, 0, (void *)imgBuffer);
				sbsSet(ssdIDG, DATA_OUT, 0, (void *)tempPosBuffer);
				sbsSet(trajGenIDG, DATA_IN, 0, (void *)tempPosBuffer);
			break;

			case SBS_SET_TEST_OFF:
				sbsControl(camReaderIDG, SBS_OFF);
				sbsControl(ssdIDG, SBS_OFF);
				sbsControl(trajGenIDG, SBS_OFF);
				xil_printf("Turn off all component for safe!\r\n");
			break;

			case VS_TASK_ON:
				xil_printf("Turn on visualServoTask component to test!\r\n");
				sbsControl(visualServoTaskIDG, SBS_ON);
			break;

			case VS_TASK_OFF:
				sbsControl(visualServoTaskIDG, SBS_OFF);
				sbsControl(camReaderIDG, SBS_OFF);
				sbsControl(ssdIDG, SBS_OFF);
				sbsControl(trajGenIDG, SBS_OFF);
				xil_printf("Turn off visualServoTask component for safe!\r\n");
			break;

#if !DEBUG
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




