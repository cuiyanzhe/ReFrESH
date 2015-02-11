/* --- time.c       Version 1.0
   --- This module provides function related to system clock.
   ---
   --- Copyright 2001, Mark V Automation Corp.
   ---
   --- 12/07/14 GYJ	Initial coding. V1.0
   ---
   --- */

/* ********************************************************************************************************************************************* */
/*      Include Files						                                              														 */
/* ********************************************************************************************************************************************* */
#include <stdint.h>
#include "xtime_l.h"
#include "sleep.h"
#include "time.h"

/* ********************************************************************************************************************************************* */
/*      Function prototype                                              																		 */
/* ********************************************************************************************************************************************* */

/* ******************************************************************** */
/*      system time											            */
/* ******************************************************************** */

/* --- initialize_time() sets system clock at 0.
 * ---
 */
void initialize_time()
{
	xil_printf("System Time Initializing... \r\n");

	XTime_SetTime(0);
}

/* --- get_sys_time_ms() give system time in ms.
 * ---
 */
uint32_t get_sys_time_ms()
{
	uint32_t sys_time_ms;
	XTime sys_time;
	XTime tmp_time;

	XTime_GetTime(&sys_time);
	tmp_time = sys_time;
	sys_time_ms = (uint32_t)(tmp_time/100000);

	return sys_time_ms;
}

/* --- get_sys_time_us() give system time in us.
 * ---
 */
uint32_t get_sys_time_us()
{
	uint32_t sys_time_us;
	XTime sys_time;
	XTime tmp_time;

	XTime_GetTime(&sys_time);
	tmp_time = sys_time;
	sys_time_us = (uint32_t)(tmp_time/100);

	return sys_time_us;
}

/* ******************************************************************** */
/*      delay												            */
/* ******************************************************************** */

/* --- delay_us() offer a delay in us.
 * ---
 */
void delay_us(uint32_t num_us)
{
	usleep(num_us);
}

/* --- delay_ms() offer a delay in ms.
 * ---
 */
void delay_ms(uint32_t num_ms)
{
	usleep(1000*num_ms);
}

/* --- delay_s() offer a delay in s.
 * ---
 */
void delay_s(uint32_t num_s)
{
	sleep(num_s);
}
