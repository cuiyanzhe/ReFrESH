/* --- SERIAL.H
   ---
   --- Header of serial.c
   ---
   --- 01/29/05 JWB	Initial coding. V1.0
   ---
   --- */

#ifndef __SERIAL_H
#define __SERIAL_H

#include "xparameters.h"
#include "xuartlite_l.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"


//#include "sched.h"		/* for I_ERROR only */

/* --- Initialize UART1. baud can be either 9600, 19200, 38400, 57600 or 115200. */
char serInit1(unsigned int baud);

/* --- Transmit the first n chars of string str on UART1 */
unsigned char serXmit1(unsigned char *str, unsigned char n);

/* --- Blocking receive on UART1. This routine will block until it receives n chars. */
unsigned char serRecvStr1(unsigned char *str, unsigned char n);

/* --- getchar() from UART1 */
char serGetChar1(unsigned char *str);

/* --- Check if a char is available on UART1. Non-blocking. Immediately returns 1 or 0 if a char
   --- is available or not. Does not return the char. */
char serRecvChk1(void);

unsigned char serGetDLline1(unsigned char *str);

void serDLinit1();

unsigned char serGetDLline1(unsigned char *str);

void print_float(float fval);
void print_decimal(int32_t data, int32_t magnification);
int32_t wait_for_enter();

#endif
