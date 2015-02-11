/* --- SERIAL.C
   ---
   --- This is the polling version of the serial drivers for the DU100
   --- series of powerpc405 processor boards.
   ---
   --- 01/29/05 JWB	v 2.4 separates termbot.c into serveral of C codes.
   ---			Dedugging the slow speed at the Bumpy gait.
   --- 10/15/08 RMV v 3.0 - Cleaned up the header files.
   ---			Changed the types returned to chars in most cases.
   --- 01/15/10	Changed baud rate via the Conifg VHDL IP dialog in XPS.
   --- 12/25/14 GYJ add wait_for_enter().
   ---
   --- */


#include "serial.h"
#include "xparameters.h"
#include "xuartlite_l.h"
#include "pbort.h"
#include "spi.h"

uint8_t		bootLoadCmd1G;  // BootLoadCmd (BLC)
#define		BLC_NORMAL		0
#define		BLC_DL_WAITING	0x80  //Download Line (DL)
#define		BLC_DL_RECVD	0x81

#define		TODO			0



/* --- XPAR_UARTLITE_0_BASEADDR comes from xparameters.h file which could
   --- change names.  Check xparameters.h file for correct name
   --- */

#define UARTLITE_BASEADDRESS XPAR_UARTLITE_0_BASEADDR

/* ******************************************************************** */
/*      Serial Routines                                                 */
/* ******************************************************************** */

/* --- This function is here only for compatibility purposes as Xilinx's
   --- UART Lite has its Baud setting via the Conifg IP dialog in XPS
   --- which in turn hard codes this setting into VHDL. */

char serInit1(unsigned int baud)
{
	bootLoadCmd1G = BLC_NORMAL;
	return 1;
}

/* --- This routine transmits a string of chars to the UART Lite */
/* --- No more than 255 characters can be transmitted */

unsigned char serXmit1(unsigned char *str, unsigned char n)
{
  unsigned int i;
 
  i = 0;
  while(n>i){
	  //if(XUartLite_IsReceiveEmpty(UARTLITE_BASEADDRESS))
      XUartLite_SendByte(UARTLITE_BASEADDRESS, str[i++]);
  }
  return n;
}

/* --- serRecvStr1()  Receive n chars from the UART Lite.
   --- If n == 0, get only available chars (don't block) 
   --- n cannot be larger than 255 */
unsigned char serRecvStr1(unsigned char *str, unsigned char n)
{
  unsigned char i;

  i = 0;
  if (n > 0){
    /* If n > 0, block for n chars */
    while(i < n){
        (str[i++]) = XUartLite_RecvByte(UARTLITE_BASEADDRESS);
    }
    return n;
  } else if (XUartLite_IsReceiveEmpty(UARTLITE_BASEADDRESS)){
  	return 0;
  } else {
    /* if n == 0, get as many as available now (non-blocking) */
      (str[i++]) = XUartLite_RecvByte(UARTLITE_BASEADDRESS);
    return i;
  }
}

/* --- serGetChar1()  Get 1 char from the UART Lite, but do not block.
   --- Return number of chars received. */
char serGetChar1(unsigned char *str)
{
	if (bootLoadCmd1G == BLC_NORMAL){
		/* Check if data is available */

		if (!XUartLite_IsReceiveEmpty(UARTLITE_BASEADDRESS)){
			*str = XUartLite_ReadReg(UARTLITE_BASEADDRESS, XUL_RX_FIFO_OFFSET);
			return 1;
		}
		return 0;
	}
	else if (bootLoadCmd1G == BLC_DL_RECVD){
		return 0;
	}
	else if (bootLoadCmd1G == BLC_DL_WAITING){
		if (!XUartLite_IsReceiveEmpty(UARTLITE_BASEADDRESS)){
			*str = XUartLite_ReadReg(UARTLITE_BASEADDRESS, XUL_RX_FIFO_OFFSET);
			if (*str == ':'){
				bootLoadCmd1G = BLC_DL_RECVD;
				return 0;
			}
			return 1;
		}
		return 0;
	}
	else{
		*str = bootLoadCmd1G;
		bootLoadCmd1G = BLC_DL_WAITING;
		return 1;
	}
}


/* --- serRecvChk1()  Checks for incoming data on USART 1.
   --- Also checks for overrun. Returns true, fail, or error. */
char serRecvChk1(void)
{
	if ((bootLoadCmd1G != BLC_NORMAL) && (bootLoadCmd1G != BLC_DL_WAITING) && (bootLoadCmd1G != BLC_DL_RECVD))
		return 1;

	if (!XUartLite_IsReceiveEmpty(UARTLITE_BASEADDRESS))
		return 1;

	return 0;
}

/* --- serGetDLline1()  Receives a full Download Line (DL) of chars from USART 1.
   --- It strips off the ':' */
unsigned char serGetDLline1(unsigned char *str)
{
	unsigned char i, slen[5], *dummy;
	uint8_t len;

	if (bootLoadCmd1G == BLC_NORMAL){
		return 0;
	}
	else if (bootLoadCmd1G == BLC_DL_WAITING){
		/* Check if data is available */
		if (!XUartLite_IsReceiveEmpty(UARTLITE_BASEADDRESS)){
			*slen = XUartLite_ReadReg(UARTLITE_BASEADDRESS, XUL_RX_FIFO_OFFSET);
			if (*slen != ':'){
				bootLoadCmd1G = *slen;
				return 0;
			}
			bootLoadCmd1G = BLC_DL_RECVD;
		}
		else{
			return 0;
		}
	}

	// If we made it here, a char has been received
	if (bootLoadCmd1G == BLC_DL_RECVD){
		// ':' has been received, get the number of data bytes
		i = 0;
		str[2] = 0;
		// get first two chars to determine length of line
		while(i < 2){
			if(!XUartLite_IsReceiveEmpty(UARTLITE_BASEADDRESS)){
				str[i++] = XUartLite_ReadReg(UARTLITE_BASEADDRESS, XUL_RX_FIFO_OFFSET);
			}
		}

		len = (uint8_t)strtol(str,&dummy,16);

		// Now get the 2-byte address, the 1-byte type, the data and the 1-byte Checksum (2 chars per byte)
		while(i < (10 + (len <<1))){
			if(!XUartLite_IsReceiveEmpty(UARTLITE_BASEADDRESS)){
				str[i++] = XUartLite_ReadReg(UARTLITE_BASEADDRESS, XUL_RX_FIFO_OFFSET);
			}
		}
		str[i] = 0;			// place the end-of-string char
		bootLoadCmd1G = BLC_DL_WAITING;
		return i;
	}
	else{
    	return 0;		// A char other than ':' was received - not a download string
    }
}

void serDLinit1()
{
	if (bootLoadCmd1G == BLC_NORMAL)
		bootLoadCmd1G = BLC_DL_WAITING;
}


#if 0 /* serGetChar1 & serRecvChk1 baks */
/* --- serGetChar1()  Get 1 char from the UART Lite, but do not block.
   --- Return number of chars received. */
char serGetChar1(unsigned char *str)
{

  /* Check if data is available */
    *str = XUartLite_RecvByte(UARTLITE_BASEADDRESS);
    return 1;

  return 0;
}

/* --- serRecvChk1()  Checks for incoming data on USART 1.
   --- Also checks for overrun. Returns true, fail, or error. */
char serRecvChk1(void)
{
  return (char)!XUartLite_IsReceiveEmpty(UARTLITE_BASEADDRESS);

}
#endif

/* ---	print_float() prints a float number
 * ---	bug: 1.03 would be printed as 1. 3
 */
void print_float(float fval)
{
	int whole, thousandths;

	whole = fval;
	thousandths = (fval - whole) * 1000;
    xil_printf("%d.%3d ", whole, thousandths);
}

/* --- print_decimal() prints a magnified integer in its original decimal format
 * ---
 */
void print_decimal(int32_t data, int32_t magnification)
{
	int32_t whole, fraction, digit;

	if(data < 0){
		xil_printf("-");
		data = -data;
	}

	whole = data/magnification;
	xil_printf("%d.", whole);

	fraction = data - whole*magnification;
	magnification /= 10;
	while(magnification)
	{
		digit = fraction/magnification;
		fraction -= digit*magnification;
		xil_printf("%d", digit);
		magnification /= 10;
	}

    xil_printf("	");
}


/* --- wait_for_enter()check and wait for one keyboard input.
 * ---
 */
int32_t wait_for_enter()
{
	unsigned char str[5];

	while(!serRecvChk1()) {}
	serGetChar1(str);

	return 1;
}

/* --- TODO:
 * slen = rfRxInfo.pPayload has the problem. Debug it.
 */
#if TODO
unsigned char serGetOSCmd1(unsigned char *str)
{
	unsigned char j, *dummy;
	unsigned char node_dest;
	int i;
	uint8_t *slen,len;
	j=0;
	bootLoadCmd1G = BLC_DL_WAITING;
	//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,0x1A);

	if (bootLoadCmd1G == BLC_NORMAL){
		return 0;
	}
	else if (bootLoadCmd1G == BLC_DL_WAITING){
		//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,0x1B);
		/* Check if data is available */
		//if (!XUartLite_IsReceiveEmpty(UARTLITE_BASEADDRESS)){
			//*slen = XUartLite_ReadReg(UARTLITE_BASEADDRESS, XUL_RX_FIFO_OFFSET);
			j = Zigbee_Receive();
			if (j==1)
			{
				slen = rfRxInfo.pPayload;
			if ((slen[0] != '!')&&(slen[1] != 'O')&&(slen[2] != 'S')){
				XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,0xFF);
				bootLoadCmd1G = *slen;
				return 0;
			}
			bootLoadCmd1G = BLC_DL_RECVD;

		}
		else{
			return 0;
		}
	}

	// If we made it here, a char has been received
	if (bootLoadCmd1G == BLC_DL_RECVD){
		// '!' has been received

		i = 0;
		str[2] = 0;
		//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,0x1D);
		// get first two chars to determine destination node
		while(i<2)
		 {
			str[i] = slen[i+3];
			i++;
			//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,0x1D);
		}

		//check if this message is meant for me
	   // XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,0x1E);
		node_dest = (unsigned char)strtol(str,&dummy,16);

		//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,node_dest);
		if(node_dest != myNodeG)
			return 0;

		i = 0;
		str[2] = 0;

		//get the next two chars to determine data length
		while(i < 2){
			str[i] = slen[i+15];
			i++;
			}

		//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,0x1F);
		len = (uint8_t)strtol(str,&dummy,16);
		//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,len);
		i =0;
		//XUartLite_SendByte(XPAR_RS232_UART_1_BASEADDR,len);
		// Now get the 1-byte record type, 1-byte comp_ID, and data bytes
		while(i < (12 + (len <<1))){

				str[i] = slen[i+5] ;
				i++;

		}
		str[i] = 0;			// place the end-of-string char
		bootLoadCmd1G = BLC_DL_WAITING;
		return 1;
	}
	else{
		return 0;
	}
}
#endif



