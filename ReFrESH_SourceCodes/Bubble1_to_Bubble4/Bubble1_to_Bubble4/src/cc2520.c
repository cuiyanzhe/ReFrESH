/* --- CC2520.C       Version 1.1
   --- This driver code implements basic Zigbee functionality in conjunction with the CC2520 Zigbee communication
   --- processor chip from TI.
   ---
   ---
   --- Copyright 2014, Collaborative Robotics Lab
   ---
   --- 12/13/14 RMV 	Initial coding using lab coding standards
   --- 12/14/14 RMV		Small messages are working, but bug in large messages (108 bytes is limit)
   --- 12/20/14 GYJ 	Sending and receiving large messages is fine. Excluding Zigbee overhead (11 bytes), 2 RSSI bytes and OS head (7 bytes), 108 bytes are maximum payload for one packet.
   --- 12/21/14 RMV		Changed API for sbsZigbeeRegister(), added sbsZigbeeGetRecvMsgSize, v1.1.
   ---
   --- */

/* ********************************************************************************************************************************************* */
/*      Include Files						                                              														 */
/* ********************************************************************************************************************************************* */
#include <string.h>
#include <stdlib.h>
#include "xparameters.h"
#include "xgpio.h"
#include "spi.h"
#include "time.h"
#include "refresh_main.h"
#include "BasicCC2520.h"
#include "cc2520.h"



/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */

#define	DEBUG	0

#define cc2520_reset_DEVICE_ID	((uint16_t) XPAR_CC2520_RESET_BASEADDR)
#define cc2520_reset_CHANNEL 1

#define PBORT_HEADER_SIZE 7
#define PBORT_MAX_CMD_SIZE	(RF_MAX_PAYLOAD_SIZE - PBORT_HEADER_SIZE)



/* ********************************************************************************************************************************************* */
/*      Global variables  for local function                                              														 */
/* ********************************************************************************************************************************************* */
char othrBuf[256];	// add for zigbee
char rx_bufG[RF_MAX_PAYLOAD_SIZE];

XGpio rstG;	/* GPIO instance for cc2520 reset pin control */

MessageRec_t	MsgRegArrayG[MSG_ARRAY_MAX];
uint8_t			MsgRegHeadG, MsgRegTailG;

/* ********************************************************************************************************************************************* */
/*      Function prototype                                              																		 */
/* ********************************************************************************************************************************************* */


/* ********************************************************************************************************************************************* */
/*      Local Functions                         	                     																		 */
/* ********************************************************************************************************************************************* */

/* ******************************************************************** */
/*      this function puts cc2520 reset pin at low						*/
/* ******************************************************************** */
void cc2520_reset_low()
{
	uint32_t Data = 0x0;
	uint32_t Direction = 0x0;
	XGpio_Initialize(&rstG, cc2520_reset_DEVICE_ID);
	XGpio_SetDataDirection(&rstG, cc2520_reset_CHANNEL, Direction);
	XGpio_DiscreteWrite(&rstG, cc2520_reset_CHANNEL, Data);
}

/* ******************************************************************** */
/*      this function puts cc2520 reset pin at high						*/
/* ******************************************************************** */
void cc2520_reset_high()
{
	uint32_t Data = 0xFFFFFFFF;
	uint32_t Direction = 0x0;
	XGpio_Initialize(&rstG, cc2520_reset_DEVICE_ID);
	XGpio_SetDataDirection(&rstG, cc2520_reset_CHANNEL, Direction);
	XGpio_DiscreteWrite(&rstG, cc2520_reset_CHANNEL, Data);
}

/* ******************************************************************** */
/*      this function runs cc2520 power up procedure					*/
/* ******************************************************************** */
void cc2520_reset()
{
	delay_ms(200);

	cc2520_reset_low();

	delay_ms(200);

	cc2520_reset_high();
}

/* --- This function initializes the CC2520 chip, setting the local PAN ID and local address.
 * --- The broadcast PAN ID is 0xFFFF, else the sender must match the receiver's PAN ID.
 * --- Likewise, the broadcast addr is 0xFFFF, else the sender must match the receiver's addr
 */
void initialize_zigbee(uint8_t channel, uint16_t panId, uint16_t myAddr)
{
	 uint8_t i, status, chipID;

	 xil_printf("Zigbee Initializing... \r\n");

	 cc2520_reset();	/*	 It seems like GPIO reset is not necessary	*/


	 for(i=0;i<5;i++)	/*	 read and check cc2520 chipID five times to make sure it has powered on successfully	*/
	 {
		do	//read cc2520 register to get ChipID
	 	{
	 		SPI_ENABLE();

	 		status = FASTSPI_TX(0x10);	//memory reading, different from register reading
	 		status = FASTSPI_TX(0x40);
	 		chipID = FASTSPI_RX();

	 		SPI_DISABLE();

//	 		xil_printf("Status = 0x%x, ChipID = 0x%x\r\n", status, chipID);

	 		delay_ms(100);

	 	}while(!((status == 0x80) && (chipID == 0x84)));
	 }

	if(chipID == 0x84){
		xil_printf("cc2520 powered on successfully! ID = 0x%x \r\n", chipID);
	}
	else{
		//initFlagG = 0;
		xil_printf("cc2520 powered on failed! ID = 0x%x \r\n", chipID);
	}

	 rfRxInfoG.pPayload = rx_bufG;
	 rfRxInfoG.max_length = RF_MAX_PAYLOAD_SIZE;
	 rfRxInfoG.ackRequest= 1;				//read from FCF in the package received, doesn't matter here
	 rfRxInfoG.srcAddr = 0x1111;

	 cc2520_init(&rfRxInfoG, channel, panId, myAddr);
}


/* ******************************************************************** */
/*      this function receives one character command from other nodes*/
/* ******************************************************************** */
char Zigbee_CMD_Receive()
{
	uint8_t i, n;
	char cmd;

	if(!rxCycleG)
	{
		rf_polling_rx_on();
	}

	if(rf_rx_check_sfd())
	{
		n = rf_polling_rx_packet();

		if (n == 1)
		{
			cmd = rfRxInfoG.pPayload[0];

			if(cmd >= 'a' && cmd <= 'z')
				return cmd;

#if DEBUG
				xil_printf("Packet: " );

				for(i=0; i<rfRxInfoG.length; i++ )
					xil_printf("%d", rfRxInfoG.pPayload[i]);

				xil_printf("Packet: [ ");

				xil_printf("LEN %3d  NO. %3d  SRCADD: 0x%x  RSSI: %d  Payload: ", rfRxInfoG.length, rfRxInfoG.seqNumber, rfRxInfoG.srcAddr, rfRxInfoG.rssi);

				for(i=0; i<rfRxInfoG.length; i++ )
					xil_printf("%c", rfRxInfoG.pPayload[i]);

				xil_printf(" ] has been received!\r\n" );
#endif
		}
	}
	return 0;
}

/* ******************************************************************** */
/*   this function reads and prints message received from other nodes	*/
/* ******************************************************************** */
char Zigbee_Receive()
{
	uint8_t i, n;

	if(!rxCycleG)	/* check if radio is reading a message. if yes, skip and read message from RX buffer. if not, turn on the radio  */
	{
		rf_polling_rx_on();	/* turn on the radio */
	}

	if(rf_rx_check_sfd())	/* check if the message is ready in RX buffer */
	{
		n = rf_polling_rx_packet();	/* check and see if a packet has been read successfully	*/

#if !DEBUG	/* print out message in RX buffer for debugging*/
		if (n == 1)
		{
//				xil_printf("Message received: " );
//
//				for(i=0; i<rfRxInfoG.length; i++ )
//					xil_printf("%d", rfRxInfoG.pPayload[i]);
//
//				xil_printf("  " );

				xil_printf("Packet: [ ");

				xil_printf("SEQNUM: %d  SRCADDR: 0x%x  RSSI: %d  Payload:",	rfRxInfoG.seqNumber, rfRxInfoG.srcAddr, rfRxInfoG.rssi);

				for(i=0; i<rfRxInfoG.length; i++ )
					xil_printf("%c", rfRxInfoG.pPayload[i]);
				xil_printf(" ] has been received!\r\n" );
		}
#endif
	}

	return n;
}



/* --- sbsZigbeeGetRecvMsgSize() returns the length of the last message received from a particular source.
   ---    This is a one-time-use function! Once the length is read, it is reset to zero until a new message comes in.
 * --- myRec is a pointer to a buffer that is allocated in the requester's space.
 * --- src is the Zigbee address of the responding node
 * --- key is a unique 16-bit identifier for the request.
 * --- This works with sbsZigbeeRegister() to parse special OS commands from the Zigbee packets.
   ---
 * --- Returns the size of the last full message returned, zero if no new compelete msg recevied, or -1 if the pointer is stale
 */

int16_t sbsZigbeeGetRecvMsgSize(MessageRec_t *myRec, uint16_t src, uint16_t key)
{
	uint16_t	len;

	if ((myRec->src == src) && (myRec->key == key)){
		len = myRec->length;		/* make local copy of length before resetting it to zero */
		if (len != 0){

		    /* Should disable interrupts to update this value */
		    myRec->length = 0;		/* reset length to zero (forgetting that there is a message in the buf) */
		//    myRec->next = 0;		/* reset next index to zero (allowing reuse of buf[]) */
		    /* End of DISABLE() */

		} /* endif */
		return len;
	}
	return -1;	/* ERROR - src and/or key do not match - STALE POINTER! */
}


/* --- sbsZigbeeRegister() allows an entity to register for a response from another node.
 * --- buf is a pointer to a buffer that is allocated in the requester's space.
 * --- src is the Zigbee address of the responding node
 * --- key is a unique 16-bit identifier for the request.
 * --- This works with sbsZigbeeOSparse() to parse special OS commands from the Zigbee packets.
 * ---
   --- What about double-buffering? Current implementation will cause problems with re-use of buf
   ---
   --- Returns a pointer to the message record stored
*/

MessageRec_t * sbsZigbeeRegister(char *buf, uint16_t src, uint16_t key)
{
	MessageRec_t * currRec = &MsgRegArrayG[MsgRegTailG];

	MsgRegArrayG[MsgRegTailG].buf = buf;
	MsgRegArrayG[MsgRegTailG].src = src;
	MsgRegArrayG[MsgRegTailG].key = key;
	MsgRegArrayG[MsgRegTailG].next = 0;		/* index into buf[] pointing to the start of the empty buffer */
	MsgRegArrayG[MsgRegTailG].length = 0;		/* length of complete msg stored in buf[] */

	if (MsgRegTailG++ > MSG_ARRAY_MAX)		/* Update MsgRegTailG */
		MsgRegTailG = 0;

	return currRec;
}



/* --- sbsZigbeeOSparse() is a function that parses Zigbee packets and allows large packets to be sent asynchronously.
 * --- Should be called whenever a packet arrives. (interrupt based, not polling, but polling is all we have now)
 * --- The search over the Registration database is not yet implemented.
 */
uint16_t sbsZigbeeOSparse()
{
	static uint8_t	state = 0;
	uint8_t 	i, n;
	uint16_t	*totLen;
	uint8_t		*buf;
	uint8_t 	*bufPtr;
	uint8_t		fndItem;
	MessageRec_t *thisRec;

	if(!rxCycleG)	/* check if radio is reading a message. if yes, skip and read message from RX buffer. if not, turn on the radio  */
	{
		rf_polling_rx_on();	/* turn on the radio */
	}

	if(rf_rx_check_sfd())	/* check if the message is ready in RX buffer */
	{
		n = rf_polling_rx_packet();	/* check and see if a packet has been read successfully	*/

		if (n != 1)
			return 0;		/* If no complete packet is in, don't block */
		else {
			/* A complete packet has arrived, check for an OS command */
			if ((rfRxInfoG.pPayload[0] == '%') && (rfRxInfoG.pPayload[1] == 'O') && (rfRxInfoG.pPayload[2] == 'S')){
				if ((rfRxInfoG.pPayload[3] == 'P') && (rfRxInfoG.pPayload[4] == 'C')){
					/* It's a COMPLETE PBO/RT Command %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

					thisRec->length = 0;	/* Reset length to zero while copying data */
#if DEBUG
					xil_printf("OS: " );

					totLen = (uint16_t *)&(rfRxInfoG.pPayload[5]);
					xil_printf("len: %d  ",	*totLen);

					xil_printf("SEQNUM: %d  SRCADDR: 0x%x  RSSI: %d  Payload:",	rfRxInfoG.seqNumber, rfRxInfoG.srcAddr, rfRxInfoG.rssi);

					for(i=0; i<rfRxInfoG.length; i++ )
						xil_printf("%c", rfRxInfoG.pPayload[i]);
					xil_printf(" ] COMPLETE!\r\n" );
#endif
					/* Copy it into the correct buffer (I'll assume there is only one, for now, else just search the list) */
					/* Match the src and key during search */
					fndItem = 0;				/* Replace with search of list */
					thisRec = &MsgRegArrayG[fndItem];	/* This is the found record that matches src and key */

//					xil_printf("\r\n\Pointer1 %d \r\n",	thisRec->next);
					memcpy(thisRec->buf, &(rfRxInfoG.pPayload[7]), *totLen);
					state = 0;			/* not using this yet */
					thisRec->next = 0;		/* Keep next at zero so it is ready for next message (double buffering would come in handy) */
					thisRec->length = *totLen;	/* Set the length value to payload length since the message is complete */

					/* Release a remote semaphore, if process is blocked on it */

					return thisRec->key;
				} else if ((rfRxInfoG.pPayload[3] == 'P') && (rfRxInfoG.pPayload[4] == 'P')){
					/* PARTIAL - Part of a larger message */

					thisRec->length = 0;	/* Reset length to zero while copying data */

					totLen = (uint16_t *)&(rfRxInfoG.pPayload[5]);		/* Retrieve the total length of the message */
//					bufPtr = buf = malloc(*totLen);
//					memcpy(bufPtr, &(rfRxInfoG.pPayload[7]), PBORT_MAX_CMD_SIZE);

					/* Need to search the list to match src and key, but assume it's the only one for now */
					fndItem = 0;
					thisRec = &MsgRegArrayG[fndItem];

//					xil_printf("\r\n\Pointer2 %d ",	MsgRegArrayG[fndItem].next);
					memcpy(&(thisRec->buf[thisRec->next]), &(rfRxInfoG.pPayload[7]), PBORT_MAX_CMD_SIZE);
					thisRec->next += PBORT_MAX_CMD_SIZE;		/* Next updated to next empty spot in buf[] */

#if DEBUG
					xil_printf("OS: " );
					xil_printf("len: %d  ",	*totLen);
					xil_printf("SEQNUM: %d  SRCADDR: 0x%x  RSSI: %d  Payload:",	rfRxInfoG.seqNumber, rfRxInfoG.srcAddr, rfRxInfoG.rssi);

					for(i=0; i<rfRxInfoG.length; i++ )
						xil_printf("%c", rfRxInfoG.pPayload[i]);
					xil_printf(" ] PARTIAL!\r\n" );
#endif
					return 0;

				} else if ((rfRxInfoG.pPayload[3] == 'P') && (rfRxInfoG.pPayload[4] == 'D')){
					/* End of a larger message - DONE */

					/* Need to search the list to match src and key, but assume it's the only one for now */
					fndItem = 0;
					thisRec = &MsgRegArrayG[fndItem];

//					xil_printf("\r\n\Pointer3 %d \r\n",	MsgRegArrayG[fndItem].next);
					memcpy(&(thisRec->buf[thisRec->next]), &(rfRxInfoG.pPayload[7]), rfRxInfoG.length - 7);

					/* Set the value to total payload length now that the message is fully copied */
					/* Should compare this to expected message length */
					thisRec->length = thisRec->next + rfRxInfoG.length - 7;

					thisRec->next = 0;		/* Reset the next index to point to the beginning for the next use */

					/* Implement the release of a remote semaphore, if a requesting process is blocked on it */

#if DEBUG
					xil_printf("OS: " );

					xil_printf("SEQNUM: %d  SRCADDR: 0x%x  RSSI: %d  Payload:",	rfRxInfoG.seqNumber, rfRxInfoG.srcAddr, rfRxInfoG.rssi);

					for(i=0; i<rfRxInfoG.length; i++ )
						xil_printf("%c", rfRxInfoG.pPayload[i]);
					xil_printf(" DONE!\r\n" );
#endif
					return thisRec->key;
				}
			} else {
				/* Copy the message to some buffer, because it is not a PBO/RT command message */
			}
				xil_printf("Not OS CMD: " );

				for(i=0; i<rfRxInfoG.length; i++ )
					xil_printf("%d", rfRxInfoG.pPayload[i]);

				xil_printf(" Packet: [ ");

				xil_printf("SEQNUM: %d  SRCADDR: 0x%x  RSSI: %d  Payload:",	rfRxInfoG.seqNumber, rfRxInfoG.srcAddr, rfRxInfoG.rssi);

				for(i=0; i<rfRxInfoG.length; i++ )
					xil_printf("%c", rfRxInfoG.pPayload[i]);
				xil_printf(" ] has been received!\r\n" );
		}
	}
	return n;
}



/* ******************************************************************** */
/*      this function sends a message to other nodes					*/
/* ******************************************************************** */
void Zigbee_Transmit(uint16_t destPan, uint16_t destAddr, char * Tx_Message, uint16_t len)
{
	uint8_t TXFIFO[128];
	uint8_t i;

	rfTxInfoG.destAddr = destAddr;
	rfTxInfoG.destPanId = destPan;

	rfTxInfoG.length = len;
	rfTxInfoG.pPayload = Tx_Message;

	rf_tx_packet(&rfTxInfoG);

#if DEBUG
	FASTSPI_READ_RAM(TXFIFO,0x01,0x00,rfTxInfoG.length + RF_PACKET_OVERHEAD_SIZE);

	xil_printf("Packet: ");
//	xil_printf("[ Len %3d ", TXFIFO[0]- RF_PACKET_OVERHEAD_SIZE);
	xil_printf("[ Len %3d ", TXFIFO[0]);
	xil_printf("NO. %3d ", TXFIFO[3]);
	xil_printf("PanID 0x%x%x ", TXFIFO[4],TXFIFO[5]);
	xil_printf("DesAdd 0x%x%x ", TXFIFO[6],TXFIFO[7]);
	xil_printf("MyAdd 0x%x%x ", TXFIFO[8],TXFIFO[9]);
	xil_printf("Payload ");
	for(i = 10;i < rfTxInfoG.length+RF_PACKET_OVERHEAD_SIZE-1;i++)
		xil_printf("%c", TXFIFO[i]);
	xil_printf("  CSM %3d ", TXFIFO[rfTxInfoG.length+RF_PACKET_OVERHEAD_SIZE-1]);
	xil_printf(" ] has been sent!\r\n");
#endif
}



/* --- sbsZigbeeSend()
 * --- This function sends an arbitrary number of bytes via the Zigbee TRC1150 Morphing Bus wedge.
 * --- This wedge uses the CC2520 chip from TI.
 * --- It breaks the data up into chunks less than 127 bytes. An extra PBO/RT header is prepended.
 */
void sbsZigbeeSend(uint16_t destPan, uint16_t destAddr, uint8_t * Tx_Message, uint16_t len)
{
	uint8_t bufP[128] = {"%OSPPXX********"};
	uint8_t bufD[128] = {"%OSPDXX********"};
	uint8_t bufC[128] = {"%OSPCXX$$$$$$$$"};
	uint8_t i, ii;
	uint16_t	*totLen;

	uint8_t TXFIFO[128];

	rfTxInfoG.destAddr = destAddr;
	rfTxInfoG.destPanId = destPan;

	/* Check to see if the data record is too big for one packet. */
	if (len > PBORT_MAX_CMD_SIZE){
		/* If it is, send the PARTIAL command... */

		totLen = (uint16_t *)&bufP[5];
		*totLen = len;
		totLen = (uint16_t *)&bufD[5];
		*totLen = len;

//		xil_printf("OSPP: 0x%x ", rfTxInfoG.destPanId);
//		xil_printf("len: %d ", *totLen);

		rfTxInfoG.length = RF_MAX_PAYLOAD_SIZE;		/* ...and use the maximum size */
		/* Loop through the PARTIAL packets until the data is within one packet to go */
		while (len > PBORT_MAX_CMD_SIZE){
			memcpy(&bufP[7], Tx_Message, PBORT_MAX_CMD_SIZE);	/* Copy the front of the buf into the buffer with the command header */
			rfTxInfoG.pPayload = bufP;
//			rfTxInfoG.destAddr = destAddr;	/* add to avoid a bug */
//			rfTxInfoG.destPanId = destPan;	/* add to avoid a bug */
			rf_tx_packet(&rfTxInfoG);

			delay_ms(100);		/*  a big delay is added to give transmitting node and receiving node enough time to print received packets for debug.	*/

//			for (ii=0; ii<5; ii++){
//				xil_printf("%c", bufP[ii]);
//			}
//			xil_printf("%d", *((uint16_t *)&bufP[5]));
//			for (ii=7; ii<14; ii++){
//				xil_printf("%c", bufP[ii]);
//			}

//			FASTSPI_READ_RAM(TXFIFO,0x01,0x00,rfTxInfoG.length + RF_PACKET_OVERHEAD_SIZE);

//			xil_printf("PanID: ");
//			for(i=4;i<6;i++)
//				xil_printf("0x%x ", TXFIFO[i]);
//			xil_printf("DesAdd: ");
//			for(i=6;i<8;i++)
//				xil_printf("0x%x ", TXFIFO[i]);
//			xil_printf("PanID2: ");
//				xil_printf("0x%x ", rfTxInfoG.destPanId);
//			xil_printf("DesAdd2: ");
//				xil_printf("0x%x ", rfTxInfoG.destAddr);

//			xil_printf("PAYLOAD: ");
//			for(i=10;i<rfTxInfoG.length + RF_PACKET_OVERHEAD_SIZE + CHECKSUM_OVERHEAD - 1 -1;i++)
//				xil_printf("%c", TXFIFO[i]);

#if DEBUG
			FASTSPI_READ_RAM(TXFIFO,0x01,0x00,rfTxInfoG.length + RF_PACKET_OVERHEAD_SIZE);

			xil_printf("Packet: ");
			xil_printf("LEN %3d ", (int16_t) TXFIFO[15]<<8 | TXFIFO[16]);
			xil_printf("[ Len %3d ", TXFIFO[0]);
			xil_printf("NO. %3d ", TXFIFO[3]);
			xil_printf("PanID 0x%x%x ", TXFIFO[4],TXFIFO[5]);
			xil_printf("DesAdd 0x%x%x ", TXFIFO[6],TXFIFO[7]);
			xil_printf("SrcAdd 0x%x%x ", TXFIFO[8],TXFIFO[9]);
			xil_printf("PBORTHeader");
			for(i = 10;i < 15;i++)
				xil_printf("%c", TXFIFO[i]);
			xil_printf(" Payload ");
			for(i = 17;i < rfTxInfoG.length+RF_PACKET_OVERHEAD_SIZE-1;i++)
				xil_printf("%c", TXFIFO[i]);
			xil_printf(" ] has been sent!\r\n");
#endif

			Tx_Message += PBORT_MAX_CMD_SIZE;					/* increment the ptr past the data just copied */
			len -= PBORT_MAX_CMD_SIZE;							/* decrement the len to indicate bytes remaining */
		}

		/* Check to see if there is any data left over */
		if (len > 0){
			/* If so, pack it and send it with a completion command */
			rfTxInfoG.length = len + PBORT_HEADER_SIZE;
			memcpy(&bufD[7], Tx_Message, len);
//			rfTxInfoG.destAddr = destAddr;	/* add to avoid a bug */
//			rfTxInfoG.destPanId = destPan;	/* add to avoid a bug */
			rfTxInfoG.pPayload = bufD;

			rf_tx_packet(&rfTxInfoG);

#if DEBUG
			FASTSPI_READ_RAM(TXFIFO,0x01,0x00,rfTxInfoG.length + RF_PACKET_OVERHEAD_SIZE);

			xil_printf("Packet: ");
			xil_printf("LEN %3d ", (int16_t) TXFIFO[15]<<8 | TXFIFO[16]);
			xil_printf("[ Len %3d ", TXFIFO[0]);
			xil_printf("NO. %3d ", TXFIFO[3]);
			xil_printf("PanID 0x%x%x ", TXFIFO[4],TXFIFO[5]);
			xil_printf("DesAdd 0x%x%x ", TXFIFO[6],TXFIFO[7]);
			xil_printf("SrcAdd 0x%x%x ", TXFIFO[8],TXFIFO[9]);
			xil_printf("PBORTHeader");
			for(i = 10;i < 15;i++)
				xil_printf("%c", TXFIFO[i]);
			xil_printf(" Payload ");
			for(i = 17;i < rfTxInfoG.length+RF_PACKET_OVERHEAD_SIZE-1;i++)
				xil_printf("%c", TXFIFO[i]);
			xil_printf(" ] has been sent!\r\n");
#endif
		}
	} else {
		totLen = (uint16_t *)&bufC[5];
		*totLen = len;
		/* The data fits into one packet */
		rfTxInfoG.length = len + PBORT_HEADER_SIZE;
		memcpy(&bufC[7], Tx_Message, len);
		rfTxInfoG.pPayload = bufC;

		rf_tx_packet(&rfTxInfoG);

#if DEBUG
		FASTSPI_READ_RAM(TXFIFO,0x01,0x00,rfTxInfoG.length + RF_PACKET_OVERHEAD_SIZE);

		xil_printf("Packet: ");
		xil_printf("LEN %3d ", (int16_t) TXFIFO[15]<<8 | TXFIFO[16]);
		xil_printf("[ Len %3d ", TXFIFO[0]);
		xil_printf("NO. %3d ", TXFIFO[3]);
		xil_printf("PanID 0x%x%x ", TXFIFO[4],TXFIFO[5]);
		xil_printf("DesAdd 0x%x%x ", TXFIFO[6],TXFIFO[7]);
		xil_printf("SrcAdd 0x%x%x ", TXFIFO[8],TXFIFO[9]);
		xil_printf("PBORTHeader");
		for(i = 10;i < 15;i++)
			xil_printf("%c", TXFIFO[i]);
		xil_printf(" Payload ");
		for(i = 17;i < rfTxInfoG.length+RF_PACKET_OVERHEAD_SIZE-1;i++)
			xil_printf("%c", TXFIFO[i]);
		xil_printf(" ] has been sent!\r\n");
#endif
	} /* endif */
}
