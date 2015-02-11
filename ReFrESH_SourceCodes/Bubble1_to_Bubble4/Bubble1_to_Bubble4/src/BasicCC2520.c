/* --- BasicCC2520.c       Version 1.1
   --- this module provides low level driver for cc2520
   ---
   --- Copyright 2001, Mark V Automation Corp.
   ---
   --- 12/07/14 GYJ	Initial coding. V1.0
   --- 12/15/14 RMV	Add more comments. V1.1
   ---
   --- */

/* ********************************************************************************************************************************************* */
/*      Include Files						                                              														 */
/* ********************************************************************************************************************************************* */

#include <stdio.h>
#include "spi.h"
#include "cc2520register.h"
#include "BasicCC2520.h"
#include "time.h"

/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */

#define  ACK_REQUEST	0
#define  SECURITY	0
#define  CCA	0

//---------(CC2520 variables)---------//
#define TRUE    1
#define FALSE   0

#define MIN_CHANNEL     		11   // 11---26
#define CHANNEL_SPACING			 5   //5MHz

/* ********************************************************************************************************************************************* */
/*      Global variables 					                                              														 */
/* ********************************************************************************************************************************************* */

RF_TX_INFO	 	rfTxInfoG;
RF_RX_INFO 		rfRxInfoG;
RF_SETTINGS 	rfSettingsG;

uint8_t rxCycleG;	/* 	a flag which shows whether the radio is reading a message	*/

uint8_t auto_ack_enable;
uint8_t security_enable;
uint8_t last_pkt_encrypted;

/* ********************************************************************************************************************************************* */
/*      Local Functions                         	                     																		 */
/* ********************************************************************************************************************************************* */
//---------CC2520 Initialization---------//
/* --- CC2520_init()
   --- This function sets the channel number, PAN ID and address of the Zigbee radio.
   --- These are important as the mode is set to filter on addr and PAN ID. If the sender does not match these
   --- (or does not broadcast) the receiver will not receive the message.
   --- Do NOT set this PAN ID or ADDR to 0xFFFF as this is the broadcast flag.
 */
void cc2520_init(RF_RX_INFO *pRRI, uint8_t channel, uint16_t panId, uint16_t myAddr)
{

    FASTSPI_STROBE(SXOSCON);

	FASTSPI_SETREG(FRMCTRL0,0x40);	//AUTOCRC ENABLE: 0b0100 0000, AUTOACK ENABLE: 0b0010 0000;	(AUTOCRC will put two bytes for RSSI at the end of each frame, decrease capacity of payload)
	FASTSPI_SETREG(FRMCTRL1,0x0);	//RXENABLE:0x01, RXDISABLE:0x0

	FASTSPI_SETREG(FRMFILT0,0x0D);	//Filter Enable:0x0D, Filter Disable:0x0C
	FASTSPI_SETREG(TXPOWER, 0xF7);	//TXPOWER
    FASTSPI_SETREG(MDMCTRL1, 0x14); // Set the correlation threshold = 20
    FASTSPI_SETREG(FIFOPCTRL, 0x7F);   // Set the FIFOP threshold to maximum

	FASTSPI_STROBE(SFLUSHRX);
	FASTSPI_STROBE(SFLUSHTX);

	// Set the protocol configuration
	rfSettingsG.pRxInfo = pRRI;
	rfSettingsG.myPanId = panId;
	rfSettingsG.myAddr = myAddr;
	rfSettingsG.txSeqNumber = 0;
    rfSettingsG.receiveOn = FALSE;

	FASTSPI_SETREG(SRCMATCH,0x01);		//Filter
	FASTSPI_SETREG(SRCSHORTEN0,0x01);

	//Filter Configuration
	FASTSPI_WRITE_RAM((uint8_t *)&rfSettingsG.myAddr,0x03,0xF4,2);
	FASTSPI_WRITE_RAM((uint8_t *)&rfSettingsG.myPanId,0x03,0xF2,2);
	FASTSPI_WRITE_RAM((uint8_t *)&rfSettingsG.myPanId,0x03,0x80,2);
	FASTSPI_WRITE_RAM((uint8_t *)&rfSettingsG.myAddr,0x03,0x82,2);


    // Set the RF channel
    //halRfSetChannel(channel);

	// Wait for the crystal oscillator to become stable
    //halRfWaitForCrystalOscillator();

	auto_ack_enable = ACK_REQUEST;
	security_enable = SECURITY;
	last_pkt_encrypted = 0;

}




/* ******************************************************************** */
/*      this function transmit a message stored in TX buffer			*/
/* ******************************************************************** */
/*	rf_tx_packet() developed based on TI library 	*/
uint8_t rf_tx_packet(RF_TX_INFO *pRTI)
{
	uint16_t frameControlField;
    uint8_t packetLength, length;
    uint8_t success;
    uint8_t spiStatusByte;
    uint8_t checksum,i;
	uint8_t ACK_flag = 1;


	//if(security_enable) FASTSPI_STROBE(STXENC);

    checksum=0;

    for(i=0; i<pRTI->length; i++ )
    {
		// lets do our own payload checksum because we don't trust the CRC
		checksum+=pRTI->pPayload[i];
    }
    // Write the packet to the TX FIFO (the FCS is appended automatically when AUTOCRC is enabled)

    // These are only the MAC AGNOSTIC parameters...
    // Slots for example are at a slighly higher later since they assume TDMA
    packetLength = pRTI->length + RF_PACKET_OVERHEAD_SIZE + 1;		/*	calculate packet length as payload + overhead + 1. It's still not clear why it has to be 1, not 2 as we have 2 RSSI bytes.	*/

    if(security_enable) packetLength+=4;  // for CTR counter

   	//  2 below are hacks...
	FASTSPI_STROBE(SFLUSHRX);
	FASTSPI_STROBE(SFLUSHRX);

	// Flush the TX FIFO just in case...
    FASTSPI_STROBE(SFLUSHTX);
    FASTSPI_STROBE(SFLUSHTX);

/*
    // Turn on RX if necessary
    if (!rfSettingsG.receiveOn) {
		FASTSPI_STROBE(CC2420_SRXON);
		}

    // Wait for the RSSI value to become valid
    do {
        FASTSPI_UPD_STATUS(spiStatusByte);
    } while (!(spiStatusByte & BM(CC2420_RSSI_VALID)));

	// TX begins after the CCA check has passed
    do {
		FASTSPI_STROBE(CC2420_STXONCCA);
		FASTSPI_UPD_STATUS(spiStatusByte);
		halWait(100);
    } while (!(spiStatusByte & BM(CC2420_TX_ACTIVE)));
*/

    /*		writing TX FIFO		*/
    FASTSPI_WRITE_FIFO((uint8_t*)&packetLength, 1);               // Packet length

    if(ACK_REQUEST)
    	frameControlField = RF_FCF_ACK;
    else
    	frameControlField = RF_FCF_NOACK;
    if(auto_ack_enable) frameControlField |= RF_ACK_BM;
    if(security_enable) frameControlField |= RF_SEC_BM;

    FASTSPI_WRITE_FIFO((uint8_t*) &frameControlField, 2);         // Frame control field
    FASTSPI_WRITE_FIFO((uint8_t*) &rfSettingsG.txSeqNumber, 1);    // Sequence number
    FASTSPI_WRITE_FIFO((uint8_t*) &pRTI->destPanId, 2);          // Dest. PAN ID
    FASTSPI_WRITE_FIFO((uint8_t*) &pRTI->destAddr, 2);            // Dest. address
    FASTSPI_WRITE_FIFO((uint8_t*) &rfSettingsG.myAddr, 2);         // Source address
    //if(security_enable)	FASTSPI_WRITE_FIFO((uint8_t*) &tx_ctr, 4);         // CTR counter
    FASTSPI_WRITE_FIFO((uint8_t*) pRTI->pPayload, pRTI->length);  // Payload
    FASTSPI_WRITE_FIFO((uint8_t*) &checksum, 1);         // Checksum

    /*		send what is in TX FIFO		*/

//    uint32_t time;

	do
	{
		FASTSPI_STROBE(STXON);

//		time = get_sys_time_us();

		// Wait for the transmission to finish
		while(!(FASTSPI_GETREG(EXCFLAG0) & 0x02))	// flag of TX_FRM_DONE
		{
//			xil_printf(".");
//			delay_us(500);	/*	delay some time for transmission. Transmitting a 128 bytes packet needs 4.5 ms.	Transmitting a 21 bytes packet needs 1.1 ms. */
		}

//		time = get_sys_time_us() - time;
//		xil_printf("time %d \r\n", time);

		FASTSPI_SETREG(EXCFLAG0,0x0);
		success = TRUE;

		// Wait for the acknowledge to be received, if any
		if(auto_ack_enable)
		{
			rfSettingsG.ackReceived = FALSE;

			if(FASTSPI_GETREG(EXCFLAG1) & 0x01)
				ACK_flag = 1;
			else
				ACK_flag = 0;

		// We'll enter RX automatically, so just wait until we can be sure that the
		// ack reception should have finished
		// The timeout consists of a 12-symbol turnaround time, the ack packet duration,
		// and a small margin
		// halWait((12 * RF_SYMBOL_DURATION) + (RF_ACK_DURATION) + (2 * RF_SYMBOL_DURATION) + 100);
		}
	}while(!ACK_flag);

	 /*		reset after finish transmission		*/

	FASTSPI_SETREG(EXCFLAG1,0x0);

	// Turn off the receiver if it should not continue to be enabled
	FASTSPI_STROBE(SRFOFF);  // shut down radio

	rxCycleG = 0;	/* 	to open the radio when receiving function is being called	*/

    // Increment the sequence number, and return the result
	rfSettingsG.txSeqNumber++;

    return success;

}

/* ******************************************************************** */
/*      this function turn on radio in RX mode							*/
/* ******************************************************************** */
void rf_polling_rx_on(void)
{
    rfSettingsG.receiveOn = TRUE;

	FASTSPI_STROBE(SRXON);

	FASTSPI_STROBE(SFLUSHRX);

} // rf_rx_on()

/* ******************************************************************** */
/*      this function check and see if the message is ready in RX buffer*/
/* ******************************************************************** */
uint8_t rf_rx_check_sfd()
{
 	 if(auto_ack_enable)	/* if acknowledgment is on, check and wait for it*/
 	 {
 		 while(!(FASTSPI_GETREG(EXCFLAG0) & 0x04));
 		 FASTSPI_SETREG(EXCFLAG0,0x0);
 	 }

 	 if(FASTSPI_GETREG(EXCFLAG1) & 0x01)	/* check and see if radio is idle. If it is, message is ready in RX buffer and turn off radio*/
 	 {
 		rxCycleG = 0;
 		FASTSPI_STROBE(SRFOFF);
 		return 1;
 	 }
 	 else
 	 {
 		rxCycleG = 1;
 		return 0;
 	 }
}

/* --- This function checks to see if a packet has been read.
 * --- It returns a variety of values, 1 being "Packet read"
 */
int8_t rf_polling_rx_packet()
{
	uint8_t tmp;

    if(FASTSPI_GETREG(EXCFLAG1) & 0x10)		//if(FIFOP_IS_1)
    {
    	FASTSPI_SETREG(EXCFLAG1,0x0);		//reset EXCFLAG1!

		uint16_t frameControlField;
		uint8_t length;
		uint8_t pFooter[2];
		uint8_t checksum,rx_checksum;
		uint8_t i;

		last_pkt_encrypted=0;

		// Payload length
		length = FASTSPI_READ_FIFO_BYTE();

		// print RX FIFO for debugging
//		xil_printf("len %d ", length);
//		for(i=0; i<2; i++)
//			xil_printf("%x", FASTSPI_READ_FIFO_BYTE());
//		xil_printf(" %3d ", FASTSPI_READ_FIFO_BYTE());
//		for(i=0; i<6; i++)
//			xil_printf("%x", FASTSPI_READ_FIFO_BYTE());
//		xil_printf(" ");
//		for(i=0; i<(length-RF_PACKET_OVERHEAD_SIZE-1); i++)
//			xil_printf("%c", FASTSPI_READ_FIFO_BYTE());
//		xil_printf(" %d ", FASTSPI_READ_FIFO_BYTE());
//		xil_printf(" %d ", FASTSPI_READ_FIFO_BYTE());
//		xil_printf(" %d \r\n", FASTSPI_READ_FIFO_BYTE());

		// Register the payload length
		rfSettingsG.pRxInfo->length = length - RF_PACKET_OVERHEAD_SIZE - 1;
//		xil_printf("len %d ", rfSettingsG.pRxInfo->length);

		// Read the frame control field
		FASTSPI_READ_FIFO_NO_WAIT((uint8_t*) &frameControlField, 2);
		rfSettingsG.pRxInfo->ackRequest = !!(frameControlField & RF_FCF_ACK_BM);

		//read the data sequence number
		rfSettingsG.pRxInfo->seqNumber = FASTSPI_READ_FIFO_BYTE();

		// Is this an acknowledgment packet?
		if ((length == RF_ACK_PACKET_SIZE) && (frameControlField == RF_ACK_FCF)/* && (rfSettingsG.pRxInfo->seqNumber == (rfSettingsG.txSeqNumber - 1))*/)
		{

			// Read the footer and check for CRC OK
			FASTSPI_READ_FIFO_NO_WAIT((uint8_t*) pFooter, 2);

			// Indicate the successful ack reception (this flag is polled by the transmission routine)
			if (pFooter[1] & RF_CRC_OK_BM) rfSettingsG.ackReceived = TRUE;

			return 5;
		}
		else
		{
			// Skip the destination PAN and address (that's taken care of by hardware address recognition!)
			FASTSPI_READ_FIFO_GARBAGE(4);

			// Read the source address
			FASTSPI_READ_FIFO_NO_WAIT((uint8_t*) &rfSettingsG.pRxInfo->srcAddr, 2);

		/* if(frameControlField & RF_SEC_BM)
			{
				uint8_t n;
				// READ rx_ctr and set it
				FASTSPI_READ_FIFO_NO_WAIT((uint8_t*) &rx_ctr, 4);
				FASTSPI_WRITE_RAM(&rx_ctr[0],(CC2420RAM_RXNONCE+9),2,n);
				FASTSPI_WRITE_RAM(&rx_ctr[2],(CC2420RAM_RXNONCE+11),2,n);
				FASTSPI_STROBE(CC2420_SRXDEC);  // if packet is encrypted then decrypt
				last_pkt_encrypted=1;
				rfSettingsG.pRxInfo->length -= 4;
			}*/

			// Read the packet payload
			FASTSPI_READ_FIFO_NO_WAIT(rfSettingsG.pRxInfo->pPayload, rfSettingsG.pRxInfo->length);

//			for(i=0; i<rfSettingsG.pRxInfo->length; i++ )
//				xil_printf("%c", rfSettingsG.pRxInfo->pPayload[i]);

//			uint8_t *p = rfSettingsG.pRxInfo->pPayload;

			// Read the packet checksum byte
			rx_checksum = FASTSPI_READ_FIFO_BYTE();

			// Read the footer to get the RSSI value
			FASTSPI_READ_FIFO_NO_WAIT((uint8_t*) pFooter, 2);
			rfSettingsG.pRxInfo->rssi = pFooter[0];

			checksum=0;
			for(i=0; i<rfSettingsG.pRxInfo->length; i++ )
				checksum += rfSettingsG.pRxInfo->pPayload[i];

			if(checksum!=rx_checksum)
			{
				xil_printf("checksum failed %3d %3d\r\n",rx_checksum, checksum);
				// always read 1 byte before flush (data sheet pg 62)
				tmp = FASTSPI_READ_FIFO_BYTE();
				FASTSPI_STROBE(SFLUSHRX);
				FASTSPI_STROBE(SFLUSHRX);

				return 4;
			}

			/* This is the good condition */
			if (pFooter[1] & RF_CRC_OK_BM)
			{
				//rfSettingsG.pRxInfo = rf_rx_callback(rfSettingsG.pRxInfo);
				return 1;
			}
			else
			{
				// always read 1 byte before flush (data sheet pg 62)
				tmp = FASTSPI_READ_FIFO_BYTE();
				FASTSPI_STROBE(SFLUSHRX);
				FASTSPI_STROBE(SFLUSHRX);

				return 3;
			}

		}
 	}

	FASTSPI_SETREG(EXCFLAG1,0x0);

	return 0;
}
