/* --- BasicCC2520.h       Version 1.0
   --- header for BasicCC2520.c
   ---
   --- Copyright 2001, Mark V Automation Corp.
   ---
   --- 03/31/11 GY Jiang    Initial coding. V1.0
   ---
   --- */

#ifndef BASICRF_H_
#define BASICRF_H_

/* ********************************************************************************************************************************************* */
/*      Include Files						                                              														 */
/* ********************************************************************************************************************************************* */
#include <stdint.h>
#include "xparameters.h"

/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */

#define  RF_PACKET_OVERHEAD_SIZE   11	/*including one length byte, two FCF bytes, one sequence byte, six address bytes (DestAdd, PanID, MyAdd) and one checksum byte in the end	*/

#define  RF_MAX_PAYLOAD_SIZE	   (128 - RF_PACKET_OVERHEAD_SIZE - 2)	/* 128 - 11 - two RSSI bytes	*/


/*	SPI register and operations for zigbee	*/
#define ZIGBEE	XPAR_XPS_SPI_0_BASEADDR

#define SPICR (*((volatile unsigned int *) (ZIGBEE+0x60)))
#define SPISR (*((volatile unsigned int *) (ZIGBEE+0x64)))
#define SPIDTR (*((volatile unsigned int *) (ZIGBEE+0x68)))
#define SPIDRR (*((volatile unsigned int *) (ZIGBEE+0x6C)))
#define SPISSR (*((volatile unsigned int *) (ZIGBEE+0x70)))

#define  REG_READ				(0x80)
#define  REG_WRITE				(0xC0)
#define  MEM_READ				(0x10)
#define  MEM_WRITE				(0x20)
#define  RXFIFO_READ			(0x30)
#define  TXFIFO_WRITE			(0x3A)


//-------------------------------------------------------------------------------------------------------
// The data structure which is used to transmit packets
typedef struct {
	uint16_t destPanId;
	uint16_t destAddr;
	uint8_t length;
    uint8_t *pPayload;
	uint8_t cca;
	uint8_t ackRequest;
} RF_TX_INFO;


//-------------------------------------------------------------------------------------------------------
// The receive struct:
typedef struct {
    uint8_t seqNumber;
	uint16_t srcAddr;
//	uint16_t srcPanId;
	uint8_t length;
	uint8_t max_length;
    uint8_t *pPayload;
	uint8_t ackRequest;
	int8_t rssi;
} RF_RX_INFO;


//-------------------------------------------------------------------------------------------------------
// The RF settings structure:
typedef struct {
    RF_RX_INFO *pRxInfo;
    uint8_t txSeqNumber;
    volatile uint8_t ackReceived;
    uint16_t myPanId;
    uint16_t myAddr;
    uint8_t	 receiveOn;
} RF_SETTINGS;
//extern volatile RF_SETTINGS rfSettings;


/* ********************************************************************************************************************************************* */
/*      Global variables 					                                              														 */
/* ********************************************************************************************************************************************* */

extern RF_TX_INFO	 	rfTxInfoG;
extern RF_RX_INFO 		rfRxInfoG;
extern RF_SETTINGS 	rfSettingsG;

extern uint8_t rxCycleG;	/* 	a flag which shows whether the radio is reading a message	*/

/* ********************************************************************************************************************************************* */
/*      Function prototype                                              																		 */
/* ********************************************************************************************************************************************* */

void 		cc2520_init(RF_RX_INFO *pRRI, uint8_t channel, uint16_t panId, uint16_t myAddr);
uint8_t 	rf_tx_packet(RF_TX_INFO *pRTI);
void 		rf_polling_rx_on(void);
uint8_t 	rf_rx_check_sfd();
int8_t 		rf_polling_rx_packet();

#endif /* BASICRF_H_ */
