/* --- BasicCC2520.h       Version 1.0
   --- header for BasicCC2520.c
   ---
   --- Copyright 2001, Mark V Automation Corp.
   ---
   --- 03/31/11 GYJ	Initial coding. V1.0
   ---
   --- */

#ifndef CC2520_H_
#define CC2520_H_

#include <stdint.h>

/* ********************************************************************************************************************************************* */
/*      definition							                                              														 */
/* ********************************************************************************************************************************************* */
#define MSG_ARRAY_MAX		10

typedef struct {
	char 	 *buf;
	uint16_t src;
	uint16_t key;
	uint16_t next;		/* Index pointing to empty space in buf[] */
	int16_t  length;	/* Length of last full msg recvd - Added with v 1.1 */
} MessageRec_t;

/* ********************************************************************************************************************************************* */
/*      Global variables 					                                              														 */
/* ********************************************************************************************************************************************* */

extern char othrBuf[256];

extern MessageRec_t	MsgRegArrayG[MSG_ARRAY_MAX];
extern uint8_t		MsgRegHeadG, MsgRegTailG;

/* ********************************************************************************************************************************************* */
/*      Function prototype                                              																		 */
/* ********************************************************************************************************************************************* */

void initialize_zigbee(uint8_t channel, uint16_t panId, uint16_t myAddr);

int16_t sbsZigbeeGetRecvMsgSize(MessageRec_t *myRec, uint16_t src, uint16_t key);
void sbsZigbeeSend(uint16_t destPan, uint16_t destAddr, uint8_t * Tx_Message, uint16_t len);
MessageRec_t * sbsZigbeeRegister(char *buf, uint16_t src, uint16_t key);

uint16_t sbsZigbeeOSparse();

char Zigbee_Receive();
char Zigbee_CMD_Receive();
void Zigbee_Transmit(uint16_t destPan, uint16_t destAddr, char * Tx_Message, uint16_t len);

#endif /* CC2520_H_ */
