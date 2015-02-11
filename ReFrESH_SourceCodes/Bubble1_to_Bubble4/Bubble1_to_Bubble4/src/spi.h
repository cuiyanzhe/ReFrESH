/* --- spi.h       Version 1.0
   --- header for spi.c
   ---
   --- Copyright 2001, Mark V Automation Corp.
   ---
   --- 03/31/11 GYJ	Initial coding. V1.0
   ---
   --- */


#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

/* ********************************************************************************************************************************************* */
/*      Function prototype                                              																		 */
/* ********************************************************************************************************************************************* */

/*	functions for basic SPI operation	*/
void SPI_ENABLE();
void SPI_DISABLE();
uint8_t FASTSPI_TX(uint8_t Data);
uint8_t FASTSPI_RX();

/*	functions for zigbee operation	*/
void  		FASTSPI_SETREG(uint8_t addr, uint8_t value);
uint8_t 	FASTSPI_GETREG(uint8_t addr);
void 		FASTSPI_WRITE_RAM(uint8_t *p, uint8_t AddH, uint8_t AddL, uint8_t c);
void 		FASTSPI_READ_RAM(uint8_t *p, uint8_t AddH, uint8_t AddL, uint8_t c);
void 		FASTSPI_WRITE_FIFO(uint8_t *p, uint8_t c);
void 		FASTSPI_READ_FIFO(uint8_t *p, uint8_t c);
void 		FASTSPI_READ_FIFO_NO_WAIT(uint8_t *p, uint8_t c);
uint8_t 	FASTSPI_READ_FIFO_BYTE();
void 		FASTSPI_STROBE(uint8_t cmd);
void 		FASTSPI_READ_FIFO_GARBAGE(uint8_t c);
void 		FASTSPI_RX_GARBAGE();
void 		FASTSPI_WAIT();

#endif /* SPI_H_ */
