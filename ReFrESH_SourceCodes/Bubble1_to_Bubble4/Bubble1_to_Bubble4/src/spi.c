/* --- spi.c       Version 1.0
   --- this file provides low level driver for SPI and SPI functions  required by zigbee cc2520
   ---
   --- Copyright 2001, Mark V Automation Corp.
   ---
   --- 03/31/11 GYJ	Initial coding. V1.0
   ---
   --- */

#include "spi.h"
#include "BasicCC2520.h"	// required for REG_READ and REG_WRITE

/* ********************************************************************************************************************************************* */
/*      basic spi functions                                              																		 */
/* ********************************************************************************************************************************************* */
/* --- SPI_ENABLE() enables SPI communication.
 * ---
 */
void SPI_ENABLE()
{
	SPISSR = 0x0;
}

/* --- SPI_DISABLE() disables SPI communication.
 * ---
 */
void SPI_DISABLE()
{
	SPISSR = 0x1;
}

/* --- FASTSPI_TX() sends one byte over SPI.
 * --- uint8_t Data is the byte to be sent
 */
uint8_t FASTSPI_TX(uint8_t Data)
{
	SPIDTR = Data;
	SPICR=0x086;
	while(!(SPISR & 0x04));
	SPICR = 0x186;

	return SPIDRR;
}

/* --- FASTSPI_RX() reads one byte over SPI.
 * ---
 */
uint8_t FASTSPI_RX()
{
	SPIDTR = 0x0;
	SPICR=0x086;
	while(!(SPISR & 0x04));
	SPICR = 0x186;

	return SPIDRR;
}


/* ********************************************************************************************************************************************* */
/*      SPI functions for zigbee cc2520                                              														 */
/* ********************************************************************************************************************************************* */

/* --- FASTSPI_SETREG() sets one register of cc2520 over SPI.
 * ---
 */
void  FASTSPI_SETREG(uint8_t addr,uint8_t value)
{
	SPI_ENABLE();
	FASTSPI_TX(addr|REG_WRITE);
	FASTSPI_TX(value);
	SPI_DISABLE();
}

/* --- FASTSPI_GETREG() reads the value of one register of cc2520 over SPI.
 * ---
 */
uint8_t FASTSPI_GETREG(uint8_t addr)
{
	uint16_t value;
	SPI_ENABLE();
	FASTSPI_TX(addr|REG_READ);
	value = FASTSPI_RX();
	SPI_DISABLE();
	return value;
}

/* --- FASTSPI_WRITE_RAM() writes one byte to the RAM of cc2520 over SPI.
 * ---
 */
void FASTSPI_WRITE_RAM(uint8_t *p,uint8_t AddH,uint8_t AddL,uint8_t c)
{
	uint8_t i;

    for(i=0;i<c;i++)
	{
	    SPI_ENABLE();
        FASTSPI_TX(0x20 | AddH);
        FASTSPI_TX(AddL++);
        FASTSPI_TX(*(p++));
        SPI_DISABLE();
	}
 }

/* --- FASTSPI_READ_RAM() reads one byte in the RAM of cc2520 over SPI.
 * ---
 */
void FASTSPI_READ_RAM(uint8_t *p,uint8_t AddH,uint8_t AddL,uint8_t c)
{
	uint8_t i;

    for(i=0;i<c;i++)
	{
	    SPI_ENABLE();
        FASTSPI_TX(0x10 | AddH);
        FASTSPI_TX(AddL++);
        *(p++) = FASTSPI_RX();
        SPI_DISABLE();
	}
}

/* --- FASTSPI_WRITE_FIFO() writes (c) number of bytes to the FIFO of cc2520 over SPI.
 * ---
 */
void FASTSPI_WRITE_FIFO(uint8_t *p,uint8_t c)
{
	uint8_t i;

    for(i=0;i<c;i++)
	{
	    SPI_ENABLE();
        FASTSPI_TX(TXFIFO_WRITE);
        FASTSPI_TX(*(p++));
        SPI_DISABLE();
	}
}

/* --- FASTSPI_READ_FIFO() reads (c) number of bytes in the FIFO of cc2520 over SPI.
 * ---
 */
void FASTSPI_READ_FIFO(uint8_t *p,uint8_t c)
{
	uint8_t i;

    for(i=0;i<c;i++)
	{
	    SPI_ENABLE();
        FASTSPI_TX(RXFIFO_READ);
        *(p++) = FASTSPI_RX();
        SPI_DISABLE();
	}
}

/* --- FASTSPI_READ_FIFO_NO_WAIT() reads (c) number of bytes in the FIFO of cc2520 over SPI.
 * ---
 */
void FASTSPI_READ_FIFO_NO_WAIT(uint8_t *p,uint8_t c)
{
	uint8_t i;

	for(i=0;i<c;i++)
	{
		*(p++) = FASTSPI_READ_FIFO_BYTE();
	}
}

/* --- FASTSPI_READ_RAM() reads one byte in the FIFO of cc2520 over SPI.
 * ---
 */
uint8_t FASTSPI_READ_FIFO_BYTE()
{
	 uint8_t value;
	 SPI_ENABLE();
     FASTSPI_TX(RXFIFO_READ);
     value = FASTSPI_RX();
     SPI_DISABLE();
	 return value;
}

/* --- FASTSPI_STROBE() sends one command to cc2520 over SPI.
 * ---
 */
void FASTSPI_STROBE(uint8_t cmd)
{
	SPI_ENABLE();
	FASTSPI_TX(cmd);
	SPI_DISABLE();
}

/* --- FASTSPI_STROBE() discards (c) number of bytes in the FIFO of cc2520 over SPI.
 * ---
 */
void FASTSPI_READ_FIFO_GARBAGE(uint8_t c)
{
	uint8_t i;

    for(i=0;i<c;i++)
	{
    	FASTSPI_READ_FIFO_BYTE();
	}

}

/* --- FASTSPI_RX_GARBAGE().
 * ---
 */
void FASTSPI_RX_GARBAGE()
{
	SPIDTR = 0x0;
	SPICR=0x086;
    FASTSPI_WAIT();

}

/* --- FASTSPI_WAIT().
 * ---
 */
void FASTSPI_WAIT()
{
	while(!(SPISR & 0x04));
	SPICR = 0x186;
}
