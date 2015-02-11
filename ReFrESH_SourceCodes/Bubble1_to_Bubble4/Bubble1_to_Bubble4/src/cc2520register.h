/* --- cc2520register.h       Version 1.0
   --- This file provides addresses of cc2520 registers.
   ---
   --- Copyright 2001, Mark V Automation Corp.
   ---
   --- 12/31/14 GYJ	Initial coding. Moved from MPU6000.c V1.0
   ---
   --- */

#ifndef CC2520REGISTER_H_
#define CC2520REGISTER_H_

// The time it takes for the acknowledgment packet to be received after the data packet has been transmitted
#define  RF_ACK_DURATION		  (0.5 * 32 * 2 * ((4 + 1) + (1) + (2 + 1) + (2)))
#define  RF_SYMBOL_DURATION	      (32 * 0.5)
#define  CHECKSUM_OVERHEAD        1

#define  RF_ACK_PACKET_SIZE		   5

// The length byte
#define  RF_LENGTH_MASK           0x7F

// Frame control field
#define  RF_FCF_NOACK	          0x4188
#define  RF_FCF_ACK               0x8861

#define  RF_SEC_BM		  	      0x0008
#define  RF_ACK_BM		 		  0x0020
#define  RF_FCF_ACK_BM            0x0020
#define  RF_FCF_BM                (~RF_FCF_ACK_BM)
#define  RF_ACK_FCF		          0x0002

// Footer
#define  RF_CRC_OK_BM             0x80

#define  FIFO_IS_1        GPIO1_IN
#define  CCA_IS_1         GPIO3_IN
#define  RESET_IS_1       RESET_DDR_OUT()
#define  VREG_IS_1        VREG_DDR_OUT()
#define  FIFOP_IS_1       GPIO2_IN
#define  SFD_IS_1         GPIO4_IN

//---------(RF chip definitions)---------//
//the hex numbers are same as Instruction implementation in CC2520.h by TI

#define  CC2520_RAM_PANID               0x3F2
#define  CC2520_RAM_SHORTADDR           0x3F4


//--------- (CC2520 command definitions)---------//
//the hex numbers are same as Instruction implementation in CC2520.h by TI
#define  SNOP				(0x00)
#define  IBUFLD				(0x02)
#define  SIBUFEX			(0x03)
#define  SSAMPLECCA			(0x04)
#define  SRES 	 		    (0x09)
#define  SXOSCON			(0x40)
#define  SRXON	            (0x42)
#define  STXON	            (0x43)
#define  SRFOFF	            (0x45)
#define  SFLUSHRX 			(0x47)
#define  SFLUSHTX			(0x48)


//--------- (CC2520 Configure register)---------//
//the hex numbers are same as FREG definitions in CC2520.h by TI
#define  FRMFILT0				(0x00)
#define  FRMFILT1				(0x01)
#define  SRCMATCH				(0x02)
#define  SRCSHORTEN0			(0x04)
#define  SRCSHORTEN1			(0x05)
#define  SRCSHORTEN2			(0x06)
#define  SRCEXTEN0				(0x08)
#define  SRCEXTEN1				(0x09)
#define  SRCEXTEN2				(0x0A)
#define  FRMCTRL0				(0x0C)
#define  FRMCTRL1				(0x0D)
#define  RXENABLE0				(0x0E)
#define  RXENABLE1				(0x0F)
#define  EXCFLAG0				(0x10)
#define  EXCFLAG1				(0x11)
#define  EXCFLAG2				(0x12)
#define  EXCMASKA0				(0x14)
#define  EXCMASKA1              (0x15)
#define  EXCMASKA2              (0x16)
#define  EXCMASKB0              (0x18)
#define  EXCMASKB1              (0x19)
#define  EXCMASKB2              (0x1A)
#define  EXCBINDX0              (0x1C)
#define  EXCBINDX1              (0x1D)
#define  EXCBINDY0              (0x1E)
#define  EXCBINDY1              (0x1F)
#define  GPIOCTRL0              (0x20)
#define  GPIOCTRL1              (0x21)
#define  GPIOCTRL2              (0x22)
#define  GPIOCTRL3              (0x23)
#define  GPIOCTRL4              (0x24)
#define  GPIOCTRL5              (0x25)
#define  GPIOPOLARITY           (0x26)
#define  GPIOCTRL               (0x28)
#define  DPUCON                 (0x2A)
#define  DPUSTAT                (0x2C)
#define  FREQCTRL               (0x2E)
#define  FREQTUNE               (0x2F)
#define  TXPOWER                (0x30)
#define  FSMSTAT0               (0x32)
#define  FSMSTAT1               (0x33)
#define  FIFOPCTRL              (0x34)
#define  FSMCTRL                (0x35)
#define  CCACTRL0               (0x36)
#define  CCACTRL1               (0x37)
#define  RSSI                   (0x38)
#define  RSSISTAT               (0x39)
#define  RXFIRST				(0x3C)
#define  RXFIFOCNT				(0x3E)
#define  TXFIFOCNT				(0x3F)

#define  CHIPIDADD				(0x40)

#define  MDMCTRL0               (0x46)
#define  MDMCTRL1               (0x47)

//---------(PSDU definitions)---------//
//FRAME_CONTROL_FIELD
//FRAME_TYPE
#define  FRAME_TYPE_BEACCON	    0x00
#define  FRAME_TYPE_DATA        0x01
#define  FRAME_TYPE_ACK         0x02
#define  FRAME_TYPE_MAC			0x03

#define  SECURITY_ENABLE		0x00
#define  FRAME_PENDING			0x00
#define  ACKNOWLEDGMENT_REQUEST 0x00
#define  PAN_ID_COMPRESSION		0x00
#define  DEST_ADDRESSING_MODE	0x03
#define  FRAME_VERSION			0x01
#define  SOURCE_ADDRESSING_MODE	0x03
#define  SEQUENCE_NUMBER		0x02


//---------(CC2520 RAM Address)---------//
#define  RAM_IEEEADR          (0xEA)
#define  RAM_PANID		      (0xF2)
#define  RAM_SHORTADR         (0xF4)

#endif /* CC2520REGISTER_H_ */
