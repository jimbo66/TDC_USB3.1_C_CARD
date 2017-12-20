#pragma NOIV               // Do not generate interrupt vectors
//-----------------------------------------------------------------------------
//   File:      CYStream.c
//   Contents:   USB Bulk and Isoc streaming example code.
//
// Copyright (c) 2003, Cypress Semiconductor Corporation All rights reserved
//
// This software is owned by Cypress Semiconductor Corporation
// (Cypress) and is protected by United States copyright laws and
// international treaty provisions.  Therefore, unless otherwise specified in a
// separate license agreement, you must treat this
// software like any other copyrighted material.  Reproduction, modification, translation,
// compilation, or representation of this software in any other form
// (e.g., paper, magnetic, optical, silicon, etc.) is prohibited
// without the express written permission of Cypress.
//
// Disclaimer: Cypress makes no warranty of any kind, express or implied, with
// regard to this material, including, but not limited to, the implied warranties
// of merchantability and fitness for a particular purpose. Cypress reserves the
// right to make changes without further notice to the materials described
// herein. Cypress does not assume any liability arising out of the application
// or use of any product or circuit described herein. Cypress’ products described
// herein are not authorized for use as components in life-support devices.
//
// This software is protected by and subject to worldwide patent
// coverage, including U.S. and foreign patents. Use may be limited by
// and subject to the Cypress Software License Agreement.
//
//-----------------------------------------------------------------------------
#include <include\fx2.h>
#include <include\fx2regs.h>
#include <include\fx2sdly.h>
#include <stdlib.h>

extern BOOL   GotSUD;         // Received setup data flag
extern BOOL   Sleep;
extern BOOL   Rwuen;
extern BOOL   Selfpwr;
extern bit	Switch_Change_FLAG;
extern bit	Sudav_Happen_Flag;

typedef struct
{
//	BYTE SLV_ADR;
	WORD REG_ADR;
	BYTE *Pdat;
}I2C_Buf;

enum {
    Alt0_BulkIN = 0,
    Alt1_IntINOUT,		//
    Alt2_BulkINOUT,		//
    Alt3_IsocIN,
    Alt4_IsocOUT,
    Alt5_IsocIN,
    Alt6_IsocINOUT,		//
	Alt7_BulkSpeed		//bulk speed check,EP2 Bulk IN,EP6 Bulk Out.
};

enum {
    Full_Alt0_BulkINOUT = 0,	//
    Full_Alt1_IntINOUT,			//
    Full_Alt2_IsocIN,
    Full_Alt3_IsocOUT
};



BYTE    Configuration;      // Current configuration
BYTE    AlternateSetting = Alt0_BulkIN;   // Alternate settings

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------
#define	VR_UPLOAD		0xc0
#define VR_DOWNLOAD		0x40

#define VR_ANCHOR_DLD   0xa0 // handled by core
#define VR_EEPROM		0xa2 // loads (uploads) EEPROM
#define	VR_RAM			0xa3 // loads (uploads) external ram
#define VR_SETI2CADDR	0xa4
#define VR_GETI2C_TYPE  0xa5 // 8 or 16 byte address
#define VR_GET_CHIP_REV 0xa6 // Rev A, B = 0, Rev C = 2 // NOTE: New TNG Rev
#define VR_TEST_MEM     0xa7 // runs mem test and returns result
#define VR_RENUM	    0xa8 // renum
#define VR_DB_FX	    0xa9 // Force use of double byte address EEPROM (for FX)
#define VR_I2C_100    0xaa // put the i2c bus in 100Khz mode
#define VR_I2C_400    0xab // put the i2c bus in 400Khz mode
#define VR_NOSDPAUTO  0xac // test code. does uploads using SUDPTR with manual length override


#define GET_CHIP_REV()		((CPUCS >> 4) & 0x00FF) // EzUSB Chip Rev Field


#define SERIAL_ADDR		0x50
#define EP0BUFF_SIZE	0x40




//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
BYTE			DB_Addr;					//TPM Dual Byte Address stat
BYTE			I2C_Addr;					//TPM I2C address

//-----------------------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------------------
//void EEPROMWrite(WORD addr, BYTE length, BYTE xdata *buf); //TPM EEPROM Write
void EEPROMRead(WORD addr, BYTE length, BYTE xdata *buf);  //TPM EEPROM Read
void EEPROMWritePage(WORD addr, BYTE length);

//-----------------------------------------------------------------------------
// Task Dispatcher hooks
//   The following hooks are called by the task dispatcher.
//-----------------------------------------------------------------------------

WORD mycount;

void TD_Init(void)             // Called once at startup
{

   int i,j;
 
   // set the CPU clock to 48MHz
   CPUCS = ((CPUCS & ~bmCLKSPD) | bmCLKSPD1) ;
   SYNCDELAY;

   // set the slave FIFO interface to 48MHz
   IFCONFIG |= 0x40;
   SYNCDELAY;

    // Default interface uses endpoint 2, zero the valid bit on all others
    // Just using endpoint 2, zero the valid bit on all others
    EP1OUTCFG = (EP1OUTCFG & 0x7F);
	SYNCDELAY;
	EP1INCFG = (EP1INCFG & 0x7F);
	SYNCDELAY;
	EP4CFG = (EP4CFG & 0x7F);
	SYNCDELAY;
	EP6CFG = (EP6CFG & 0x7F);
	SYNCDELAY;
	EP8CFG = (EP8CFG & 0x7F);
	SYNCDELAY;
    EP2CFG = 0xE0;  // EP2 is DIR=IN, TYPE=BULK, SIZE=512, BUF=4x

   EIE|=0x08;	//test int5
   mycount = 0;

   // Prepare data
   for (i=1;i<5;i++)                   ///???Why excute 4 times?
   {
      EP2FIFOBUF[0] = LSB(mycount);
      EP2FIFOBUF[1] = MSB(mycount);
      EP2FIFOBUF[2] = USBFRAMEL;
      EP2FIFOBUF[3] = USBFRAMEH;
      EP2FIFOBUF[4] = MICROFRAME;
      for (j=5;j<1024;j++)
      {
         EP2FIFOBUF[j] = i;
      }
      EP2BCH = 0x02; ///512
      EP2BCL = 0x00;
   }
   		
	AUTOPTRSETUP |= 0x01;

    Rwuen = TRUE;                 // Enable remote-wakeup
	EZUSB_InitI2C();			// Initialize I2C Bus    

}

void TD_Poll(void)             // Called repeatedly while the device is idle
{  
	unsigned int i;
  	WORD count;
	// ...FX2 in high speed mode
	if(EZUSB_HIGHSPEED())
	{ 
	    // Perform USB activity based upon the Alt. Interface selected 		
	    switch (AlternateSetting)
	    {
	        case Alt6_IsocINOUT: 	 
	 			if(!(EP2468STAT & bmEP2FULL))   
	            {                			
					EP2BCH=0x00;
					EP2BCL=0x20; 
			    }
				break;
			case Alt0_BulkIN:
	            // Send data on EP2
	            if(!(EP2468STAT & bmEP2FULL))
	            {
	                EP2FIFOBUF[0] = LSB(mycount);
	                EP2FIFOBUF[1] = MSB(mycount);
	                EP2FIFOBUF[2] = USBFRAMEL;
	                EP2FIFOBUF[3] = USBFRAMEH;
	                EP2FIFOBUF[4] = MICROFRAME;
	
	                EP2BCH = 0x02;		//512 byte
	                EP2BCL = 0x00;
	
	                mycount++;
	            }
	        break;
			case Alt1_IntINOUT:
		    case Alt2_BulkINOUT:	           				  
				if(!(EP2468STAT & bmEP6EMPTY))
  				{ // check EP6 EMPTY(busy) bit in EP2468STAT (SFR), core set's this bit when FIFO is empty
    				if(!(EP2468STAT & bmEP2FULL))
    				{  // check EP2 FULL(busy) bit in EP2468STAT (SFR), core set's this bit when FIFO is full
      					APTR1H = MSB( &EP6FIFOBUF );
        				APTR1L = LSB( &EP6FIFOBUF );

       					AUTOPTRH2 = MSB( &EP2FIFOBUF );
    					AUTOPTRL2 = LSB( &EP2FIFOBUF );

      					count = (EP6BCH << 8) + EP6BCL;

        				// loop EP2OUT buffer data to EP6IN
        				for( i = 0x0000; i < count; i++ )
        				{
           					// setup to transfer EP2OUT buffer to EP6IN buffer using AUTOPOINTER(s)
          					EXTAUTODAT2 = EXTAUTODAT1;
       					}
        				EP2BCH = EP6BCH;  
        				SYNCDELAY;  
        				EP2BCL = EP6BCL;        // arm EP6IN
        				SYNCDELAY;                    
       					EP6BCL = 0x80;          // re(arm) EP2OUT
     				}
				}
	        break;
	        case Alt3_IsocIN:
	        case Alt5_IsocIN:
	            // Send data on EP2
	            if(!(EP2468STAT & bmEP2FULL))
	            {
	                EP2FIFOBUF[0] = LSB(mycount);
	                EP2FIFOBUF[1] = MSB(mycount);
	                EP2FIFOBUF[2] = USBFRAMEL;
	                EP2FIFOBUF[3] = USBFRAMEH;
	                EP2FIFOBUF[4] = MICROFRAME;
	
	                EP2BCH = 0x04;
	                EP2BCL = 0x00;
	
	                mycount++;
	            }
	        break;
	        case Alt4_IsocOUT:
	            // check EP2 EMPTY(busy) bit in EP2468STAT (SFR), core set's this bit when FIFO is empty
	            if(!(EP2468STAT & bmEP2EMPTY))
	            { 
	                EP2BCL = 0x80;          // re(arm) EP2OUT
	            }
	        break;
	        default:break;    
	   }
	}
	else	// Full Speed
	{
	    // Perform USB activity based upon the Alt. Interface selected 
	    switch (AlternateSetting)
	    {
	        case Full_Alt0_BulkINOUT:
	        case Full_Alt1_IntINOUT:	
	            // check EP2 EMPTY(busy) bit in EP2468STAT (SFR), core set's this bit when FIFO is empty
	            if(!(EP2468STAT & bmEP6EMPTY))
  				{ // check EP6 EMPTY(busy) bit in EP2468STAT (SFR), core set's this bit when FIFO is empty
    				if(!(EP2468STAT & bmEP2FULL))
    				{  // check EP2 FULL(busy) bit in EP2468STAT (SFR), core set's this bit when FIFO is full
      					APTR1H = MSB( &EP6FIFOBUF );
        				APTR1L = LSB( &EP6FIFOBUF );

       					AUTOPTRH2 = MSB( &EP2FIFOBUF );
    					AUTOPTRL2 = LSB( &EP2FIFOBUF );

      					count = (EP6BCH << 8) + EP6BCL;

        				// loop EP2OUT buffer data to EP6IN
        				for( i = 0x0000; i < count; i++ )
        				{
           					// setup to transfer EP2OUT buffer to EP6IN buffer using AUTOPOINTER(s)
          					EXTAUTODAT2 = EXTAUTODAT1;
       					}
        				EP2BCH = EP6BCH;  
        				SYNCDELAY;  
        				EP2BCL = EP6BCL;        // arm EP2IN
        				SYNCDELAY;                    
       					EP6BCL = 0x80;          // re(arm) EP6OUT
     				}
				}
	        	break;
	        //case Full_Alt2_IsocIN:
	            // Send data on EP2
	        	//break;
	        case Full_Alt3_IsocOUT:
	            // check EP2 EMPTY(busy) bit in EP2468STAT (SFR), core set's this bit when FIFO is empty
	            if(!(EP2468STAT & bmEP2EMPTY))
	            { 
	                EP2BCL = 0x80;          // re(arm) EP2OUT
	            }
	        	break;
			default:break;
	   }	
	}
}

//-----------------------------------------------------------------------------
// Device Request hooks
//   The following hooks are called by the end point 0 device request parser.
//-----------------------------------------------------------------------------

BOOL DR_GetDescriptor(void)
{
   return(TRUE);
}

BOOL DR_SetConfiguration(void)   // Called when a Set Configuration command is received
{
   Configuration = SETUPDAT[2];
   return(TRUE);            // Handled by user code
}

BOOL DR_GetConfiguration(void)   // Called when a Get Configuration command is received
{
   EP0BUF[0] = Configuration;
   EP0BCH = 0;
   EP0BCL = 1;
   return(TRUE);            // Handled by user code
}

BOOL DR_SetInterface(void)       // Called when a Set Interface command is received
{ 
	unsigned int i;
	unsigned char j;
    AlternateSetting = SETUPDAT[2];

	EPIE  &= ~(bmBIT4 | bmBIT6);
    EPIRQ = 0xff;//clear IRQ request
	// ...FX2 in high speed mode
	if( EZUSB_HIGHSPEED( ) )
	{ 
	    // Change configuration based upon the Alt. Interface selected 
	    switch (AlternateSetting)
	    {
	        case Alt0_BulkIN:
	            // Only using endpoint 2, zero the valid bit on all others
	            // Just using endpoint 2, zero the valid bit on all others
	            EP2CFG = 0xE0;  // EP2 is DIR=IN, TYPE=BULK, SIZE=512, BUF=4x
	            SYNCDELAY;
	
	            EP1OUTCFG = (EP1OUTCFG & 0x7F);
	        	SYNCDELAY;
	        	EP1INCFG = (EP1INCFG & 0x7F);
	        	SYNCDELAY;
	        	EP4CFG = (EP4CFG & 0x7F);
	        	SYNCDELAY;
	        	EP6CFG = (EP6CFG & 0x7F);
	        	SYNCDELAY;
	        	EP8CFG = (EP8CFG & 0x7F);
	        	SYNCDELAY;
	
	            // Clear out any committed packets
	            FIFORESET = 0x80;
	            SYNCDELAY;
	            FIFORESET = 0x02;
	            SYNCDELAY;
	            FIFORESET = 0x00;
	            SYNCDELAY;
	
	            // Reset data toggle to 0
	            TOGCTL = 0x12;  // EP2 IN
	            TOGCTL = 0x32;  // EP2 IN Reset
	
	        break;
	
	        case Alt1_IntINOUT:
	            // Only using endpoint 2, zero the valid bit on all others
	            EP2CFG = 0xF2;  // EP2 is DIR=IN, TYPE=INT, SIZE=512, BUF=2x
	            SYNCDELAY;
				EP6CFG = 0xB2;  // EP6 is DIR=OUT, TYPE=INT, SIZE=512, BUF=2x
	            SYNCDELAY;
	
	            EP1OUTCFG = (EP1OUTCFG & 0x7F);
	        	SYNCDELAY;
	        	EP1INCFG = (EP1INCFG & 0x7F);
	        	SYNCDELAY;
	        	EP4CFG = (EP4CFG & 0x7F);
	        	SYNCDELAY;
	           	EP8CFG = (EP8CFG & 0x7F);
	        	SYNCDELAY;
	           
	         	  // Clear out any committed packets
	            FIFORESET = 0x80;
	            SYNCDELAY;
	            FIFORESET = 0x02;
	            SYNCDELAY;							
	            FIFORESET = 0x00;
	            SYNCDELAY;
	
	            // Reset data toggle to 0
	            TOGCTL = 0x12;  // EP2 IN
	            TOGCTL = 0x32;  // EP2 IN Reset
	
	            // OUT endpoints do NOT come up armed
	            EP6BCL = 0x80; // arm first buffer by writing BC w/skip=1
	            SYNCDELAY;
	            EP6BCL = 0x80; // arm second buffer by writing BC w/skip=1
	        	SYNCDELAY;
	
			//	AUTOPTRSETUP |= 0x01;
	            
	        break;
	        case Alt2_BulkINOUT:
	            // Using endpoints 2 & 6, zero the valid bit on all others
	            EP2CFG = 0xE2; // EP2 is DIR=IN, TYPE=BULK, SIZE=512, BUF=2x
	        	SYNCDELAY;
	            EP6CFG = 0xA2; // EP6 is DIR=OUT, TYPE=BULK, SIZE=512, BUF=2x   
	        	SYNCDELAY;
	            
	            EP1OUTCFG = (EP1OUTCFG & 0x7F);
	        	SYNCDELAY;
	        	EP1INCFG = (EP1INCFG & 0x7F);
	        	SYNCDELAY;
	        	EP4CFG = (EP4CFG & 0x7F);
	        	SYNCDELAY;
	        	EP8CFG = (EP8CFG & 0x7F);
	        	SYNCDELAY;
	
	            // Clear out any committed packets
	            FIFORESET = 0x80;
	            SYNCDELAY;
	            FIFORESET = 0x02;
	            SYNCDELAY;				 
	            FIFORESET = 0x00;
	            SYNCDELAY;
	
	            // Reset data toggle to 0
	            TOGCTL = 0x12;  // EP2 IN
	            TOGCTL = 0x32;  // EP2 IN Reset
	
	            // OUT endpoints do NOT come up armed
	            EP6BCL = 0x80; // arm first buffer by writing BC w/skip=1
	            SYNCDELAY;
	            EP6BCL = 0x80; // arm second buffer by writing BC w/skip=1
	        	SYNCDELAY;
	        break;
	
	        case Alt3_IsocIN:
	            // Only using endpoint 2, zero the valid bit on all others
	            EP2CFG = 0xD8;  // EP2 is DIR=IN, TYPE=ISOC, SIZE=1024, BUF=4x
	            SYNCDELAY;
	            
	            EP1OUTCFG = EP1INCFG = EP4CFG = EP6CFG = EP8CFG = 0x00; 
	            SYNCDELAY;
	
	            // Clear out any committed packets
	            FIFORESET = 0x80;
	            SYNCDELAY;
	            FIFORESET = 0x02;
	            SYNCDELAY;
	            FIFORESET = 0x00;
	            SYNCDELAY;
	
	            // This register sets the number of Isoc packets to send per
	            // uFrame.  This register is only valid in high speed.
	            EP2ISOINPKTS = 0x03;
	
	        break;
	
	        case Alt4_IsocOUT:
	        {
	            // Only using endpoint 2, zero the valid bit on all others
	            EP1OUTCFG = EP1INCFG = EP4CFG = EP6CFG = EP8CFG = 0x00; 
	            SYNCDELAY;
	            EP2CFG = 0x98;  // EP2 is DIR=OUT, TYPE=ISOC, SIZE=1024, BUF=4x
	            SYNCDELAY;
	
	            // OUT endpoints do NOT come up armed
	            EP2BCL = 0x80; // arm first buffer by writing BC w/skip=1
	            SYNCDELAY;
	            EP2BCL = 0x80; // arm second buffer by writing BC w/skip=1        break;
	
	        }
	        break;
	
	        case Alt5_IsocIN:
	        {
	            // Only using endpoint 2, zero the valid bit on all others
	            EP2CFG = 0xD8;  // EP2 is DIR=IN, TYPE=ISOC, SIZE=1024, BUF=4x
	            SYNCDELAY;
	
	            EP1OUTCFG = EP1INCFG = EP4CFG = EP6CFG = EP8CFG = 0x00; 
	            SYNCDELAY;
	
	            // Clear out any committed packets
	            FIFORESET = 0x80;
	            SYNCDELAY;
	            FIFORESET = 0x02;
	            SYNCDELAY;
	            FIFORESET = 0x00;
	            SYNCDELAY;
	
	            // This register sets the number of Isoc packets to send per
	            // uFrame.  This register is only valid in high speed.
	            EP2ISOINPKTS = 0x01;
	        }
	        break;
	
	        case Alt6_IsocINOUT:
	        {
				EP2CFG = 0xD2; // EP2 is DIR=IN, TYPE=ISOC, SIZE=512, BUF=2x
	            SYNCDELAY; 
			    EP6CFG = 0x9A; // EP6 is DIR=OUT, TYPE=ISOC, SIZE=1024, BUF=2x    
	            SYNCDELAY;
				// Clear out any committed packets
	            FIFORESET = 0x80;
	            SYNCDELAY;
	            FIFORESET = 0x02;
	            SYNCDELAY;
	            FIFORESET = 0x00;
	            SYNCDELAY;
	
				for(i=0;i<32;i++)
				{
				 EP2FIFOBUF[i]=0xa5;
				}
			
				EP2BCH=0x00;
				SYNCDELAY;
				EP2BCL=0x20;
			 	SYNCDELAY;				
				for(i=0;i<32;i++)
				{
				 EP2FIFOBUF[i]=i;
				}
			
				EP2BCH=0x00;
				SYNCDELAY;
				EP2BCL=0x20;
			 	SYNCDELAY;
			 
	            // This register sets the number of Isoc packets to send per
	            // uFrame.  This register is only valid in high speed.
	            EP2ISOINPKTS = 0x01;
		
	            // OUT endpoints do NOT come up armed
	            EP6BCL = 0x80; // arm first buffer by writing BC w/skip=1
	            SYNCDELAY;
	            EP6BCL = 0x80; // arm second buffer by writing BC w/skip=1
	        }
	        break;
			//high speed bulk transfer speed test
			case Alt7_BulkSpeed:
			{
				EPIE  |= bmBIT4 | bmBIT6;
				EPIRQ = 0xff;//clear IRQ request
				// Using endpoints 2 & 6, zero the valid bit on all others
	            EP2CFG = 0xE2; // EP2 is DIR=IN, TYPE=BULK, SIZE=512, BUF=2x
	        	SYNCDELAY;
	            EP6CFG = 0xA2; // EP6 is DIR=OUT, TYPE=BULK, SIZE=512, BUF=2x   
	        	SYNCDELAY;
	            
	            EP1OUTCFG = (EP1OUTCFG & 0x7F);
	        	SYNCDELAY;
	        	EP1INCFG = (EP1INCFG & 0x7F);
	        	SYNCDELAY;
	        	EP4CFG = (EP4CFG & 0x7F);
	        	SYNCDELAY;
	        	EP8CFG = (EP8CFG & 0x7F);
	        	SYNCDELAY;
	
	            // Clear out any committed packets
	            FIFORESET = 0x80;
	            SYNCDELAY;
	            FIFORESET = 0x02;
	            SYNCDELAY;				 
	            FIFORESET = 0x00;
	            SYNCDELAY;
	
	            // Reset data toggle to 0
	            TOGCTL = 0x12;  // EP2 IN
	            TOGCTL = 0x32;  // EP2 IN Reset
			
				for(j=0;j<4;j++)
				{
					for(i=0;i<512;i++)
						EP2FIFOBUF[i] = 0xaa;
					EP2BCH = 0x02;
					EP2BCL = 0x00;
				}
	            // OUT endpoints do NOT come up armed
	            EP6BCL = 0x80; // arm first buffer by writing BC w/skip=1
	            SYNCDELAY;
	            EP6BCL = 0x80; // arm second buffer by writing BC w/skip=1
	        	SYNCDELAY;
			}break;

			default:break;
	    }
	}
    else
    {
	    // Change configuration based upon the Alt. Interface selected 
	    switch (AlternateSetting)
	    {
	        case Full_Alt0_BulkINOUT:
	            // Only using endpoint 2, zero the valid bit on all others
	            // Just using endpoint 2, zero the valid bit on all others
	            EP2CFG = 0xE2; // EP2 is DIR=IN, TYPE=BULK, SIZE=512, BUF=2x
	        	SYNCDELAY;
	            EP6CFG = 0xA2; // EP6 is DIR=OUT, TYPE=BULK, SIZE=512, BUF=2x   
	        	SYNCDELAY;
	            
	            EP1OUTCFG = (EP1OUTCFG & 0x7F);
	        	SYNCDELAY;
	        	EP1INCFG = (EP1INCFG & 0x7F);
	        	SYNCDELAY;
	        	EP4CFG = (EP4CFG & 0x7F);
	        	SYNCDELAY;
	        	EP8CFG = (EP8CFG & 0x7F);
	        	SYNCDELAY;
	
	            // Clear out any committed packets
	            FIFORESET = 0x80;
	            SYNCDELAY;
	            FIFORESET = 0x02;
	            SYNCDELAY;
	            FIFORESET = 0x00;
	            SYNCDELAY;
	
	            // Reset data toggle to 0
	            TOGCTL = 0x12;  // EP2 IN
	            TOGCTL = 0x32;  // EP2 IN Reset
	
	            // OUT endpoints do NOT come up armed
	            EP6BCL = 0x80; // arm first buffer by writing BC w/skip=1
	            SYNCDELAY;
	            EP6BCL = 0x80; // arm second buffer by writing BC w/skip=1
	        	break;
					
	        case Full_Alt1_IntINOUT:
	          	EP2CFG = 0xF2;  // EP2 is DIR=IN, TYPE=INT, SIZE=512, BUF=2x
	            SYNCDELAY;
				EP6CFG = 0xB2;  // EP6 is DIR=OUT, TYPE=INT, SIZE=512, BUF=2x
	            SYNCDELAY;
	
	            EP1OUTCFG = (EP1OUTCFG & 0x7F);
	        	SYNCDELAY;
	        	EP1INCFG = (EP1INCFG & 0x7F);
	        	SYNCDELAY;
	        	EP4CFG = (EP4CFG & 0x7F);
	        	SYNCDELAY;
	           	EP8CFG = (EP8CFG & 0x7F);
	        	SYNCDELAY;
	           
	         	  // Clear out any committed packets
	            FIFORESET = 0x80;
	            SYNCDELAY;
	            FIFORESET = 0x02;
	            SYNCDELAY;
	            FIFORESET = 0x00;
	            SYNCDELAY;
	
	            // Reset data toggle to 0
	            TOGCTL = 0x12;  // EP2 IN
	            TOGCTL = 0x32;  // EP2 IN Reset
	
	            // OUT endpoints do NOT come up armed
	            EP6BCL = 0x80; // arm first buffer by writing BC w/skip=1
	            SYNCDELAY;
	            EP6BCL = 0x80; // arm second buffer by writing BC w/skip=1
	        	SYNCDELAY;
	        	break;
	
	        case Full_Alt2_IsocIN:
	            // Only using endpoint 2, zero the valid bit on all others
	            EP2CFG = 0xDA;  // EP2 is DIR=IN, TYPE=ISOC, SIZE=1024, BUF=4x
	            SYNCDELAY;
	            
	            EP1OUTCFG = EP1INCFG = EP4CFG = EP6CFG = EP8CFG = 0x00; 
	            SYNCDELAY;
	
	            // Clear out any committed packets
	            FIFORESET = 0x80;
	            SYNCDELAY;
	            FIFORESET = 0x02;
	            SYNCDELAY;
	            FIFORESET = 0x00;
	            SYNCDELAY;
				
				for(i=0;i<256;i++)
				{
					EP2FIFOBUF[i]=0xa5;
				}
			
				EP2BCH=0x01;
				SYNCDELAY;
				EP2BCL=0x00;
			 	SYNCDELAY;
				for(i=0;i<256;i++)
				{
					EP2FIFOBUF[i]=i;
				}
			
				EP2BCH=0x01;
				SYNCDELAY;
				EP2BCL=0x00;
			 	SYNCDELAY;	 
	        	break;
	
	        case Full_Alt3_IsocOUT:	        
	            // Only using endpoint 2, zero the valid bit on all others
	            EP1OUTCFG = EP1INCFG = EP4CFG = EP6CFG = EP8CFG = 0x00; 
	            SYNCDELAY;
	            EP2CFG = 0x98;  // EP2 is DIR=OUT, TYPE=ISOC, SIZE=1024, BUF=4x
	            SYNCDELAY;
	
	            // OUT endpoints do NOT come up armed
	            EP2BCL = 0x80; // arm first buffer by writing BC w/skip=1
	            SYNCDELAY;
	            EP2BCL = 0x80; // arm second buffer by writing BC w/skip=1        break;	        
	        	break;	
				
			default:break;	
	    }
	}
   	return(TRUE);            // Handled by user code
}

BOOL DR_GetInterface(void)       // Called when a Set Interface command is received
{
   EP0BUF[0] = AlternateSetting;
   EP0BCH = 0;
   EP0BCL = 1;
   return(TRUE);            // Handled by user code
}

BOOL DR_GetStatus(void)
{
   return(TRUE);
}

BOOL DR_ClearFeature(void)
{
   return(TRUE);
}

BOOL DR_SetFeature(void)
{
   return(TRUE);
}

BOOL DR_VendorCmnd(void)
{

	WORD		addr, len, bc;
	WORD		ChipRev;
	WORD i;

	// Determine I2C boot eeprom device address; addr = 0x0 for 8 bit addr eeproms (24LC00)
	I2C_Addr = SERIAL_ADDR | ((I2CS & 0x10) >> 4); // addr=0x01 for 16 bit addr eeprom (LC65)
	// Indicate if it is a dual byte address part
	DB_Addr = (BOOL)(I2C_Addr & 0x01); //TPM: ID1 is 16 bit addr bit - set by rocker sw or jumper

	switch(SETUPDAT[1])
	{ //TPM handle new commands
		case VR_GETI2C_TYPE:
			*EP0BUF = DB_Addr;
			EP0BCH = 0;
			EP0BCL = 1; // Arm endpoint with # bytes to transfer
			EP0CS |= bmHSNAK; // Acknowledge handshake phase of device request
			break;
			
		case VR_GET_CHIP_REV:
			ChipRev = GET_CHIP_REV();
			*EP0BUF = ChipRev;
			EP0BCH = 0;
			EP0BCL = 1; // Arm endpoint with # bytes to transfer
			EP0CS |= bmHSNAK; // Acknowledge handshake phase of device request
			break;

		case VR_TEST_MEM:
			*EP0BUF = 0x0F; // Fail
			EP0BCH = 0;
			EP0BCL = 1; // Arm endpoint with # bytes to transfer
			EP0CS |= bmHSNAK; // Acknowledge handshake phase of device request
			break;

		case VR_SETI2CADDR:
			I2C_Addr = SETUPDAT[2];
			break;

		case VR_I2C_100:  //0xaa
			I2CTL &= ~bm400KHZ;
			EP0BCH = 0;
			EP0BCL = 0;
			break;

		case VR_I2C_400:  //0xab
			I2CTL |= bm400KHZ;
			EP0BCH = 0;
			EP0BCL = 0;
			break;

		case VR_RENUM:
			*EP0BUF = 7;
			EP0BCH = 0;
			EP0BCL = 1; // Arm endpoint with # bytes to transfer
			EP0CS |= bmHSNAK; // Acknowledge handshake phase of device request
			EZUSB_Delay(1000);
			EZUSB_Discon(TRUE);		// renumerate until setup received
			break;

		case VR_NOSDPAUTO:
         // we want specify our own length for the transfer so
         // disable the automatic length feature of the Setup Data Autopointer
         SUDPTRCTL &= ~bmSDPAUTO;
         EP0BCH = SETUPDAT[7];
         EP0BCL = SETUPDAT[6];
         SUDPTRH = SETUPDAT[3];
         SUDPTRL = SETUPDAT[2];
         break;
			
		case VR_DB_FX:	   //0xa9
			DB_Addr = 0x01;		//TPM: need to assert double byte
			I2C_Addr |= 0x01;	//TPM: need to assert double byte
        // NOTE: This case falls through !

		case VR_RAM:	  //0xa0
		case VR_EEPROM:	  //0xa2
			addr = SETUPDAT[2];		// Get address and length
			addr |= SETUPDAT[3] << 8;
			len = SETUPDAT[6];
			len |= SETUPDAT[7] << 8;

			// Is this an upload command ?
			if(SETUPDAT[0] == VR_UPLOAD)
			{
				while(len)					// Move requested data through EP0IN 
				{							// one packet at a time.
               		while(EP0CS & bmEPBUSY);

					if(len < EP0BUFF_SIZE)
						bc = len;
					else
						bc = EP0BUFF_SIZE;

					// Is this a RAM upload ?
					if(SETUPDAT[1] == VR_RAM)
					{
						for(i=0; i<bc; i++)
							*(EP0BUF+i) = *((BYTE xdata *)addr+i);
					}
					else
					{
						for(i=0; i<bc; i++)
							*(EP0BUF+i) = 0xcd;
						EEPROMRead(addr,(WORD)bc,(WORD)EP0BUF);
					}

					EP0BCH = 0;
					EP0BCL = (BYTE)bc; // Arm endpoint with # bytes to transfer

					addr += bc;
					len -= bc;

				}
			}
			else if(SETUPDAT[0] == VR_DOWNLOAD)		   //////////////////////////////////////////////////////
			{
				IOD &= ~0x20;//PD5 low, disable eeprom wp
				while(len)					// Move new data through EP0OUT 
				{							// one packet at a time.
					// Arm endpoint - do it here to clear (after sud avail)
					EP0BCH = 0;
					EP0BCL = 0; // Clear bytecount to allow new data in; also stops NAKing

					while(EP0CS & bmEPBUSY);

					bc = EP0BCL; // Get the new bytecount

					// Is this a RAM download ?
					if(SETUPDAT[1] == VR_RAM)
					{
						for(i=0; i<bc; i++)
							 *((BYTE xdata *)addr+i) = *(EP0BUF+i);
					}
					else if(DB_Addr)
					{
						 EEPROMWritePage(addr,bc);
						 //EEPROMWrite(addr,bc, (WORD)EP0BUF);
					}

					addr += bc;
					len -= bc;
				}
				IOD |= 0x20;//PD5 high, enable eeprom wp
			}
			break;

		default:
			return(TRUE);	// error; command  not handled 	
			break;				 
	}
	return(FALSE); // no error; command handled OK	
}

//-----------------------------------------------------------------------------
// USB Interrupt Handlers
//   The following functions are called by the USB interrupt jump table.
//-----------------------------------------------------------------------------

// Setup Data Available Interrupt Handler
void ISR_Sudav(void) interrupt 0
{
   Sudav_Happen_Flag=1;  

   GotSUD = TRUE;            // Set flag
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUDAV;         // Clear SUDAV IRQ
}

// Setup Token Interrupt Handler
void ISR_Sutok(void) interrupt 0
{
	Sudav_Happen_Flag=1;  
	EZUSB_IRQ_CLEAR();
	USBIRQ = bmSUTOK;         // Clear SUTOK IRQ
}

void ISR_Sof(void) interrupt 0
{	
	EZUSB_IRQ_CLEAR();
	USBIRQ = bmSOF;            // Clear SOF IRQ
}

void ISR_Ures(void) interrupt 0
{
	if(EZUSB_HIGHSPEED())
   	{
    	pConfigDscr = pHighSpeedConfigDscr;
    	pOtherConfigDscr = pFullSpeedConfigDscr;
   	}
   	else
   	{
    	pConfigDscr = pFullSpeedConfigDscr;
      	pOtherConfigDscr = pHighSpeedConfigDscr;
   	}

   	EZUSB_IRQ_CLEAR();
   	USBIRQ = bmURES;         // Clear URES IRQ
}

void ISR_Susp(void) interrupt 0
{
   	Sleep = TRUE;
   	EZUSB_IRQ_CLEAR();
   	USBIRQ = bmSUSP;
}

void ISR_Highspeed(void) interrupt 0
{
   	Sudav_Happen_Flag=1;  
 	if(EZUSB_HIGHSPEED())
   	{
    	pConfigDscr = pHighSpeedConfigDscr;
    	pOtherConfigDscr = pFullSpeedConfigDscr;
     	// This register sets the number of Isoc packets to send per
      	// uFrame.  This register is only valid in high speed.
      	EP2ISOINPKTS = 0x01;
   	}
   	else
   	{
      	pConfigDscr = pFullSpeedConfigDscr;
      	pOtherConfigDscr = pHighSpeedConfigDscr;
   	}

   	EZUSB_IRQ_CLEAR();
   	USBIRQ = bmHSGRANT;
}

void ISR_int5(void) interrupt INT5_VECT 
{ 
	Switch_Change_FLAG=1;
	EXIF&=~0x80;
} 

void ISR_Ep0ack(void) interrupt 0
{
}
void ISR_Stub(void) interrupt 0
{
}
void ISR_Ep0in(void) interrupt 0
{
}
void ISR_Ep0out(void) interrupt 0
{
}
void ISR_Ep1in(void) interrupt 0
{
}
void ISR_Ep1out(void) interrupt 0
{
}

// ISR_Ep2inout is called on every OUT packet receieved.
// We don't do anything with the data.  We just indicate we are done with the buffer.
void ISR_Ep2inout(void) interrupt 0	 //speed check handled in here
{
	if(EZUSB_HIGHSPEED())
	{
		if(AlternateSetting == Alt7_BulkSpeed)
		{
			//read from ep2
			if(!(EP2468STAT & bmEP2FULL))
			{
				EP2BCH = 0x02;//512 byte
				EP2BCL = 0x00;
			}
		}
	}
   	EZUSB_IRQ_CLEAR();
   	EPIRQ = bmBIT4;
}
void ISR_Ep4inout(void) interrupt 0
{
}
void ISR_Ep6inout(void) interrupt 0
{
	if( EZUSB_HIGHSPEED( ) )
	{
		if(AlternateSetting == Alt7_BulkSpeed)
		{	
			//write to ep6
			if(!(EP2468STAT & bmEP6EMPTY))
			{
				EP6BCL = 0x80;
			}
		}		
	}
	EZUSB_IRQ_CLEAR();
	EPIRQ = bmBIT6;	
}
void ISR_Ep8inout(void) interrupt 0
{
}
void ISR_Ibn(void) interrupt 0
{
}
void ISR_Ep0pingnak(void) interrupt 0
{
}
void ISR_Ep1pingnak(void) interrupt 0
{
}
void ISR_Ep2pingnak(void) interrupt 0
{
}
void ISR_Ep4pingnak(void) interrupt 0
{
}
void ISR_Ep6pingnak(void) interrupt 0
{
}
void ISR_Ep8pingnak(void) interrupt 0
{
}
void ISR_Errorlimit(void) interrupt 0
{
}
void ISR_Ep2piderror(void) interrupt 0
{
}
void ISR_Ep4piderror(void) interrupt 0
{
}
void ISR_Ep6piderror(void) interrupt 0
{
}
void ISR_Ep8piderror(void) interrupt 0
{
}
void ISR_Ep2pflag(void) interrupt 0
{
}
void ISR_Ep4pflag(void) interrupt 0
{
}
void ISR_Ep6pflag(void) interrupt 0
{
}
void ISR_Ep8pflag(void) interrupt 0
{
}
void ISR_Ep2eflag(void) interrupt 0
{
}
void ISR_Ep4eflag(void) interrupt 0
{
}
void ISR_Ep6eflag(void) interrupt 0
{
}
void ISR_Ep8eflag(void) interrupt 0
{
}
void ISR_Ep2fflag(void) interrupt 0
{
}
void ISR_Ep4fflag(void) interrupt 0
{
}
void ISR_Ep6fflag(void) interrupt 0
{
}
void ISR_Ep8fflag(void) interrupt 0
{
}
void ISR_GpifComplete(void) interrupt 0
{
}
void ISR_GpifWaveform(void) interrupt 0
{
}

void EEPROMRead(WORD addr, BYTE length, BYTE xdata *buf)
{
	BYTE		i = 0;
	BYTE		j = 0;
	BYTE xdata 	ee_str[2];

	if(DB_Addr)
		ee_str[i++] = MSB(addr);

	ee_str[i++] = LSB(addr);

	EZUSB_WriteI2C(I2C_Addr, i, ee_str);
	EZUSB_ReadI2C(I2C_Addr, length, buf);
}

void EEPROMWritePage(WORD addr, BYTE length)
{

	BYTE i;
	BYTE xdata Buffer[2];

		 
	Buffer[0] = EP0BUF[62];
	Buffer[1] = EP0BUF[63];

	for(i=62;i>0;i--)
	{
		EP0BUF[i+1] = EP0BUF[i-1];
	}
	EP0BUF[0] =  MSB(addr);
	EP0BUF[1] =  LSB(addr);	 
	
	if(length < 63)
	{
		EZUSB_WriteI2C(I2C_Addr,length+2,(WORD)EP0BUF);
		EZUSB_WaitForEEPROMWrite(I2C_Addr);
	}	
	else
	{
		EZUSB_WriteI2C(I2C_Addr,64,(WORD)EP0BUF);
		EZUSB_WaitForEEPROMWrite(I2C_Addr);	 
		addr = addr + 62;
		EP0BUF[0] = MSB(addr);
		EP0BUF[1] = LSB(addr);
		EP0BUF[2] = Buffer[0];
		EP0BUF[3] = Buffer[1];
		EZUSB_WriteI2C(I2C_Addr,length-60,EP0BUF);
		EZUSB_WaitForEEPROMWrite(I2C_Addr);	
	}
}
