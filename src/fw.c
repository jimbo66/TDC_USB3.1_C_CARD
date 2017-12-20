//-----------------------------------------------------------------------------
//   File:      fw.c
//   Contents:   Firmware frameworks task dispatcher and device request parser
//            source.
//
// indent 3.  NO TABS!
//
// $Revision: 1.00
// $Date: 16/01/25 20:30
// author : jimbo zhang
//   Copyright (c) 1997 AnchorChips, Inc. All rights reserved
//-----------------------------------------------------------------------------
#include <include\fx2.h>
#include <include\fx2regs.h>
#include <include\fx2sdly.h> 

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------
#define DELAY_COUNT   0x9248*8L  // Delay for 8 sec at 24Mhz, 4 sec at 48
#define _IFREQ  48000            // IFCLK constant for Synchronization Delay
#define _CFREQ  48000            // CLKOUT constant for Synchronization Delay

//-----------------------------------------------------------------------------
// Random Macros
//-----------------------------------------------------------------------------
#define   min(a,b) (((a)<(b))?(a):(b))
#define   max(a,b) (((a)>(b))?(a):(b))

  // Registers which require a synchronization delay, see section 15.14
  // FIFORESET        FIFOPINPOLAR
  // INPKTEND         OUTPKTEND
  // EPxBCH:L         REVCTL
  // GPIFTCB3         GPIFTCB2
  // GPIFTCB1         GPIFTCB0
  // EPxFIFOPFH:L     EPxAUTOINLENH:L
  // EPxFIFOCFG       EPxGPIFFLGSEL
  // PINFLAGSxx       EPxFIFOIRQ
  // EPxFIFOIE        GPIFIRQ
  // GPIFIE           GPIFADRH:L
  // UDMACRCH:L       EPxGPIFTRIG
  // GPIFTRIG
  
  // Note: The pre-REVE EPxGPIFTCH/L register are affected, as well...
  //      ...these have been replaced by GPIFTC[B3:B0] registers
  
          

//-----------------------------------------------------------------------------
// Global Variables		   system
//-----------------------------------------------------------------------------
volatile BOOL   GotSUD;
volatile BOOL   Sleep;                  // Sleep mode enable flag
BOOL      Rwuen;
BOOL      Selfpwr;

BYTE   Ls_err;
WORD   pDeviceDscr;   // Pointer to Device Descriptor; Descriptors may be moved
WORD   pDeviceQualDscr;
WORD   pHighSpeedConfigDscr;
WORD   pFullSpeedConfigDscr;   
WORD   pConfigDscr;
WORD   pOtherConfigDscr;   
WORD   pStringDscr; 

WORD	AD_result=0;
WORD 	AD_temp;
BYTE	TJ_out,TK_out;
WORD	  AD[10]={0,0,0,0,0,0,0,0,0,0};
WORD	  AD1[5]={0,0,0,0,0};

//Flags
bit 	FULL_TEST_FLAG 	= 0;
bit     HIGH_TEST_FLAG 	= 0;
bit		LOW_TEST_FLAG	= 0;
bit		CHECK_PWR_FLAG	= 0;
bit 	CHECK_K_FLAG	= 0;
bit		CHECK_J_FLAG	= 0;
bit		FULL_DATA_J		= 0;
bit		FULL_DATA_K		= 0;
bit 	DIS_FLAG		= 0;
bit		CHIRP_J_FLAG	= 0;
bit		CHIRP_K_FLAG	= 0;
bit		CHIRP_J_FLAG_T	= 0;
bit		CHIRP_K_FLAG_T	= 0;
bit		CHIRP_PJ_FLAG	= 0;
bit		CHIRP_PK_FLAG	= 0;
bit		CHECK_PIN_FLAG	= 0;	//2015.11.30
bit		J_STATE			= 0;
bit		K_STATE			= 0;
bit 	Switch_Change_FLAG = 0;  
bit		Sudav_Happen_Flag  = 0;
bit		GetStr_Happen_Flag = 0;
bit   	Led_On_Flag		= 0;
bit 	CC_CTRL_FLAG	= 0;
bit 	RENUM_CMD_FLAG	= 0;
bit 	CC_WITH_U20_FLAG = 0;
bit		CheckOn_Flg			= 1;
bit		RST_VIDEO_FLAG		= 0;
//Global Variables	  User
BYTE xdata  CC_Value		= 0;
BYTE xdata	 ADC_CTRL		= 0;
BYTE xdata  channels		= 0;
BYTE xdata  U3sel_Flag		= 2;
BYTE xdata  Rd_Flag			= 0;
BYTE xdata  RD_SPI_FLAG		= 0;
BYTE xdata  U2sel_Flag		= 0;             //cc adds
BYTE xdata  Timeout_Flag	= 0;   		  //cc adds
BYTE xdata  U2sel_JK_Flag	= 0;      // cc adds
BYTE xdata  Pcb_rev;
BYTE xdata	float_time		= 10;
BYTE xdata	call_USB30		= 0;//no usb3.0
WORD xdata	Tcount=0,OnCount= 0;
WORD xdata	waitTIME		= 1;

//-----------------------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------------------
void SetupCommand(void);
void TD_Init(void);
void TD_Poll(void);
//BOOL TD_Suspend(void);
//BOOL TD_Resume(void);

BOOL DR_GetDescriptor(void);
BOOL DR_SetConfiguration(void);
BOOL DR_GetConfiguration(void);
BOOL DR_SetInterface(void);
BOOL DR_GetInterface(void);
BOOL DR_GetStatus(void);
BOOL DR_ClearFeature(void);
BOOL DR_SetFeature(void);
BOOL DR_VendorCmnd(void);
BYTE GetSubID(void);

void delay(void);
//void Rd_Spi_Info(int cmd,int len,int chip);
WORD GET_AD(unsigned char channel);
WORD average(void);
BYTE GetSubID(void);
//BYTE GetAsmGpio(void);
void Timer0_Init(void);		//10ms

// this table is used by the epcs macro 
const char code  EPCS_Offset_Lookup_Table[] =
{
   0,    // EP1OUT
   1,    // EP1IN
   2,    // EP2OUT
   2,    // EP2IN
   3,    // EP4OUT
   3,    // EP4IN
   4,    // EP6OUT
   4,    // EP6IN
   5,    // EP8OUT
   5,    // EP8IN
};

// macro for generating the address of an endpoint's control and status register (EPnCS)
#define epcs(EP) (EPCS_Offset_Lookup_Table[(EP & 0x7E) | (EP > 128)] + 0xE6A1)

#define SPI_CS_A_1 IOD|=0x01 		//PD0 high
#define SPI_CS_A_0 IOD&=~0x01 	//PD0 low
#define SPI_CS_B_1 IOD|=0x02 		//PD1 high
#define SPI_CS_B_0 IOD&=~0x02 	//PD1 low
#define SPI_SO_1	IOD|=0x04			//PD2 high
#define SPI_SO_0	IOD&=~0x04		//PD2 low
#define SPI_WP_1 IOD|=0x08			//PD3 high
#define SPI_WP_0 IOD&=~0x08			//PD3 low
#define SPI_CLK_1 IOD|=0x10			//PD4 high
#define SPI_CLK_0 IOD&=~0x10 		//PD4 low
#define SPI_SI_1 IOD|=0x20			//PD5 high
#define SPI_SI_0 IOD&=~0x20			//PD5 low
#define SPI_READ_CMD	0x03
#define SPI_RDSR_CMD	0x05
#define SPI_RDID_CMD	0x9F
//-----------------------------------------------------------------------------
// Code
//-----------------------------------------------------------------------------

// Task dispatcher
void main(void)
{
   DWORD  DevDescrLen;
   WORD   IntDescrAddr;
   WORD   ExtDescrAddr;

   DWORD  xdata i,offset,j,p;
   DWORD  xdata led_cnt=0;
   WORD   xdata temp1,temp2;
   BYTE   xdata ok=0,fail=0;

   BYTE xdata OldGPIO=0x03,CurGPIO=0x03;
   BYTE xdata tmp;
 
	Pcb_rev= GetSubID();	//jimbo add	

   // Initialize Global States
 //  Sleep = FALSE;               // Disable sleep mode
   Rwuen = FALSE;               // Disable remote wakeup
   Selfpwr = FALSE;            // Disable self powered
   GotSUD = FALSE;               // Clear "Got setup data" flag

  
  
   // Initialize user device
   TD_Init();



			


	//initial GPIO
	if((Pcb_rev & 0x01) == 0x00)//new card
	{
	//-----set PA  -------
		OEA &= ~0x0F;					//set PA3(ID3)/PA2(ID2)/PA1(ID1)/PA0(ID0) input
		OEA |= 0xF0;					//set PA7(G1)/PA6(HOP)/PA5(ADC_SC)/PA4(Q7_EN)/ output
	
		IOA &= ~0xD0;					//set PA7 low lowspeed test ,G1
										//set PA5 low ADC default switch to GND with 50 ohm
										//set PA4 low The second USB2.0 pair default no connect										
		IOA |= 0x40;					//set PA6 high USB2.0 led off	

	//-----set PB  -------						
		OEB &= ~0xFF;	// input
//		IOB &= ~0xFF;

	//-----set PC  -------	
		OEC &= ~0x0A;						//set PC3(AD_DO)/PC1(AD_BSY) input
		OEC |= 0xF5;						//set PC7(ADC_EN)/PC6(ADC_A2)/PC5(ADC_A1)/PC4(ADC_A0)/PC2(AD_DI)/PC0(AD_CLK) output
		
		IOC &= ~0xF5;						//set PC7 low	ADC_EN low
											//set PC6/5/4 ADC_A2/A1/A0 low
											//set PC2 low AD_DI low	
											//set PC0 lwo AD_CLK low

	//-----set PD  -------
	    OED &= ~0x17;						//set/PD4/PD2/PD1/PD0 input
		OED |= 0xE8;						//set PD7 PD6 PD3 PD5 output 			
											 
		IOD &= ~0xC8;						//set PD6(CP),PD3(ASM1351 GPIO7) low
											//	PD3		<--->	 U3_GPIO7_A/B
											//  PD6    <-->	 CP
											//	PD7		<----> Q8_EN
		IOD |= 0x20;						//	PD5 eeprom wp,enable

											  
	//-----set PE  -------	
		OEE|=0xFF;							//set PE7(U3_SEL_CTL_B) output
											//set PE6(U3_SEL_CTL_A) output
											//set PE5(U3_RST_CTL_B) output
											//set PE4(U3_RST_CTL_A) output
											//set PE3(C0) output
											//set PE2(C1) output
											//set PE1(U3_CC1_CTL) output
											//set PE0(U3_CC2_CTL) output
	
		IOE=(IOE|0xFC)&0xFC;				//set PE7(U3_SEL_CTL_B) high															//attention
											//set PE6(U3_SEL_CTL_A) high
											//set PE5(U3_RST_CTL_B) high
											//set PE4(U3_RST_CTL_A) high
											//set PE3(C0) high
											//set PE2(C1) high
											//set PE1(U3_CC1_CTL) low
											//set PE0(U3_CC2_CTL) low

	  ///CP
	IOD |= 0x40;					///PD6 hight
	EZUSB_Delay(1);
	IOD &= ~0x40;					///PD6 low
			
	}
	else if((Pcb_rev & 0x01) == 0x01)//old card
	{		
	//-----set PA  -------
		OEA &= ~0xD9;						//PA7 input ,PCB1
											//PA6 input ,PCB2
											//PA4 input ,TALK
											//PA3 input ,LOP
											//PA0 input ,PCB_SELECT

		OEA |= 0x26;						//PA5 output ,TEST_I
											//PA2 output ,HOP
											//PA1 output ,test point

		IOA &= ~0x22;						//PA5(TEST_I) low  	
											//PA1(test point) low

		IOA |= 0x04;						//PA2 high (USB2.0 LED OFF)

	//-----set PB  -------						
		OEB &= ~0xFF;						//PB0~PB7 input GPIO

	//-----set PC  -------	
		OEC &= ~0x30;						//PC5,4 input ,G3,G4 not used
		OEC |= 0xCF;						//PC7,6 output ,G1,G2
											//PC3,2,1,0 output ,testpoint
		
		IOC &= ~0xCF;						//low	
	//-----set PD  -------
	    OED &= ~0xC1;						//PD7 input 	,DOUT
											//PD6 input  	,BUSY
											//PD0 input GPIO2
		
		OED |= 0x3E;						//PD5 output ,DIN
											//PD4 output ,CLK
											//PD3,2,1 output testpiont
		
		IOD &= ~0x3E;						//low			
	//-----set PE  -------		
		OEE &= ~0x03;						//PE0,1 input N/C
		OEE |= 0xFC;						//PE7,6,5,4 output  ,testpoint
											//PE3 output ,C0
											//PE2 output ,C1

		IOE |= 0x0C;						//PE3(C0) high
											//PE2(C1) high

		IOE &= ~0xF0; 						//PE7~PE4 (testpoint) low	

	} 
	//header card
	if(Pcb_rev == 0x02)
	{
   		Timer0_Init();
		CheckOn_Flg = 1;
		OnCount = 0;
	}
	//video card
	if(Pcb_rev == 0x04)
	{
		OEB |= 0xA0;
		IOB &= ~0x20;//PB5 is low;RLS_HPD.
		IOB |= 0x80;//PB7 is high; VIDEO_RST
	}

/*********************USB enumerate***************************************/

   // The following section of code is used to relocate the descriptor table. 
   // Since the SUDPTRH and SUDPTRL are assigned the address of the descriptor 
   // table, the descriptor table must be located in on-part memory.
   // The 4K demo tools locate all code sections in external memory.
   // The descriptor table is relocated by the frameworks ONLY if it is found 
   // to be located in external memory.
   pDeviceDscr = (WORD)&DeviceDscr;
   pDeviceQualDscr = (WORD)&DeviceQualDscr;
   pHighSpeedConfigDscr = (WORD)&HighSpeedConfigDscr;
   pFullSpeedConfigDscr = (WORD)&FullSpeedConfigDscr;
   pStringDscr = (WORD)&StringDscr;


   if ((WORD)&DeviceDscr & 0xe000)
   {	
      IntDescrAddr = INTERNAL_DSCR_ADDR;
      ExtDescrAddr = (WORD)&DeviceDscr;
      DevDescrLen = (WORD)&UserDscr - (WORD)&DeviceDscr + 2;
      for (i = 0; i < DevDescrLen; i++)
         *((BYTE xdata *)IntDescrAddr+i) = 0xCD;
      for (i = 0; i < DevDescrLen; i++)
         *((BYTE xdata *)IntDescrAddr+i) = *((BYTE xdata *)ExtDescrAddr+i);
      pDeviceDscr = IntDescrAddr;
      offset = (WORD)&DeviceDscr - INTERNAL_DSCR_ADDR;
      pDeviceQualDscr -= offset;
      pConfigDscr -= offset;
      pOtherConfigDscr -= offset;
      pHighSpeedConfigDscr -= offset;
      pFullSpeedConfigDscr -= offset;
      pStringDscr -= offset;
   }

   EZUSB_IRQ_ENABLE();            // Enable USB interrupt (INT2)
   EZUSB_ENABLE_RSMIRQ();            // Wake-up interrupt

   // What is INT2 is for USB & INT4 is for the Slave FIFOs
   INTSETUP |= (bmAV2EN | bmAV4EN);     // Enable INT 2 & 4 autovectoring

   // I don't think we care about Setup PIDs only the Setup data; commented out
   // bmSUTOK but we want bmSUDAV.
   USBIE |= bmSUDAV | bmSUTOK | bmSUSP | bmURES | bmHSGRANT;   // Enable selected interrupts
   // Global interrupt enable. Controls masking of all interrupts except USB wakeup
   // (resume). EA = 0 disables all interrupts except USB wakeup. When EA = 1, interrupts are
   // enabled or masked by their individual enable bits.
   EA = 1;                  // Enable 8051 interrupts




#ifndef NO_RENUM
   // Renumerate if necessary.  Do this by checking the renum bit.  If it
   // is already set, there is no need to renumerate.  The renum bit will
   // already be set if this firmware was loaded from an eeprom.
   if(!(USBCS & bmRENUM))
   {
       EZUSB_Discon(TRUE);   // renumerate
   }
#endif

   // unconditionally re-connect.  If we loaded from eeprom we are
   // disconnected and need to connect.  If we just renumerated this
   // is not necessary but doesn't hurt anything
   USBCS |= bmDISCON;
   EZUSB_Delay(10);
   USBCS &=~bmDISCON;


   // The three LSBs of the Clock Control Register (CKCON, at SFR location 0x8E) control the stretch
   // value; stretch values between zero and seven may be used. A stretch value of zero adds zero
   // instruction cycles, resulting in MOVX instructions which execute in two instruction cycles.
   CKCON = (CKCON&(~bmSTRETCH)) | FW_STRETCH_VALUE; // Set stretch to 0 (after renumeration)

/**************************************/


   while(TRUE)               // Main Loop
   {
 	/***************New card**************/
		if((Pcb_rev & 0x01) == 0x00)//new card
		{
			//for video card
			if(RST_VIDEO_FLAG)
			{
				RST_VIDEO_FLAG = 0;
				IOB &= ~0x80;//reset pin;PB7
				EZUSB_Delay(2);//CY3014 reset pulse width minimus 1ms
				IOB |= 0x80;
			}
			//for header card
			if(Pcb_rev == 0x02)	 
			{
				if((IOB & 0x21) != 0x21)		//check turn on
				{
					Timer0_Init();
					CheckOn_Flg = 1;
					OnCount = 0;
				}

				if(TF0)
				{
					TF0 = 0;
					TH0 = (65535-40000)/256;
					TL0 = (65535-40000)%256;
					Tcount++;
			
					if(CheckOn_Flg)
					{
						if((Tcount == 100) && (OnCount < 65535))//1s
						{
							OnCount++;	
							Tcount = 0;
						}
					}
				}
			}
			//force modify port type  
			else if((Pcb_rev == 0x0a)&&(led_cnt == 501))			 
			{
				if(Timeout_Flag == 1)
				{
					EZUSB_Delay(waitTIME);
					USBCS |= bmDISCON; //2017.07.18
					EZUSB_Delay(5);

				    IOD &= ~0x80;
					IOA &= ~0x10;
					IOE |= 0x08; 
					Timeout_Flag = 0;
					
					EZUSB_Delay(5);
					USBCS &=~bmDISCON;
				}
				else if(Timeout_Flag == 0)
				{
					EZUSB_Delay(waitTIME);
					USBCS |= bmDISCON;	//2017.07.18
					EZUSB_Delay(5);

					IOD &= ~0x80;
					IOA |= 0x10;
					IOE |= 0x08; 
					Timeout_Flag = 1;
					
					EZUSB_Delay(5);
					USBCS &=~bmDISCON;
				}
				
				led_cnt = 502; 		
			}
			

			if(!GetStr_Happen_Flag & Sudav_Happen_Flag )
	   		{ 
	   			if(led_cnt<500)
   				{
					led_cnt++;
					EZUSB_Delay(10);
   				}
	   			else if(led_cnt == 500)
   				{
					EZUSB_Delay(waitTIME);
					USBCS |= bmDISCON;	//2017.07.18
					EZUSB_Delay(5);

					Timeout_Flag = 1;
					led_cnt=501; 
					IOA|=0x10;		//test set PA4 high U3_U2_CTL set B connect with A 				    //cc adds
					
					EZUSB_Delay(5);
					USBCS &=~bmDISCON;
   				}	   		
	   		}

			if(GetStr_Happen_Flag & (!Led_On_Flag))
			{
				IOA&=~0x40;		//set PA6 low led on
	   			Led_On_Flag=1;
				led_cnt = 501;
			}
			//for AR chip
			if((call_USB30 == 1)&&(GetStr_Happen_Flag==1))	 //usb3.0 A
			{
				call_USB30 = 0;
				//tmp = IOD;
				OED |= 0x08;
				IOD &= ~0x08;//keep U3.0 GPIO7 Low

				EZUSB_Delay(10);
				IOE&=~0x10;						//PE4 U3_RST_CTL_A low	

				EZUSB_Delay(500);				//keep U3.0	 has run correct
			}
			else if((call_USB30 == 2)&&(GetStr_Happen_Flag==1))	   //usb3.0 B
			{
				call_USB30 = 0;
				OED |= 0x08;
				IOD &= ~0x08;//keep U3.0 GPIO7 Low

				EZUSB_Delay(10);
				IOE&=~0x20;						//PE5 U3_RST_CTL_B low	

				EZUSB_Delay(500);				//keep U3.0	 has run correct
			}

			if(CC_CTRL_FLAG)
			{
				CC_CTRL_FLAG=0;
			
				if(CC_Value==0)	//TOP layer
				{
					IOD &= ~0x40;					///PD6 low
					
					IOE|=0x30;						//PE4 U3_RST_CTL_A high	
													//PE5 U3_RST_CTL_B high							
		
					if(CC_WITH_U20_FLAG == 0)//cc pin switch with USB3.0						
					{	
						if(float_time != 0)	//CC pin floating
						{/*There is a motherboard,CC pin must keep floating 3ms
						  *or more when switch layer*/
							IOE |= 0x02;
							IOE&=~0x01;						//PE1 U3_CC1_CTL high
															///PE0 U3_CC2_CTL low		
							EZUSB_Delay(1);		
								///CP	 must place in the last 
							IOD |= 0x40;					///PD6 hight
							EZUSB_Delay(1);
							IOD &= ~0x40;					///PD6 low
				
							EZUSB_Delay(float_time);
						}
									
						IOE&=~0x03;	  ///cc pin
						EZUSB_Delay(1);		
									
						///CP
						IOD |= 0x40;					///PD6 hight
						EZUSB_Delay(1);
						IOD &= ~0x40;					///PD6 low
					}

					U3sel_Flag=0;					///indicate top layer was select	
					tmp = IOD;
					OED |= 0x08;
					IOD &= ~0x08;//keep U3.0 GPIO7 Low

					IOE|=0x40;						//PE6 U3_SEL_CTL_A high	
					IOE&=~0x80;						//PE7 U3_SEL_CTL_B low

					EZUSB_Delay(10);
					IOE&=~0x10;						//PE4 U3_RST_CTL_A low	

					EZUSB_Delay(500);				//keep U3.0	 has run correct
		
				}
				else  if(CC_Value==1)	//bottom layer
				{
					IOD &= ~0x40;					///PD6 low
					
					IOE|=0x30;						//PE4 U3_RST_CTL_A high	
													//PE5 U3_RST_CTL_B high							
						
					if(CC_WITH_U20_FLAG == 0)//cc pin switch with USB3.0						
					{
						if(float_time != 0)
						{
							IOE |= 0x02;
							IOE&=~0x01;						//PE1 U3_CC1_CTL high
															///PE0 U3_CC2_CTL low		
							EZUSB_Delay(1);		
								///CP	 must place in the last 
							IOD |= 0x40;					///PD6 hight
							EZUSB_Delay(1);
							IOD &= ~0x40;					///PD6 low
				
							EZUSB_Delay(float_time);
						}
		
						IOE |= 0x03;///cc pin			
						EZUSB_Delay(1);
						///CP
						IOD |= 0x40;					///PD6 hight
						EZUSB_Delay(1);
						IOD &= ~0x40;					///PD6 low
					}

					U3sel_Flag=1;					///indicate bottom layer was select	
					tmp = IOD;
					OED |= 0x08;
					IOD &= ~0x08;//keep U3.0 GPIO7 Low

					IOE|=0x80;						//PE7 U3_SEL_CTL_B high		 
					IOE&=~0x40;						//PE6 U3_SEL_CTL_A low

					EZUSB_Delay(10);
					IOE&=~0x20;						//PE5 U3_RST_CTL_B low	

					EZUSB_Delay(500);				//keep U3.0	 has run correct
				}		
				else if(CC_Value==3)		        //cc adds
				{
				     if(Timeout_Flag == 0)
					 {
					 	 EZUSB_Delay(waitTIME);
						 USBCS |= bmDISCON;
						 EZUSB_Delay(5);	
	
			             //switch usb2.0 signal to A
			             IOA &= ~0x10;
						 delay();
						 IOD &= ~0x80;
						 delay();												    
						 U2sel_Flag = 0;			
						
			             EZUSB_Delay(5);
						 USBCS &=~bmDISCON;
					 }
					 else if(Timeout_Flag == 1)		//single port
					 {
					 	IOA |= 0x10;       //set PA4  high to connect USB2.0-B with USB2.0-A
						IOD &= ~0x80;	 
						U2sel_JK_Flag = 1; // for JK test use	
					 }

					 if(CC_WITH_U20_FLAG)//switch cc pin with USB2.0
					 {
					 	//reset usb3.0
						IOE|=0x30;						//PE4 U3_RST_CTL_A high	
														//PE5 U3_RST_CTL_B high	
					 	if(float_time != 0)
						{
							IOE |= 0x02;
							IOE&=~0x01;						//PE1 U3_CC1_CTL high
															///PE0 U3_CC2_CTL low		
							EZUSB_Delay(1);		
								///CP	 must place in the last 
							IOD |= 0x40;					///PD6 hight
							EZUSB_Delay(1);
							IOD &= ~0x40;					///PD6 low
				
							EZUSB_Delay(float_time);
						}
									
						IOE&=~0x03;	  ///cc pin
						U3sel_Flag=0;					///indicate top layer was select
						EZUSB_Delay(10);		
									
						///CP
						IOD |= 0x40;					///PD6 hight
						EZUSB_Delay(1);
						IOD &= ~0x40;					///PD6 low
							
						IOE|=0x40;						//PE6 U3_SEL_CTL_A high	
						IOE&=~0x80;						//PE7 U3_SEL_CTL_B low
						
						GetStr_Happen_Flag = 0;	
						call_USB30 = 1;//call usb3.0 A	
					 }
				}
		
				else if(CC_Value==4)	   // cc adds
				{
					//switch usb2.0 signal to B
					 if(Timeout_Flag == 0)
				     {
				   		EZUSB_Delay(waitTIME);
					    USBCS |= bmDISCON;
					    EZUSB_Delay(5);
	
						IOA |= 0x10;
						delay();
						IOD |= 0x80;
						U2sel_Flag = 1;
						
			            EZUSB_Delay(5);
						USBCS &=~bmDISCON;
				    }
				    else if(Timeout_Flag == 1)
				    {
						IOA |= 0x10;   //set PA4 high to connnect USB2.0-B with USB2.0-A
						IOD &= ~0x80;
					   	U2sel_JK_Flag =2;    //for JK test use
				    }

					if(CC_WITH_U20_FLAG)//cc pin switch with USB2.0						
					{
						//reset usb3.0
						IOE|=0x30;						//PE4 U3_RST_CTL_A high	
														//PE5 U3_RST_CTL_B high	
						
						if(float_time != 0)
						{
							IOE |= 0x02;
							IOE&=~0x01;						//PE1 U3_CC1_CTL high
															///PE0 U3_CC2_CTL low		
							EZUSB_Delay(1);		
								///CP	 must place in the last 
							IOD |= 0x40;					///PD6 hight
							EZUSB_Delay(1);
							IOD &= ~0x40;					///PD6 low
				
							EZUSB_Delay(float_time);
						}
		
						IOE |= 0x03;///cc pin		
						U3sel_Flag=1;					///indicate bottom layer was select		
						EZUSB_Delay(1);
						///CP
						IOD |= 0x40;					///PD6 hight
						EZUSB_Delay(1);
						IOD &= ~0x40;					///PD6 low

						IOE|=0x80;						//PE7 U3_SEL_CTL_B high		 
						IOE&=~0x40;						//PE6 U3_SEL_CTL_A low	
						
						GetStr_Happen_Flag = 0;
						call_USB30 = 2;//call usb3.0 B
					}
				}
				
				else if(CC_Value == 0xff)//for debug
				{
					//TODO: debug code
				}
			}

			//---------------pbtest 2015.11.30--------------------
			if(CHECK_PIN_FLAG)
			{
				CHECK_PIN_FLAG=0;
				
				IOC	=(ADC_CTRL & 0xF0)|(IOC & 0x0F) ;   			//set PC7 (ADC_EN)
																									//set	PC4	(ADC_A0)	
				IOA =((ADC_CTRL & 0x01)<<5)|(IOA & 0xDF);			//set PA5 (ADC_SC)			  pull up to 1.2v
																								
				IOE	=((ADC_CTRL & 0x0C)<<4)|(IOE & 0x3F);  		//set PE7(U3_SEL_CTL_B)

				channels=0xD7; 						//Y+ test pb test 2015.10.26
	
				for(i=0;i<10;i++)
				{
					AD[i]=GET_AD(channels);
					for(j=0;j<10;j++);
				}	
				AD_result=average();	
			}
   	}
	else if((Pcb_rev & 0x01) == 0x01)//old card
	{			
		if(GetStr_Happen_Flag & (!Led_On_Flag))
		{
			IOA&=~0x04;		//set PA2 low led on
   			Led_On_Flag=1;
		}
	}
	/****************New or Old card***************/
	if(Switch_Change_FLAG)
	{
//		EZUSB_Delay(waitACK);	//has disconnected in low speed test code
//		USBCS |= bmDISCON; 
//		EZUSB_Delay(5);

		Switch_Change_FLAG=0;
		if((Pcb_rev & 0x01) == 0x00)//new card
		{						//pbtest 2015.11.30
			IOA&=~0x80;					//set PA7 low  G1  low speed test over			
		}
		else if((Pcb_rev & 0x01) == 0x01)//old card
		{			
			IOC&=~0x80;					//set PC7 low  G1  low speed test over			
		}				
	 	EIE&=~0x08;			
		IOE|=0x0c;							//set PE2,PE3 high				 
		CT1&=~0x02;	  //switch to high speed
		EZUSB_Delay(10);
		USBIRQ = 0xff;          //Clear any pending USB interrupt requests.  They're for our old life.
  		EPIRQ = 0xff;
   		EZUSB_IRQ_CLEAR();

		EZUSB_Delay(5);
		USBCS &=~bmDISCON;				  
	}	
   
	if(LOW_TEST_FLAG)
	{
		EZUSB_Delay(waitTIME);
		USBCS |= bmDISCON;
		EZUSB_Delay(5); 

	 	LOW_TEST_FLAG=0; 	
		IOE&=~0x0c;							//set PE2,PE3 low 			
		EIE|=0x08;
		EZUSB_Delay(10);				//2010.12.14
		
		if((Pcb_rev & 0x01) == 0x00)//new card
		{						//pbtest 2015.11.30
				IOA|=0x80;					//set PA7 high  G1  low speed test				
		}
		else if((Pcb_rev & 0x01) == 0x01)//old card
		{			
				IOC|=0x80;					//set PC7 high  G1  low speed test	2010.12.14
		}		

    }
	 
	if(FULL_DATA_J)
    {
	    FULL_DATA_J=0;
		channels=0xC7; 	//channel choose CD+

		for(i=0;i<10;i++)
		{
			AD[i]=GET_AD(channels);
			for(j=0;j<10;j++);
		}
		AD_result=average();		
    }

//-------------------test---09.09.28----------------------
     if(CHIRP_J_FLAG_T)
	 {
	   	EZUSB_Delay(waitTIME);
	   	USBCS |= bmDISCON;
		EZUSB_Delay(5);

		CHIRP_J_FLAG_T=0;
		AD_result=0;
		channels=0;

		if((Pcb_rev & 0x01) == 0x00)//new card
		{					
			if(Timeout_Flag == 1&&U2sel_JK_Flag == 1)	  // cc adds
			{
				IOD &= ~0x80;
				delay();	  
				IOA &= ~0x10;
			}
	
			else if(Timeout_Flag ==1&&U2sel_JK_Flag == 2)  // cc adds
			{
				IOA |= 0x10;
				delay();
				IOD  |= 0x80;	
			}		
		}
	
		EZUSB_Delay(10);
		
		IOE= (IOE|0x04)&~0x08;	//set PE2 high,PE3 low  		//2.04G	 C1 high C0 LOW		
	
		EZUSB_Delay(10);
		for(i=0;i<5;i++){AD[i]=0xffff;AD1[i]=0xffff;}
		i=0;P=0;ok=0;fail=0;
		while(!ok)
		{   
   			p++;
			temp1=0;temp2=0;
			channels=0xC7; 			//channel choose CD+
			temp1=GET_AD(channels);								//configure the ADC7873
			for(j=0;j<50;j++);		//delay
			channels=0xE7; 			//channel choose CD-
			temp2=GET_AD(channels);
					if(temp1<600&&temp1>100&&temp2<300&&i<5){AD[i]=temp1;AD1[i]=temp2;i++;}			//the true level is 400*5000/4096
					else
		 			{
						for(j=0;j<5;j++){AD[j]=0xffff;AD1[j]=0xffff;}
						i=0;
					}

			if(i==5 && CHIRP_PJ_FLAG ==0)ok=1;
			for(j=0;j<300;j++);		//delay
			if(p>100*TJ_out){p=0;ok=1;fail=1;}//timeout	

		}
		if(!fail)
		{
			for(i=0;i<5;i++)
			AD_result+=(AD[i]-AD1[i]);
			AD_result/=5;
		}
		else
		{
 			AD_result=0;
		}
		CHIRP_PJ_FLAG=0;
		TJ_out=0;					 //clear timeout reg

		IOE|=0x0c;				     //set PE2,PE3 high

		if((Pcb_rev & 0x01) == 0x00)//new card
		{
	   		if(Timeout_Flag == 1&&U2sel_JK_Flag == 1)	  // cc adds
			{
				IOA |= 0x10;
			}
			else if(Timeout_Flag ==1&&U2sel_JK_Flag == 2)  // cc adds
			{
				IOD  &= ~0x80;	
			}			
		}
			
		EZUSB_Delay(5);
		USBCS &=~bmDISCON;	
		}

  
	   if(CHIRP_K_FLAG_T)
	   {
		   	EZUSB_Delay(waitTIME);
			USBCS |= bmDISCON;
			EZUSB_Delay(5);
	
			CHIRP_K_FLAG_T=0;
			AD_result=0;
			channels=0;
	
			if((Pcb_rev & 0x01) == 0x00)//new card
			{
				if(Timeout_Flag == 1&&U2sel_JK_Flag == 1)	  // cc adds
				{
					IOD &= ~0x80;
					delay();
					IOA &= ~0x10;
				}
		
				else if(Timeout_Flag ==1&&U2sel_JK_Flag == 2)  // cc adds
				{
					IOA |= 0x10;
					delay();
					IOD  |= 0x80;	
				}
			}
	     	EZUSB_Delay(10);
			
			IOE= (IOE|0x04)&~0x08;	//set PE2 high,PE3 low  		//2.04G	   C1 low C0 high
			
			EZUSB_Delay(10);
			for(i=0;i<5;i++){AD[i]=0xffff;AD1[i]=0xffff;}
			i=0;P=0;ok=0;fail=0;
			while(!ok)
			{   
	   			p++;
				temp1=0;temp2=0;
				channels=0xE7; 			//channel choose CD-
				temp1=GET_AD(channels);
				for(j=0;j<50;j++);		//delay
				channels=0xC7; 			//channel choose CD+
				temp2=GET_AD(channels);	
						if(temp1<600&&temp1>100&&temp2<300&&i<5){AD[i]=temp1;AD1[i]=temp2;i++;}			//the true level is 400*5000/4096
						else
			 			{
							for(j=0;j<5;j++){AD[j]=0xffff;AD1[j]=0xffff;}
							i=0;
						}
	
				if(i==5 && CHIRP_PK_FLAG ==0)ok=1;
				for(j=0;j<300;j++);		//delay
				if(p>100*TK_out){p=0;ok=1;fail=1;}//timeout		
			}

			if(!fail)
			{
				for(i=0;i<5;i++)
				AD_result+=(AD[i]-AD1[i]);
				AD_result/=5;
			}
			else
			{
	 			AD_result=0;
			}
			CHIRP_PK_FLAG=0;
			TK_out=0;					 //clear timeout reg
	
			IOE|=0x0c;				     //set PE2,PE3 high
	
			if((Pcb_rev & 0x01) == 0x00)//new card
			{
				if(Timeout_Flag == 1&&U2sel_JK_Flag == 1)	  // cc adds
				{
					IOA |= 0x10;
				}
	
				else if(Timeout_Flag ==1&&U2sel_JK_Flag == 2)  // cc adds
				{
					IOD  &= ~0x80;
				}
			}

			EZUSB_Delay(5);
			USBCS &=~bmDISCON;		
		}

		/************General***********/
   	
		
		if(RENUM_CMD_FLAG)
		{
			RENUM_CMD_FLAG=0;
			EZUSB_Discon(TRUE);   // renumerate
		}
   	
		if(GotSUD)            			// Wait for SUDAV
     	{
         	SetupCommand();          	// Implement setup command
          	GotSUD = FALSE;             // Clear SUDAV flag
      	}

		if(FULL_TEST_FLAG)
		{
			EZUSB_Delay(waitTIME);
			USBCS |= bmDISCON; 
			EZUSB_Delay(5);

			FULL_TEST_FLAG=0;			 
			CT1|=0x02; 	//switch to full speed
			EZUSB_Delay(10);
			USBIRQ = 0xff;          // Clear any pending USB interrupt requests.  They're for our old life.
	  		EPIRQ = 0xff;
	   		EZUSB_IRQ_CLEAR();

			EZUSB_Delay(5);
			USBCS &=~bmDISCON;	
		}
     	 
		if(HIGH_TEST_FLAG)
		{
			EZUSB_Delay(waitTIME);
			USBCS |= bmDISCON; 
			EZUSB_Delay(5);

			HIGH_TEST_FLAG=0;
			CT1&=~0x02;		//switch to high speed
			EZUSB_Delay(10);
			USBIRQ = 0xff;          // Clear any pending USB interrupt requests.  They're for our old life.
	  		EPIRQ = 0xff;
	   		EZUSB_IRQ_CLEAR();

			EZUSB_Delay(5);
			USBCS &=~bmDISCON;		
		}

		if(FULL_DATA_K)
	    {
		   	FULL_DATA_K=0;
			
			channels=0xE7; 	//channel choose CD-
			for(i=0;i<10;i++)
			{
				AD[i]=GET_AD(channels);
				for(j=0;j<10;j++);
			}
			AD_result=average();
	    }

		if(CHECK_PWR_FLAG)
	    {
			CHECK_PWR_FLAG=0;		
			channels=0xA7; 	//channel choose   BAT

			for(i=0;i<10;i++)
			{
				AD[i]=GET_AD(channels);
				for(j=0;j<10;j++);
			}

			AD_result=average();	 
	    }

		TD_Poll();
	/***********************************/

   }
}

void delay(void)
{
	int i;
	for(i=0;i<10;i++);
}
WORD average(void)
{
	WORD aver,max1,min1;
	int i,j;
	aver=0;
	max1=AD[0];
	j=0;

	for(i=1;i<10;i++)
	{
		 max1=max(max1,AD[i]);
		 if(max1==AD[i])j=i;
 	}
	for(i=0;i<j;i++)
	{
	 	AD[i]=AD[i];
	}
	for(i=j;i<9;i++)
	{	
	 	AD[i]=AD[i+1];
	}


	max1=AD[0];
	j=0;
	for(i=1;i<9;i++)
	{
		 max1=max(max1,AD[i]);
		 if(max1==AD[i])j=i;
 	}
	for(i=0;i<j;i++)
	{
	 	AD[i]=AD[i];
	}
	for(i=j;i<8;i++)
	{	
	 	AD[i]=AD[i+1];
	}


	min1=AD[0];
	j=0;
	for(i=1;i<8;i++)
	{
		 min1=min(min1,AD[i]);
		 if(min1==AD[i])j=i;
 	}
	for(i=0;i<j;i++)
	{
		 AD[i]=AD[i];
	}
	for(i=j;i<7;i++)
	{	
		 AD[i]=AD[i+1];
	}


	min1=AD[0];
	j=0;
	for(i=1;i<7;i++)
	{
		 min1=min(min1,AD[i]);
		 if(min1==AD[i])j=i;
 	}
	for(i=0;i<j;i++)
	{
	 	AD[i]=AD[i];
	}
	for(i=j;i<6;i++)
	{	
	 	AD[i]=AD[i+1];
	}
	for(i=0;i<6;i++)
	{
		aver+=AD[i];
	}
	aver/=6;


	return(aver);
}

 
WORD GET_AD(unsigned char channel)
{
		int i;	
		bit busy_flag;

		busy_flag=0;	
		AD_temp=0;
		if((Pcb_rev & 0x01) == 0x00)//new card
		{
			IOC&=~0x01;//CLK_0;		
			IOC&=~0x04;//DIN_0;
		}
		else if((Pcb_rev & 0x01) == 0x001)//old card
		{
			IOD &= ~0x10;  //CLK_0;	
			IOD &= ~0x20;//DIN_0;
			 	
		}
	
		for(i=0;i<8;i++)			 //configure the AD7873
		{
			 if((Pcb_rev & 0x01) == 0x00)//new card
			{
				IOC&=~0x01;//CLK_0;
			}
			else if((Pcb_rev & 0x01) == 0x001)//old card
			{
				IOD &= ~0x10;  //CLK_0;
				 	
			}		//clk low
			 if((channel<<i)&0x80)
			 { 
			 	if((Pcb_rev & 0x01) == 0x00)//new card
				{
					IOC|=0x04;//DIN_1;
				}
				else if((Pcb_rev & 0x01) == 0x001)//old card
				{
					IOD |= 0x20;//DIN_1;
					 	
				}
			}
			 else 
			 {
			 	if((Pcb_rev & 0x01) == 0x00)//new card
				{
					IOC&=~0x04;//DIN_0;
				}
				else if((Pcb_rev & 0x01) == 0x001)//old card
				{
					IOD &= ~0x20;//DIN_0;	
				}	
			}
			 delay();
			if((Pcb_rev & 0x01) == 0x00)//new card
			{
				IOC|=0x01;//CLK_1;		//clk high
			}
			else if((Pcb_rev & 0x01) == 0x001)//old card
			{
				IOD |= 0x10;  //CLK_1;		//clk high
				 	
			}
		     delay();
		}
		if((Pcb_rev & 0x01) == 0x00)//new card
		{
			IOC&=~0x01;//CLK_0;	
			IOC&=~0x04;//DIN_0;
		}
		else if((Pcb_rev & 0x01) == 0x001)//old card
		{
			IOD &= ~0x10;  //CLK_0;
			IOD &= ~0x20;//DIN_0;	 	
		}
	
		delay();
		if((Pcb_rev & 0x01) == 0x00)//new card
		{
			IOC|=0x01;//CLK_1;		//clk high
		}
		else if((Pcb_rev & 0x01) == 0x001)//old card
		{
			IOD |= 0x10;  //CLK_1;		//clk high
			 	
		}
		delay();
		if((Pcb_rev & 0x01) == 0x00)//new card
		{
			IOC&=~0x01;//CLK_0;
		}
		else if((Pcb_rev & 0x01) == 0x001)//old card
		{
			IOD &= ~0x10;  //CLK_0;
			 	
		}
		for(i=0;i<12;i++)
		{
			if((Pcb_rev & 0x01) == 0x00)//new card
			{
				IOC&=~0x01;//CLK_0;
			}
			else if((Pcb_rev & 0x01) == 0x001)//old card
			{
				IOD &= ~0x10;  //CLK_0;
				 	
			}
			delay();
			if((Pcb_rev & 0x01) == 0x00)//new card
			{
				IOC|=0x01;//CLK_1;		//clk high
			}
			else if((Pcb_rev & 0x01) == 0x001)//old card
			{
				IOD |= 0x10;  //CLK_1;		//clk high
				 	
			}
			AD_temp<<=1;
			if((Pcb_rev & 0x01) == 0x00)//new card
			{	
				if(IOC&0x08)AD_temp+=1; //pbtest  2015.9.15		 PD7 change to PC3
				else AD_temp+=0;
			}
			else if((Pcb_rev & 0x01) == 0x001)//Old card
			{
				if(IOD&0x80)AD_temp+=1;
				else AD_temp+=0;
			}
			delay();
		}
		for(i=0;i<3;i++)
		
		{
			if((Pcb_rev & 0x01) == 0x00)//new card
			{
				IOC&=~0x01;//CLK_0;
			}
			else if((Pcb_rev & 0x01) == 0x001)//old card
			{
				IOD &= ~0x10;  //CLK_0;
				 	
			}	   	
			delay();
			if((Pcb_rev & 0x01) == 0x00)//new card
			{
				IOC|=0x01;//CLK_1;		//clk high
			}
			else if((Pcb_rev & 0x01) == 0x001)//old card
			{
				IOD |= 0x10;  //CLK_1;		//clk high
				 	
			}
			delay();
		}
		
		if((Pcb_rev & 0x01) == 0x00)//new card
		{
			IOC&=~0x01;//CLK_0;	
			IOC&=~0x04;//DIN_0;
		}
		else if((Pcb_rev & 0x01) == 0x01)//old card
		{
			IOD &= ~0x10;  //CLK_0;
			IOD &= ~0x20;//DIN_0;	 	
		}

		AD_temp&=0x0fff;	//the PWR value=AD_result*4*2.5/4096
							//the D+ value=AD_result*2.5*1000/4096*2
		
		return(AD_temp);
}

// Device request parser
void SetupCommand(void)
{
   void   *dscr_ptr;
   unsigned  char i;

   switch(SETUPDAT[1])
   {
   //for video card
   case RLS_HPD:
   		if((Pcb_rev == 0x04) && (SETUPDAT[3] == 0) && (SETUPDAT[2] == 0x00))
			IOB &= ~0x20;
		else if((Pcb_rev == 0x04) && (SETUPDAT[3] == 0) && (SETUPDAT[2] == 0x01))
			IOB |= 0x20;
		break;
	case GET_RSLT:
		if(Pcb_rev == 0x04)
		{
			 EP0BUF[0] = IOB & 0x1f;//get video check result.
		}
		else
		{
			 EP0BUF[0] = IOB;			
		}
		EP0BCH = 0;
		EP0BCL = 1; 
		break;
	case RST_VIDEO:
		if(Pcb_rev == 0x04)
			RST_VIDEO_FLAG = 1;
		break;

   //for header card
   case GET_TURNON_TIME:
	 	if((SETUPDAT[3] == 0) && (SETUPDAT[2] == 0x06))
		{
			CheckOn_Flg = 0;
			TR0 = 0;
			EP0BUF[0] = OnCount >> 8;
			EP0BUF[1] = OnCount;
			EP0BCH = 0;
			EP0BCL = 2; 
		} 
		break;
    //for AR chip
	case Def_CCSW_TIME:	 //0xc0
		float_time = SETUPDAT[2];
		break;  
	case SWITCH_CC_CFG:
		if(SETUPDAT[2] == 0 && SETUPDAT[3] == 0)
			CC_WITH_U20_FLAG = 0;//cc pin switch with USB2.0
		else if(SETUPDAT[2] == 1 && SETUPDAT[3] == 0)
			CC_WITH_U20_FLAG = 1;
		break;  
   //pausebreak add
    case SET_WAIT_TIME:
		waitTIME = (SETUPDAT[3] << 8) | SETUPDAT[2];		
		break;
	case PCB_CS:  //0xbd
		if((Pcb_rev & 0x01) == 0x00)//new card
		{
			// SPI_WP_1;				//PD3 high
			 if(Rd_Flag == 0x01)
			 {
			 	if(Pcb_rev == 0x0a)
					EP0BUF[0] = 0x0e;
				else
			 		EP0BUF[0] = Pcb_rev;
	            EP0BCH = 0;
	            EP0BCL = 1;
			 }
			 else if(Rd_Flag ==0x02)
			 {
				EP0BUF[0] = U3sel_Flag;
	         	EP0BCH = 0;
	       	    EP0BCL = 1;
			 }
			 else if(Rd_Flag ==0x03 || Rd_Flag ==0x04)	//delete
			 {
		      EP0BCH = 0;
	       	  EP0BCL = 32;	 		
			 }
			  else if(Rd_Flag == 0x07)
			 {
			  EP0BUF[0] = U2sel_Flag;
			  EP0BCH = 0;
			  EP0BCL = 1;
			 }
			 else if(Rd_Flag == 0x08)
			 {
			 	EP0BUF[0] = Timeout_Flag;
			  	EP0BCH = 0;
			  	EP0BCL = 1;
			 }
			 Rd_Flag =0;
		}
		else if((Pcb_rev & 0x01) == 0x01)//old card
		{
			EP0BUF[0] = Pcb_rev & 0x01;
            EP0BCH = 0;
            EP0BCL = 1;
		}
		 break;
	case SET_FULL_TEST:
		   FULL_TEST_FLAG=1;
		 break;
	case SET_HIGH_TEST:
		   HIGH_TEST_FLAG=1;
		 break;
	case SET_LOW_TEST:
			LOW_TEST_FLAG=1;
		 break;
	case CHECK_PWR:
			CHECK_PWR_FLAG=1;
		 break;
 	case CHECK_J:
			CHIRP_J_FLAG=1;

		 break;
 	case CHECK_K:
			CHIRP_K_FLAG=1;
		 break;

//----------test-----09.09.28--13:47------
	case READ_DATA_T:
		 	//for(i=0;i<10;i++)
			for(i=0;i<5;i++)
			{
			 EP0BUF[i*4] 	 = AD[i]>>8;
			 EP0BUF[i*4+1] 	 = AD[i];
 			 EP0BUF[i*4+2] 	 = AD1[i]>>8;
			 EP0BUF[i*4+3] 	 = AD1[i];
			}		
			 EP0BCH = 0;
			 EP0BCL = 0x14;		//20 bytes
         //    EP0BCL = 0x28;		//40 bytes
		 break;
 	case CHECK_J_T:		  //0xbe
			CHIRP_J_FLAG_T=1;
			TJ_out=SETUPDAT[2] ;
			CHIRP_PJ_FLAG=SETUPDAT[3]&0x01 ;
		 break;
 	case CHECK_K_T:		//0xbf
			CHIRP_K_FLAG_T=1;
			TK_out=SETUPDAT[2] ;
			CHIRP_PK_FLAG=SETUPDAT[3]&0x01 ;
		 break;
//----------------------------------------
	case CHECK_FULL_J:
	       FULL_DATA_J=1;
		   //JK_A_OR_B_Flag = SETUPDAT[2];// cc adds

		 break;
	case CHECK_FULL_K:
			FULL_DATA_K=1;
		 break;
	case VOLT_DATA_UP:
		 	 EP0BUF[0] = AD_result>>8;
             EP0BUF[1] = AD_result;
             EP0BCH = 0;
             EP0BCL = 2;			//2 bytes
		 break;
	case  RENUM_CMD:
			RENUM_CMD_FLAG=1;
		 break;
	case  U30_CTRL:	//0xbb	//pbtest 2015.09.16
			CC_CTRL_FLAG=1;
			CC_Value=SETUPDAT[2] ;
			Rd_Flag=SETUPDAT[3] ;
		  break;

//----------test-----15.11.30-------------
	case CHECK_PIN:
			CHECK_PIN_FLAG=1;
			ADC_CTRL=SETUPDAT[2] ;
		 break;
   	case SC_GET_DESCRIPTOR:                  // *** Get Descriptor
         if(DR_GetDescriptor())
            switch(SETUPDAT[3])         
            {
               case GD_DEVICE:            // Device
                  SUDPTRH = MSB(pDeviceDscr);
                  SUDPTRL = LSB(pDeviceDscr);
                  break;
               case GD_DEVICE_QUALIFIER:            // Device Qualifier
                  SUDPTRH = MSB(pDeviceQualDscr);
                  SUDPTRL = LSB(pDeviceQualDscr);
                  break;
               case GD_CONFIGURATION:         // Configuration
                  SUDPTRH = MSB(pConfigDscr);
                  SUDPTRL = LSB(pConfigDscr);
                  break;
               case GD_OTHER_SPEED_CONFIGURATION:  // Other Speed Configuration
                  SUDPTRH = MSB(pOtherConfigDscr);
                  SUDPTRL = LSB(pOtherConfigDscr);
                  break;
               case GD_STRING:            // String
                  if(dscr_ptr = (void *)EZUSB_GetStringDscr(SETUPDAT[2]))
                  {
                     SUDPTRH = MSB(dscr_ptr);
                     SUDPTRL = LSB(dscr_ptr);
					 GetStr_Happen_Flag= 1;
                  }
                  else 
                     EZUSB_STALL_EP0();   // Stall End Point 0
                  break;
               default:            // Invalid request
                  EZUSB_STALL_EP0();      // Stall End Point 0
            }
         break;
      case SC_GET_INTERFACE:                  // *** Get Interface
         DR_GetInterface();
         break;
      case SC_SET_INTERFACE:                  // *** Set Interface
         DR_SetInterface();
         break;
      case SC_SET_CONFIGURATION:               // *** Set Configuration
         DR_SetConfiguration();
         break;
      case SC_GET_CONFIGURATION:               // *** Get Configuration
         DR_GetConfiguration();
         break;
      case SC_GET_STATUS:                  // *** Get Status
         if(DR_GetStatus())
            switch(SETUPDAT[0])
            {
               case GS_DEVICE:            // Device
                  EP0BUF[0] = ((BYTE)Rwuen << 1) | (BYTE)Selfpwr;
                  EP0BUF[1] = 0;
                  EP0BCH = 0;
                  EP0BCL = 2;
                  break;
               case GS_INTERFACE:         // Interface
                  EP0BUF[0] = 0;
                  EP0BUF[1] = 0;
                  EP0BCH = 0;
                  EP0BCL = 2;
                  break;
               case GS_ENDPOINT:         // End Point
                  EP0BUF[0] = *(BYTE xdata *) epcs(SETUPDAT[4]) & bmEPSTALL;
                  EP0BUF[1] = 0;
                  EP0BCH = 0;
                  EP0BCL = 2;
                  break;
               default:            // Invalid Command
                  EZUSB_STALL_EP0();      // Stall End Point 0
            }
         break;
      case SC_CLEAR_FEATURE:                  // *** Clear Feature
         if(DR_ClearFeature())
            switch(SETUPDAT[0])
            {
               case FT_DEVICE:            // Device
                  if(SETUPDAT[2] == 1)
                     Rwuen = FALSE;       // Disable Remote Wakeup
                  else
                     EZUSB_STALL_EP0();   // Stall End Point 0
                  break;
               case FT_ENDPOINT:         // End Point
                  if(SETUPDAT[2] == 0)
                  {
                     *(BYTE xdata *) epcs(SETUPDAT[4]) &= ~bmEPSTALL;
                     EZUSB_RESET_DATA_TOGGLE( SETUPDAT[4] );
                  }
                  else
                     EZUSB_STALL_EP0();   // Stall End Point 0
                  break;
            }
         break;
      case SC_SET_FEATURE:                  // *** Set Feature
         if(DR_SetFeature())
            switch(SETUPDAT[0])
            {
               case FT_DEVICE:            // Device
                  if(SETUPDAT[2] == 1)
                     Rwuen = TRUE;      // Enable Remote Wakeup
                  else if(SETUPDAT[2] == 2){
				     // Set Feature Test Mode.  The core handles this request.  However, it is
                     // necessary for the firmware to complete the handshake phase of the
                     // control transfer before the chip will enter test mode.  It is also
                     // necessary for FX2 to be physically disconnected (D+ and D-)
                     // from the host before it will enter test mode.
					 
                     break;}
                  else
                     EZUSB_STALL_EP0();   // Stall End Point 0
                  break;
               case FT_ENDPOINT:         // End Point
                  *(BYTE xdata *) epcs(SETUPDAT[4]) |= bmEPSTALL;
                  break;
            }
         break;
	

      default:                     // *** Invalid Command
         if(DR_VendorCmnd())
            EZUSB_STALL_EP0();            // Stall End Point 0
   }

   // Acknowledge handshake phase of device request
   EP0CS |= bmHSNAK;
}

// Wake-up interrupt handler
void resume_isr(void) interrupt WKUP_VECT
{
   EZUSB_CLEAR_RSMIRQ();
}

//get sub card ID
BYTE GetSubID(void)
{
	BYTE tmp=0x0f;
	BYTE regtmp;

	regtmp = OEA;

 	OEA &= ~0x0f;//set PA0~PA3 input	
	tmp &= IOA;

	OEA = regtmp;
	return tmp; 
}
/*********add for header card******************/
void Timer0_Init(void)		//10ms
{
	TMOD = ((TMOD & 0xF1) | 0x01);
	T0M	=	0;		//0:use CLKOUT/12 as clk source  1:use CLKOUT/4 as clk source 
	TH0 = (65535-40000)/256;
	TL0 = (65535-40000)%256;	
	TR0 = 1;
	Tcount = 0;
}
