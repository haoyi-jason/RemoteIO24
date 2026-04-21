#include "ch.h"
#include "hal.h"
#include "dev_bc3601.h"
#include "bc3601_def.h"
#include "bc3601.h"
#include <string.h>
#include "rf_parameters.h"
#include "bc3603_cmd_reg.h"

#define  _IRQ_ENABLE_   (1)
#define _IRQ_LINE_  (1)		
//       		 BC3601 IRQ for GPIO1/2/3/4
//          <0=> GPIO1
//          <1=> GPIO2
//          <2=> GPIO3
//          <3=> GPIO4
#define _IRQ_POLARITY_ (0)
#define  _LIRC_ENABLE_   (0)
#define  _BC3601_REGS_IRQ1_   (0x04)
#define  _BC3601_REGS_ARK1_   (82)
#define  _BC3601_REGS_ARK2_   (20)

enum
{
	_TX_SEL_ = 0,
	_RX_SEL_
};

uint8_t   rf_power_value[][5] = 
{
	//   0,   5,  10,  13,  17
  /* BC3601 */
//	{ 0x09,0x27,0x3C,0x46,0x85 },		//315 
//	{ 0x08,0x1A,0x3C,0x49,0x8A },		//433
//	{ 0x08,0x19,0x3A,0x45,0x82 },		//470 no gived and copy from 433
//	{ 0x06,0x17,0x3A,0x47,0x7A },		//868
//	{ 0x16,0x27,0x3B,0x47,0x95 }		//915
  
  /* BC3603*/
  	/*0dBm,10dBm,13dBm,17dBm,19dBm */
	{ 0x18, 0x29, 0x32, 0x3B, 0x3B },		/* 315MHz */
	{ 0x18, 0x27, 0x30, 0x37, 0x3B },		/* 433MHz */
	{ 0x16, 0x2B, 0x33, 0x39, 0x3C },		/* 470MHz */
	{ 0x20, 0x2F, 0x35, 0x3A, 0x3D },		/* 868MHz */
	{ 0x16, 0x2B, 0x33, 0x3A, 0x3D }	        /* 915MHz */
};

//uc8		margin_len[] = { 4, 8, 16, 32};
uint8_t	rx_preamble_value[] = { 0x00,0x01,0x02,0x03 };
uint8_t sync_width_value[] = { 0x04,0x08,0x0c };											
uint8_t sync_word_address[] = { 0x47,0x5C,0x58,0xCC,0x73,0x34,0x5E,0x72};
uint8_t sync_word_buff[10];
uint8_t pn9_data[] = 
{
  0xFF,0x83,0xDF,0x17,0x32,0x09,0x4E,0xD1, 
	0xE7,0xCD,0x8A,0x91,0xC6,0xD5,0xC4,0xC4,
	0x40,0x21,0x18,0x4E,0x55,0x86,0xF4,0xDC,
	0x8A,0x15,0xA7,0xEC,0x92,0xDF,0x93,0x53,
	0x30,0x18,0xCA,0x34,0xBF,0xA2,0xC7,0x59,
	0x67,0x8F,0xBA,0x0D,0x6D,0xD8,0x2D,0x7D,
	0x54,0x0A,0x57,0x97,0x70,0x39,0xD2,0x7A,
	0xEA,0x24,0x33,0x85,0xED,0x9A,0x1D,0xE0,
};

//#define RF_SPI_line     3
const unsigned short IRQIO_REGS_TABLE[]=
{
	((	IO2_REGS	<< 8)	+	0x00	),	//b[3:0]=GIO3S[3:0]
#if (RF_SPI_line!=3)
	((	IO1_REGS	<< 8)	+	0x69	),	//b[7:6]=PADDS[1:0],b[5:3]=GIO2S[2:0],b[2:0]=GIO1S[2:0]
	((	IO3_REGS	<< 8)	+	0xF9	),	//b7=SDO_TEN,b6=SPIPU,b[5:4]=reserved,b[3:1]=GIOPU[3:1],b0=reserved
#else
	((	IO1_REGS	<< 8)	+	0x45	),	//b[7:6]=PADDS[1:0],b[5:3]=GIO2S[2:0],b[2:0]=GIO1S[2:0]
#endif
	((	IRQ1_REGS	<< 8)	+	0x0D	),	//b7=RXTO, b6=RXFFOW, b4=RXCRCF, b[3:2]=RXDETS[1:0], b1=IRQCPOR, b0=IRQPOR
	((	IRQ2_REGS	<< 8)	+	0x13	),	//b7=ARKTFIE, b6=ARTCIE, b5=FIFOLTIE, b4=RXERRIE, b3=RXDETIE, b2=CALCMPIE, b1=RXCMPIE, b0=TXCMPIE
	((	IRQ3_REGS	<< 8)	+	0x13	),	//b7=ARKTFIF, b6=ARTCIF, b5=FIFOLTIF, b4=RXERRIF, b3=RXDETIF, b2=CALCMPIF, b1=RXCMPIF, b0=TXCMPIF
};

#define	RF_SYNCLEN_			((RF_SYNC_Length-1)>>1)
#define	RF_SYNCLENLB_			((RF_SYNC_Length-1)&0x01)
#define	RF_WHTFMT1_			((RF_whitening_Format)>>1)
#define	RF_WHTFMT0_			((RF_whitening_Format)&0x01)

#define	RF_TX_PREAMBLE_PATTERN_H	((RF_TX_PREAMBLE_PATTERN>>8)&0xFF)
#define	RF_TX_PREAMBLE_PATTERN_L	((RF_TX_PREAMBLE_PATTERN)&0xFF)

#define	RF_CRC_SEED_H			((RF_CRC_SEED>>8)&0xFF)
#define	RF_CRC_SEED_L			((RF_CRC_SEED)&0xFF)


#define	RF_PLHEA			((RF_PLHA_Address>>8)&0xFF)
#define	RF_PLHA				((RF_PLHA_Address)&0x3F)




//	BC3603 packet format Configure	//
/* CRC_EN=1, PLLEN_EN=0, PAYLOAD = 16-byte */
const unsigned short PACKET_REGS_TABLE[]=
{
	((	FIFO1_REGS	<< 8)	+	0x00	),	//b[5:0]=TXFFSA[5:0]
	((	FIFO2_REGS	<< 8)	+	0x01	),	//b4=RXPL2F_EN,b3=FFINF_EN,b2=FFMG_EN,b[1:0]=FFMG[1:0]
	((	PKT1_REGS	<< 8)	+	RF_TxPreamble_Length	),	//TXPMLEN[7:0] (+1)
	((	PKT2_REGS	<< 8)	+	((RF_Trailer_EN<<5)|(RF_WHTFMT0_<<4)|(RF_SYNCLEN_<<2))),	//b[7:6]=PID[1:0],b[5]=TRAILER_EN,b[4]=WHTFMT[0],b[3:2]=SYNCLEN[1:0],b[1:0]=reserved
//	((	PKT2_REGS	<< 8)	+	(0x08|(RF_Trailer_EN<<5))	),
	#if	RF_ARK_Enable == 1
	((	PKT3_REGS	<< 8)	+	(0x00|(RF_Manchester_EN<<7)|(RF_FEC_EN<<6)|(RF_ARK_Enable<<5)|(RF_CRC_Format<<4)|(RF_ARK_Enable<<3)|(RF_PLHAC_EN<<2)|(RF_Header_Length<<1)|RF_ARK_Enable)	),	//b7=MCH_EN, b6=FEC_EN, b5=CRC_EN, b4=CRCFMT, b3=PLLEN_EN, b2=PLHAC_EN, b1=PLHLEN, b0=PLH_EN
	#else
	((	PKT3_REGS	<< 8)	+	(0x00|(RF_Manchester_EN<<7)|(RF_FEC_EN<<6)|(RF_CRC_EN<<5)|(RF_CRC_Format<<4)|(RF_PLLEN_EN<<3)|(RF_PLHAC_EN<<2)|(RF_Header_Length<<1)|RF_Header_EN)	),	//b7=MCH_EN, b6=FEC_EN, b5=CRC_EN, b4=CRCFMT, b3=PLLEN_EN, b2=PLHAC_EN, b1=PLHLEN, b0=PLH_EN
	#endif
	((	PKT4_REGS	<< 8)	+	(0x00|(RF_whitening_EN<<7)|RF_whitening_Seed)	),	//b7=WHT_EN, b[6:0]=WHTSD[6:0]
	((	PKT5_REGS	<< 8)	+	RF_Payload_Length	),	//TXDLEN
	#if RF_ARK_Enable==1
	((	PKT6_REGS	<< 8)	+	0x00	),				//RXDLEN
	#else
	((	PKT6_REGS	<< 8)	+	RF_Payload_Length	),	//RXDLEN
	#endif
	((	PKT7_REGS	<< 8)	+	0x27	),	//b[7:6]=RXPID[1:0], b[5:3]=DLY_RXS[2:0], b[2:0]=DLY_TXS[2:0]
	((	PKT8_REGS	<< 8)	+	RF_PLHA	),	//b[5:0]=PLHA[5:0]
	((	PKT9_REGS	<< 8)	+	RF_PLHEA	),	//b[7:0]=PLHEA[7:0]
	((	PKT10_REGS	<< 8)	+	((RF_WHTFMT1_<<6)|(RF_CRC_BYTE_ORDER<<5)|(RF_CRC_BIT_ORDER<<4)|(RF_CRC_INVERTED<<3)|(RF_SYNCLENLB_<<2)|(RF_TX_PREAMBLE_CLASS<<1)|(RF_Preamble_Pattern_EN))),//b[7:]=reserved,b[6]=WHTFMT[1],b[5]=CRCBYTEO,b[4]=CRCBITO,b[3]=CRCINV,b[2]=SYNCLENLB,b[1]=PMLPLEN,b[0]=PMLP_EN
	((	PKT11_REGS	<< 8)	+	RF_TX_PREAMBLE_PATTERN_L	),	//b[7:0]=PMLPAT[7:0]
	((	PKT12_REGS	<< 8)	+	RF_TX_PREAMBLE_PATTERN_H	),	//b[7:0]=PMLPAT[15:8]
	((	PKT13_REGS	<< 8)	+	RF_CRC_SEED_L	),	//b[7:0]=CRCSD[7:0]
	((	PKT14_REGS	<< 8)	+	RF_CRC_SEED_H	),	//b[7:0]=CRCSD[15:8]
	((	PKT15_REGS	<< 8)	+	RF_CRC_SEED_H	),	//b[7:6]=WHTSD[8:7] 
	((	XO1_REGS	<< 8)	+	(RF_CYL_COARSE_TUNE_<<6|RF_CYL_FINE_TUNE_)),	//b[7:6]=XSHIFT[1:0] b[4:0]= XO_TRIM[4:0]
};

const unsigned short COMMON_REGS_TABLE[]=
{
	((	CFG1_REGS	<< 8)	+	0x40	),	//b[5:0]=TXFFSA[5:0]
};


#define		ATR_Cycle			(((RF_ATR_Cycle*1024L)/125)-1)
#define		ATR_Cycle_low		(ATR_Cycle&0xff)
#define		ATR_Cycle_high		((ATR_Cycle>>8)&0xff)
#define		ATR_RX_Window		(RF_ATR_RX_Window/250L)
#define		ATR_RX_Window_low	(ATR_RX_Window&0xff)
#define		ATR_RX_Window_high	((ATR_RX_Window>>8)&0x07)
#define		ATR_RX_extra		(RF_ATR_RX_extra*4L)
#define		ATR_RX_extra_low	(ATR_RX_extra&0xff)
#define		ATR_RX_extra_high	((ATR_RX_extra>>8)&0xff)
#define		ARK_RX_Window		(RF_ARK_RX_Window/250L)



//	BC3603 Bank0 Configure	//
const unsigned short BANK0_REG_TABLE[]=
{
	((	ATR1_REGS	<< 8)	+	0x42	),	                //_ATR1_		ATRCLK = 8192Hz / WOR
	((	ATR2_REGS	<< 8)	+	ATR_Cycle_low	),	        //_ATR2_
	((	ATR3_REGS	<< 8)	+	ATR_Cycle_high	),	        //_ATR3_	ATR Cycle = 0x8000 * 0.125 = 4s
	((	ATR4_REGS	<< 8)	+	ATR_RX_Window_low	),	//_ATR4_	RX cycle = (0x25+1) * 250us = 9.5ms
	((	ATR5_REGS	<< 8)	+	ATR_RX_extra_low	),	//_ATR5_	
	((	ATR6_REGS	<< 8)	+	ATR_RX_extra_high	),	//_ATR6_	RX extra time = (0x3E7+1) * 250us = 250ms
	((	ATR7_REGS	<< 8)	+	(((0x02|((RF_ARK_Resend_Count)<<4)))|RF_ARK_Enable	)),	//_ATR7_
	((	ATR8_REGS	<< 8)	+	(ARK_RX_Window&0xFF)		),	//_ATR8_	RX cycle = (ARK_RX_Window+1) * 250us = ms
	((	ATR11_REGS	<< 8)	+	ATR_RX_Window_high		),
	((	XO1_REGS	<< 8)	+	0x20	),	//
	((	XO2_REGS	<< 8)	+	0x03	),	//
};
//	BC3603 Bank1 Configure	//
const unsigned short BANK1_REG_TABLE[]=
{
	((	AGC2_REGS	<< 8)	+	0x14	),	//
};

// Bank2		
uint8_t   Analog_RegisterTable[][2] = 
{  {0x2A,0x57},{0x3A,0x94},{0x34,0xD0}};     


//---BC3603---
const uint16_t BANK2_REG_TABLE[]={
#if	RF_BAND==_915MHz_	
	((	RXG_REGS	<< 8)	+	0X97	),
	((	RX1_REGS	<< 8)	+	0x68	),	
#if	(RF_Datarate==_125kbps_	||RF_Datarate==_250kbps_)
	((	RX2_REGS	<< 8)	+	0x96	),
#else
	((	RX2_REGS	<< 8)	+	0x86	),
#endif
	((	TX3_REGS	<< 8)	+	0x41	),
	((	CA1_REGS	<< 8)	+	0x90	),
	((	LDC1_REGS	<< 8)	+	0x9C	),	
#endif	
	((	0x3B	   << 8)	+	0x5D	),
};
/* Crystal = 16MHz */ 

//uint32_t data_rate_Table16[] =
//{
//	2000,5000,10000,25000,50000,125000,250000
//};
uint32_t data_rate_Table16[] =
{
	2000,10000,50000,125000,250000
};

#define DEFAULT_RX_Preamble _RX_Preamble_4B_
uint8_t MOD_RegisterTable16[][8]=
{
 /* MOD1 MOD2 MOD3     Crystal 16MHz    */
   {0xF9,0x60,0x67},   /*data rate = 2K */	//h=8
   {0x31,0x60,0x66},   /*data rate = 10K */
   {0x09,0x60,0x67},   /*data rate = 50K */	//h=0.75
   {0x03,0x90,0x9A},   /*data rate = 125K */ 
   {0x01,0x90,0x9A}    /*data rate = 250K */
};

#if ((DEFAULT_RX_Preamble ==_RX_Preamble_0B_) || (DEFAULT_RX_Preamble ==_RX_Preamble_1B_))
uint8_t DM_RegisterTable16[][8]=
{
 /*  DM1  DM2  DM3  DM4  DM5  DM6  DM7  DM8     Crystal 16MHz    */
   {0x31,0x09,0xE6,0x08,0x1F,0x40,0x66,0x05},   /*data rate = 2K */
//   {0x13,0x09,0xE6,0x08,0x1F,0x40,0x66,0x0D},   /*data rate = 5K */
   {0x09,0x09,0xE6,0x08,0x1F,0x00,0x66,0x1A},   /*data rate = 10K */
//   {0x07,0x04,0xE6,0x08,0x1F,0x00,0x66,0x20},   /*data rate = 25K */
   {0x13,0x00,0xE0,0x08,0x3A,0x40,0x66,0x0D},   /*data rate = 50K */
   {0x07,0x00,0xE0,0x08,0x3A,0x00,0x9A,0x20},   /*data rate = 125K */
   {0x03,0x00,0xE0,0x08,0x3A,0x00,0x9A,0x40}    /*data rate = 250K */
};
#elif (DEFAULT_RX_Preamble ==_RX_Preamble_2B_)
uint8_t DM_RegisterTable16[][8]=
{
 /*  DM1  DM2  DM3  DM4  DM5            DM8     Crystal 16MHz    */
   {0x31,0x09,0xE6,0x08,0x1F,0x40,0x66,0x05},   /*data rate = 2K */
   {0x09,0xC9,0xE6,0x08,0x1F,0x00,0x66,0x1A},   /*data rate = 10K */
   {0x13,0xC0,0xE0,0x08,0x30,0x40,0x66,0x0D},   /*data rate = 50K */
   {0x07,0xC0,0xE0,0x08,0x30,0x00,0x9A,0x20},   /*data rate = 125K */
   {0x03,0x40,0x40,0x08,0x30,0x00,0x9A,0x40}    /*data rate = 250K */
};
#elif (DEFAULT_RX_Preamble ==_RX_Preamble_4B_)
uint8_t DM_RegisterTable16[][8]=
{
 /*  DM1  DM2  DM3  DM4  DM5            DM8     Crystal 16MHz    */
   {0x31,0xC9,0xE6,0x48,0x1A,0x40,0x66,0x05},   /*data rate = 2K */
//   {0x13,0xC9,0xE6,0x48,0x1A,0x40,0x66,0x0D},   /*data rate = 5K */
   {0x09,0xC9,0xE6,0x48,0x1A,0x00,0x66,0x1A},   /*data rate = 10K */
//  {0x07,0xC4,0xE6,0x48,0x1A,0x00,0x66,0x20},   /*data rate = 25K */
   {0x13,0xC0,0xE0,0x48,0x30,0x40,0x66,0x0D},   /*data rate = 50K */
   {0x07,0xC0,0xE0,0x48,0x30,0x00,0x9A,0x20},   /*data rate = 125K */
   {0x03,0xC0,0xE0,0x48,0x30,0x00,0x9A,0x40}    /*data rate = 250K */
};
#endif

uint8_t RX2_REGS_Table[] ={
  0x86, // 2K
//  0x86, // 5k
  0x86, // 10k
//  0x86, // 25k
  0x86, // 50k
  0x96, // 125k
  0x96, // 125k
};

uint8_t AGC_CTL1_REGS_Table[][3] = {
  // {AGC1, AGC3, AGC7}
  {0x00,0x04,0x30}, // 2k
  {0x00,0x04,0x30}, // 5k
//  {0x00,0x04,0x30}, // 10k
  {0x00,0x04,0x30}, // 25k
//  {0x00,0x04,0x30}, // 50k
  {0x1C,0x07,0x50}, // 125k
  {0x18,0x04,0x50}, // 250k
  
};

uint16_t FCF_RegisterTable16[][10] = 
{
//	FCF1 FCF3,2,  5,4   7,6	    9,8	  11,10	 13,12	15,14  17,16  19,18		
//   {0x0036,0x0021,0x0000,0x0000,0x0302,0x0000,0x0000,0x0000,0x0000,0x0000 }, /* data rate=2K */ 	
//   {0x0036,0x0052,0x0000,0x0000,0x0302,0x0000,0x0000,0x0000,0x0000,0x0000 }, /* data rate=5K */	
//   {0x0016,0x00A4,0x0000,0x0000,0x0310,0x0000,0x0000,0x0000,0x0000,0x0000 }, /* data rate=10K */ 	
//   {0x0016,0x00CD,0x0000,0x0000,0x0310,0x0000,0x0000,0x0000,0x0000,0x0000 }, /* data rate=25K */	
//   {0x0000,0x004C,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000 }, /* data rate=50K */ 	
//   {0x0000,0x0119,0x001D,0x0346,0x0022,0x0331,0x0386,0x0012,0x0008,0x0008 }, /* data rate=125K */ 	
//   {0x0000,0x0444,0x0285,0x008A,0x0012,0x032B,0x0114,0x0021,0x0078,0x0028 }  /* data rate=250K */   
//	FCF1 FCF3,2,  5,4   7,6	    9,8	  11,10	 13,12	15,14  17,16  19,18		
   {0x0030,0x0020,0x0000,0x0000,0x0302,0x0000,0x0000,0x0000,0x0000,0x0000 }, /* data rate=2K */ 	
//   {0x0036,0x0052,0x0000,0x0000,0x0302,0x0000,0x0000,0x0000,0x0000,0x0000 }, /* data rate=5K */	
   {0x0010,0x00A4,0x0000,0x0000,0x0310,0x0000,0x0000,0x0000,0x0000,0x0000 }, /* data rate=10K */ 	
//   {0x0016,0x00CD,0x0000,0x0000,0x0310,0x0000,0x0000,0x0000,0x0000,0x0000 }, /* data rate=25K */	
   {0x0000,0x004C,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000 }, /* data rate=50K */ 	
   {0x0000,0x0119,0x001D,0x0346,0x0022,0x0331,0x0386,0x0012,0x0008,0x0008 }, /* data rate=125K */ 	
   {0x0006,0x9402,0xCA02,0x0062,0x0358,0x03E9,0x03B3,0x3E00,0x03E9,0x0039 }  /* data rate=250K */   
};

uint32_t *data_rate_index[1] =
{
	(uint32_t *)&data_rate_Table16,
};

uint8_t *MOD_Register_index[1] =
{
   (uint8_t *)&MOD_RegisterTable16,   
};

uint8_t *DM_Register_index[1] =
{
   (uint8_t *)&DM_RegisterTable16,   
};

uint16_t *FCF_Register_index[1] = 
{
   (uint16_t *)&FCF_RegisterTable16,      
};

uc8 AGC_RegisterTable[][2] = 	
{ {AGC_CTL2_REGS,0x40},{AGC_CTL3_REGS,0x24},{AGC_CTL5_REGS,0x00},{AGC_CTL7_REGS,0x30} };



uint8_t  TxPayloadData[64];
uint8_t  RxPayloadData[64];


void bc3601_lightSleepMode(BC3601Driver *dev)
{
  BC3601_LITE_SLEEP(dev);
}

/*
  LIRC calibration, set bit7 to start calibration, and 
  wait bit7 set to zero when finished.

  bit0 must set to 1 to enable LIRC
*/

static void lirc_calibration(BC3601Driver *dev)
{
  uint8_t reg;
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_READ(dev,LIRC_CTL_REGS,&reg);
  reg |= 0x80;  // enable calibration
  BC3601_REG_WRITE(dev,LIRC_CTL_REGS,&reg);
  do{
    chThdSleepMilliseconds(1);
    BC3601_REG_READ(dev,LIRC_CTL_REGS,&reg);
  }while(reg & 0x80);
}

static void vco_calibration(BC3601Driver *dev)
{
  uint8_t reg;
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_READ(dev,OPERATION_CTL_REGS,&reg);
  reg |= 0x08;
  BC3601_REG_WRITE(dev,OPERATION_CTL_REGS,&reg);
  do{
    chThdSleepMilliseconds(1);
    BC3601_REG_READ(dev,OPERATION_CTL_REGS,&reg);
  }while(reg & 0x80);
}
//
//static void gpio_calibration(BC3601Driver *dev, uint8_t gio, uint8_t func)
//{
//  uint8_t reg;
//  BC3601_REG_READ(dev,OPERATION_CTL_REGS,&reg);
//  reg |= 0x08;
//  BC3601_REG_WRITE(dev,OPERATION_CTL_REGS,&reg);
//  do{
//    chThdSleepMilliseconds(1);
//    BC3601_REG_READ(dev,OPERATION_CTL_REGS,&reg);
//  }while(reg & 0x80);
//}

static void gio_config(BC3601Driver *dev,uint8_t gio,uint8_t func)
{
  uint8_t reg , regv;
  BC3601_SETBANK(dev,REGS_BANK0);
  regv = GIO12_CTL_REGS + (gio/2);
  BC3601_REG_READ(dev,regv,&reg);
  if(gio < 2){
    reg &= ~(0x07 << (3*(gio %2)));
    reg |= ((func & 0x07) << (3*(gio %2)));
  }
  else{
    reg &= ~(0xf << (4*(gio %2)));
    reg |= ((func & 0x0f) << (4*(gio%2)));
  }
  BC3601_REG_WRITE(dev,regv,&reg);
}

void irq_config(BC3601Driver *dev, uint8_t enable)
{
  uint8_t reg = enable;
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_WRITE(dev,IRQ_ENABLE_REGS,&reg);
}

static bool wait_crystal_ready(BC3601Driver *dev)
{
  uint8_t reg;
  BC3601_SETBANK(dev,REGS_BANK0);
  uint32_t cntr = 0xFFFF;
  do{
    chThdSleepMilliseconds(1);
    cntr--;
    BC3601_REG_READ(dev,CLOCK_CTL_REGS,&reg);
    if(cntr == 0) break;
  }while(!(reg & 0x20));
  
  if(cntr == 0) return false;
  return true;
}


static void agc_configuration(BC3601Driver *dev, uint8_t enable)
{
  uint8_t reg;
  BC3601_SETBANK(dev,REGS_BANK0);
  
  // enable AGC
  BC3601_REG_READ(dev,CONFIG_REGS,&reg);
  if(enable == 1){
    reg |= 0x40;
  }
  else{
    reg &= ~0x40;
  }
  BC3601_REG_WRITE(dev,CONFIG_REGS,&reg);

  
  BC3601_SETBANK(dev,REGS_BANK2);
  reg = _RXG_VALUE_;
  BC3601_REG_WRITE(dev,GAIN_CTL_REGS,&reg);
  uint8_t x = sizeof(AGC_RegisterTable)/2;
  BC3601_SETBANK(dev,REGS_BANK1);
  for(uint8_t i=0;i<x;i++){
    BC3601_REG_WRITE(dev,AGC_RegisterTable[i][0],(void*)&AGC_RegisterTable[i][1]);
  }
  BC3601_SETBANK(dev,REGS_BANK0);
}

static void crysstal_config(BC3601Driver *dev)
{
  uint8_t reg = 0x03; // 16MHz
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_WRITE(dev,XO_SEL_CTL_REGS,&reg);
  reg = 0x18;
  BC3601_REG_WRITE(dev,XO_CAP_CTL_REGS,&reg);
}

static void set_crystal_cload(BC3601Driver *dev,uint8_t xshift, uint8_t xtim)
{
  uint8_t reg = 0x03; // 16MHz
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_READ(dev,XO_CAP_CTL_REGS,&reg);
  reg = (reg & ~XO_SHIFT_MASK) | ((xshift  << 6) & XO_SHIFT_MASK);
  reg = (reg & ~XO_TRIM_MASK) | (xtim & XO_TRIM_MASK);
  BC3601_REG_WRITE(dev,XO_CAP_CTL_REGS,&reg);
}

static void premble_config(BC3601Driver *dev,uint8_t txp, uint8_t rxp)
{
  uint8_t reg = txp-1;
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_WRITE(dev,PREAMBLE_LENG_REGS,&reg);
  
  BC3601_REG_READ(dev,PACKET_CTL2_REGS,&reg);
  reg &= 0xfc;
  reg = reg + rx_preamble_value[rxp];
  BC3601_REG_WRITE(dev,PACKET_CTL2_REGS,&reg);
  
}

static void syncword_config(BC3601Driver *dev,uint8_t length, uint8_t id)
{
  uint8_t reg;
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_READ(dev,PACKET_CTL2_REGS,&reg);
  reg &= 0xf3;
  reg |= sync_width_value[length];
  BC3601_REG_WRITE(dev,PACKET_CTL2_REGS,&reg); 
  
  for(uint8_t i=0;i<8;i++){
    sync_word_buff[i] = (i+1)*2;
  }
  
  BC3601_SYNC_WRITE(dev,sync_word_buff,8); 
}

static void header_config(BC3601Driver *dev,uint8_t pllen_en, uint8_t plhac_en, uint8_t plhen, uint8_t plh_en)
{
  uint8_t reg;
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_READ(dev,PACKET_CTL3_REGS,&reg);
  reg &= 0xf0;
  reg |= (pllen_en << 3) | (plhac_en <<2) | (plhen<<1) | (plh_en);
  BC3601_REG_WRITE(dev,PACKET_CTL3_REGS,&reg); 
  BC3601_REG_READ(dev,PACKET_CTL3_REGS,&reg);

  // set address, 14-bit format
  reg = dev->destAddr & 0x3F;
  BC3601_REG_WRITE(dev,HEADER_ADDR0_REGS,&reg); 
  BC3601_REG_READ(dev,HEADER_ADDR0_REGS,&reg); 
  reg = (dev->destAddr>>8);
  BC3601_REG_WRITE(dev,HEADER_ADDR1_REGS,&reg); 
  BC3601_REG_READ(dev,HEADER_ADDR1_REGS,&reg); 
}

static void bc3601_menchesterConfig(BC3601Driver *dev,uint8_t id)
{
  uint8_t reg;
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_READ(dev,PACKET_CTL3_REGS,&reg);
  reg &= 0x7F;
  reg |= (id << 7);
  BC3601_REG_WRITE(dev,PACKET_CTL3_REGS,&reg); 
}

static void bc3601_fecConfig(BC3601Driver *dev,uint8_t en)
{
  uint8_t reg;
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_READ(dev,PACKET_CTL3_REGS,&reg);
  reg &= 0xBF;
  reg |= (en << 6);
  BC3601_REG_WRITE(dev,PACKET_CTL3_REGS,&reg); 
}

static void bc3601_crcConfig(BC3601Driver *dev,uint8_t en, uint8_t fmt)
{
  uint8_t reg;
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_READ(dev,PACKET_CTL3_REGS,&reg);
  reg &= 0xCF;
  reg |= ((en << 5) | (fmt << 4));
  BC3601_REG_WRITE(dev,PACKET_CTL3_REGS,&reg); 
}

static void bc3601_whiteningConfig(BC3601Driver *dev,uint8_t en, uint8_t fmt)
{
  uint8_t reg;
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_READ(dev,PACKET_CTL4_REGS,&reg);
//  reg &= 0xCF;
  reg |= ((en << 7) | (fmt));
  BC3601_REG_WRITE(dev,PACKET_CTL4_REGS,&reg); 
}

void bc3601_freqConfig(BC3601Driver *dev,float freq)
{
  uint8_t reg,dn;
  float fn;
  uint32_t dk;
  
  if(freq < 400.0){
    dn = 0x0;
    fn = (freq * 1000000)*4; // 315
  }
  else{
    if(freq >= 800.0){
      dn = 0x60;
      fn = (freq * 1000000) *1; //868/915
    }
    else{
      fn = (freq * 1000000) *2;
      if(freq > 450.0) dn = 0x40;
      else dn = 0x20;
    }
  }
    
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_READ(dev,OPERATION_CTL_REGS,&reg);
  reg &= 0x9F;
  dn |= reg;
  BC3601_REG_WRITE(dev,OPERATION_CTL_REGS,&dn);
  
  fn /= (float)16000000;
  dn = (uint8_t)fn;
  fn -= (float)dn;
  fn *= (float)(1048576.0);
  dk = (uint32_t)fn;
  
  BC3601_REG_WRITE(dev,FRACTIONAL_N_DN_REGS,&dn);
  dn = dk & 0xFF;
  BC3601_REG_WRITE(dev,FRACTIONAL_N_DKL_REGS,&dn);
  dn = (dk >> 8) & 0xFF;
  BC3601_REG_WRITE(dev,FRACTIONAL_N_DKM_REGS,&dn);
  dn = (dk >> 16) & 0xFF;
  BC3601_REG_WRITE(dev,FRACTIONAL_N_DKH_REGS,&dn);
}


void bc3601_powerConfig(BC3601Driver *dev,uint8_t pwr)
{
  uint8_t reg;
  BC3601_SETBANK(dev,REGS_BANK0);
  reg = rf_power_value[DEFAULT_RF_Band][pwr];
  BC3601_REG_WRITE(dev,TX_POWER_CTL_REGS,&reg); 
}


void bc3601_datarateConfig(BC3601Driver *dev,uint8_t dr)
{
  uint8_t reg;
  
  //bc3601StrobeCommand(dev,REGS_BANK_CMD + REGS_BANK0);
  BC3601_SETBANK(dev,REGS_BANK2);
  

  BC3601_REG_WRITE(dev,RX2_CTL_REGS,&RX2_REGS_Table[dr]);
  
  BC3601_SETBANK(dev,REGS_BANK0);
  
  /* MOD configuration */
  BC3601_REGS_WRITE(dev,MODULATOR_CTL1_REGS,(void*)&MOD_RegisterTable16[dr],3);

//  BC3601_REGS_WRITE(dev,DEMOULATOR_CTL1_REGS,(void*)&DM_RegisterTable16[dr],8);
  BC3601_REG_WRITE(dev,DEMOULATOR_CTL1_REGS,(void*)&DM_RegisterTable16[dr][0]);
  BC3601_REG_WRITE(dev,DEMOULATOR_CTL2_REGS,(void*)&DM_RegisterTable16[dr][1]);
  BC3601_REG_WRITE(dev,DEMOULATOR_CTL3_REGS,(void*)&DM_RegisterTable16[dr][2]);
  BC3601_REG_WRITE(dev,DEMOULATOR_CTL5_REGS,(void*)&DM_RegisterTable16[dr][4]);
  BC3601_REG_WRITE(dev,DEMOULATOR_CTL8_REGS,(void*)&DM_RegisterTable16[dr][7]);
  
  /* filter coefficient */
  BC3601_SETBANK(dev,REGS_BANK1);
  uint16_t *ptr = &FCF_RegisterTable16[0][0];
//  reg = (FCF_RegisterTable16[dr*10]) & 0xFF;
  reg = *(ptr+dr*10) & 0xFF;
  BC3601_REG_WRITE(dev,FILTER_CTL1_REGS,&reg);
  BC3601_REGS_WRITE(dev,FILTER_CTL2_REGS,(uint8_t*)(&FCF_RegisterTable16[dr][1]),18);
  
  BC3601_REG_WRITE(dev,AGC_CTL1_REGS,&AGC_CTL1_REGS_Table[0][dr]);
  BC3601_REG_WRITE(dev,AGC_CTL3_REGS,&AGC_CTL1_REGS_Table[1][dr]);
  BC3601_REG_WRITE(dev,AGC_CTL7_REGS,&AGC_CTL1_REGS_Table[2][dr]);
  
  BC3601_SETBANK(dev,REGS_BANK0);
  
}


static void bc3601_extenedMarginConfig(BC3601Driver *dev,uint8_t mar_en, uint8_t length)
{
  uint8_t reg;
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_READ(dev,FIFO2_CTL_REGS,&reg);
  reg &= 0xF3;
  reg |= ((mar_en << 2) | length);
  BC3601_REG_WRITE(dev,FIFO2_CTL_REGS,&reg); 
}


static void bc3601_analogRegisterconfig(BC3601Driver *dev)
{
  BC3601_SETBANK(dev,REGS_BANK2);
//  for(uint8_t i=0;i<3;i++){
//    BC3601_REG_WRITE(dev,Analog_RegisterTable[i][0],&Analog_RegisterTable[i][1]); 
//  }
  uint8_t addr, value;
  for(uint8_t i=0;i<sizeof(BANK2_REG_TABLE)/2;i++){
    addr = BANK2_REG_TABLE[i]>>8;
    value = BANK2_REG_TABLE[i]&0xff;
    BC3601_REG_WRITE(dev,addr,&value); 
  }
  BC3601_SETBANK(dev,REGS_BANK0);
}
                                              
static uint32_t bc3601_getDataRate(BC3601Driver *dev, uint8_t dr)
{
  return *data_rate_index[dr];
}

static void bc3601_enableDirectTx(BC3601Driver *dev)
{
  uint8_t reg;
  BC3601_LITE_SLEEP(dev);

  BC3601_REG_READ(dev,CONFIG_REGS,&reg); 
  reg |= 0x10;
  BC3601_REG_WRITE(dev,CONFIG_REGS,&reg); 
  BC3601_SETBANK(dev,REGS_BANK0);
  
  BC3601_REG_READ(dev,OPERATION_CTL_REGS,&reg); 
  reg &= 0xFD;
  reg |= 0x03;
  BC3601_REG_WRITE(dev,OPERATION_CTL_REGS,&reg); 
  reg |= 0x40;
  chThdSleepMilliseconds(1);
  BC3601_REG_WRITE(dev,OPERATION_CTL_REGS,&reg);   
}
                                              
static void bc3601_enableDirectRx(BC3601Driver *dev)
{
  uint8_t reg;
  BC3601_LITE_SLEEP(dev);

  BC3601_REG_READ(dev,CONFIG_REGS,&reg); 
  reg |= 0x10;
  BC3601_REG_WRITE(dev,CONFIG_REGS,&reg); 
  BC3601_SETBANK(dev,REGS_BANK0);
  
  BC3601_REG_READ(dev,OPERATION_CTL_REGS,&reg); 
  reg &= 0xFD;
  reg |= 0x01;
  BC3601_REG_WRITE(dev,OPERATION_CTL_REGS,&reg); 
  reg |= 0x40;
  chThdSleepMilliseconds(1);
  BC3601_REG_WRITE(dev,OPERATION_CTL_REGS,&reg);   
}

static void bc3601_disableDirectRx(BC3601Driver *dev)
{
  uint8_t reg;
  BC3601_SETBANK(dev,REGS_BANK0);

  BC3601_REG_READ(dev,CONFIG_REGS,&reg); 
  reg &= ~0x04;
  BC3601_REG_WRITE(dev,CONFIG_REGS,&reg); 
  
  BC3601_REG_READ(dev,OPERATION_CTL_REGS,&reg); 
  reg &= ~0x01;
  BC3601_REG_WRITE(dev,OPERATION_CTL_REGS,&reg); 
  chThdSleepMilliseconds(1);

  BC3601_REG_READ(dev,OPERATION_CTL_REGS,&reg); 
  reg &= ~0x10;
  BC3601_REG_WRITE(dev,OPERATION_CTL_REGS,&reg);   
}

uint32_t cal_packet_time(BC3601Driver *dev,uint8_t sel)
{
  uint32_t	time, dr;
  uint8_t reg;
  uint8_t	c;
  time = DEFAULT_PKT_Length;
  BC3601_REG_READ(dev,PACKET_CTL3_REGS,&c);
  if((c & 0x03) == 0x01) time++;						//PLH_EN=1
  else if((c & 0x03) == 0x03) time += 2;				//PLHLEN=1 & PLH_EN=1
  if(c & 0x08) time++;										//PLHAC_EN=1
  if(c & 0x20) time += 2;									//CRC_EN=1
  if(c & 0x40) time = (time * 7 + 3) / 4; 	      //*1.75	FEC_EN=1
  if(c & 0x80) time *= 2;									//*2		MCH_EN=1
  if(sel == _TX_SEL_)
          time += DEFAULT_TX_Preamble;
  else		
  {
          time += rx_preamble_value[DEFAULT_RX_Preamble];
          if(rx_preamble_value[DEFAULT_RX_Preamble] == 3)
                  time++;
  }		
  time +=  (sync_width_value[DEFAULT_SyncWidth] >> 1) + 2;
  time = time * 8 + 4 + 8;	//4 trailer & 8 dummy clock
  dr = bc3601_getDataRate(dev,DEFAULT_DATA_RATE);
  dr = 1000000 / dr;
  time *= dr;
  return time;
}

static void bc3601_setAutoCycleMode(BC3601Driver *dev,uint8_t mode, bool en)
{
  uint8_t reg;
  BC3601_REG_READ(dev,ATR_CONTROL_REGS,&reg); 
  reg &= 0xF8;
  reg |= (((mode & 0x03) <<1) | en);
  BC3601_REG_WRITE(dev,ATR_CONTROL_REGS,&reg); 
}

static void bc3601_setCycleClockUnit(BC3601Driver *dev,uint32_t tm)
{
  uint8_t reg;
  BC3601_REG_READ(dev,ATR_CONTROL_REGS,&reg); 
  reg &= 0xC0;
  
  if(tm >= 16000000)
    reg |= 0xc0;
  else if(tm >= 8000000)
    reg |= 0x80;
  else if(tm > 2000000)
    reg |= 0x40;
  BC3601_REG_WRITE(dev,ATR_CONTROL_REGS,&reg); 
}

uint16_t BC3601_CalCycleCounter(uint32_t tm)
{
   float clk_tim;
   uint16_t   cyc_value;
	 uint8_t		cc;
	 cc = BC3601_ReadRegister(ATR_CONTROL_REGS);	
   switch(cc & 0xC0)
   {
      case 0x00 : clk_tim = (float)1000000.0/32768.0; break;
      case 0x40 : clk_tim = (float)1000000.0/8192.0; break;
      case 0x80 : clk_tim = (float)1000000.0/4096.0; break;
      case 0xC0 : clk_tim = (float)1000000.0/2048.0; break;
   }   
   clk_tim = (float)tm / clk_tim;
   cyc_value = (u16)(clk_tim + 0.5);
	if(cyc_value > 0)   
   	cyc_value--;
	return cyc_value;
}

#define	_RXG_VALUE_		0x8F		//Gain=80dB

static void bc3601_enableATRCTM(BC3601Driver *dev, bool en)
{
  uint8_t reg;
  BC3601_REG_READ(dev,ATR_CONTROL_REGS,&reg); 
  if(en)
    reg |= 0x08;
  else
    reg &= ~0x08;
  BC3601_REG_WRITE(dev,ATR_CONTROL_REGS,&reg); 
}

static void bc3601_setAutoCycleTime(BC3601Driver *dev, uint32_t tm)
{
  uint16_t reg;
  bc3601_setCycleClockUnit(dev,tm);
  reg = BC3601_CalCycleCounter(tm);
  
  BC3601_REGS_WRITE(dev,WRITE_REGS_CMD,(uint8_t*)&reg,2); 
}

static void bc3601_tuneAutoCycleCounter(BC3601Driver *dev, int16_t cnt)
{
  uint32_t reg;
  reg = bc3601_getAutoCycleCounter(dev);
  reg += cnt;
  bc3601_setCycleClockUnit(dev,cnt);
  reg = BC3601_CalCycleCounter(cnt);
  
  BC3601_REGS_WRITE(dev,WRITE_REGS_CMD,(uint8_t*)&reg,2); 
}

static void bc3601_setATRCT_Timer(BC3601Driver *dev, uint32_t tm)
{
  uint16_t reg;
  reg = BC3601_CalCycleCounter(tm);
  
  BC3601_REGS_WRITE(dev,ATRCT_L_REGS,(uint8_t*)&reg,2); 
}

void bc3601_setATRCT_Counter(BC3601Driver *dev, uint16_t cntr)
{
  BC3601_REGS_WRITE(dev,ATRCT_L_REGS,(uint8_t*)&cntr,2); 
}


uint32_t bc3601_getATRCT_Counter(BC3601Driver *dev)
{
  uint32_t reg;
  BC3601_REGS_READ(dev,ATRCT_L_REGS,(uint8_t*)&reg,2);
  return reg;
}

void bc3601_enableARK(BC3601Driver *dev, bool en)
{
  uint8_t reg = _BC3601_REGS_ARK1_;
  if(en)
    reg |= 0x01;
  else
    reg &= ~0x01;
  
  BC3601_REG_WRITE(dev,ARK_CONTROL_REGS,&reg);
  reg = _BC3601_REGS_ARK2_;
  BC3601_REG_WRITE(dev,ARK_CONTROL_REGS,&reg);
}

bool bc3601_setARKRXAP_Timer(BC3601Driver *dev, uint32_t tm)
{
  bool flag = TRUE;
  uint8_t reg;
  uint16_t uart;
  BC3601_REG_READ(dev,ATR_CONTROL_REGS,&reg);
  
  if(tm > 64000)
    reg |= 0x10;
  else
    reg &= ~0x10;
  BC3601_REG_WRITE(dev,ATR_CONTROL_REGS,&reg);
  
  if(reg & 0x10)
    uart = (tm + 999) / 1000;
  else
    uart = (tm+249)/250;
  uart--;
  
  if(uart > 0x00ff){
    flag = FALSE;
    uart = 0x00ff;
  }
  
  reg = uart & 0xff;
  BC3601_REG_WRITE(dev,ARK_ACTIVE_REGS,&reg);
  
  return flag;
}

uint8_t bc3601_getAutoRetryNumber(BC3601Driver *dev)
{
  BC3601_SETBANK(dev,REGS_BANK0);
  uint8_t reg;
  BC3601_REG_READ(dev,ARK_CONTROL_REGS,&reg);
  reg>>=4;
  reg +=1;
  return reg;
}

void bc3601_setAutoRetryNumber(BC3601Driver *dev, uint8_t counter)
{
  uint8_t reg;
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_READ(dev,ARK_CONTROL_REGS,&reg);
  reg <<= 4;
  reg = (reg >> 4) + ((counter-1)<<4);
  BC3601_REG_WRITE(dev,ARK_CONTROL_REGS,&reg);
  BC3601_SETBANK(dev,REGS_BANK0);
}

void bc3601_incPID(BC3601Driver *dev)
{
  uint8_t reg;
  BC3601_REG_READ(dev,PACKET_CTL2_REGS,&reg);
  reg += 0x40;
  BC3601_REG_WRITE(dev,PACKET_CTL2_REGS,&reg);
}

uint8_t bc3601_readRSSI(BC3601Driver *dev)
{
  uint8_t reg;
  BC3601_REG_READ(dev,RSSI_VALUE_ID_REGS,&reg);
  
  return reg;
}

uint8_t bc3601_readEnvRSSI(BC3601Driver *dev)
{
  uint8_t reg;
  BC3601_REG_READ(dev,RSSI_VALUE_REGS,&reg);
  
  return reg;
}

static void bc3601_setAutoRxActivePeriod(BC3601Driver *dev,uint32_t tm)
{
  uint8_t reg;
  uint16_t uart;
  BC3601_REG_READ(dev,ATR_CONTROL_REGS,&reg);
  
  if(tm > 255750)
    reg |= 0x10;
  else 
    reg &= ~0x10;  
  BC3601_REG_WRITE(dev,ATR_CONTROL_REGS,&reg); 
  
  if(reg&0x10)
    uart = (tm + 999)/1000;
  else
    uart = (tm + 249)/250;
  if(uart > 0)
    uart--;
  
  reg = (uint8_t)uart;
  BC3601_REG_WRITE(dev,ATR_ACTIVE_REGS,&reg); 
  reg = (uint8_t)(uart >> 8);
  BC3601_REG_WRITE(dev,ATR_ACTIVE_H_REGS,&reg); 
  
}

static void bc3601_setAutoRxExtendPeriod(BC3601Driver *dev,uint32_t tm)
{
  uint8_t reg;
  uint16_t uart;
  BC3601_REG_READ(dev,ATR_CONTROL_REGS,&reg);
    
  if(reg&0x10)
    uart = (tm + 999)/1000;
  else
    uart = (tm + 249)/250;
  
  uart--;
  
  BC3601_REGS_WRITE(dev,ATR_EACTIVE_L_REGS,(uint8_t*)&uart,2);   
}

uint16_t bc3601_getAutoCycleCounter(BC3601Driver *dev)
{
  uint16_t reg;
  BC3601_REGS_READ(dev,ATR_CYCLE_L_REGS,(uint8_t*)&reg,2);
  return reg;
}

void parameter_initialization(BC3601Driver *dev)
{
  
    uint8_t addr, value;
    uint8_t *ptr = (uint8_t*)&dev->reg_cmd;
    BC3601_SETBANK(dev,REGS_BANK0);
    for(uint8_t i=0;i<sizeof(IRQIO_REGS_TABLE)/2;i++){
      addr = IRQIO_REGS_TABLE[i]>>8;
      value = IRQIO_REGS_TABLE[i]&0xff;
      BC3601_REG_WRITE(dev,addr,&value); 
    }  
    
    for(uint8_t i=0;i<sizeof(PACKET_REGS_TABLE)/2;i++){
      addr = PACKET_REGS_TABLE[i]>>8;
      value = PACKET_REGS_TABLE[i]&0xff;
      BC3601_REG_WRITE(dev,addr,&value); 
    }  

    for(uint8_t i=0;i<sizeof(COMMON_REGS_TABLE)/2;i++){
      addr = COMMON_REGS_TABLE[i]>>8;
      value = COMMON_REGS_TABLE[i]&0xff;
      BC3601_REG_WRITE(dev,addr,&value); 
    }  
    
    BC3601_SETBANK(dev,REGS_BANK0);
    for(uint8_t i=0;i<sizeof(BANK0_REG_TABLE)/2;i++){
      addr = BANK0_REG_TABLE[i]>>8;
      value = BANK0_REG_TABLE[i]&0xff;
      BC3601_REG_WRITE(dev,addr,&value); 
    }  
    
    BC3601_SETBANK(dev,REGS_BANK1);
    for(uint8_t i=0;i<sizeof(BANK1_REG_TABLE)/2;i++){
      addr = BANK1_REG_TABLE[i]>>8;
      value = BANK1_REG_TABLE[i]&0xff;
      BC3601_REG_WRITE(dev,addr,&value); 
    }  
    
    BC3601_SETBANK(dev,REGS_BANK2);
    for(uint8_t i=0;i<sizeof(BANK2_REG_TABLE)/2;i++){
      addr = BANK2_REG_TABLE[i]>>8;
      value = BANK2_REG_TABLE[i]&0xff;
      BC3601_REG_WRITE(dev,addr,&value); 
    }  
    BC3601_SETBANK(dev,REGS_BANK0);

//	memcpy((void *)TxPayloadData, (void *)pn9_data, DEFAULT_PKT_Length);
//	memcpy((void *)RxPayloadData, (void *)pn9_data, DEFAULT_PKT_Length);
	
	/* set BC3601 bank2 register */	 
//	bc3601_analogRegisterconfig(dev);
        
	/* set Crystal */
	//crysstal_config(dev);

	/* set AGC, not necessary for 3603 */
	//agc_configuration(dev,Enable);
	
	/* set tx and rx preamble */
	premble_config(dev,DEFAULT_TX_Preamble,DEFAULT_RX_Preamble);

	/* set Syncword */

	/* set tx packet length */
	//bc3601_setTxPayloadWidth(dev,DEFAULT_PKT_Length);   
	
	/* set rx packet length */
	//bc3601_setRxPayloadWidth(dev,DEFAULT_PKT_Length);   

  /* set header type */
	header_config(dev,DEFAULT_PLLEN_EN, DEFAULT_PLHAC_EN, DEFAULT_PLHLEN, DEFAULT_PLH_EN); 
    //header_config(dev,1, 1, 1, 1); 

	/* set codeing type */
	bc3601_menchesterConfig(dev,DEFAULT_Man_EN);
	bc3601_fecConfig(dev,DEFAULT_FEC_EN);
        bc3601_crcConfig(dev,DEFAULT_CRC_EN, DEFAULT_CRCFMT);

	/* set whitening type */
	bc3601_whiteningConfig(dev,DEFAULT_WHT_EN, DEFAULT_WHTSEED);

	/* set RF band */
//  bc3601_freqConfig(dev,DEFAULT_RF_Fr0equency);
  

  /* set Tx power */
//  bc3601_powerConfig(dev,DEFAULT_TX_Power);
  bc3601_powerConfig(dev,dev->txPower);
  
  /* set data rate */
//  bc3601_datarateConfig(dev,DEFAULT_DATA_RATE);
  
   
	wait_crystal_ready(dev);	  
}

void bc3601_vcoCalibration(BC3601Driver *dev)
{
  uint8_t reg;
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_READ(dev,OPERATION_CTL_REGS,&reg);
  reg |= 0x08;

  BC3601_REG_WRITE(dev,OPERATION_CTL_REGS,&reg);

  do{
    BC3601_REG_READ(dev,OPERATION_CTL_REGS,&reg);
  }while(reg & 0x08);
}

void bc3601_lircCalibration(BC3601Driver *dev)
{
  uint8_t reg;
  
  BC3601_SETBANK(dev,REGS_BANK0);
  BC3601_REG_READ(dev,LIRC_CTL_REGS,&reg);
  reg |= 0x80;

  BC3601_REG_WRITE(dev,LIRC_CTL_REGS,&reg);

  do{
    BC3601_REG_READ(dev,LIRC_CTL_REGS,&reg);
  }while(reg & 0x80);
}

void bc3601_gotoDefaultMode(BC3601Driver *dev)
{
  bc3601_lightSleepMode(dev);
  
}

uint8_t bc3601_irqState(BC3601Driver *dev)
{
  uint8_t reg;
  uint8_t ret;
  
  BC3601_REG_READ(dev,IRQ_STATUS_REGS,&reg);
  ret = reg;
  if(ret != 0x00){
    reg = 0x0;
    BC3601_REG_WRITE(dev,IRQ_STATUS_REGS,&reg);
  }
  return ret;
}

void task_sendTestPacket(BC3601Driver *dev)
{
  uint8_t reg;
  BC3601_RESET_TXFIFO(dev);
  reg = 0;
  BC3601_SET_TX_PAYLOAD_SADDR(dev,&reg);
  reg = DEFAULT_PKT_Length;
  BC3601_SET_TX_PAYLOAD_WIDTH(dev,&reg);
  BC3601_FIFO_WRITE(dev,TxPayloadData,DEFAULT_PKT_Length);
  BC3601_TX_MODE(dev);
}

void task_enterRxPacket(BC3601Driver *dev)
{
  uint8_t reg;
  // enter RX
  irq_config(dev,IRQ2_RXERRIE | IRQ2_RXCMPIE);
  reg = DEFAULT_PKT_Length;
  BC3601_SET_RX_PAYLOAD_WIDTH(dev,&reg);
  BC3601_RESET_RXFIFO(dev);
  BC3601_RX_MODE(dev);
}
void dev_bc3601Init(BC3601Driver *dev,const bc3601_config_t *config)
{
  uint8_t reg;
  dev->config = config;
  // 4-wire spi
//  gio_config(dev,_GPIO1_,1);
  // set to sleep mode
  // reset chip
  BC3601_RESET_CHIP(dev);
  bc3601_refresh_registers(dev);

  bc3601_lightSleepMode(dev);
  
  if(!wait_crystal_ready(dev)){
    return;
  }
  set_crystal_cload(dev,RF_CYL_COARSE_TUNE_,RF_CYL_FINE_TUNE_);

  parameter_initialization(dev);
  bc3601_refresh_registers(dev);
  
  BC3601_SETBANK(dev,REGS_BANK0);
  
  // set GIO2 to MISO
  //reg = IO1_PADDS_1MA | IO1_GIO2S_SDO | IO1_GIO1S_INPUT;
  //BC3601_REG_WRITE(dev,GIO12_CTL_REGS,&reg);
  
  //reg = IO3_GIOPU2;
  //BC3601_REG_WRITE(dev,GPIO_PULL_UP_REGS,&reg);
  
  // set gio 1 as interrupt
  //gio_config(dev,_IRQ_LINE, INT_REQUEST);
  
  /* 1.2v rstll */
  
//  BC3601_REG_READ(dev,CLOCK_CTL_REGS,&reg);
//  reg |= 0x01;
//  BC3601_REG_WRITE(dev,CLOCK_CTL_REGS,&reg);
//  reg &= ~0x01;
//  BC3601_REG_WRITE(dev,CLOCK_CTL_REGS,&reg);
  
  //BC3601_REG_READ(dev,CLOCK_CTL_REGS,&reg);

  
  // write sync word
  syncword_config(dev,DEFAULT_SyncWidth, DEFAULT_DeviceID);
  
  // write freq table
  bc3601_freqConfig(dev,dev->frequency);
  // set dr param
  bc3601_datarateConfig(dev,dev->dataRate);
  // set power
  
  // stalready
  
  bc3601_vcoCalibration(dev);
  bc3601_lircCalibration(dev);
  bc3601_gotoDefaultMode(dev);

  // set pid
  //reg = dev->destAddr & 0xFF;
  //BC3601_REG_WRITE(dev,GPIO_PULL_UP_REGS,&reg);
  
  bc3601_refresh_registers(dev);
  
}

void dev_bc3601DeInit(BC3601Driver *dev)
{
  BC3601_RESET_CHIP(dev);
  BC3601_SETBANK(dev,REGS_BANK0);
  uint8_t reg = 0x98;
  BC3601_REG_WRITE(dev,0x39,&reg);
  bc3601_lightSleepMode(dev);
  
  // set to default 
  BC3601_SETBANK(dev,REGS_BANK0);
  reg = 0x11;
  BC3601_REG_WRITE(dev,CLOCK_CTL_REGS,&reg);
  reg = 0x10;
  BC3601_REG_WRITE(dev,CLOCK_CTL_REGS,&reg);
  reg = 0x40;
  BC3601_REG_WRITE(dev,CONFIG_REGS,&reg);
  reg = 0x38;
  BC3601_REG_WRITE(dev,TRX_MODE_DELAY_REGS,&reg);
  reg = 0x58;
  BC3601_REG_WRITE(dev,DEMOULATOR_CTL4_REGS,&reg);
  
  BC3601_SETBANK(dev,REGS_BANK1);
  reg = 0x14;
  BC3601_REG_WRITE(dev,AGC_CTL2_REGS,&reg);
  
  BC3601_SETBANK(dev,REGS_BANK2);
  reg = 0x03;
  BC3601_REG_WRITE(dev,0x26,&reg);
  reg = 0x88;
  BC3601_REG_WRITE(dev,0x29,&reg);
  reg = 0x68;
  BC3601_REG_WRITE(dev,0x2e,&reg);
  reg = 0x06;
  BC3601_REG_WRITE(dev,0x2F,&reg);
  reg = 0x90;
  BC3601_REG_WRITE(dev,0x34,&reg);
  reg = 0x9C;
  BC3601_REG_WRITE(dev,0x39,&reg);
  reg = 0x5D;
  BC3601_REG_WRITE(dev,0x3B,&reg);
  BC3601_SETBANK(dev,REGS_BANK0);

  if(!wait_crystal_ready(dev)){
    return;
  }
//  BC3601_STBY(dev);
  BC3601_DEEP_SLEEP(dev);

  
}

void bc3601_WaitCrystalReady(BC3601Driver *dev)
{
  wait_crystal_ready(dev);
}

void bc3601_clearIRQ(BC3601Driver *dev, uint8_t flag_mask)
{
  uint8_t reg;
  BC3601_REG_READ(dev,IRQ_CTL_REGS,&reg);
  if(reg & 0x01){
    reg = flag_mask;
  }
  else{
    reg = ~flag_mask;
  }
  BC3601_REG_WRITE(dev,IRQ_CTL_REGS,&reg);
  
}