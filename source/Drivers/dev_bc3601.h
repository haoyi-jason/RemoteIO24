#ifndef _TASK_BC3601_
#define _TASK_BC3601_

#include "hal.h"
#include "bc3601.h"
#include "bc3601_def.h"

#define Enable 1
#define Disable 0

#define	DEFAULT_RF_Band						_RF_Band_915MHz_				//433MHz
#define	DEFAULT_RF_Frequency			915									//433.92MHz
#define	DEFAULT_TX_Preamble				4	  										//1~256			 	4 = 4 byte  	 
#define	DEFAULT_RX_Preamble				_RX_Preamble_2B_				//2 byte
#define	DEFAULT_SyncWidth					_Syncword_4B_						//4 byte
#define	DEFAULT_DeviceID					0xab00111111	  				//0~0xFFFFFFFFFF //40bit
#define	DEFAULT_TX_Power					_TX_Power_0dBm_				//+10 dBm
#define	DEFAULT_DATA_RATE					_DataRate_10K_					//50K bps
					
#define	DEFAULT_Man_EN						Disable									//Manchester code enable
#define	DEFAULT_FEC_EN						Disable									//FEC enable
#define	DEFAULT_CRC_EN						Enable									//CRC field enable
#define	DEFAULT_CRCFMT						1												//CRC format selection

#define	DEFAULT_WHT_EN						Disable									//Data whitening enable
#define	DEFAULT_WHTSEED						54											//whitening Seed selection

#define	DEFAULT_PLLEN_EN					Disable									//Payload length field enable
#define	DEFAULT_PLHAC_EN					Enable									//Payload header address correction enable control  Enable:set address   Disable: set software flags	
#define	DEFAULT_PLHLEN						Enable									//1~2byte Payload header length	 Enable:2byte	 Disable: 1byte		
#define	DEFAULT_PLH_EN						Enable									//Payload header field enable	
		
#define	DEFAULT_PKT_Length    		64	  									//1~64

#define	DEFAULT_PKT_Ext_Length    180	  									//1~255														
#define	DEFAULT_Margin_Length    	Margin_32byte						//FIFO length margin selection		
//******************************************************************

#define	_DEEP_SLEEP_MODE_ 	0
#define	_LIGHT_SLEEP_MODE_	1
#define	_IDLE_MODE_				  2


#define Simple_TX_Mode				0
#define Simple_RX_Mode				1
#define Block_TX_Mode					2
#define Block_RX_Mode					3
#define Extend_TX_Mode				4
#define Extend_RX_Mode				5
#define Infinite_TX_Mode			6
#define Infinite_RX_Mode			7
#define PER_Master_Mode				8
#define PER_Slave_Mode				9
#define ATR_WOT_Mode					10
#define ATR_WOR_Mode					11
#define ARK_AAK_Mode					12
#define ARK_ARS_Mode					13
#define WOTARS_Mode						14
#define WORAAK_Mode						15


#define	_RF_Band_315MHz_ 0
#define	_RF_Band_433MHz_ 1
#define	_RF_Band_470MHz_ 2
#define	_RF_Band_868MHz_ 3
#define	_RF_Band_915MHz_ 4


#define	_RX_Preamble_0B_ 0
#define	_RX_Preamble_1B_ 1
#define	_RX_Preamble_2B_ 2
#define	_RX_Preamble_4B_ 3


#define	_Syncword_4B_ 0
#define	_Syncword_6B_ 1
#define	_Syncword_8B_ 2


#define	_TX_Power_0dBm_ 0
#define	_TX_Power_5dBm_ 1
#define	_TX_Power_10dBm_ 2
#define	_TX_Power_13dBm_ 3
#define	_TX_Power_17dBm_ 4

#define	_DataRate_2K_ 0
#define	_DataRate_5K_ 1
#define	_DataRate_10K_ 2
#define	_DataRate_25K_ 3
#define	_DataRate_50K_ 4
#define	_DataRate_125K_ 5
#define	_DataRate_250K_ 6


#define	Margin_4byte 0
#define	Margin_8byte 1
#define	Margin_16byte 2
#define	Margin_32byte 3


//    Selection default mode
//       <0=>  Deep Sleep
//       <1=>  Light Sleep
//       <2=>  Idle Mode
#define	_DEFAULT_MODE_ (1)

//    GIO Driving
//			<0x00=>	 0.5mA
//			<0x40=>  1mA
//			<0x80=>  5mA
//			<0xC0=>  10mA
#define _GIO_DRIVING_  	(0xC0)

#define  _GPIO1_     	0 																
#define  _GPIO2_     	1																	
#define  _GPIO3_     	2																
#define  _GPIO4_     	3																	

#define  GPIO1P   		(1UL << _GPIO1_)
#define  GPIO2P   		(1UL << _GPIO2_)
#define  GPIO3P   		(1UL << _GPIO3_)
#define  GPIO4P   		(1UL << _GPIO4_)

#define  GPIO3_IN    	(HT_GPIOC->DINR & GPIO3P)																
#define  GPIO3_LOW   	(HT_GPIOC->DOUTR &= ~GPIO3P)														
#define  GPIO3_HIGH  	(HT_GPIOC->DOUTR |= GPIO3P)															
#define  GPIO3_TOGGLE   (HT_GPIOC->DOUTR ^= GPIO3P)														

#define  GPIO4_IN    	(HT_GPIOC->DINR & GPIO4P)																
#define  GPIO4_LOW   	(HT_GPIOC->DOUTR &= ~GPIO4P)														
#define  GPIO4_HIGH  	(HT_GPIOC->DOUTR |= GPIO4P)															
#define  GPIO4_TOGGLE   (HT_GPIOC->DOUTR ^= GPIO4P)														

#define  IRQ_ENABLE     (HT_EXTI->CR |= (1UL << EXTI_CHANNEL_10))
#define  IRQ_DISABLE    (HT_EXTI->CR &= ~(1UL << EXTI_CHANNEL_10))

#define  RF_IO_CFGR     HT_AFIO->GPCCFGR[0]										
#define  RF_IO_PORT     HT_GPIOC														
#define  _RF_SDIO_      1																	
#define  _RF_IRQ_       4																	
#define  RF_SDIO        (1UL << _RF_SDIO_)   
#define  RF_IRQ         (1UL << _RF_IRQ_)
#define  RF_SDIO_DIRO   (RF_SPI_PORT->DIRCR |= RF_SDIO)
#define  RF_SDIO_DIRI   (RF_SPI_PORT->DIRCR &= ~RF_SDIO)
#define  RF_SDIO_IN     (RF_SPI_PORT->DINR & RF_SDIO)
#define  RF_SDIO_LOW    (RF_SPI_PORT->DOUTR &= ~RF_SDIO)
#define  RF_SDIO_HIGH   (RF_SPI_PORT->DOUTR |= RF_SDIO)
#define	 RF_IRQ_IN		(RF_IO_PORT->DINR & RF_IRQ)

#define  Up_Key      0x01
#define  Down_Key    0x02
#define  Enter_Key   0x04
#define  Back_Key    0x08

#define	_RXG_VALUE_		0x8F		//Gain=80dB


uint16_t bc3601_getAutoCycleCounter(BC3601Driver *dev);


void dev_bc3601Init(BC3601Driver *dev,const bc3601_config_t *config);
#endif