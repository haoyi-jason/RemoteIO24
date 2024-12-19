#ifndef _BC3601_DEF_
#define _BC3601_DEF_

enum reg_def{
  BC_CFG1,
  BC_RC1,
  BC_IRQ1,
  BC_IRQ2,
  BC_IRQ3,
  BC_RSV1,
  BC_IO1,
  BC_IO2,
  BC_IO3,
  BC_FIFO1,
  BC_FIFO2,
  BC_PKT1,
  BC_PKT2,
  BC_PKT3,
  BC_PKT4,
  BC_PKT5,
  BC_PKT6,
  BC_PKT7,
  BC_PKT8,
  BC_PKT9,
  BC_MOD1,
  BC_MOD2,
  BC_MOD3,
  BC_DM1,
  BC_DM2,
  BC_DM3,
  BC_DM4,
  BC_DM5,
  BC_DM6,
  BC_DM7,
  BC_DM8,
};

/* BC3601 strobe command */
#define WRITE_REGS_CMD		0x40  /* Register write command */
#define READ_REGS_CMD		0xC0  /* Register read command */
#define REGS_BANK_CMD      	0x20  /* Set Register Bank command */
#define WRITE_SYNCWORD_CMD  	0x10  /* Write Sync-Word command */
#define READ_SYNCWORD_CMD   	0x90  /* Read Sync-Word command */
#define WRITE_FIFO_CMD      	0x11  /* Write TX FIFO command */
#define READ_FIFO_CMD       	0x91  /* Read RX FIFO command */
#define SOFT_RESET_CMD      	0x08  /* Software reset command */
#define REST_TX_POS_CMD     	0x09  /* TX FIFO address pointer reset command */
#define REST_RX_POS_CMD     	0x89  /* RX FIFO address pointer reset command */
#define DEEP_SLEEP_CMD      	0x0A  /* Enter Deep Sleep mode */
#define IDLE_MODE_CMD       	0x0B  /* Enter Idle mode */
#define LIGHT_SLEEP_CMD     	0x0C  /* Enter Light Sleep mode */
#define STANDBY_MODE_CMD    	0x0D  /* Enter Standby mode */
#define TX_MODE_CMD         	0x0E  /* Enter TX mode */
#define RX_MODE_CMD         	0x8E  /* Enter RX mode */
#define REGSADDR_MASK       	0x3F

/* BC3601 register bank */
enum 
{
   REGS_BANK0 = 0,                  /* register bank 0 */
   REGS_BANK1,                      /* register bank 1 */
   REGS_BANK2,                      /* register bank 2 */
   REGS_BANK3                       /* register bank 3 */
};

/* BC3601 Common & Bank 0 register */
enum 
{
   CONFIG_REGS = 0x00,           /* Configuration Control Register */
   CLOCK_CTL_REGS,               /* Reset/Clock Control Register */
   IRQ_CTL_REGS,                 /* Interrupt Control Register */
   IRQ_ENABLE_REGS,              /* Interrupt enable register */
   IRQ_STATUS_REGS,              /* interrupt status register */
   GIO12_CTL_REGS = 0x06,       /* GPIO 1/2 control register */
   GIO34_CTL_REGS,              /* GPIO 3/4 control register */
   GPIO_PULL_UP_REGS,            /* SPI/GPIO pull high control register */
   TX_FIFO_SA_REGS,              /* TX FIFO start address register */
   FIFO2_CTL_REGS,               /* Packet mode FIFO control regster */
   PREAMBLE_LENG_REGS,           /* Packet preamble length register */
   PACKET_CTL2_REGS,
   PACKET_CTL3_REGS,
   PACKET_CTL4_REGS,
   TX_DATA_LENG_REGS,
   RX_DATA_LENG_REGS,
   TRX_MODE_DELAY_REGS,
   HEADER_ADDR0_REGS,
   HEADER_ADDR1_REGS,
   MODULATOR_CTL1_REGS = 0x14,   
   MODULATOR_CTL2_REGS,
   MODULATOR_CTL3_REGS,
   DEMOULATOR_CTL1_REGS = 0x17,
   DEMOULATOR_CTL2_REGS,
   DEMOULATOR_CTL3_REGS,
   DEMOULATOR_CTL4_REGS,
   DEMOULATOR_CTL5_REGS,
   DEMOULATOR_CTL6_REGS,
   DEMOULATOR_CTL7_REGS,
   DEMOULATOR_CTL8_REGS = 0x1E,
   OPERATION_CTL_REGS = 0x20,
   FRACTIONAL_N_DN_REGS = 0x22,   
   FRACTIONAL_N_DKL_REGS,   
   FRACTIONAL_N_DKM_REGS,   
   FRACTIONAL_N_DKH_REGS,   
   MODE_STATUS_REGS,
   RSSI_CTL_REGS = 0x28,
   RSSI_VALUE_REGS,
   RSSI_VALUE_ID_REGS,
   ATR_CONTROL_REGS,
   ATR_CYCLE_L_REGS,
   ATR_CYCLE_H_REGS,
   ATR_ACTIVE_REGS,
   ATR_EACTIVE_L_REGS,
   ATR_EACTIVE_H_REGS,
   ARK_CONTROL_REGS,
   ARK_ACTIVE_REGS,
   ATRCT_L_REGS,
   ATRCT_H_REGS,
   ATR_ACTIVE_H_REGS,
   XO_CAP_CTL_REGS = 0x3C,
   XO_SEL_CTL_REGS,
   LIRC_CTL_REGS,
   TX_POWER_CTL_REGS
};

/* BC3601 Bank 1 register */
enum 
{
   AGC_CTL1_REGS = 0x20,
   AGC_CTL2_REGS,
   AGC_CTL3_REGS,
   AGC_CTL4_REGS,
   AGC_CTL5_REGS,
   AGC_CTL6_REGS,
   AGC_CTL7_REGS,
   FILTER_CTL1_REGS = 0x2C,
   FILTER_CTL2_REGS,
   FILTER_CTL3_REGS,
   FILTER_CTL4_REGS,
   FILTER_CTL5_REGS,
   FILTER_CTL6_REGS,
   FILTER_CTL7_REGS,
   FILTER_CTL8_REGS,
   FILTER_CTL9_REGS,
   FILTER_CTL10_REGS,
   FILTER_CTL11_REGS,
   FILTER_CTL12_REGS,
   FILTER_CTL13_REGS,
   FILTER_CTL14_REGS,
   FILTER_CTL15_REGS,
   FILTER_CTL16_REGS,
   FILTER_CTL17_REGS,
   FILTER_CTL18_REGS,
   FILTER_CTL19_REGS,   
};

/* BC3601 Bank 2 register */
enum
{
	GAIN_CTL_REGS = 0x21,
	RX2_CTL_REGS = 0x2F,
};

#define ENABLE  1
#define DISABLE 0

#define CFG1_AGC_EN_BIT                 (1 << 6)
#define CFG1_RXCON_EN_BIT               (1 << 5)
#define CFG1_DIR_EN_BIT                 (1 << 4)

#define RC1_PWRON_BIT                   (1 << 7)
#define RC1_FSYCK_RDY_BIT               (1 << 6)
#define RC1_XCLK_RDY_BIT                (1 << 5)
#define RC1_XCLK_EN_BIT                 (1 << 4)
#define RC1_FSYCK_DIV1                  (0 << 2)
#define RC1_FSYCK_DIV2                  (1 << 2)
#define RC1_FSYCK_DIV4                  (2 << 2)
#define RC1_FSYCK_DIV8                  (3 << 2)
#define RC1_FSCYK_EN                    (1 << 1)
#define RC1_RST_LL                      (1 << 0)

#define IRQ1_RXTO                       (1 << 7)
#define IRQ1_RXFFOW                     (1 << 6)
#define IRQ1_RXDETS_CARRY               (0 << 2)
#define IRQ1_RXDETS_PREM                (1 << 2)
#define IRQ1_RXDETS_SYNC                (2 << 2)
#define IRQ1_IRQCPOR                    (1 << 1)
#define IRQ1_IRQPOR                     (1 << 0)

#define IRQ2_ARKTFIE                    (1 << 7)
#define IRQ2_ATRCTIE                    (1 << 6)
#define IRQ2_FIFOLTIE                   (1 << 5)
#define IRQ2_RXERRIE                    (1 << 4)
#define IRQ2_RXDETIE                    (1 << 3)
#define IRQ2_CALCMPIE                   (1 << 2)
#define IRQ2_RXCMPIE                    (1 << 1)
#define IRQ2_TXCMPIE                    (1 << 0)

#define IRQ3_ARKTFIF                    (1 << 7)
#define IRQ3_ATRCTIF                    (1 << 6)
#define IRQ3_FIFOLTIF                   (1 << 5)
#define IRQ3_RXERRIF                    (1 << 4)
#define IRQ3_RXDETIF                    (1 << 3)
#define IRQ3_CALCMPIF                   (1 << 2)
#define IRQ3_RXCMPIF                    (1 << 1)
#define IRQ3_TXCMPIF                    (1 << 0)

#define IO1_PADDS_0_5MA                 (0 << 6)
#define IO1_PADDS_1MA                   (1 << 6)
#define IO1_PADDS_5MA                   (2 << 6)
#define IO1_PADDS_10MA                  (3 << 6)

#define IO1_GIO2S_INPUT                 (0 << 3)
#define IO1_GIO2S_SDO                   (1 << 3)
#define IO1_GIO2S_TRXD                  (2 << 3)
#define IO1_GIO2S_TXD                   (3 << 3)
#define IO1_GIO2S_RXD                   (4 << 3)
#define IO1_GIO2S_IRQ                   (5 << 3)
#define IO1_GIO2S_ROSCI                 (6 << 3)

#define IO1_GIO1S_INPUT                 (0 << 0)
#define IO1_GIO1S_SDO                   (1 << 0)
#define IO1_GIO1S_TRXD                  (2 << 0)
#define IO1_GIO1S_TXD                   (3 << 0)
#define IO1_GIO1S_RXD                   (4 << 0)
#define IO1_GIO1S_IRQ                   (5 << 0)
#define IO1_GIO1S_ROSCI                 (6 << 0)

#define IO2_GIO4S_INPUT                 (0 << 4)
#define IO2_GIO4S_SDO                   (1 << 4)
#define IO2_GIO4S_TRXD                  (2 << 4)
#define IO2_GIO4S_TXD                   (3 << 4)
#define IO2_GIO4S_RXD                   (4 << 4)
#define IO2_GIO4S_IRQ                   (5 << 4)
#define IO2_GIO4S_ROSCI                 (6 << 4)
#define IO2_GIO4S_TBCLK                 (8 << 4)
#define IO2_GIO4S_RBCLK                 (9 << 4)
#define IO2_GIO4S_FSYCK                 (10 << 4)
#define IO2_GIO4S_LIRCCLK               (11 << 4)
#define IO2_GIO4S_EPA_EN                (12 << 4)
#define IO2_GIO4S_ELAN_EN               (13 << 4)
#define IO2_GIO4S_TRBCLK                (14 << 4)

#define IO2_GIO3S_INPUT                 (0 << 0)
#define IO2_GIO3S_SDO                   (1 << 0)
#define IO2_GIO3S_TRXD                  (2 << 0)
#define IO2_GIO3S_TXD                   (3 << 0)
#define IO2_GIO3S_RXD                   (4 << 0)
#define IO2_GIO3S_IRQ                   (5 << 0)
#define IO2_GIO3S_ROSCI                 (6 << 0)
#define IO2_GIO3S_TBCLK                 (8 << 0)
#define IO2_GIO3S_RBCLK                 (9 << 0)
#define IO2_GIO3S_FSYCK                 (10 << 0)
#define IO2_GIO3S_LIRCCLK               (11 << 0)
#define IO2_GIO3S_EPA_EN                (12 << 0)
#define IO2_GIO3S_ELAN_EN               (13 << 0)
#define IO2_GIO3S_TRBCLK                (14 << 0)

#define IO3_SDO_TEN                     (1 << 7)
#define IO3_SPIPU                       (1 << 6)
#define IO3_GIOPU4                      (1 << 4)                  
#define IO3_GIOPU3                      (1 << 3)                  
#define IO3_GIOPU2                      (1 << 2)                  
#define IO3_GIOPU1                      (1 << 1)                  

#define FIFO2_RXPL2F_EN                 (1 << 4)
#define FIFO2_FFINF_EN                  (1 << 3)
#define FFMG_EN                         (1 << 2)
#define FFMG_4                          (0)
#define FFMG_8                          (1)
#define FFMG_16                         (2)
#define FFMG_32                         (3)

#define PKT2_PID(x)                     (x << 6)
#define PKT2_SYNCLEN(x)                 (x << 2)
#define PKT2_RXPMLEN(x)                 (x)

#define PKT3_MCH_EN                     (1 << 7)
#define PKT3_FEC_EN                     (1 << 6)
#define PKT3_CRC_EN                     (1 << 5)
#define PKT3_CRCFMT                     (1 << 4)
#define PKT3_PLL_EN                     (1 << 3)
#define PKT3_PLLHAC_EN                  (1 << 2)
#define PKT3_PLHLEN                     (1 << 1)
#define PKT3_PLH_EN                     (1 << 0)

#define PKT4_WHT_EN                     (1 << 7)

#define CMD_REG_READ(x)                 (0xC0 | x)
#define CMD_REG_WRITE(x)                (0x40 | x)
#define CMD_SET_REG_BANK(x)             (0x20 | x)
#define CMD_WRITE_SYNC_WORD             (0x10)
#define CMD_READ_SYNC_WORD              (0x90)
#define CMD_WRITE_TX_FIFO               (0x11)
#define CMD_READ_RX_FIFO                (0x91)
#define CMD_SOFT_RESET                  (0x08)
#define CMD_TXFIFO_POINTER_RESET        (0x09)
#define CMD_RXFIFO_POINTER_RESET        (0x89)
#define CMD_DEEPSLEEP                   (0x0A)
#define CMD_IDLE_MODE                   (0x0B)
#define CMD_LIGHT_SLEEP                 (0x0C)
#define CMD_STANDBY                     (0x0D)
#define CMD_TX_MODE                     (0x0E)
#define CMD_RX_MODE                     (0x1E)


#endif