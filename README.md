# same70-can
Implementation of CAN driver for SAME70

In order to use this driver you must:

[0] Add this driver to your project (include "same70_can_driver.h" where you need to use it).

[1] Have the dependency "c-utils" in your project that you can find: https://github.com/LentzTechMaster/c-utils.git

[2] Add MCAN to your project using the ASF Wizard.

[3] You have to setup the conf_mcan.h like the following:

It has to be exactly the same for baudrate and clock settings. The remaining should not be changed except for: CONF_MCAN_ELEMENT_DATA_SIZE, CONF_MCANx_RX_STANDARD_ID_FILTER_NUM and CONF_MCANx_RX_EXTENDED_ID_FILTER_NUM that you can change at your will.
```
#ifndef CONF_MCAN_H_INCLUDED
#define CONF_MCAN_H_INCLUDED

/*
 * Below is the message RAM setting, it will be stored in the system RAM.
 * Please adjust the message size according to your application.
 */
/** Range: 1..64 */ 
#define CONF_MCAN0_RX_FIFO_0_NUM         16     
/** Range: 1..64 */        
#define CONF_MCAN0_RX_FIFO_1_NUM         16      
/** Range: 1..64 */      
#define CONF_MCAN0_RX_BUFFER_NUM         1
/** Range: 1..16 */
#define CONF_MCAN0_TX_BUFFER_NUM         1   
/** Range: 1..16 */        
#define  CONF_MCAN0_TX_FIFO_QUEUE_NUM     1     
/** Range: 1..32 */       
#define CONF_MCAN0_TX_EVENT_FIFO         (CONF_MCAN0_TX_BUFFER_NUM + CONF_MCAN0_TX_FIFO_QUEUE_NUM)             
/** Range: 1..128 */
#define CONF_MCAN0_RX_STANDARD_ID_FILTER_NUM     32    
/** Range: 1..64 */
#define CONF_MCAN0_RX_EXTENDED_ID_FILTER_NUM     16    
/** Range: 1..64 */
#define CONF_MCAN1_RX_FIFO_0_NUM         16             
/** Range: 1..64 */
#define CONF_MCAN1_RX_FIFO_1_NUM         16  
/** Range: 1..64 */          
#define CONF_MCAN1_RX_BUFFER_NUM         1      
/** Range: 1..16 */     
#define CONF_MCAN1_TX_BUFFER_NUM         1 
/** Range: 1..16 */     
#define CONF_MCAN1_TX_FIFO_QUEUE_NUM     1     
/** Range: 1..32 */        
#define CONF_MCAN1_TX_EVENT_FIFO         (CONF_MCAN1_TX_BUFFER_NUM + CONF_MCAN1_TX_FIFO_QUEUE_NUM)             
/** Range: 1..128 */
#define CONF_MCAN1_RX_STANDARD_ID_FILTER_NUM     32    
/** Range: 1..64 */
#define CONF_MCAN1_RX_EXTENDED_ID_FILTER_NUM     16    

/** The value should be 8/12/16/20/24/32/48/64. */
#define CONF_MCAN_ELEMENT_DATA_SIZE         8

/**
 * The setting of the nominal bit rate is based on the PCK5 which is 30M which you can
 * change in the conf_clock.h. Below is the default configuration. The
 * time quanta is 30MHz / (2+1) =  10MHz. And each bit is (1 + NTSEG1 + 1 + NTSEG2 + 1) = 20 time
 * quanta which means the bit rate is 10MHz/20=500KHz.
 */
/** Nominal bit Baud Rate Prescaler */
//#define CONF_MCAN_NBTP_NBRP_VALUE    2
//The UPLL Clock has been configured in the mcan driver to run at 80 MHz in MCANx_configure(). We divide it by 8 to get 10 MHz.
#define CONF_MCAN_NBTP_NBRP_VALUE      7//7 correspond to a division by 8
/** Nominal bit (Re)Synchronization Jump Width */
//#define CONF_MCAN_NBTP_NSJW_VALUE    3
#define CONF_MCAN_NBTP_NSJW_VALUE      3
/** Nominal bit Time segment before sample point */
//#define CONF_MCAN_NBTP_NTSEG1_VALUE  10
#define CONF_MCAN_NBTP_NTSEG1_VALUE    10
/** Nominal bit Time segment after sample point */
//#define CONF_MCAN_NBTP_NTSEG2_VALUE  7
#define CONF_MCAN_NBTP_NTSEG2_VALUE    7

/*
 * The setting of the data bit rate is based on the GCLK_MCAN is 48M which you can
 * change in the conf_clock.h. Below is the default configuration. The
 * time quanta is 48MHz / (5+1) =  8MHz. And each bit is (1 + FTSEG1 + 1 + FTSEG2 + 1) = 16 time
 * quanta which means the bit rate is 8MHz/16=500KHz.
 */
/** Data bit Baud Rate Prescaler */
#define CONF_MCAN_FBTP_FBRP_VALUE    5
/** Data bit (Re)Synchronization Jump Width */
#define CONF_MCAN_FBTP_FSJW_VALUE    3
/** Data bit Time segment before sample point */
#define CONF_MCAN_FBTP_FTSEG1_VALUE  10
/** Data bit Time segment after sample point */
#define CONF_MCAN_FBTP_FTSEG2_VALUE  7

#endif
```

[4] Activate the ports for MCAN0 and MCAN1 by adding these 2 lines inside the conf_board.h:
```
#define CONF_BOARD_CAN0
#define CONF_BOARD_CAN1
```
Remark: If you want to only use one of the 2 CAN lines you can define only the one you will use.

[5] Enjoy!

/!\ /!\ /!\ 

WARNING: This driver does not work when caching is enabled.
This means that "CONF_BOARD_ENABLE_CACHE" SHOULD NOT be defined in conf_board.h.

/!\ /!\ /!\ 
