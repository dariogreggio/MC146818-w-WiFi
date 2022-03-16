// AT WINC 1500 by G.Dar, based upon shit/crap from harmony github and more MERDAAAAA ;) #cancroagliumani #morteaibambini 2021
// PIC16 XC8 version (2022)

#include "at_winc1500.h"
#include "MC146818.h"
#include <stdint.h>
#include <stdio.h>


// nm_common.c	*************************************************************************************
//#include "common/include/nm_common.h"

#define m2m_memcpy(a,b,c) memcpy(a,b,c)
#define m2m_memcmp(a,b,c) memcmp(a,b,c)
#define m2m_memset(a,b,c) memset(a,b,c)

uint8 m2m_checksum(uint8 *buf, int sz) {
	uint8 cs=0;
  
	while(--sz)	{
		cs ^= *buf;
		buf++;
    }

	return cs;
  }



// nm_bsp_samg55.c ****************************************************************************************

//#include "bsp/include/nm_bsp.h"
//#include "common/include/nm_common.h"
#ifdef __CONF_ASF__
//#include "asf.h"
#endif
//#include "config/conf_winc.h"

#define  CUT_OFF 0

/*static*/ tpfNmBspIsr gpfIsr;

#if CONF_MBED_SAMG55_BTN
extern void at_button_cb();
extern void at_oled_button1_cb();
extern void at_oled_button2_cb();
extern void at_oled_button2_cb();


static void sw0_isr(uint32 id, uint32 mask) {
  
	if ((id == ID_PIOA) && (mask == PIO_PA2)) {
		at_button_cb();
    }
  }

static void oled_btn1_isr(uint32 id, uint32 mask) {
	int delay_cnt = 5;
	
	if ((id == ID_PIOB) && (mask == PIO_PB3)) {
		pio_disable_interrupt(PIOB, PIO_PB3);
		at_oled_button1_cb();

		while(delay_cnt--) {
			if ((PIOB->PIO_PDSR & mask) != PIO_PB3) {
				nm_bsp_sleep(100);
				if ((PIOB->PIO_PDSR & mask) != PIO_PB3) {
					nm_bsp_sleep(100);
					//printf("johnny d_cnt 1 = %d\n\r",delay_cnt);
					break;
          }
        }
			nm_bsp_sleep(100);
      }
		//printf("johnny d_cnt 2 = %d\n\r",delay_cnt);
		pio_enable_interrupt(PIOB, PIO_PB3);
    }	
  }

static void oled_btn2_isr(uint32 id, uint32 mask) {
	/*if ((id == ID_PIOA) && (mask == PIO_PA19)) {
		at_oled_button2_cb();
	}*/
	
	int delay_cnt = 5;
	
	if((id == ID_PIOA) && (mask == PIO_PA19)) {
		pio_disable_interrupt(PIOB, PIO_PA19);
		at_oled_button2_cb();

		while(delay_cnt--) {
			if ((PIOA->PIO_PDSR & mask) != PIO_PA19) {
				nm_bsp_sleep(100);
				if ((PIOA->PIO_PDSR & mask) != PIO_PA19) {
					nm_bsp_sleep(100);
					//printf("johnny d_cnt 1 = %d\n\r",delay_cnt);
					break;
				}
        }
			nm_bsp_sleep(100);
      }
		//printf("johnny d_cnt 2 = %d\n\r",delay_cnt);
		pio_enable_interrupt(PIOA, PIO_PA19);
    }
  }

static void oled_btn3_isr(uint32 id, uint32 mask) {
	int delay_cnt = 5;

	if ((id == ID_PIOA) && (mask == PIO_PA20)) {
		pio_disable_interrupt(PIOA, PIO_PA20);
		at_oled_button3_cb();

		while(delay_cnt--) {
			if ((PIOA->PIO_PDSR & mask) != PIO_PA20) {
				nm_bsp_sleep(100);
				if ((PIOA->PIO_PDSR & mask) != PIO_PA20) {
					nm_bsp_sleep(100);
					//printf("johnny d_cnt 1 = %d\n\r",delay_cnt);
					break;
				}
			}
			nm_bsp_sleep(100);
    	}
		//printf("johnny d_cnt 2 = %d\n\r",delay_cnt);
		pio_enable_interrupt(PIOA, PIO_PA20);
    }
  }
#endif


static void chip_isr(uint32_t id, uint32_t mask) {
  
//	if((id == CONF_WINC_SPI_INT_PIO_ID) && (mask == CONF_WINC_SPI_INT_MASK)) {
		if(gpfIsr) {
			gpfIsr();
		}
//	}
  }

/*
 *	@fn		init_chip_pins
 *	@brief	Initialize reset, chip enable and wake pin
 */
static void init_chip_pins(void) {
  
//	ioport_init();
//	ioport_set_pin_dir(CONF_WINC_PIN_RESET, IOPORT_DIR_OUTPUT);
  WINCResetTris=0;
//	ioport_set_pin_level(CONF_WINC_PIN_RESET, IOPORT_PIN_LEVEL_HIGH);
  m_WINCResetBit=1;
//	ioport_set_pin_dir(CONF_WINC_PIN_CHIP_ENABLE, IOPORT_DIR_OUTPUT);
//  WINCCETris=0;
  SPICSTris=0;
//	ioport_set_pin_level(CONF_WINC_PIN_CHIP_ENABLE, IOPORT_PIN_LEVEL_HIGH);
//  m_WINCCEBit=1;
  m_SPICSBit=1;
//	ioport_set_pin_dir(CONF_WINC_PIN_WAKE, IOPORT_DIR_OUTPUT);
//  WINCWakeTris=0;
//	ioport_set_pin_level(CONF_WINC_PIN_WAKE, IOPORT_PIN_LEVEL_HIGH);
//  m_WINCWakeBit=1;
  }

/*
*	@fn		nm_bsp_init
*	@brief	Initialize BSP
*	@return	0 in case of success and -1 in case of failure
*/
sint8 nm_bsp_init(void) {
  
	gpfIsr = NULL;

	/* Initialize chip IOs. */
	init_chip_pins();

    /* Make sure a 1ms Systick is configured. */
//    if (!(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk && SysTick->CTRL & SysTick_CTRL_TICKINT_Msk)) {
//	    delay_init();
//    }

	/* Perform chip reset. */
	nm_bsp_reset();

	return 0;
  }

/**
 *	@fn		nm_bsp_reset
 *	@brief	Reset WINC1500 SoC by setting CHIP_EN and RESET_N signals low,
 *           CHIP_EN high then RESET_N high
 */
void nm_bsp_reset(void) {
  
//ioport_set_pin_level(CONF_WINC_PIN_CHIP_ENABLE, IOPORT_PIN_LEVEL_LOW);
  m_SPICSBit=0;
//  ioport_set_pin_level(CONF_WINC_PIN_RESET, IOPORT_PIN_LEVEL_LOW);
  m_WINCResetBit=0;
	nm_bsp_sleep(100);
//	ioport_set_pin_level(CONF_WINC_PIN_CHIP_ENABLE, IOPORT_PIN_LEVEL_HIGH);
  m_SPICSBit=1;
	nm_bsp_sleep(100);
//	ioport_set_pin_level(CONF_WINC_PIN_RESET, IOPORT_PIN_LEVEL_HIGH);
  m_WINCResetBit=1;
  nm_bsp_sleep(100);
  
  }

/*
*	@fn		nm_bsp_sleep
*	@brief	Sleep in units of mSec
*	@param[IN]	u32TimeMsec
*				Time in milliseconds
*/
void nm_bsp_sleep(uint32 u32TimeMsec) {
  
	while(u32TimeMsec--) {
		__delay_ms(1);
    }
  }

/*
*	@fn		nm_bsp_register_isr
*	@brief	Register interrupt service routine
*	@param[IN]	pfIsr
*				Pointer to ISR handler
*/
void nm_bsp_register_isr(tpfNmBspIsr pfIsr) {
  
	gpfIsr = pfIsr;


	/* Configure PGIO pin for interrupt from SPI slave, used when slave has data to send. */
	//pmc_enable_periph_clk(CONF_WINC_SPI_INT_PIO_ID);
//	pio_configure_pin(CONF_WINC_SPI_INT_PIN, PIO_TYPE_PIO_INPUT);
//	pio_pull_up(CONF_WINC_SPI_INT_PIO, CONF_WINC_SPI_INT_MASK, PIO_PULLUP);
	/*Interrupt on falling edge*/
//	pio_handler_set(CONF_WINC_SPI_INT_PIO, CONF_WINC_SPI_INT_PIO_ID,
//	CONF_WINC_SPI_INT_MASK, PIO_PULLUP | PIO_IT_FALL_EDGE, chip_isr);
//	pio_enable_interrupt(CONF_WINC_SPI_INT_PIO, CONF_WINC_SPI_INT_MASK);
//	NVIC_EnableIRQ((IRQn_Type) CONF_WINC_SPI_INT_PIO_ID);
//	pio_handler_set_priority(CONF_WINC_SPI_INT_PIO, (IRQn_Type)CONF_WINC_SPI_INT_PIO_ID,
//			CONF_WINC_SPI_INT_PRIORITY);
  
//  v. anche enable_interrupt
    WINCIRQTris=1;
/*    IPC29bits.CNBIP=5; IPC29bits.CNBIS=0;
    CNPUBbits.CNPUB15=1;   // WINC IRQ pullup
    CNENBbits.CNIEB15=0;
    CNNEBbits.CNNEB15=1;
    CNCONBbits.ON=1; CNCONBbits.EDGEDETECT=1;
    IFS3bits.CNBIF=0;
    IEC3bits.CNBIE=1;
 v. in main, 
 */
//    CNPUFbits.CNPUF0=1;   // IRQ pullup
    CNENFbits.CNIEF0=0;
    CNNEFbits.CNNEF0=1;

#if CONF_MBED_SAMG55_BTN
#if 0
	// SW0 Button
	pio_configure_pin(IOPORT_CREATE_PIN(PIOA, 2), PIO_TYPE_PIO_INPUT);
	pio_pull_up(PIOA, PIO_PA2, PIO_PULLUP);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA2, PIO_PULLUP | PIO_IT_FALL_EDGE, sw0_isr);
	pio_enable_interrupt(PIOA, PIO_PA2);
	NVIC_EnableIRQ((IRQn_Type) ID_PIOA);
	//pio_handler_set_priority(PIOA, (IRQn_Type)ID_PIOA, CONF_WINC_SPI_INT_PRIORITY);

	// OLED Button1
	pio_configure_pin(IOPORT_CREATE_PIN(PIOB, 3), PIO_TYPE_PIO_INPUT);
	pio_pull_up(PIOB, PIO_PB3, PIO_PULLUP);
	pio_handler_set(PIOB, ID_PIOB, PIO_PB3, PIO_PULLUP | PIO_IT_FALL_EDGE, oled_btn1_isr);
	pio_enable_interrupt(PIOB, PIO_PB3);
	NVIC_EnableIRQ((IRQn_Type) ID_PIOB);
	//pio_handler_set_priority(PIOA, (IRQn_Type)ID_PIOA, CONF_WINC_SPI_INT_PRIORITY);
	
	// OLED Button2
	pio_configure_pin(IOPORT_CREATE_PIN(PIOA, 19), PIO_TYPE_PIO_INPUT);
	pio_pull_up(PIOA, PIO_PA19, PIO_PULLUP);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA19, PIO_PULLUP | PIO_IT_FALL_EDGE, oled_btn2_isr);
	pio_enable_interrupt(PIOA, PIO_PA19);
	NVIC_EnableIRQ((IRQn_Type) ID_PIOA);
	//pio_handler_set_priority(PIOA, (IRQn_Type)ID_PIOA, CONF_WINC_SPI_INT_PRIORITY);

	// OLED Button3
	pio_configure_pin(IOPORT_CREATE_PIN(PIOA, 20), PIO_TYPE_PIO_INPUT);
	pio_pull_up(PIOA, PIO_PA20, PIO_PULLUP);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA20, PIO_PULLUP | PIO_IT_FALL_EDGE, oled_btn3_isr);
	pio_enable_interrupt(PIOA, PIO_PA20);
	NVIC_EnableIRQ((IRQn_Type) ID_PIOA);
	//pio_handler_set_priority(PIOA, (IRQn_Type)ID_PIOA, CONF_WINC_SPI_INT_PRIORITY);
#else
	// SW0 Button
	/* Adjust PIO debounce filter parameters, using 10 Hz filter. */
    pio_set_debounce_filter(PIOA, PIO_PA2, 10);
    /* Initialize PIO interrupt handlers, see PIO definition in board.h. */
    pio_handler_set(PIOA, ID_PIOA, PIO_PA2, PIO_PULLUP | PIO_IT_RISE_EDGE, sw0_isr);
    /* Enable PIO controller IRQs. */
    NVIC_EnableIRQ((IRQn_Type)ID_PIOA);
    /* Enable PIO line interrupts. */
    pio_enable_interrupt(PIOA, PIO_PA2);
	

	// OLED Button1
	/* Adjust PIO debounce filter parameters, using 10 Hz filter. */
    pio_set_debounce_filter(PIOB, PIO_PB3, CUT_OFF);
    /* Initialize PIO interrupt handlers, see PIO definition in board.h. */
    pio_handler_set(PIOB, ID_PIOB, PIO_PB3, PIO_PULLUP | PIO_IT_RISE_EDGE, oled_btn1_isr);
    /* Enable PIO controller IRQs. */
    NVIC_EnableIRQ((IRQn_Type) ID_PIOB);
    /* Enable PIO line interrupts. */
    pio_enable_interrupt(PIOB, PIO_PB3);
	
	
	// OLED Button2
	/* Adjust PIO debounce filter parameters, using 10 Hz filter. */
    pio_set_debounce_filter(PIOA, PIO_PA19, CUT_OFF);
    /* Initialize PIO interrupt handlers, see PIO definition in board.h. */
    pio_handler_set(PIOA, ID_PIOA, PIO_PA19, PIO_PULLUP | PIO_IT_RISE_EDGE, oled_btn2_isr);
    /* Enable PIO controller IRQs. */
    NVIC_EnableIRQ((IRQn_Type) ID_PIOA);
    /* Enable PIO line interrupts. */
    pio_enable_interrupt(PIOA, PIO_PA19);


	// OLED Button3
	/* Adjust PIO debounce filter parameters, using 10 Hz filter. */
    pio_set_debounce_filter(PIOA, PIO_PA20, CUT_OFF);
    /* Initialize PIO interrupt handlers, see PIO definition in board.h. */
    pio_handler_set(PIOA, ID_PIOA, PIO_PA20, PIO_PULLUP | PIO_IT_RISE_EDGE, oled_btn3_isr);
    /* Enable PIO controller IRQs. */
    NVIC_EnableIRQ((IRQn_Type) ID_PIOA);
    /* Enable PIO line interrupts. */
    pio_enable_interrupt(PIOA, PIO_PA20);

#endif
    
  WINCIRQTris=1;
  CNPUBbits.CNPUB8=1;   // WINC IRQ pullup
  IPC2bits.CNBIP=5; IPC2bits.CNBIS=0;

#endif
	
  }

/*
*	@fn		nm_bsp_interrupt_ctrl
*	@brief	Enable/Disable interrupts
*	@param[IN]	u8Enable
*				'0' disable interrupts. '1' enable interrupts
*/
void nm_bsp_interrupt_ctrl(uint8 u8Enable) {
  
	if(u8Enable) {
//		pio_enable_interrupt(CONF_WINC_SPI_INT_PIO, CONF_WINC_SPI_INT_MASK);
    WINCIRQTris=1;
    CNENFbits.CNIEF0=0;
    CNNEFbits.CNNEF0=1;
    // v. altrove
    }
	else {
//		pio_disable_interrupt(CONF_WINC_SPI_INT_PIO, CONF_WINC_SPI_INT_MASK);
    WINCIRQTris=1;
    CNENFbits.CNIEF1=0;
    CNNEFbits.CNNEF1=0;
    // v. altrove
    }
  }



// m2m_ate_mode.c *************************************************************************************
#ifdef _M2M_ATE_FW_
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include "driver/include/m2m_ate_mode.h"
#include "driver/source/nmasic.h"
#include "driver/source/nmdrv.h"
#include "m2m_hif.h"
#include "driver/source/nmbus.h"
#include "bsp/include/nm_bsp.h"

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#define rInterrupt_CORTUS_0				(0x10a8)
#define rInterrupt_CORTUS_1				(0x10ac)
#define rInterrupt_CORTUS_2				(0x10b0)

#define rBurstTx_NMI_TX_RATE				(0x161d00)
#define rBurstTx_NMI_NUM_TX_FRAMES			(0x161d04)
#define rBurstTx_NMI_TX_FRAME_LEN			(0x161d08)
#define rBurstTx_NMI_TX_CW_PARAM			(0x161d0c)
#define rBurstTx_NMI_TX_GAIN				(0x161d10)
#define rBurstTx_NMI_TX_DPD_CTRL			(0x161d14)
#define rBurstTx_NMI_USE_PMU				(0x161d18)
#define rBurstTx_NMI_TEST_CH				(0x161d1c)
#define rBurstTx_NMI_TX_PHY_CONT			(0x161d20)
#define rBurstTx_NMI_TX_CW_MODE				(0x161d24)
#define rBurstTx_NMI_TEST_XO_OFF			(0x161d28)
#define rBurstTx_NMI_USE_EFUSE_XO_OFF 		(0x161d2c)

#define rBurstTx_NMI_MAC_FILTER_ENABLE_DA 	(0x161d30)
#define rBurstTx_NMI_MAC_ADDR_LO_PEER 		(0x161d34)
#define rBurstTx_NMI_MAC_ADDR_LO_SELF 		(0x161d38)
#define rBurstTx_NMI_MAC_ADDR_HI_PEER 		(0x161d3c)
#define rBurstTx_NMI_MAC_ADDR_HI_SELF		(0x161d40)
#define rBurstTx_NMI_RX_PKT_CNT_SUCCESS 	(0x161d44)
#define rBurstTx_NMI_RX_PKT_CNT_FAIL 		(0x161d48)
#define rBurstTx_NMI_SET_SELF_MAC_ADDR 		(0x161d4c)
#define rBurstTx_NMI_MAC_ADDR_LO_SA 		(0x161d50)
#define rBurstTx_NMI_MAC_ADDR_HI_SA 		(0x161d54)
#define rBurstTx_NMI_MAC_FILTER_ENABLE_SA 	(0x161d58)

#define rBurstRx_NMI_RX_ALL_PKTS_CONT	(0x9898)
#define rBurstRx_NMI_RX_ERR_PKTS_CONT	(0x988c)

#define TX_DGAIN_MAX_NUM_REGS			(4)
#define TX_DGAIN_REG_BASE_ADDRESS		(0x1240)
#define TX_GAIN_CODE_MAX_NUM_REGS		(3)
#define TX_GAIN_CODE_BASE_ADDRESS		(0x1250)
#define TX_PA_MAX_NUM_REGS				(3)
#define TX_PA_BASE_ADDRESS				(0x1e58)
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
VARIABLES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
volatile static uint8	gu8AteIsRunning	= 0;	/*!< ATE firmware status, 1 means ATE is running otherwise stopped */
volatile static uint8	gu8RxState		= 0;	/*!< RX status, 1 means Rx is running otherwise stopped */
volatile static uint8	gu8TxState		= 0;	/*!< TX status, 1 means Tx is running otherwise stopped */
volatile static uint32	gaAteFwTxRates[M2M_ATE_MAX_NUM_OF_RATES] = {
	0x01, 0x02, 0x05, 0x0B,							/*B-Rats*/
	0x06, 0x09, 0x0C, 0x12, 0x18, 0x24, 0x30, 0x36,	/*G-Rats*/
	0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87	/*N-Rats*/
};

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
STATIC FUNCTIONS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
static void m2m_ate_set_rx_status(uint8 u8Value) {
  
	gu8RxState = u8Value;
  }

static void m2m_ate_set_tx_status(uint8 u8Value) {
	gu8TxState = u8Value;
}

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION IMPLEMENTATION
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/*!
@fn	\
	sint8 m2m_ate_init(void);

@brief
	This function used to download ATE firmware from flash and start it

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
sint8 m2m_ate_init(void) {
	sint8 s8Ret = M2M_SUCCESS;
	uint8 u8WifiMode = M2M_WIFI_MODE_ATE_HIGH;
	
	s8Ret = nm_drv_init(&u8WifiMode);
	
	return s8Ret;
  }

/*!
@fn	\
	sint8 m2m_ate_init(tstrM2mAteInit *pstrInit);

@brief
	This function used to download ATE firmware from flash and start it

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
sint8 m2m_ate_init_param(tstrM2mAteInit *pstrInit) {
	sint8 s8Ret = M2M_SUCCESS;
	
	s8Ret = nm_drv_init((void*)&pstrInit->u8RxPwrMode);
	
	return s8Ret;
  }

/*!
@fn	\
	sint8 m2m_ate_deinit(void);

@brief
	De-Initialization of ATE firmware mode 

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
sint8 m2m_ate_deinit(void) {
  
	return nm_drv_deinit(NULL);
  }

/*!
@fn	\
	sint8 m2m_ate_set_fw_state(uint8);

@brief
	This function used to change ATE firmware status from running to stopped or vice versa.

@param [in]	u8State
		Required state of ATE firmware, one of \ref tenuM2mAteFwState enumeration values.
@return
	The function SHALL return 0 for success and a negative value otherwise.
\sa
	m2m_ate_init
*/
sint8 m2m_ate_set_fw_state(uint8 u8State) {
	sint8		s8Ret	= M2M_SUCCESS;
	uint32_t	u32Val	= 0;
	
	if((M2M_ATE_FW_STATE_STOP == u8State) && (M2M_ATE_FW_STATE_STOP != gu8AteIsRunning)) {
		u32Val = nm_read_reg(rNMI_GLB_RESET);
		u32Val &= ~(1 << 10);
		s8Ret = nm_write_reg(rNMI_GLB_RESET, u32Val);
		gu8AteIsRunning = M2M_ATE_FW_STATE_STOP;
	}
	else if((M2M_ATE_FW_STATE_RUN == u8State) && (M2M_ATE_FW_STATE_RUN != gu8AteIsRunning))	{
		/* 0x1118[0]=0 at power-on-reset: pad-based control. */
		/* Switch cortus reset register to register control. 0x1118[0]=1. */
		u32Val = nm_read_reg(rNMI_BOOT_RESET_MUX);
		u32Val |= (1 << 0);
		s8Ret = nm_write_reg(rNMI_BOOT_RESET_MUX, u32Val);
		if(M2M_SUCCESS != s8Ret) {
			goto __EXIT;
		}
		/**
		Write the firmware download complete magic value 0x10ADD09E at
		location 0xFFFF000C (Cortus map) or C000C (AHB map).
		This will let the boot-rom code execute from RAM.
		**/
		s8Ret = nm_write_reg(0xc0000, 0x71);
		if(M2M_SUCCESS != s8Ret) {
			goto __EXIT;
		}

		u32Val = nm_read_reg(rNMI_GLB_RESET);
		if((u32Val & (1ul << 10)) == (1ul << 10)) {
			u32Val &= ~(1ul << 10);
			s8Ret = nm_write_reg(rNMI_GLB_RESET, u32Val);
			if(M2M_SUCCESS != s8Ret) {
				goto __EXIT;
			}
		}
		
		u32Val |= (1ul << 10);
		s8Ret = nm_write_reg(rNMI_GLB_RESET, u32Val);
		if(M2M_SUCCESS != s8Ret) {
			goto __EXIT;
      }
		gu8AteIsRunning = M2M_ATE_FW_STATE_RUN;
    }
	else {
		s8Ret = M2M_ATE_ERR_UNHANDLED_CASE;
    }
	
__EXIT:
	if((M2M_SUCCESS == s8Ret) && (M2M_ATE_FW_STATE_RUN == gu8AteIsRunning))	{
		nm_bsp_sleep(500);	/*wait for ATE firmware start up*/
    }
	return s8Ret;
  }

/*!
@fn	\
	sint8 m2m_ate_get_fw_state(uint8);

@brief
	This function used to return status of ATE firmware.

@return
	The function SHALL return status of ATE firmware, one of \ref tenuM2mAteFwState enumeration values.
\sa
	m2m_ate_init, m2m_ate_set_fw_state
*/
sint8 m2m_ate_get_fw_state(void) {
	
  return gu8AteIsRunning;
  }

/*!
@fn	\
	uint32 m2m_ate_get_tx_rate(uint8);

@brief
	This function used to return value of TX rate required by application developer.

@param [in]	u8Index
		Index of required rate , one of \ref tenuM2mAteTxIndexOfRates enumeration values.
@return
	The function SHALL return 0 for in case of failure otherwise selected rate value.
\sa
	tenuM2mAteTxIndexOfRates
*/
uint32 m2m_ate_get_tx_rate(uint8 u8Index) {
  
	if(M2M_ATE_MAX_NUM_OF_RATES <= u8Index)	{
		return 0;
	}
	return gaAteFwTxRates[u8Index];
}

/*!
@fn	\
	sint8 m2m_ate_get_tx_status(void);

@brief
	This function used to return status of TX test case either running or stopped.

@return
	The function SHALL return status of ATE firmware, 1 if TX is running otherwise 0.
\sa
	m2m_ate_start_tx, m2m_ate_stop_tx
*/
sint8 m2m_ate_get_tx_status(void) {
  
	return gu8TxState;
  }

/*!
@fn	\
	sint8 m2m_ate_start_tx(tstrM2mAteTx *)

@brief
	This function used to start TX test case.

@param [in]	strM2mAteTx
		Type of \ref tstrM2mAteTx, with the values required to enable TX test case. You must use \ref m2m_ate_init first.
@return
	The function SHALL return 0 for success and a negative value otherwise.
\sa
	m2m_ate_init, m2m_ate_stop_tx, m2m_ate_get_tx_status
*/
sint8 m2m_ate_start_tx(tstrM2mAteTx *strM2mAteTx) {
	sint8		s8Ret = M2M_SUCCESS;
	uint8	u8LoopCntr = 0;
	uint32_t 	val32;

	
	if(NULL == strM2mAteTx) {
		s8Ret = M2M_ATE_ERR_VALIDATE;
		goto __EXIT;
    }
	
	if(0 != m2m_ate_get_tx_status()) {
		s8Ret = M2M_ATE_ERR_TX_ALREADY_RUNNING;
		goto __EXIT;
    }
	
	if(0 != m2m_ate_get_rx_status()) {
		s8Ret = M2M_ATE_ERR_RX_ALREADY_RUNNING;
		goto __EXIT;
  	}
	
	if(	(strM2mAteTx->channel_num < M2M_ATE_CHANNEL_1) || 
		(strM2mAteTx->channel_num > M2M_ATE_CHANNEL_14) || 
		(strM2mAteTx->tx_gain_sel < M2M_ATE_TX_GAIN_DYNAMIC)	||
		(strM2mAteTx->tx_gain_sel > M2M_ATE_TX_GAIN_TELEC) ||
		(strM2mAteTx->frame_len > M2M_ATE_MAX_FRAME_LENGTH) ||
		(strM2mAteTx->frame_len < M2M_ATE_MIN_FRAME_LENGTH)
    )	{
		s8Ret = M2M_ATE_ERR_VALIDATE;
		goto __EXIT;
    }
	
	if(	(strM2mAteTx->duty_cycle < M2M_ATE_TX_DUTY_MAX_VALUE /*1*/) ||
		(strM2mAteTx->duty_cycle > M2M_ATE_TX_DUTY_MIN_VALUE /*10*/ ) ||
		(strM2mAteTx->dpd_ctrl < M2M_ATE_TX_DPD_DYNAMIC)	||
		(strM2mAteTx->dpd_ctrl > M2M_ATE_TX_DPD_ENABLED)  ||
		(strM2mAteTx->use_pmu < M2M_ATE_PMU_DISBLE)	||
		(strM2mAteTx->use_pmu > M2M_ATE_PMU_ENABLE)	||
		(strM2mAteTx->phy_burst_tx < M2M_ATE_TX_SRC_MAC) ||
		(strM2mAteTx->phy_burst_tx > M2M_ATE_TX_SRC_PHY) ||
		(strM2mAteTx->cw_tx < M2M_ATE_TX_MODE_NORM) ||
		(strM2mAteTx->cw_tx > M2M_ATE_TX_MODE_CW)
	)	{
		s8Ret = M2M_ATE_ERR_VALIDATE;
		goto __EXIT;
    }
	
	for(u8LoopCntr=0; u8LoopCntr<M2M_ATE_MAX_NUM_OF_RATES; u8LoopCntr++) {
		if(gaAteFwTxRates[u8LoopCntr] == strM2mAteTx->data_rate) {
			break;
      }
    }
	
	if(M2M_ATE_MAX_NUM_OF_RATES == u8LoopCntr) {
		s8Ret = M2M_ATE_ERR_VALIDATE;
		goto __EXIT;
    }
	
	s8Ret += nm_write_reg(rBurstTx_NMI_USE_PMU, strM2mAteTx->use_pmu);
	s8Ret += nm_write_reg(rBurstTx_NMI_TX_PHY_CONT, strM2mAteTx->phy_burst_tx);
	s8Ret += nm_write_reg(rBurstTx_NMI_NUM_TX_FRAMES, strM2mAteTx->num_frames);
	s8Ret += nm_write_reg(rBurstTx_NMI_TX_GAIN, strM2mAteTx->tx_gain_sel);
	s8Ret += nm_write_reg(rBurstTx_NMI_TEST_CH, strM2mAteTx->channel_num);
	s8Ret += nm_write_reg(rBurstTx_NMI_TX_FRAME_LEN, strM2mAteTx->frame_len);
	s8Ret += nm_write_reg(rBurstTx_NMI_TX_CW_PARAM, strM2mAteTx->duty_cycle);
	s8Ret += nm_write_reg(rBurstTx_NMI_TX_DPD_CTRL, strM2mAteTx->dpd_ctrl);
	s8Ret += nm_write_reg(rBurstTx_NMI_TX_RATE, strM2mAteTx->data_rate);
	s8Ret += nm_write_reg(rBurstTx_NMI_TX_CW_MODE, strM2mAteTx->cw_tx);
	s8Ret += nm_write_reg(rBurstTx_NMI_TEST_XO_OFF, strM2mAteTx->xo_offset_x1000);
	s8Ret += nm_write_reg(rBurstTx_NMI_USE_EFUSE_XO_OFF, strM2mAteTx->use_efuse_xo_offset);

	val32	 = strM2mAteTx->peer_mac_addr[5]   << 0;
	val32	|= strM2mAteTx->peer_mac_addr[4]  << 8;
	val32	|= strM2mAteTx->peer_mac_addr[3]  << 16;
	nm_write_reg(rBurstTx_NMI_MAC_ADDR_LO_PEER, val32 );
	
	val32	 = strM2mAteTx->peer_mac_addr[2]  << 0;
	val32	|= strM2mAteTx->peer_mac_addr[1] << 8;
	val32	|= strM2mAteTx->peer_mac_addr[0] << 16;
	nm_write_reg(rBurstTx_NMI_MAC_ADDR_HI_PEER, val32 );

	if(M2M_SUCCESS == s8Ret) {
		s8Ret += nm_write_reg(rInterrupt_CORTUS_0, 1);	/*Interrupt Cortus*/
		m2m_ate_set_tx_status(1);
		nm_bsp_sleep(200);  /*Recommended*/	
    }
	
__EXIT:
	return s8Ret;
  }

/*!
@fn	\
	sint8 m2m_ate_stop_tx(void)

@brief
	This function used to stop TX test case.

@return
	The function SHALL return 0 for success and a negative value otherwise.
\sa
	m2m_ate_init, m2m_ate_start_tx, m2m_ate_get_tx_status
*/
sint8 m2m_ate_stop_tx(void) {
	sint8	s8Ret = M2M_SUCCESS;
	
	s8Ret = nm_write_reg(rInterrupt_CORTUS_1, 1);
	if(M2M_SUCCESS == s8Ret)	{
		m2m_ate_set_tx_status(0);
    }
	
	return s8Ret;
  }

/*!
@fn	\
	sint8 m2m_ate_get_rx_status(uint8);

@brief
	This function used to return status of RX test case either running or stopped.

@return
	The function SHALL return status of ATE firmware, 1 if RX is running otherwise 0.
\sa
	m2m_ate_start_rx, m2m_ate_stop_rx
*/
sint8 m2m_ate_get_rx_status(void) {
	return gu8RxState;
  }

/*!
@fn	\
	sint8 m2m_ate_start_rx(tstrM2mAteRx *)

@brief
	This function used to start RX test case.

@param [in]	strM2mAteRx
		Type of \ref tstrM2mAteRx, with the values required to enable RX test case. You must use \ref m2m_ate_init first.
@return
	The function SHALL return 0 for success and a negative value otherwise.
\sa
	m2m_ate_init, m2m_ate_stop_rx, m2m_ate_get_rx_status
*/
sint8 m2m_ate_start_rx(tstrM2mAteRx  *strM2mAteRxStr) {
	sint8		s8Ret = M2M_SUCCESS;
	uint32  	val32;
  
	if(!strM2mAteRxStr) {
		s8Ret = M2M_ATE_ERR_VALIDATE;
		goto __EXIT;
    }
	
	if(0 != m2m_ate_get_tx_status()) {
		s8Ret = M2M_ATE_ERR_TX_ALREADY_RUNNING;
		goto __EXIT;
    }
	
	if(0 != m2m_ate_get_rx_status()) {
		s8Ret = M2M_ATE_ERR_RX_ALREADY_RUNNING;
		goto __EXIT;
  	}
	
	if(	(strM2mAteRxStr->channel_num < M2M_ATE_CHANNEL_1) ||
		(strM2mAteRxStr->channel_num > M2M_ATE_CHANNEL_14)||
		(strM2mAteRxStr->use_pmu < M2M_ATE_PMU_DISBLE)	 ||
		(strM2mAteRxStr->use_pmu > M2M_ATE_PMU_ENABLE)
    )	{
		s8Ret = M2M_ATE_ERR_VALIDATE;
		goto __EXIT;
    }
	
	s8Ret += nm_write_reg(rBurstTx_NMI_TEST_CH, strM2mAteRxStr->channel_num);
	s8Ret += nm_write_reg(rBurstTx_NMI_USE_PMU, strM2mAteRxStr->use_pmu);
	s8Ret += nm_write_reg(rBurstTx_NMI_TEST_XO_OFF, strM2mAteRxStr->xo_offset_x1000);
	s8Ret += nm_write_reg(rBurstTx_NMI_USE_EFUSE_XO_OFF, strM2mAteRxStr->use_efuse_xo_offset);

	if(strM2mAteRxStr->override_self_mac_addr)	{
		val32	 = strM2mAteRxStr->self_mac_addr[5] << 0;
		val32	|= strM2mAteRxStr->self_mac_addr[4] << 8;
		val32	|= strM2mAteRxStr->self_mac_addr[3] << 16;
		nm_write_reg(rBurstTx_NMI_MAC_ADDR_LO_SELF, val32 );

		val32	 = strM2mAteRxStr->self_mac_addr[2] << 0;
		val32	|= strM2mAteRxStr->self_mac_addr[1] << 8;
		val32	|= strM2mAteRxStr->self_mac_addr[0] << 16;
		nm_write_reg(rBurstTx_NMI_MAC_ADDR_HI_SELF, val32 );
	}
	
	if(strM2mAteRxStr->mac_filter_en_sa)	{
		val32	 = strM2mAteRxStr->peer_mac_addr[5] << 0;
		val32	|= strM2mAteRxStr->peer_mac_addr[4] << 8;
		val32	|= strM2mAteRxStr->peer_mac_addr[3] << 16;
		nm_write_reg(rBurstTx_NMI_MAC_ADDR_LO_SA, val32 );
	
		val32	 = strM2mAteRxStr->peer_mac_addr[2] << 0;
		val32	|= strM2mAteRxStr->peer_mac_addr[1] << 8;
		val32	|= strM2mAteRxStr->peer_mac_addr[0] << 16;
		nm_write_reg(rBurstTx_NMI_MAC_ADDR_HI_SA, val32 );
    }
	
	nm_write_reg(rBurstTx_NMI_MAC_FILTER_ENABLE_DA, strM2mAteRxStr->mac_filter_en_da);
	nm_write_reg(rBurstTx_NMI_MAC_FILTER_ENABLE_SA, strM2mAteRxStr->mac_filter_en_sa);
	nm_write_reg(rBurstTx_NMI_SET_SELF_MAC_ADDR, strM2mAteRxStr->override_self_mac_addr);
	
	if(M2M_SUCCESS == s8Ret) {
		s8Ret += nm_write_reg(rInterrupt_CORTUS_2, 1);	/*Interrupt Cortus*/
		m2m_ate_set_rx_status(1);
		nm_bsp_sleep(10);  /*Recommended*/	
    }
	
__EXIT:
	return s8Ret;
  }

/*!
@fn	\
	sint8 m2m_ate_stop_rx(void)

@brief
	This function used to stop RX test case.

@return
	The function SHALL return 0 for success and a negative value otherwise.
\sa
	m2m_ate_init, m2m_ate_start_rx, m2m_ate_get_rx_status
*/
sint8 m2m_ate_stop_rx(void) {
  
	m2m_ate_set_rx_status(0);
	nm_bsp_sleep(200);  /*Recommended*/	
	return M2M_SUCCESS;
  }

/*!
@fn	\
	sint8 m2m_ate_read_rx_status(tstrM2mAteRxStatus *)

@brief
	This function used to read RX statistics from ATE firmware.

@param [out]	strM2mAteRxStatus
		Type of \ref tstrM2mAteRxStatus used to save statistics of RX test case. You must use \ref m2m_ate_start_rx first.
@return
	The function SHALL return 0 for success and a negative value otherwise.
\sa
	m2m_ate_init, m2m_ate_start_rx
*/
sint8 m2m_ate_read_rx_status(tstrM2mAteRxStatus *strM2mAteRxStatus) {
	sint8	s8Ret = M2M_SUCCESS;
	
	if(!strM2mAteRxStatus) {
		s8Ret = M2M_ATE_ERR_VALIDATE;
		goto __EXIT;
	}
	
	if(0 != m2m_ate_get_tx_status()) {
		s8Ret = M2M_ATE_ERR_TX_ALREADY_RUNNING;
		goto __EXIT;
    }

	if(nm_read_reg(rBurstTx_NMI_MAC_FILTER_ENABLE_DA) || nm_read_reg(rBurstTx_NMI_MAC_FILTER_ENABLE_SA)) {
		strM2mAteRxStatus->num_rx_pkts 		= 		nm_read_reg(rBurstTx_NMI_RX_PKT_CNT_SUCCESS) + nm_read_reg(rBurstTx_NMI_RX_PKT_CNT_FAIL);
		strM2mAteRxStatus->num_good_pkts 	= 		nm_read_reg(rBurstTx_NMI_RX_PKT_CNT_SUCCESS);
		strM2mAteRxStatus->num_err_pkts 	= 		nm_read_reg(rBurstTx_NMI_RX_PKT_CNT_FAIL);
    } 
	else {
		strM2mAteRxStatus->num_rx_pkts = nm_read_reg(rBurstRx_NMI_RX_ALL_PKTS_CONT) + nm_read_reg(0x989c);
		strM2mAteRxStatus->num_err_pkts = nm_read_reg(rBurstRx_NMI_RX_ERR_PKTS_CONT);
		strM2mAteRxStatus->num_good_pkts = strM2mAteRxStatus->num_rx_pkts - strM2mAteRxStatus->num_err_pkts;
    }

__EXIT:
	return s8Ret;
  }

/*!
@fn	\
	sint8 m2m_ate_set_dig_gain(double dGaindB)

@brief
	This function is used to set the digital gain

@param [in]	double dGaindB
		The digital gain value required to be set.
@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
sint8 m2m_ate_set_dig_gain(double dGaindB) {
	uint32_t dGain, val32;
  
	dGain = (uint32_t)(pow(10, dGaindB/20.0) * 1024.0);

	val32 = nm_read_reg(0x160cd0);
	val32 &= ~(0x1ffful << 0);
	val32 |= (((uint32_t)dGain) << 0);
	nm_write_reg(0x160cd0, val32);
	return M2M_SUCCESS;
  }

/*!
@fn	\
	sint8 m2m_ate_get_dig_gain(double *dGaindB)

@brief
	This function is used to get the digital gain

@param [out]	double *dGaindB
		The retrieved digital gain value obtained from HW registers in dB.
@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
sint8 m2m_ate_get_dig_gain(double *dGaindB) {
	uint32 dGain, val32;
	
	if(!dGaindB) 
    return M2M_ERR_INVALID_ARG;
	
	val32 = nm_read_reg(0x160cd0);
	
	dGain = (val32 >> 0) & 0x1ffful;
	*dGaindB = 20.0*log10((double)dGain / 1024.0);
	
	return M2M_SUCCESS;
  }

/*!
@fn	\
	sint8 m2m_ate_get_pa_gain(double *paGaindB)

@brief
	This function is used to get the PA gain

@param [out]	double *paGaindB
		The retrieved PA gain value obtained from HW registers in dB.
@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
sint8 m2m_ate_get_pa_gain(double *paGaindB) {
	uint32 val32, paGain;
	uint32 m_cmbPAGainStep;
	
	if(!paGaindB) 
		return M2M_ERR_INVALID_ARG;
	
	val32 = nm_read_reg(0x1e9c);
	
	paGain = (val32 >> 8) & 0x3f;
	
	switch(paGain){
		case 0x1:
      m_cmbPAGainStep = 5;
      break;
		case 0x3:
      m_cmbPAGainStep = 4;
      break;
		case 0x7:
      m_cmbPAGainStep = 3;
      break;
		case 0xf:
      m_cmbPAGainStep = 2;
      break;
		case 0x1f:
      m_cmbPAGainStep = 1;
      break;
		case 0x3f:
      m_cmbPAGainStep = 0;
      break;
		default:
      m_cmbPAGainStep = 0;
      break;
    }
	
	*paGaindB = 18 - m_cmbPAGainStep*3;

	return M2M_SUCCESS;
  }

/*!
@fn	\
	sint8 m2m_ate_get_ppa_gain(double *ppaGaindB)

@brief
	This function is used to get the PPA gain

@param [out]	uint32 * ppaGaindB
		The retrieved PPA gain value obtained from HW registers in dB.
@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
sint8 m2m_ate_get_ppa_gain(double *ppaGaindB) {
	uint32 val32, ppaGain, m_cmbPPAGainStep;
	
	if(!ppaGaindB) 
    return M2M_ERR_INVALID_ARG;

	val32 = nm_read_reg(0x1ea0);
		
	ppaGain = (val32 >> 5) & 0x7;
	
	switch(ppaGain){
		case 0x1:
      m_cmbPPAGainStep = 2;
      break;
		case 0x3:
      m_cmbPPAGainStep = 1;
      break;
		case 0x7:
      m_cmbPPAGainStep = 0;
      break;
		default:
      m_cmbPPAGainStep = 3;
      break;
    } 
	
	*ppaGaindB = 9 - m_cmbPPAGainStep*3;
				
	return M2M_SUCCESS;
  }

/*!
@fn	\
	sint8 m2m_ate_get_tot_gain(double *totGaindB)

@brief
	This function is used to calculate the total gain

@param [out]	double *totGaindB
		The retrieved total gain value obtained from calculations made based on the digital gain, PA and PPA gain values.
@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
sint8 m2m_ate_get_tot_gain(double *totGaindB) {
	double dGaindB, paGaindB, ppaGaindB;	
	
	if(!totGaindB) 
    return M2M_ERR_INVALID_ARG;
	
	m2m_ate_get_pa_gain(&paGaindB);
	m2m_ate_get_ppa_gain(&ppaGaindB);
	m2m_ate_get_dig_gain(&dGaindB);
	
	*totGaindB = dGaindB + paGaindB + ppaGaindB;
	
	return M2M_SUCCESS;
  }

#endif //_M2M_ATE_FW_


// m2m_crypto.c *************************************************************************************
//#include "driver/include/m2m_crypto.h"
//#include "driver/source/nmbus.h"
//#include "driver/source/nmasic.h"

#ifdef CONF_CRYPTO_HW

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/*======*======*======*======*======*=======*
* WINC SHA256 HW Engine Register Definition * 
*======*======*======*======*======*========*/

#define SHA_BLOCK_SIZE											(64)

#define SHARED_MEM_BASE											(0xd0000)


#define SHA256_MEM_BASE											(0x180000UL)
#define SHA256_ENGINE_ADDR										(0x180000ul)

/* SHA256 Registers */
#define SHA256_CTRL												(SHA256_MEM_BASE+0x00)
#define SHA256_CTRL_START_CALC_MASK								(NBIT0)
#define SHA256_CTRL_START_CALC_SHIFT							(0)
#define SHA256_CTRL_PREPROCESS_MASK								(NBIT1)
#define SHA256_CTRL_PREPROCESS_SHIFT							(1)
#define SHA256_CTRL_HASH_HASH_MASK								(NBIT2)
#define SHA256_CTRL_HASH_HASH_SHIFT								(2)
#define SHA256_CTRL_INIT_SHA256_STATE_MASK						(NBIT3)
#define SHA256_CTRL_INIT_SHA256_STATE_SHIFT						(3)
#define SHA256_CTRL_WR_BACK_HASH_VALUE_MASK						(NBIT4)
#define SHA256_CTRL_WR_BACK_HASH_VALUE_SHIFT					(4)
#define SHA256_CTRL_FORCE_SHA256_QUIT_MASK						(NBIT5)
#define SHA256_CTRL_FORCE_SHA256_QUIT_SHIFT						(5)

#define SHA256_REGS_SHA256_CTRL_AHB_BYTE_REV_EN                 (NBIT6)
#define SHA256_REGS_SHA256_CTRL_RESERVED						(NBIT7)
#define SHA256_REGS_SHA256_CTRL_CORE_TO_AHB_CLK_RATIO			(NBIT8+ NBIT9+ NBIT10)
#define SHA256_REGS_SHA256_CTRL_CORE_TO_AHB_CLK_RATIO_MASK		(NBIT2+ NBIT1+ NBIT0)
#define SHA256_REGS_SHA256_CTRL_CORE_TO_AHB_CLK_RATIO_SHIFT		(8)
#define SHA256_REGS_SHA256_CTRL_RESERVED_11						(NBIT11)
#define SHA256_REGS_SHA256_CTRL_SHA1_CALC                       (NBIT12)
#define SHA256_REGS_SHA256_CTRL_PBKDF2_SHA1_CALC                (NBIT13)


#define SHA256_START_RD_ADDR									(SHA256_MEM_BASE+0x04UL)
#define SHA256_DATA_LENGTH										(SHA256_MEM_BASE+0x08UL)
#define SHA256_START_WR_ADDR									(SHA256_MEM_BASE+0x0cUL)
#define SHA256_COND_CHK_CTRL									(SHA256_MEM_BASE+0x10)
#define SHA256_COND_CHK_CTRL_HASH_VAL_COND_CHK_MASK				(NBIT1 | NBIT0)
#define SHA256_COND_CHK_CTRL_HASH_VAL_COND_CHK_SHIFT			(0)
#define SHA256_COND_CHK_CTRL_STEP_VAL_MASK						(NBIT6 | NBIT5 | NBIT4 | NBIT3 | NBIT2)
#define SHA256_COND_CHK_CTRL_STEP_VAL_SHIFT						(2)
#define SHA256_COND_CHK_CTRL_COND_CHK_RESULT_MASK				(NBIT7)
#define SHA256_COND_CHK_CTRL_COND_CHK_RESULT_SHIFT				(7)

#define SHA256_MOD_DATA_RANGE									(SHA256_MEM_BASE+0x14)
#define SHA256_MOD_DATA_RANGE_ST_BYTE_2_ADD_STP_MASK			(NBIT24-1)
#define SHA256_MOD_DATA_RANGE_ST_BYTE_2_ADD_STP_SHIFT			(0)
#define SHA256_MOD_DATA_RANGE_MOD_DATA_LEN_MASK					(NBIT24 | NBIT25| NBIT26)
#define SHA256_MOD_DATA_RANGE_MOD_DATA_LEN_SHIFT				(24)


#define SHA256_COND_CHK_STS_1									(SHA256_MEM_BASE+0x18)
#define SHA256_COND_CHK_STS_2									(SHA256_MEM_BASE+0x1c)
#define SHA256_DONE_INTR_ENABLE									(SHA256_MEM_BASE+0x20)
#define SHA256_DONE_INTR_STS									(SHA256_MEM_BASE+0x24)
#define SHA256_TARGET_HASH_H1									(SHA256_MEM_BASE+0x28)
#define SHA256_TARGET_HASH_H2									(SHA256_MEM_BASE+0x2c)
#define SHA256_TARGET_HASH_H3									(SHA256_MEM_BASE+0x30)
#define SHA256_TARGET_HASH_H4									(SHA256_MEM_BASE+0x34)
#define SHA256_TARGET_HASH_H5									(SHA256_MEM_BASE+0x38)
#define SHA256_TARGET_HASH_H6									(SHA256_MEM_BASE+0x3c)
#define SHA256_TARGET_HASH_H7									(SHA256_MEM_BASE+0x40)
#define SHA256_TARGET_HASH_H8									(SHA256_MEM_BASE+0x44)

/*======*======*======*======*======*=======*
* WINC BIGINT HW Engine Register Definition *
*======*======*======*======*======*========*/


#define BIGINT_ENGINE_ADDR							(0x180080ul)
#define BIGINT_VERSION								(BIGINT_ENGINE_ADDR + 0x00)

#define BIGINT_MISC_CTRL							(BIGINT_ENGINE_ADDR + 0x04)
#define BIGINT_MISC_CTRL_CTL_START					(NBIT0)
#define BIGINT_MISC_CTRL_CTL_RESET					(NBIT1)
#define BIGINT_MISC_CTRL_CTL_MSW_FIRST				(NBIT2)
#define BIGINT_MISC_CTRL_CTL_SWAP_BYTE_ORDER		(NBIT3)
#define BIGINT_MISC_CTRL_CTL_FORCE_BARRETT			(NBIT4)
#define BIGINT_MISC_CTRL_CTL_M_PRIME_VALID			(NBIT5)

#define BIGINT_M_PRIME								(BIGINT_ENGINE_ADDR + 0x08)

#define BIGINT_STATUS								(BIGINT_ENGINE_ADDR + 0x0C)
#define BIGINT_STATUS_STS_DONE						(NBIT0)

#define BIGINT_CLK_COUNT							(BIGINT_ENGINE_ADDR + 0x10)
#define BIGINT_ADDR_X								(BIGINT_ENGINE_ADDR + 0x14)
#define BIGINT_ADDR_E								(BIGINT_ENGINE_ADDR + 0x18)
#define BIGINT_ADDR_M								(BIGINT_ENGINE_ADDR + 0x1C)
#define BIGINT_ADDR_R								(BIGINT_ENGINE_ADDR + 0x20)
#define BIGINT_LENGTH								(BIGINT_ENGINE_ADDR + 0x24)

#define BIGINT_IRQ_STS								(BIGINT_ENGINE_ADDR + 0x28)
#define BIGINT_IRQ_STS_DONE							(NBIT0)
#define BIGINT_IRQ_STS_CHOOSE_MONT					(NBIT1)
#define BIGINT_IRQ_STS_M_READ						(NBIT2)
#define BIGINT_IRQ_STS_X_READ						(NBIT3)
#define BIGINT_IRQ_STS_START						(NBIT4)
#define BIGINT_IRQ_STS_PRECOMP_FINISH				(NBIT5)

#define BIGINT_IRQ_MASK								(BIGINT_ENGINE_ADDR + 0x2C)
#define BIGINT_IRQ_MASK_CTL_IRQ_MASK_START			(NBIT4)

#define ENABLE_FLIPPING			1




#define GET_UINT32(BUF,OFFSET)			(((uint32)((BUF)[OFFSET])) | ((uint32)(((BUF)[OFFSET + 1]) << 8))  | \
((uint32)(((BUF)[OFFSET + 2]) << 16)) | ((uint32)(((BUF)[OFFSET + 3]) << 24)))

#define PUTU32(VAL32,BUF,OFFSET)	\
do	\
{	\
	(BUF)[OFFSET	] = BYTE_3((VAL32));	\
	(BUF)[OFFSET +1	] = BYTE_2((VAL32));	\
	(BUF)[OFFSET +2	] = BYTE_1((VAL32));	\
	(BUF)[OFFSET +3	] = BYTE_0((VAL32));	\
  } while(0)


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
DATA TYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/*!
@struct	\
	tstrHashContext
	
@brief
*/
typedef struct{
	uint32		au32HashState[M2M_SHA256_DIGEST_LEN/4];
	uint8		au8CurrentBlock[64];
	uint32		u32TotalLength;
  uint8		u8InitHashFlag;
  } tstrSHA256HashCtxt;



/*======*======*======*======*======*=======*
*           SHA256 IMPLEMENTATION           *
*======*======*======*======*======*========*/

sint8 m2m_crypto_sha256_hash_init(tstrM2mSha256Ctxt *pstrSha256Ctxt)
{
	tstrSHA256HashCtxt	*pstrSHA256 = (tstrSHA256HashCtxt*)pstrSha256Ctxt;
	if(pstrSHA256) {
		m2m_memset((uint8*)pstrSha256Ctxt, 0, sizeof(tstrM2mSha256Ctxt));
		pstrSHA256->u8InitHashFlag = 1;
    }
	return 0;
  }

sint8 m2m_crypto_sha256_hash_update(tstrM2mSha256Ctxt *pstrSha256Ctxt, uint8 *pu8Data, uint16 u16DataLength) {
	sint8	s8Ret = M2M_ERR_FAIL;
	tstrSHA256HashCtxt	*pstrSHA256 = (tstrSHA256HashCtxt*)pstrSha256Ctxt;
  
	if(pstrSHA256) {
		uint32	u32ReadAddr;
		uint32	u32WriteAddr	= SHARED_MEM_BASE;
		uint32	u32Addr			= u32WriteAddr;
		uint32 	u32ResidualBytes;
		uint32 	u32NBlocks;
		uint32 	u32Offset;
		uint32 	u32CurrentBlock = 0;
		uint8	u8IsDone		= 0;

		/* Get the remaining bytes from the previous update (if the length is not block aligned). */
		u32ResidualBytes = pstrSHA256->u32TotalLength % SHA_BLOCK_SIZE;

		/* Update the total data length. */
		pstrSHA256->u32TotalLength += u16DataLength;

		if(u32ResidualBytes != 0)	{
			if((u32ResidualBytes + u16DataLength) >= SHA_BLOCK_SIZE) {
				u32Offset = SHA_BLOCK_SIZE - u32ResidualBytes;
				m2m_memcpy(&pstrSHA256->au8CurrentBlock[u32ResidualBytes], pu8Data, u32Offset);
				pu8Data			+= u32Offset;
				u16DataLength	-= u32Offset;

				nm_write_block(u32Addr, pstrSHA256->au8CurrentBlock, SHA_BLOCK_SIZE);
				u32Addr += SHA_BLOCK_SIZE;
				u32CurrentBlock	 = 1;
			}
			else {
				m2m_memcpy(&pstrSHA256->au8CurrentBlock[u32ResidualBytes], pu8Data, u16DataLength);
				u16DataLength = 0;
        }
      }

		/* Get the number of HASH BLOCKs and the residual bytes. */
		u32NBlocks			= u16DataLength / SHA_BLOCK_SIZE;
		u32ResidualBytes	= u16DataLength % SHA_BLOCK_SIZE;

		if(u32NBlocks != 0)	{
			nm_write_block(u32Addr, pu8Data, (uint16)(u32NBlocks * SHA_BLOCK_SIZE));
			pu8Data += (u32NBlocks * SHA_BLOCK_SIZE);
      }

		u32NBlocks += u32CurrentBlock;
		if(u32NBlocks != 0)	{
			uint32	u32RegVal = 0;

			nm_write_reg(SHA256_CTRL, u32RegVal);
			u32RegVal |= SHA256_CTRL_FORCE_SHA256_QUIT_MASK;
			nm_write_reg(SHA256_CTRL, u32RegVal);

			if(pstrSHA256->u8InitHashFlag) {
				pstrSHA256->u8InitHashFlag = 0;
				u32RegVal |= SHA256_CTRL_INIT_SHA256_STATE_MASK;
			}

			u32ReadAddr = u32WriteAddr + (u32NBlocks * SHA_BLOCK_SIZE);
			nm_write_reg(SHA256_DATA_LENGTH, (u32NBlocks * SHA_BLOCK_SIZE));
			nm_write_reg(SHA256_START_RD_ADDR, u32WriteAddr);
			nm_write_reg(SHA256_START_WR_ADDR, u32ReadAddr);

			u32RegVal |= SHA256_CTRL_START_CALC_MASK;

			u32RegVal &= ~(0x7 << 8);
			u32RegVal |= (2 << 8);

			nm_write_reg(SHA256_CTRL, u32RegVal);

			/* 5.	Wait for done_intr */
			while(!u8IsDone) {
				u32RegVal = nm_read_reg(SHA256_DONE_INTR_STS);
				u8IsDone = u32RegVal & NBIT0;
      	}
      }
		if(u32ResidualBytes != 0)	{
			m2m_memcpy(pstrSHA256->au8CurrentBlock, pu8Data, u32ResidualBytes);
      }
		s8Ret = M2M_SUCCESS;
    }
  
	return s8Ret;
  }


sint8 m2m_crypto_sha256_hash_finish(tstrM2mSha256Ctxt *pstrSha256Ctxt, uint8 *pu8Sha256Digest) {
	sint8	s8Ret = M2M_ERR_FAIL;
	tstrSHA256HashCtxt	*pstrSHA256 = (tstrSHA256HashCtxt*)pstrSha256Ctxt;
  
	if(pstrSHA256) {
		uint32	u32ReadAddr;
		uint32	u32WriteAddr	= SHARED_MEM_BASE;
		uint32	u32Addr			= u32WriteAddr;
		uint16	u16Offset;
		uint16 	u16PaddingLength;
		uint16	u16NBlocks		= 1;
		uint32	u32RegVal		= 0;
		uint32	u32Idx,u32ByteIdx;
		uint32	au32Digest[M2M_SHA256_DIGEST_LEN / 4];
		uint8	u8IsDone		= 0;

		nm_write_reg(SHA256_CTRL,u32RegVal);
		u32RegVal |= SHA256_CTRL_FORCE_SHA256_QUIT_MASK;
		nm_write_reg(SHA256_CTRL,u32RegVal);

		if(pstrSHA256->u8InitHashFlag) {
			pstrSHA256->u8InitHashFlag = 0;
			u32RegVal |= SHA256_CTRL_INIT_SHA256_STATE_MASK;
      }

		/* Calculate the offset of the last data byte in the current block. */
		u16Offset = (uint16)(pstrSHA256->u32TotalLength % SHA_BLOCK_SIZE);

		/* Add the padding byte 0x80. */
		pstrSHA256->au8CurrentBlock[u16Offset ++] = 0x80;

		/* Calculate the required padding to complete
		one Hash Block Size.
		*/
		u16PaddingLength = SHA_BLOCK_SIZE - u16Offset;
		m2m_memset(&pstrSHA256->au8CurrentBlock[u16Offset], 0, u16PaddingLength);

		/* If the padding count is not enough to hold 64-bit representation of
		the total input message length, one padding block is required.
		*/
		if(u16PaddingLength < 8) {
			nm_write_block(u32Addr, pstrSHA256->au8CurrentBlock, SHA_BLOCK_SIZE);
			u32Addr += SHA_BLOCK_SIZE;
			m2m_memset(pstrSHA256->au8CurrentBlock, 0, SHA_BLOCK_SIZE);
			u16NBlocks ++;
      }

		/* pack the length at the end of the padding block */
		PUTU32(pstrSHA256->u32TotalLength << 3, pstrSHA256->au8CurrentBlock, (SHA_BLOCK_SIZE - 4));

		u32ReadAddr = u32WriteAddr + (u16NBlocks * SHA_BLOCK_SIZE);
		nm_write_block(u32Addr, pstrSHA256->au8CurrentBlock, SHA_BLOCK_SIZE);
		nm_write_reg(SHA256_DATA_LENGTH, (u16NBlocks * SHA_BLOCK_SIZE));
		nm_write_reg(SHA256_START_RD_ADDR, u32WriteAddr);
		nm_write_reg(SHA256_START_WR_ADDR, u32ReadAddr);

		u32RegVal |= SHA256_CTRL_START_CALC_MASK;
		u32RegVal |= SHA256_CTRL_WR_BACK_HASH_VALUE_MASK;
		u32RegVal &= ~(0x7UL << 8);
		u32RegVal |= (0x2UL << 8);

		nm_write_reg(SHA256_CTRL,u32RegVal);


		/* 5.	Wait for done_intr */
		while(!u8IsDone) {
			u32RegVal = nm_read_reg(SHA256_DONE_INTR_STS);
			u8IsDone = u32RegVal & NBIT0;
		}
		nm_read_block(u32ReadAddr, (uint8*)au32Digest, 32);
		
		/* Convert the output words to an array of bytes.
		*/
		u32ByteIdx = 0;
		for(u32Idx = 0; u32Idx < (M2M_SHA256_DIGEST_LEN / 4); u32Idx ++) {
			pu8Sha256Digest[u32ByteIdx ++] = BYTE_3(au32Digest[u32Idx]);
			pu8Sha256Digest[u32ByteIdx ++] = BYTE_2(au32Digest[u32Idx]);
			pu8Sha256Digest[u32ByteIdx ++] = BYTE_1(au32Digest[u32Idx]);
			pu8Sha256Digest[u32ByteIdx ++] = BYTE_0(au32Digest[u32Idx]);
      }
		s8Ret = M2M_SUCCESS;
    }
	return s8Ret;
  }


/*======*======*======*======*======*=======*
*             RSA IMPLEMENTATION            *
*======*======*======*======*======*========*/

static void FlipBuffer(uint8 *pu8InBuffer, uint8 *pu8OutBuffer, uint16 u16BufferSize) {
	uint16	u16Idx;
  
	for(u16Idx = 0; u16Idx < u16BufferSize; u16Idx ++) {
#if ENABLE_FLIPPING == 1
		pu8OutBuffer[u16Idx] = pu8InBuffer[u16BufferSize - u16Idx - 1];
#else
		pu8OutBuffer[u16Idx] = pu8InBuffer[u16Idx];
#endif
	}
}

void BigInt_ModExp(	
 uint8	*pu8X,	uint16	u16XSize,
 uint8	*pu8E,	uint16	u16ESize,
 uint8	*pu8M,	uint16	u16MSize,
 uint8	*pu8R,	uint16	u16RSize
 ) {
	uint32	u32Reg;
	uint8	au8Tmp[780] = {0};
	uint32	u32XAddr	= SHARED_MEM_BASE;
	uint32	u32MAddr;
	uint32	u32EAddr;
	uint32	u32RAddr;
	uint8	u8EMswBits	= 32;
	uint32	u32Mprime	= 0x7F;
	uint16	u16XSizeWords,u16ESizeWords;
	uint32	u32Exponent;

	u16XSizeWords = (u16XSize + 3) / 4;
	u16ESizeWords = (u16ESize + 3) / 4;
	
	u32MAddr	= u32XAddr + (u16XSizeWords * 4);
	u32EAddr 	= u32MAddr + (u16XSizeWords * 4);
	u32RAddr	= u32EAddr + (u16ESizeWords * 4);

	/* Reset the core.
	*/
	u32Reg	= 0;
	u32Reg	|= BIGINT_MISC_CTRL_CTL_RESET;
	u32Reg	= nm_read_reg(BIGINT_MISC_CTRL);
	u32Reg	&= ~BIGINT_MISC_CTRL_CTL_RESET;
	u32Reg = nm_read_reg(BIGINT_MISC_CTRL);

	nm_write_block(u32RAddr,au8Tmp, u16RSize);

	/* Write Input Operands to Chip Memory. 
	*/
	/*------- X -------*/
	FlipBuffer(pu8X,au8Tmp,u16XSize);
	nm_write_block(u32XAddr,au8Tmp,u16XSizeWords * 4);

	/*------- E -------*/
	m2m_memset(au8Tmp, 0, sizeof(au8Tmp));
	FlipBuffer(pu8E, au8Tmp, u16ESize);
	nm_write_block(u32EAddr, au8Tmp, u16ESizeWords * 4);
	u32Exponent = GET_UINT32(au8Tmp, (u16ESizeWords * 4) - 4);
	while((u32Exponent & NBIT31)== 0)	{
		u32Exponent <<= 1;
		u8EMswBits --;
	}

	/*------- M -------*/
	m2m_memset(au8Tmp, 0, sizeof(au8Tmp));
	FlipBuffer(pu8M, au8Tmp, u16XSize);
	nm_write_block(u32MAddr, au8Tmp, u16XSizeWords * 4);

	/* Program the addresses of the input operands. 
	*/
	nm_write_reg(BIGINT_ADDR_X, u32XAddr);
	nm_write_reg(BIGINT_ADDR_E, u32EAddr);
	nm_write_reg(BIGINT_ADDR_M, u32MAddr);
	nm_write_reg(BIGINT_ADDR_R, u32RAddr);

	/* Mprime. 
	*/
	nm_write_reg(BIGINT_M_PRIME,u32Mprime);

	/* Length. 
	*/
	u32Reg	= (u16XSizeWords & 0xFF);
	u32Reg += ((u16ESizeWords & 0xFF) << 8);
	u32Reg += (u8EMswBits << 16);
	nm_write_reg(BIGINT_LENGTH,u32Reg);

	/* CTRL Register. 
	*/
	u32Reg = nm_read_reg(BIGINT_MISC_CTRL);
	u32Reg ^= BIGINT_MISC_CTRL_CTL_START;
	u32Reg |= BIGINT_MISC_CTRL_CTL_FORCE_BARRETT;
	//u32Reg |= BIGINT_MISC_CTRL_CTL_M_PRIME_VALID;
#if ENABLE_FLIPPING == 0
	u32Reg |= BIGINT_MISC_CTRL_CTL_MSW_FIRST;
#endif
	nm_write_reg(BIGINT_MISC_CTRL,u32Reg);

	/* Wait for computation to complete. */
	while(1) {
		u32Reg = nm_read_reg(BIGINT_IRQ_STS);
		if(u32Reg & BIGINT_IRQ_STS_DONE) {
			break;
		}
	}
	nm_write_reg(BIGINT_IRQ_STS,0);
	m2m_memset(au8Tmp, 0, sizeof(au8Tmp));
	nm_read_block(u32RAddr, au8Tmp, u16RSize);
	FlipBuffer(au8Tmp, pu8R, u16RSize);
}



#define MD5_DIGEST_SIZE			(16)
#define SHA1_DIGEST_SIZE		(20)

static const uint8 au8TEncodingMD5[] = {
	0x30, 0x20, 0x30, 0x0C, 0x06, 0x08, 0x2A, 0x86,
	0x48, 0x86, 0xF7, 0x0D, 0x02, 0x05, 0x05, 0x00,
	0x04
  };
/*!< Fixed part of the Encoding T for the MD5 hash algorithm.
*/


static const uint8 au8TEncodingSHA1[] = {
	0x30, 0x21, 0x30, 0x09, 0x06, 0x05, 0x2B, 0x0E,
	0x03, 0x02, 0x1A, 0x05, 0x00, 0x04 
  };
/*!< Fixed part of the Encoding T for the SHA-1 hash algorithm.
*/


static const uint8 au8TEncodingSHA2[] = {
	0x30, 0x31, 0x30, 0x0D, 0x06, 0x09, 0x60, 0x86,
	0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x01, 0x05,
	0x00, 0x04
};
/*!< Fixed part of the Encoding T for the SHA-2 hash algorithm.
*/


sint8 m2m_crypto_rsa_sign_verify(uint8 *pu8N, uint16 u16NSize, uint8 *pu8E, uint16 u16ESize, uint8 *pu8SignedMsgHash, 
						  uint16 u16HashLength, uint8 *pu8RsaSignature) {
	sint8		s8Ret = M2M_RSA_SIGN_FAIL;

	if((pu8N) && (pu8E) && (pu8RsaSignature) && (pu8SignedMsgHash))	{
		uint16	u16TLength, u16TEncodingLength;
		uint8	*pu8T;
		uint8	au8EM[512];

		/* Selection of correct T Encoding based on the hash size.
		*/
		if(u16HashLength == MD5_DIGEST_SIZE) {
			pu8T 				= (uint8*)au8TEncodingMD5;
			u16TEncodingLength	= sizeof(au8TEncodingMD5);
		}
		else if(u16HashLength == SHA1_DIGEST_SIZE) {
			pu8T 				= (uint8*)au8TEncodingSHA1;
			u16TEncodingLength	= sizeof(au8TEncodingSHA1);
		}
		else {
			pu8T 				= (uint8*)au8TEncodingSHA2;
			u16TEncodingLength	= sizeof(au8TEncodingSHA2);
		}
		u16TLength = u16TEncodingLength + 1 + u16HashLength;
					
		/* If emLen < tLen + 11.
		*/
		if(u16NSize >= (u16TLength + 11))	{
			uint32 	u32PSLength,u32Idx = 0;
	
			/*
			RSA verification
			*/
			BigInt_ModExp(pu8RsaSignature, u16NSize, pu8E, u16ESize, pu8N, u16NSize, au8EM, u16NSize);

			u32PSLength = u16NSize - u16TLength - 3;

			/* 
			The calculated EM must match the following pattern.
			*======*======*======*======*======*
			* 0x00 || 0x01 || PS || 0x00 || T  *	
			*======*======*======*======*======*
			Where PS is all 0xFF 
			T is defined based on the hash algorithm.
			*/
			if((au8EM[0] == 0x00) && (au8EM[1] == 0x01)) {
				for(u32Idx = 2; au8EM[u32Idx] == 0xFF; u32Idx ++);
				if(u32Idx == (u32PSLength + 2))	{
					if(au8EM[u32Idx ++] == 0x00) {
						if(!m2m_memcmp(&au8EM[u32Idx], pu8T, u16TEncodingLength))	{
							u32Idx += u16TEncodingLength;
							if(au8EM[u32Idx ++] == u16HashLength)
								s8Ret = m2m_memcmp(&au8EM[u32Idx], pu8SignedMsgHash, u16HashLength);
              }
            }
          }
        }
      }
    }
	return s8Ret;
  }


sint8 m2m_crypto_rsa_sign_gen(uint8 *pu8N, uint16 u16NSize, uint8 *pu8d, uint16 u16dSize, uint8 *pu8SignedMsgHash, 
					   uint16 u16HashLength, uint8 *pu8RsaSignature) {
	sint8		s8Ret = M2M_RSA_SIGN_FAIL;

	if((pu8N) && (pu8d) && (pu8RsaSignature) && (pu8SignedMsgHash))	{
		uint16	u16TLength, u16TEncodingLength;
		uint8	*pu8T;
		uint8	au8EM[512];

		/* Selection of correct T Encoding based on the hash size.
		*/
		if(u16HashLength == MD5_DIGEST_SIZE) {
			pu8T 				= (uint8*)au8TEncodingMD5;
			u16TEncodingLength	= sizeof(au8TEncodingMD5);
      }
		else if(u16HashLength == SHA1_DIGEST_SIZE) {
			pu8T 				= (uint8*)au8TEncodingSHA1;
			u16TEncodingLength	= sizeof(au8TEncodingSHA1);
      }
		else {
			pu8T 				= (uint8*)au8TEncodingSHA2;
			u16TEncodingLength	= sizeof(au8TEncodingSHA2);
      }
		u16TLength = u16TEncodingLength + 1 + u16HashLength;
					
		/* If emLen < tLen + 11.
		*/
		if(u16NSize >= (u16TLength + 11))	{
			uint16 	u16PSLength = 0;
			uint16	u16Offset	= 0;	

			/* 
			The calculated EM must match the following pattern.
			*======*======*======*======*======*
			* 0x00 || 0x01 || PS || 0x00 || T  *	
			*======*======*======*======*======*
			Where PS is all 0xFF 
			T is defined based on the hash algorithm.
			*/
			au8EM[u16Offset ++]	= 0;
			au8EM[u16Offset ++]	= 1;
			u16PSLength = u16NSize - u16TLength - 3;
			m2m_memset(&au8EM[u16Offset], 0xFF, u16PSLength);
			u16Offset += u16PSLength;
			au8EM[u16Offset ++] = 0;
			m2m_memcpy(&au8EM[u16Offset], pu8T, u16TEncodingLength);
			u16Offset += u16TEncodingLength;
			au8EM[u16Offset ++] = u16HashLength;
			m2m_memcpy(&au8EM[u16Offset], pu8SignedMsgHash, u16HashLength);
			
			/*
			RSA Signature Generation
			*/
			BigInt_ModExp(au8EM, u16NSize, pu8d, u16dSize, pu8N, u16NSize, pu8RsaSignature, u16NSize);
			s8Ret = M2M_RSA_SIGN_OK;
      }
    }
	return s8Ret;
  }

#endif /* CONF_CRYPTO */

#ifdef CONF_CRYPTO_SOFT

typedef struct {
	tpfAppCryproCb pfAppCryptoCb;
	uint8 *pu8Digest;
	uint8 *pu8Rsa;
	uint8 u8CryptoBusy;
  } tstrCryptoCtxt;

typedef struct {
	uint8 au8N[M2M_MAX_RSA_LEN];
	uint8 au8E[M2M_MAX_RSA_LEN];
	uint8 au8Hash[M2M_SHA256_DIGEST_LEN];
	uint16 u16Nsz;
	uint16 u16Esz;
	uint16 u16Hsz;
	uint8 _pad16_[2];
  } tstrRsaPayload;

static tstrCryptoCtxt gstrCryptoCtxt;


/**
*	@fn			m2m_crypto_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr)
*	@brief		WiFi call back function
*	@param [in]	u8OpCode
*					HIF Opcode type.
*	@param [in]	u16DataSize
*					HIF data length.
*	@param [in]	u32Addr
*					HIF address.
*	@author
*	@date
*	@version	1.0
*/
static void m2m_crypto_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr) {
	sint8 ret = M2M_SUCCESS;
	gstrCryptoCtxt.u8CryptoBusy = 0;
  
  SWITCH stronzoooooo!!
	if(u8OpCode == M2M_CRYPTO_RESP_SHA256_INIT)	{
		tstrM2mSha256Ctxt strCtxt;	
		if (hif_receive(u32Addr, (uint8*) &strCtxt,sizeof(tstrM2mSha256Ctxt), 0) == M2M_SUCCESS) {	
			tstrCyptoResp strResp;	
			if(hif_receive(u32Addr + sizeof(tstrM2mSha256Ctxt), (uint8*) &strResp,sizeof(tstrCyptoResp), 1) == M2M_SUCCESS)	{
				if (gstrCryptoCtxt.pfAppCryptoCb)
					gstrCryptoCtxt.pfAppCryptoCb(u8OpCode,&strResp,&strCtxt);
			}
		}
	}
	else if(u8OpCode == M2M_CRYPTO_RESP_SHA256_UPDATE)	{
		tstrM2mSha256Ctxt strCtxt;
		if (hif_receive(u32Addr, (uint8*) &strCtxt,sizeof(tstrM2mSha256Ctxt), 0) == M2M_SUCCESS) {
			tstrCyptoResp strResp;
			if (hif_receive(u32Addr + sizeof(tstrM2mSha256Ctxt), (uint8*) &strResp,sizeof(tstrCyptoResp), 1) == M2M_SUCCESS) {
				if (gstrCryptoCtxt.pfAppCryptoCb)
					gstrCryptoCtxt.pfAppCryptoCb(u8OpCode,&strResp,&strCtxt);
			}
		}

	}
	else if(u8OpCode == M2M_CRYPTO_RESP_SHA256_FINSIH) {
		tstrCyptoResp strResp;
		if(hif_receive(u32Addr + sizeof(tstrM2mSha256Ctxt), (uint8*) &strResp,sizeof(tstrCyptoResp), 0) == M2M_SUCCESS) {
			if(hif_receive(u32Addr + sizeof(tstrM2mSha256Ctxt) + sizeof(tstrCyptoResp), (uint8*)gstrCryptoCtxt.pu8Digest,M2M_SHA256_DIGEST_LEN, 1) == M2M_SUCCESS)	{
				if(gstrCryptoCtxt.pfAppCryptoCb)
					gstrCryptoCtxt.pfAppCryptoCb(u8OpCode,&strResp,gstrCryptoCtxt.pu8Digest);
				
        }
      }
    }
	else if(u8OpCode == M2M_CRYPTO_RESP_RSA_SIGN_GEN)	{
		tstrCyptoResp strResp;
		if(hif_receive(u32Addr + sizeof(tstrRsaPayload), (uint8*)&strResp,sizeof(tstrCyptoResp), 0) == M2M_SUCCESS) {
			if(hif_receive(u32Addr + sizeof(tstrRsaPayload) + sizeof(tstrCyptoResp), (uint8*)gstrCryptoCtxt.pu8Rsa,M2M_MAX_RSA_LEN, 0) == M2M_SUCCESS)	{
				if(gstrCryptoCtxt.pfAppCryptoCb)
					gstrCryptoCtxt.pfAppCryptoCb(u8OpCode,&strResp,gstrCryptoCtxt.pu8Rsa);
        }
      }
    }
	else if(u8OpCode == M2M_CRYPTO_RESP_RSA_SIGN_VERIFY) {
		tstrCyptoResp strResp;
		if(hif_receive(u32Addr + sizeof(tstrRsaPayload), (uint8*)&strResp,sizeof(tstrCyptoResp), 1) == M2M_SUCCESS) {
			if(gstrCryptoCtxt.pfAppCryptoCb)
				gstrCryptoCtxt.pfAppCryptoCb(u8OpCode,&strResp,NULL);
      }
    }
	else {
		M2M_ERR("u8Code %d ??\n",u8OpCode);
    }

  }

/*!
@fn	\
	sint8 m2m_crypto_init();
	
@brief	crypto initialization

@param[in]	pfAppCryproCb

*/
sint8 m2m_crypto_init(tpfAppCryproCb pfAppCryproCb) {
	sint8 ret = M2M_ERR_FAIL;
  
	m2m_memset((uint8*)&gstrCryptoCtxt,0,sizeof(tstrCryptoCtxt));
	if(pfAppCryproCb)	{
		gstrCryptoCtxt.pfAppCryptoCb = pfAppCryproCb;
		ret = hif_register_cb(M2M_REQ_GROUP_CRYPTO,m2m_crypto_cb);
    }
	return ret;
  }

/*!
@fn	\
	sint8 m2m_sha256_hash_init(tstrM2mSha256Ctxt *psha256Ctxt);
	
@brief	SHA256 hash initialization

@param[in]	psha256Ctxt
				Pointer to a sha256 context allocated by the caller.
*/
sint8 m2m_crypto_sha256_hash_init(tstrM2mSha256Ctxt *psha256Ctxt) {
	sint8  ret = M2M_ERR_FAIL;
  
	if((psha256Ctxt) && (!gstrCryptoCtxt.u8CryptoBusy))	{
		ret = hif_send(M2M_REQ_GROUP_CRYPTO,M2M_CRYPTO_REQ_SHA256_INIT|M2M_REQ_DATA_PKT,(uint8*)psha256Ctxt,sizeof(tstrM2mSha256Ctxt),NULL,0,0);
	}
	return ret;
}


/*!
@fn	\
	sint8 m2m_sha256_hash_update(tstrM2mSha256Ctxt *psha256Ctxt, uint8 *pu8Data, uint16 u16DataLength);
	
@brief	SHA256 hash update

@param [in]	psha256Ctxt
				Pointer to the sha256 context.
				
@param [in]	pu8Data
				Buffer holding the data submitted to the hash.
				
@param [in]	u16DataLength
				Size of the data bufefr in bytes.
*/
sint8 m2m_crypto_sha256_hash_update(tstrM2mSha256Ctxt *psha256Ctxt, uint8 *pu8Data, uint16 u16DataLength) {
	sint8  ret = M2M_ERR_FAIL;
  
	if((!gstrCryptoCtxt.u8CryptoBusy) && (psha256Ctxt) && (pu8Data) && (u16DataLength < M2M_SHA256_MAX_DATA))	{
		ret = hif_send(M2M_REQ_GROUP_CRYPTO,M2M_CRYPTO_REQ_SHA256_UPDATE|M2M_REQ_DATA_PKT,(uint8*)psha256Ctxt,sizeof(tstrM2mSha256Ctxt),pu8Data,u16DataLength,sizeof(tstrM2mSha256Ctxt) + sizeof(tstrCyptoResp));
    }
  
	return ret;
	}


/*!
@fn	\
	sint8 m2m_sha256_hash_finish(tstrM2mSha256Ctxt *psha256Ctxt, uint8 *pu8Sha256Digest);
	
@brief	SHA256 hash finalization

@param[in]	psha256Ctxt
				Pointer to a sha256 context allocated by the caller.
				
@param [in] pu8Sha256Digest
				Buffer allocated by the caller which will hold the resultant SHA256 Digest. It must be allocated no less than M2M_SHA256_DIGEST_LEN.
*/
sint8 m2m_crypto_sha256_hash_finish(tstrM2mSha256Ctxt *psha256Ctxt, uint8 *pu8Sha256Digest) {
	sint8  ret = M2M_ERR_FAIL;
  
	if((!gstrCryptoCtxt.u8CryptoBusy) && (psha256Ctxt) && (pu8Sha256Digest))	{
		gstrCryptoCtxt.pu8Digest = pu8Sha256Digest;
		ret = hif_send(M2M_REQ_GROUP_CRYPTO,M2M_CRYPTO_REQ_SHA256_FINSIH|M2M_REQ_DATA_PKT,(uint8*)psha256Ctxt,sizeof(tstrM2mSha256Ctxt),NULL,0,0);
    }
	return ret;
  }




/*!
@fn	\
	sint8 m2m_rsa_sign_verify(uint8 *pu8N, uint16 u16NSize, uint8 *pu8E, uint16 u16ESize, uint8 *pu8SignedMsgHash, \
		uint16 u16HashLength, uint8 *pu8RsaSignature);
	
@brief	RSA Signature Verification

	The function shall request the RSA Signature verification from the WINC Firmware for the given message. The signed message shall be 
	compressed to the corresponding hash algorithm before calling this function.
	The hash type is identified by the given hash length. For example, if the hash length is 32 bytes, then it is SHA256.

@param[in]	pu8N
				RSA Key modulus n.
				
@param[in]	u16NSize
				Size of the RSA modulus n in bytes.
				
@param[in]	pu8E
				RSA public exponent.
				
@param[in]	u16ESize
				Size of the RSA public exponent in bytes.

@param[in]	pu8SignedMsgHash
				The hash digest of the signed message.
				
@param[in]	u16HashLength
				The length of the hash digest.
				
@param[out] pu8RsaSignature
				Signature value to be verified.
*/


sint8 m2m_crypto_rsa_sign_verify(uint8 *pu8N, uint16 u16NSize, uint8 *pu8E, uint16 u16ESize, uint8 *pu8SignedMsgHash, 
						  uint16 u16HashLength, uint8 *pu8RsaSignature) {
	sint8 ret = M2M_ERR_FAIL;
  
	if((!gstrCryptoCtxt.u8CryptoBusy) && (pu8N) && (pu8E) && (pu8RsaSignature) && (pu8SignedMsgHash) 
	&& (u16NSize != 0) && (u16ESize != 0) && (u16HashLength != 0) && (pu8RsaSignature) )	{
		tstrRsaPayload strRsa = {0};
		
		m2m_memcpy(strRsa.au8N,pu8N,u16NSize);
		m2m_memcpy(strRsa.au8E,pu8E,u16ESize);
		m2m_memcpy(strRsa.au8Hash,pu8SignedMsgHash,u16HashLength);
		
		strRsa.u16Esz = u16ESize;
		strRsa.u16Hsz = u16HashLength;
		strRsa.u16Nsz = u16NSize;
		
		ret = hif_send(M2M_REQ_GROUP_CRYPTO,M2M_CRYPTO_REQ_RSA_SIGN_VERIFY|M2M_REQ_DATA_PKT,(uint8*)&strRsa,sizeof(tstrRsaPayload),NULL,0,0);
		
    }
	return ret;
  }


/*!
@fn	\
	sint8 m2m_rsa_sign_gen(uint8 *pu8N, uint16 u16NSize, uint8 *pu8d, uint16 u16dSize, uint8 *pu8SignedMsgHash, \
		uint16 u16HashLength, uint8 *pu8RsaSignature);
	
@brief	RSA Signature Generation

	The function shall request the RSA Signature generation from the WINC Firmware for the given message. The signed message shall be 
	compressed to the corresponding hash algorithm before calling this function.
	The hash type is identified by the given hash length. For example, if the hash length is 32 bytes, then it is SHA256.

@param[in]	pu8N
				RSA Key modulus n.
				
@param[in]	u16NSize
				Size of the RSA modulus n in bytes.
				
@param[in]	pu8d
				RSA private exponent.
				
@param[in]	u16dSize
				Size of the RSA private exponent in bytes.

@param[in]	pu8SignedMsgHash
				The hash digest of the signed message.
				
@param[in]	u16HashLength
				The length of the hash digest.
				
@param[out] pu8RsaSignature
				Pointer to a user buffer allocated by teh caller shall hold the generated signature.
*/
sint8 m2m_crypto_rsa_sign_gen(uint8 *pu8N, uint16 u16NSize, uint8 *pu8d, uint16 u16dSize, uint8 *pu8SignedMsgHash, 
					   uint16 u16HashLength, uint8 *pu8RsaSignature) {
	sint8 ret = M2M_ERR_FAIL;
  
	if((!gstrCryptoCtxt.u8CryptoBusy) && (pu8N) && (pu8d) && (pu8RsaSignature) && (pu8SignedMsgHash)
	&& (u16NSize != 0) && (u16dSize != 0) && (u16HashLength != 0) && (pu8RsaSignature))	{
		tstrRsaPayload strRsa = {0};
		
		m2m_memcpy(strRsa.au8N,pu8N,u16NSize);
		m2m_memcpy(strRsa.au8E,pu8d,u16dSize);
		m2m_memcpy(strRsa.au8Hash,pu8SignedMsgHash,u16HashLength);
		
		strRsa.u16Esz = u16dSize;
		strRsa.u16Hsz = u16HashLength;
		strRsa.u16Nsz = u16NSize;
		
		gstrCryptoCtxt.pu8Rsa = pu8RsaSignature;
		ret = hif_send(M2M_REQ_GROUP_CRYPTO,M2M_CRYPTO_REQ_RSA_SIGN_GEN|M2M_REQ_DATA_PKT,(uint8*)&strRsa,sizeof(tstrRsaPayload),NULL,0,0);
		
	}
	return ret;			   
}

#endif


// m2m_hif.c ***********************************************************************************************************
//#include "common/include/nm_common.h"
//#include "driver/source/nmbus.h"
//#include "bsp/include/nm_bsp.h"
//#include "m2m_hif.h"
//#include "driver/include/m2m_types.h"
//#include "driver/source/nmasic.h"
//#include "driver/include/m2m_periph.h"

#if (defined NM_EDGE_INTERRUPT)&&(defined NM_LEVEL_INTERRUPT)
#error "only one type of interrupt NM_EDGE_INTERRUPT,NM_LEVEL_INTERRUPT"
#endif

#if !((defined NM_EDGE_INTERRUPT)||(defined NM_LEVEL_INTERRUPT))
#error "define interrupt type NM_EDGE_INTERRUPT,NM_LEVEL_INTERRUPT"
#endif

#ifndef CORTUS_APP
#define NMI_AHB_DATA_MEM_BASE  0x30000
#define NMI_AHB_SHARE_MEM_BASE 0xd0000

#define WIFI_HOST_RCV_CTRL_0	(0x1070)
#define WIFI_HOST_RCV_CTRL_1	(0x1084)
#define WIFI_HOST_RCV_CTRL_2    (0x1078)
#define WIFI_HOST_RCV_CTRL_3    (0x106c)
#define WAKE_VALUE				(0x5678)
#define SLEEP_VALUE				(0x4321)
#define WAKE_REG				(0x1074)



static volatile uint8 gu8ChipMode = 0;
static volatile uint8 gu8ChipSleep = 0;
static volatile uint8 gu8HifSizeDone = 0;
static volatile uint8 gu8Interrupt = 0;

tpfHifCallBack pfWifiCb = NULL;		/*!< pointer to Wi-Fi call back function */
tpfHifCallBack pfIpCb  = NULL;		/*!< pointer to Socket call back function */
tpfHifCallBack pfOtaCb = NULL;		/*!< pointer to OTA call back function */
tpfHifCallBack pfSigmaCb = NULL;
tpfHifCallBack pfHifCb = NULL;
tpfHifCallBack pfCryptoCb = NULL;

static void isr(void) {
  
	gu8Interrupt++;
#ifdef NM_LEVEL_INTERRUPT
	nm_bsp_interrupt_ctrl(0);
#endif
  }

static sint8 hif_set_rx_done(void) {
	uint32 reg;
	sint8 ret = M2M_SUCCESS;
#ifdef NM_EDGE_INTERRUPT
	nm_bsp_interrupt_ctrl(1);
#endif

	ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_0,&reg);
	if(ret != M2M_SUCCESS)
    goto ERR1;
	//reg &= ~(1<<0);

	/* Set RX Done */
	reg |= (1<<1);
	ret = nm_write_reg(WIFI_HOST_RCV_CTRL_0,reg);
	if(ret != M2M_SUCCESS)
    goto ERR1;
#ifdef NM_LEVEL_INTERRUPT
	nm_bsp_interrupt_ctrl(1);
#endif

ERR1:
	return ret;
  }

/**
*	@fn			static void m2m_hif_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr)
*	@brief		WiFi call back function
*	@param [in]	u8OpCode
*					HIF Opcode type.
*	@param [in]	u16DataSize
*					HIF data length.
*	@param [in]	u32Addr
*					HIF address.
*	@param [in]	grp
*					HIF group type.
*	@author
*	@date
*	@version	1.0
*/
static void m2m_hif_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr) {


  }

/**
*	@fn		NMI_API sint8 hif_chip_wake(void);
*	@brief	To Wakeup the chip.
*    @return		The function shall return ZERO for successful operation and a negative value otherwise.
*/
sint8 hif_chip_wake(void) {
	sint8 ret = M2M_SUCCESS;
  
	if(gu8ChipSleep == 0)	{
		if((gu8ChipMode == M2M_PS_DEEP_AUTOMATIC)||(gu8ChipMode == M2M_PS_MANUAL)) {
			ret = nm_clkless_wake();
			if(ret != M2M_SUCCESS)
        goto ERR1;
			ret = nm_write_reg(WAKE_REG, WAKE_VALUE);
			if(ret != M2M_SUCCESS)
        goto ERR1;
      }
		else {
      }
    }
	gu8ChipSleep++;
ERR1:
	return ret;
}

/*!
@fn	\
	NMI_API void hif_set_sleep_mode(uint8 u8Pstype);

@brief
	Set the sleep mode of the HIF layer.

@param [in]	u8Pstype
				Sleep mode.

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/

void hif_set_sleep_mode(uint8 u8Pstype) {
  
	gu8ChipMode = u8Pstype;
  }

/*!
@fn	\
	NMI_API uint8 hif_get_sleep_mode(void);

@brief
	Get the sleep mode of the HIF layer.

@return
	The function SHALL return the sleep mode of the HIF layer.
*/
uint8 hif_get_sleep_mode(void) {
	return gu8ChipMode;
  }

/**
*	@fn		NMI_API sint8 hif_chip_sleep(void);
*	@brief	To make the chip sleep.
*    @return		The function shall return ZERO for successful operation and a negative value otherwise.
*/

sint8 hif_chip_sleep(void) {
	sint8 ret = M2M_SUCCESS;

	if(gu8ChipSleep >= 1)	{
		gu8ChipSleep--;
    }
	
	if(gu8ChipSleep == 0)	{
		if((gu8ChipMode == M2M_PS_DEEP_AUTOMATIC) || (gu8ChipMode == M2M_PS_MANUAL)) {
			uint32 reg = 0;
			ret = nm_write_reg(WAKE_REG, SLEEP_VALUE);
			if(ret != M2M_SUCCESS)goto ERR1;
			/* Clear bit 1 */
			ret = nm_read_reg_with_ret(0x1, &reg);
			if(ret != M2M_SUCCESS)goto ERR1;
			if(reg & 0x2)	{
				reg &=~(1 << 1);
				ret = nm_write_reg(0x1, reg);
        }
      }
		else		{
      }
    }
ERR1:
	return ret;
  }

/**
*   @fn		NMI_API sint8 hif_init(void *arg);
*   @brief	To initialize HIF layer.
*   @param [in]	arg
*				Pointer to the arguments.
*   @return		The function shall return ZERO for successful operation and a negative value otherwise.
*/
sint8 hif_init(void *arg) {
  
	pfWifiCb = NULL;
	pfIpCb = NULL;

	gu8ChipSleep = 0;
	gu8ChipMode = M2M_NO_PS;

	gu8Interrupt = 0;
	nm_bsp_register_isr(isr);

	hif_register_cb(M2M_REQ_GROUP_HIF,m2m_hif_cb);

	return M2M_SUCCESS;
  }

/**
*	@fn		NMI_API sint8 hif_deinit(void *arg);
*	@brief	To Deinitialize HIF layer.
*    @param [in]	arg
*				Pointer to the arguments.
*    @return		The function shall return ZERO for successful operation and a negative value otherwise.
*/
sint8 hif_deinit(void *arg) {
	sint8 ret = M2M_SUCCESS;
  
#if 0
	uint32 reg = 0, cnt=0;
	while (reg != M2M_DISABLE_PS)	{
		nm_bsp_sleep(1);
		reg = nm_read_reg(STATE_REG);
		if(++cnt > 1000)	{
			M2M_DBG("failed to stop power save\n");
			break;
		}
	}
#endif
	ret = hif_chip_wake();

	gu8ChipMode = 0;
	gu8ChipSleep = 0;
	gu8HifSizeDone = 0;
	gu8Interrupt = 0;

	pfWifiCb = NULL;
	pfIpCb  = NULL;
	pfOtaCb = NULL;
	pfHifCb = NULL;

	return ret;
  }

/**
*	@fn		NMI_API sint8 hif_send(uint8 u8Gid,uint8 u8Opcode,uint8 *pu8CtrlBuf,uint16 u16CtrlBufSize,
					   uint8 *pu8DataBuf,uint16 u16DataSize, uint16 u16DataOffset)
*	@brief	Send packet using host interface.

*	@param [in]	u8Gid
*				Group ID.
*	@param [in]	u8Opcode
*				Operation ID.
*	@param [in]	pu8CtrlBuf
*				Pointer to the Control buffer.
*	@param [in]	u16CtrlBufSize
				Control buffer size.
*	@param [in]	u16DataOffset
				Packet Data offset.
*	@param [in]	pu8DataBuf
*				Packet buffer Allocated by the caller.
*	@param [in]	u16DataSize
				Packet buffer size (including the HIF header).
*    @return		The function shall return ZERO for successful operation and a negative value otherwise.
*/

sint8 hif_send(uint8 u8Gid,uint8 u8Opcode,uint8 *pu8CtrlBuf,uint16 u16CtrlBufSize,
			   uint8 *pu8DataBuf,uint16 u16DataSize, uint16 u16DataOffset) {
	sint8		ret = M2M_ERR_SEND;
	volatile tstrHifHdr	strHif;

	strHif.u8Opcode		= u8Opcode&(~NBIT7);
	strHif.u8Gid		= u8Gid;
	strHif.u16Length	= M2M_HIF_HDR_OFFSET;
	if(pu8DataBuf)	{
		strHif.u16Length += u16DataOffset + u16DataSize;
    }
	else	{
		strHif.u16Length += u16CtrlBufSize;
    }
	ret = hif_chip_wake();
	if(ret == M2M_SUCCESS)	{
		volatile uint32 reg, dma_addr = 0;
		volatile uint16 cnt = 0;

		reg = 0UL;
		reg |= (uint32)u8Gid;
		reg |= ((uint32)u8Opcode<<8);
		reg |= ((uint32)strHif.u16Length<<16);
		ret = nm_write_reg(NMI_STATE_REG,reg);
		if(M2M_SUCCESS != ret) 
      goto ERR1;


		reg = 0;
		reg |= (1<<1);
		ret = nm_write_reg(WIFI_HOST_RCV_CTRL_2, reg);
		if(M2M_SUCCESS != ret) 
      goto ERR1;
		dma_addr = 0;

		//nm_bsp_interrupt_ctrl(0);

		for(cnt=0; cnt < 1000; cnt ++) {
			ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_2,(uint32 *)&reg);
			if(ret != M2M_SUCCESS) 
        break;
			if(!(reg & 0x2))	{
				ret = nm_read_reg_with_ret(0x150400,(uint32 *)&dma_addr);
				if(ret != M2M_SUCCESS) {
					/*in case of read error clear the dma address and return error*/
					dma_addr = 0;
        	}
				/*in case of success break */
				break;
        }
      }
		//nm_bsp_interrupt_ctrl(1);

		if(dma_addr != 0) {
			volatile uint32	u32CurrAddr;
			u32CurrAddr = dma_addr;
			strHif.u16Length=NM_BSP_B_L_16(strHif.u16Length);
			ret = nm_write_block(u32CurrAddr, (uint8*)&strHif, M2M_HIF_HDR_OFFSET);
		#ifdef CONF_WINC_USE_I2C
			nm_bsp_sleep(1);
		#endif
			if(M2M_SUCCESS != ret) 
        goto ERR1;
			u32CurrAddr += M2M_HIF_HDR_OFFSET;
			if(pu8CtrlBuf) {
				ret = nm_write_block(u32CurrAddr, pu8CtrlBuf, u16CtrlBufSize);
			#ifdef CONF_WINC_USE_I2C
				nm_bsp_sleep(1);
			#endif
				if(M2M_SUCCESS != ret) 
          goto ERR1;
				u32CurrAddr += u16CtrlBufSize;
        }
			if(pu8DataBuf) {
				u32CurrAddr += (u16DataOffset - u16CtrlBufSize);
				ret = nm_write_block(u32CurrAddr, pu8DataBuf, u16DataSize);
			#ifdef CONF_WINC_USE_I2C	
				nm_bsp_sleep(1);
			#endif
				if(M2M_SUCCESS != ret) 
          goto ERR1;
				u32CurrAddr += u16DataSize;
      	}

			reg = dma_addr << 2;
			reg |= (1 << 1);
			ret = nm_write_reg(WIFI_HOST_RCV_CTRL_3, reg);
			if(M2M_SUCCESS != ret) 
        goto ERR1;
      }
		else {
			M2M_DBG("Failed to alloc rx size\r");
			ret =  M2M_ERR_MEM_ALLOC;
			goto ERR1;
      }

    }
	else {
		M2M_ERR("(HIF)Fail to wakeup the chip\n");
		goto ERR1;
  	}
	ret = hif_chip_sleep();

ERR1:
	return ret;
  }

/**
*	@fn		hif_isr
*	@brief	Host interface interrupt service routine
*	@author	M. Abdelmawla
*	@date	15 July 2012
*	@return	1 in case of interrupt received else 0 will be returned
*	@version	1.0
*/
static sint8 hif_isr(void) {
	sint8 ret = M2M_ERR_BUS_FAIL;
	uint32 reg;
	volatile tstrHifHdr strHif;

	ret = hif_chip_wake();
	if(ret == M2M_SUCCESS) {
		ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_0, &reg);
		if(M2M_SUCCESS == ret) {
			if(reg & 0x1)	{     // New interrupt has been received
				uint16 size;

				nm_bsp_interrupt_ctrl(0);
				/*Clearing RX interrupt*/
				reg &= ~(1<<0);
				ret = nm_write_reg(WIFI_HOST_RCV_CTRL_0,reg);
				if(ret != M2M_SUCCESS)
          goto ERR1;
				gu8HifSizeDone = 0;
				size = (uint16)((reg >> 2) & 0xfff);
				if(size > 0) {
					uint32 address = 0;
					/**
					start bus transfer
					**/
					ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_1, &address);
					if(M2M_SUCCESS != ret) {
						M2M_ERR("(hif) WIFI_HOST_RCV_CTRL_1 bus fail\n");
						nm_bsp_interrupt_ctrl(1);
						goto ERR1;
            }
					ret = nm_read_block(address, (uint8*)&strHif, sizeof(tstrHifHdr));
					strHif.u16Length = NM_BSP_B_L_16(strHif.u16Length);
					if(M2M_SUCCESS != ret) {
						M2M_ERR("(hif) address bus fail\n");
						nm_bsp_interrupt_ctrl(1);
						goto ERR1;
        		}
					if(strHif.u16Length != size) {
						if((size - strHif.u16Length) > 4)	{
							M2M_ERR("(hif) Corrupted packet Size = %u <L = %u, G = %u, OP = %02X>\n",
								size, strHif.u16Length, strHif.u8Gid, strHif.u8Opcode);
							nm_bsp_interrupt_ctrl(1);
							ret = M2M_ERR_BUS_FAIL;
							goto ERR1;
              }
            }

					switch(strHif.u8Gid) {
            case M2M_REQ_GROUP_WIFI:
              if(pfWifiCb)
                pfWifiCb(strHif.u8Opcode,strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET);
              break;
            case M2M_REQ_GROUP_IP:
              if(pfIpCb)
                pfIpCb(strHif.u8Opcode,strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET);
              break;
            case M2M_REQ_GROUP_OTA:
              if(pfOtaCb)
                pfOtaCb(strHif.u8Opcode,strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET);
                break;
            case M2M_REQ_GROUP_CRYPTO:
              if(pfCryptoCb)
                pfCryptoCb(strHif.u8Opcode,strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET);
                break;
            case M2M_REQ_GROUP_SIGMA:
              if(pfSigmaCb)
                pfSigmaCb(strHif.u8Opcode,strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET);
                break;
            default:
              M2M_ERR("(hif) invalid group ID\n");
              ret = M2M_ERR_BUS_FAIL;
              goto ERR1;
              break;
          	}
					#ifndef ENABLE_UNO_BOARD
					if(!gu8HifSizeDone)	{
						M2M_ERR("(hif) host app didn't set RX Done\n");
						ret = hif_set_rx_done();
            }
					#endif
          }
				else	{
					ret = M2M_ERR_RCV;
					M2M_ERR("(hif) Wrong Size\n");
					goto ERR1;
          }
        }
			else {
#ifndef WIN32
				M2M_ERR("(hif) False interrupt %lx",reg);
#endif
        }
      }
		else {
			M2M_ERR("(hif) Fail to Read interrupt reg\n");
			goto ERR1;
    	}
  	}
	else {
		M2M_ERR("(hif) FAIL to wakeup the chip\n");
		goto ERR1;
  	}

	ret = hif_chip_sleep();
  
ERR1:
	return ret;
  }

/**
*	@fn		hif_handle_isr(void)
*	@brief	Handle interrupt received from NMC1500 firmware.
*   @return     The function SHALL return 0 for success and a negative value otherwise.
*/
sint8 hif_handle_isr(void) {
	sint8 ret = M2M_SUCCESS;

	while(gu8Interrupt) {
//	if(gu8Interrupt) {// GD per smazzarli meglio!
		/*must be at that place because of the race of interrupt increment and that decrement
		  when the interrupt enabled*/
		gu8Interrupt--;
		while(1) {
			ret = hif_isr();
			if(ret == M2M_SUCCESS) {
				/*we will try forever until we get that interrupt*/
				/*Fail return errors here due to bus errors (reading expected values)*/
				break;
    		} 
      else {
				M2M_ERR("(HIF) Fail to handle interrupt %d try Again..\n",ret);
        }
      }
    }

	return ret;
  }

/*
*	@fn		hif_receive
*	@brief	Host interface interrupt serviece routine
*	@param [in]	u32Addr
*				Receive start address
*	@param [out]	pu8Buf
*				Pointer to receive buffer. Allocated by the caller
*	@param [in]	u16Sz
*				Receive buffer size
*	@param [in]	isDone
*				If you don't need any more packets send True otherwise send false
*    @return		The function shall return ZERO for successful operation and a negative value otherwise.
*/
sint8 hif_receive(uint32 u32Addr, uint8 *pu8Buf, uint16 u16Sz, uint8 isDone) {
	uint32 address, reg;
	uint16 size;
	sint8 ret = M2M_SUCCESS;

	if(!u32Addr || !pu8Buf || u16Sz == 0)	{
		if(isDone) {
			gu8HifSizeDone = 1;
			
			/* set RX done */
			ret = hif_set_rx_done();
      }
		else {
			ret = M2M_ERR_FAIL;
			M2M_ERR(" hif_receive: Invalid argument\n");
      }
		goto ERR1;
    }

	ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_0,&reg);
	if(ret != M2M_SUCCESS)
    goto ERR1;

	size = (uint16)((reg >> 2) & 0xfff);
	ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_1,&address);
	if(ret != M2M_SUCCESS)
    goto ERR1;


	if(u16Sz > size) {
		ret = M2M_ERR_FAIL;
		M2M_ERR("APP Requested Size is larger than the recived buffer size <%d><%d>\n",u16Sz, size);
		goto ERR1;
    }
	if((u32Addr < address)||((u32Addr + u16Sz)>(address+size)))	{
		ret = M2M_ERR_FAIL;
		M2M_ERR("APP Requested Address beyond the recived buffer address and length\n");
		goto ERR1;
    }
	
	/* Receive the payload */
	ret = nm_read_block(u32Addr, pu8Buf, u16Sz);
	if(ret != M2M_SUCCESS)
    goto ERR1;

	/* check if this is the last packet */
	if((((address + size) - (u32Addr + u16Sz)) <= 0) || isDone)	{
		gu8HifSizeDone = 1;

		/* set RX done */
		ret = hif_set_rx_done();
    }

ERR1:
	return ret;
  }

/**
*	@fn		hif_register_cb
*	@brief	To set Callback function for every compantent Component
*	@param [in]	u8Grp
*				Group to which the Callback function should be set.
*	@param [in]	fn
*				function to be set
*    @return		The function shall return ZERO for successful operation and a negative value otherwise.
*/
sint8 hif_register_cb(uint8 u8Grp,tpfHifCallBack fn) {
	sint8 ret = M2M_SUCCESS;
  
	switch(u8Grp)	{
		case M2M_REQ_GROUP_IP:
			pfIpCb = fn;
			break;
		case M2M_REQ_GROUP_WIFI:
			pfWifiCb = fn;
			break;
		case M2M_REQ_GROUP_OTA:
			pfOtaCb = fn;
			break;
		case M2M_REQ_GROUP_HIF:
			pfHifCb = fn;
			break;
		case M2M_REQ_GROUP_CRYPTO:
			pfCryptoCb = fn;
			break;
		case M2M_REQ_GROUP_SIGMA:
			pfSigmaCb = fn;
			break;
		default:
			M2M_ERR("GRp ? %d\n",u8Grp);
			ret = M2M_ERR_FAIL;
			break;
    }
	return ret;
  }

#endif


// m2m_ota.c ***********************************************************************************************************

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
//#include "common/include/nm_common.h"
//#include "driver/include/m2m_types.h"
//#include "driver/include/m2m_ota.h"
//#include "driver/source/m2m_hif.h"
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
DATA TYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
static tpfOtaUpdateCb gpfOtaUpdateCb = NULL;
static tpfOtaNotifCb  gpfOtaNotifCb = NULL;


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION PROTOTYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/**
*	@fn			m2m_wifi_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr, uint8 grp)
*	@brief		WiFi call back function
*	@param [in]	u8OpCode
*					HIF Opcode type.
*	@param [in]	u16DataSize
*					HIF data length.
*	@param [in]	u32Addr
*					HIF address.
*	@param [in]	grp
*					HIF group type.
*	@author
*	@date
*	@version	1.0
*/
static void m2m_ota_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr) {
	sint8 ret = M2M_SUCCESS;
  
	switch(u8OpCode) {
    case M2M_OTA_RESP_NOTIF_UPDATE_INFO:
    {
      tstrOtaUpdateInfo strOtaUpdateInfo;
      m2m_memset((uint8*)&strOtaUpdateInfo,0,sizeof(tstrOtaUpdateInfo));
      ret = hif_receive(u32Addr,(uint8*)&strOtaUpdateInfo,sizeof(tstrOtaUpdateInfo),0);
      if(ret == M2M_SUCCESS) {
        if(gpfOtaNotifCb)
          gpfOtaNotifCb(&strOtaUpdateInfo);
        }
        }
      break;
    case M2M_OTA_RESP_UPDATE_STATUS:
    {
      tstrOtaUpdateStatusResp strOtaUpdateStatusResp;
      m2m_memset((uint8*)&strOtaUpdateStatusResp,0,sizeof(tstrOtaUpdateStatusResp));
      ret = hif_receive(u32Addr, (uint8*) &strOtaUpdateStatusResp,sizeof(tstrOtaUpdateStatusResp), 0);
      if(ret == M2M_SUCCESS) {
        if(gpfOtaUpdateCb)
          gpfOtaUpdateCb(strOtaUpdateStatusResp.u8OtaUpdateStatusType,strOtaUpdateStatusResp.u8OtaUpdateStatus);
        }
        }
      break;
    default:
  		M2M_ERR("Invaild OTA resp %d ?\n",u8OpCode);
      break;
    }

  }

/*!
@fn	\
	NMI_API sint8  m2m_ota_init(tpfOtaUpdateCb pfOtaUpdateCb, tpfOtaNotifCb pfOtaNotifCb);

@brief
	Initialize the OTA layer.

@param [in]	pfOtaUpdateCb
				OTA Update callback function

@param [in]	pfOtaNotifCb
				OTA notify callback function

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8  m2m_ota_init(tpfOtaUpdateCb pfOtaUpdateCb, tpfOtaNotifCb pfOtaNotifCb) {
	sint8 ret = M2M_SUCCESS;

	if(pfOtaUpdateCb){
		gpfOtaUpdateCb = pfOtaUpdateCb;
    }
  else {
		M2M_ERR("Invaild Ota update cb\n");
    }
	if(pfOtaNotifCb){
		gpfOtaNotifCb = pfOtaNotifCb;
    }
  else {
		M2M_ERR("Invaild Ota notify cb\n");
  	}

	hif_register_cb(M2M_REQ_GROUP_OTA,m2m_ota_cb);

	return ret;
  }

/*!
@fn	\
	NMI_API sint8  m2m_ota_notif_set_url(uint8 *u8Url);

@brief
	Set the OTA url

@param [in]	u8Url
			 The url server address

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_ota_notif_set_url(uint8 *u8Url) {
	sint8 ret = M2M_SUCCESS;
	uint16 u16UrlSize = strlen(u8Url) + 1;
  
	/*Todo: we may change it to data pkt but we need to give it higer priority
			but the priorty is not implemnted yet in data pkt
	*/
	ret = hif_send(M2M_REQ_GROUP_OTA,M2M_OTA_REQ_START_UPDATE,u8Url,u16UrlSize,NULL,0,0);
	return ret;
  }

/*!
@fn	\
	NMI_API sint8  m2m_ota_notif_check_for_update(void);

@brief
	check for ota update

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_ota_notif_check_for_update(void) {
	sint8 ret = M2M_SUCCESS;
  
	ret = hif_send(M2M_REQ_GROUP_OTA,M2M_OTA_REQ_NOTIF_CHECK_FOR_UPDATE,NULL,0,NULL,0,0);
	return ret;
  }

/*!
@fn	\
	NMI_API sint8 m2m_ota_notif_sched(uint32 u32Period);

@brief
	Schedule OTA update

@param [in]	u32Period
	Period in days

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_ota_notif_sched(uint32 u32Period) {
	sint8 ret = M2M_SUCCESS;
  
	ret = hif_send(M2M_REQ_GROUP_OTA,M2M_OTA_REQ_NOTIF_CHECK_FOR_UPDATE,NULL,0,NULL,0,0);
	return ret;
  }

/*!
@fn	\
	NMI_API sint8 m2m_ota_start_update(uint8 *u8DownloadUrl);

@brief
	Request OTA start update using the downloaded url

@param [in]	u8DownloadUrl
		The download firmware url, you get it from device info

@return
	The function SHALL return 0 for success and a negative value otherwise.

*/
NMI_API sint8 m2m_ota_start_update(uint8 *u8DownloadUrl) {
	sint8 ret = M2M_SUCCESS;
	uint16 u16DurlSize = strlen(u8DownloadUrl) + 1;
  
	/*Todo: we may change it to data pkt but we need to give it higer priority
			but the priorty is not implemnted yet in data pkt
	*/
	ret = hif_send(M2M_REQ_GROUP_OTA,M2M_OTA_REQ_START_UPDATE,u8DownloadUrl,u16DurlSize,NULL,0,0);
	return ret;
  }


/*!
@fn	\
	NMI_API sint8 m2m_ota_rollback(void);

@brief
	Request OTA Rollback image

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_ota_rollback(void) {
	sint8 ret = M2M_SUCCESS;
  
	ret = hif_send(M2M_REQ_GROUP_OTA,M2M_OTA_REQ_ROLLBACK,NULL,0,NULL,0,0);
	return ret;
  }


/*!
@fn	\
	NMI_API sint8 m2m_ota_switch_firmware(void);

@brief
	Switch to the upgraded Firmware

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_ota_switch_firmware(void) {
	sint8 ret = M2M_SUCCESS;
  
	ret = hif_send(M2M_REQ_GROUP_OTA,M2M_OTA_REQ_SWITCH_FIRMWARE,NULL,0,NULL,0,0);
	return ret;
  }

/*!
@fn	\
	NMI_API sint8 m2m_ota_get_firmware_version(tstrM2mRev *pstrRev);

@brief
	Get the OTA Firmware version.

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_ota_get_firmware_version(tstrM2mRev *pstrRev) {
	sint8 ret = M2M_SUCCESS;
  
	ret = hif_chip_wake();
	if(ret == M2M_SUCCESS)	{
    ret = nm_get_ota_firmware_info(pstrRev);
		hif_chip_sleep();
    }
	return ret;
  }

#if 0
#define M2M_OTA_FILE	"../../../m2m_ota.dat"
NMI_API sint8 m2m_ota_test(void) {
	uint32 page  = 0;
	uint8 buffer[1500];
	uint32 u32Sz = 0;
	sint8 ret = M2M_SUCCESS;
	FILE *fp =NULL;
	fp = fopen(M2M_OTA_FILE,"rb");
	if(fp) {
		fseek(fp, 0L, SEEK_END);
		u32Sz = ftell(fp);
		fseek(fp, 0L, SEEK_SET);

		while(u32Sz > 0) {
			{
				page = (rand()%1400);

				if((page<100) || (page>1400)) 
          page  = 1400;
			}

			if(u32Sz>page) {
				u32Sz-=page;
        }
			else {
				page = u32Sz;
				u32Sz = 0;
        }
			printf("page %d\n", (int)page);
			fread(buffer,page,1,fp);
			ret = hif_send(M2M_REQ_GROUP_OTA,M2M_OTA_REQ_TEST|M2M_REQ_DATA_PKT,NULL,0,(uint8*)&buffer,page,0);
			if(ret != M2M_SUCCESS) {
				M2M_ERR("\n");
        }
			nm_bsp_sleep(1);
      }

    }
	else	{
		M2M_ERR("nO err\n");
    }
  
	return ret;
  }
#endif


// m2m_periph.c ***********************************************************************************************************

//#include "driver/include/m2m_periph.h"
//#include "driver/source/nmasic.h"
//#include "m2m_hif.h"

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#define GPIO_OP_DIR     0
#define GPIO_OP_SET     1
#define GPIO_OP_GET     2
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
DATA TYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
STATIC FUNCTIONS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
static sint8 get_gpio_idx(uint8 u8GpioNum) {
  
	if(u8GpioNum >= M2M_PERIPH_GPIO_MAX) 
    return -1;
	switch(u8GpioNum) {
    case M2M_PERIPH_GPIO15:
      return 4;
      break;
    case M2M_PERIPH_GPIO16:
      return 5;
      break;
    case M2M_PERIPH_GPIO18:
      return 6;
      break;
#if 0 //non corrispondevano...
    case M2M_PERIPH_GPIO15:
      return 15;
      break;
    case M2M_PERIPH_GPIO16:
      return 16;
      break;
    case M2M_PERIPH_GPIO18:
      return 18;
      break;
    case M2M_PERIPH_GPIO3:
      return 3;
      break;
    case M2M_PERIPH_GPIO4:
      return 4;
      break;
    case M2M_PERIPH_GPIO5:
      return 5;
      break;
    case M2M_PERIPH_GPIO6:
      return 6;
      break;
#endif
    default:
      return -2;
      break;
    }
  }

/*
 * GPIO read/write skeleton with wakeup/sleep capability.
 */
static sint8 gpio_ioctl(uint8 op, uint8 u8GpioNum, uint8 u8InVal, uint8 *pu8OutVal) {
	sint8 ret, gpio;

	ret = hif_chip_wake();
	if(ret != M2M_SUCCESS) 
    goto _EXIT;

	gpio = get_gpio_idx(u8GpioNum);
	if(gpio < 0) 
    goto _EXIT1;

	switch(op) {
    case GPIO_OP_DIR:
      ret = set_gpio_dir((uint8)gpio, u8InVal);
      break;
    case GPIO_OP_SET:
      ret = set_gpio_val((uint8)gpio, u8InVal);
      break;
    case GPIO_OP_GET:
    	ret = get_gpio_val((uint8)gpio, pu8OutVal);
      break;
  	}
	if(ret != M2M_SUCCESS) 
    goto _EXIT1;

_EXIT1:
	ret = hif_chip_sleep();
_EXIT:
	return ret;
  }

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION IMPLEMENTATION
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

sint8 m2m_periph_init(tstrPerphInitParam *param) {
	return M2M_SUCCESS;
  }

sint8 m2m_periph_gpio_set_dir(uint8 u8GpioNum, uint8 u8GpioDir) {   // 1 = OUTPUT
  
	return gpio_ioctl(GPIO_OP_DIR, u8GpioNum, u8GpioDir, NULL);
  }

sint8 m2m_periph_gpio_set_val(uint8 u8GpioNum, uint8 u8GpioVal) {
  
	return gpio_ioctl(GPIO_OP_SET, u8GpioNum, u8GpioVal, NULL);
  }

sint8 m2m_periph_gpio_get_val(uint8 u8GpioNum, uint8 *pu8GpioVal) {
  
	return gpio_ioctl(GPIO_OP_GET, u8GpioNum, 0, pu8GpioVal);
  }

sint8 m2m_periph_gpio_pullup_ctrl(uint8 u8GpioNum, uint8 u8PullupEn) {
	/* TBD */
	return M2M_SUCCESS;
  }

sint8 m2m_periph_i2c_master_init(tstrI2cMasterInitParam *param) {
	/* TBD */
	return M2M_SUCCESS;
  }

sint8 m2m_periph_i2c_master_write(uint8 u8SlaveAddr, uint8 *pu8Buf, uint16 u16BufLen, uint8 flags) {
	/* TBD */
	return M2M_SUCCESS;
  }

sint8 m2m_periph_i2c_master_read(uint8 u8SlaveAddr, uint8 *pu8Buf, uint16 u16BufLen, uint16 *pu16ReadLen, uint8 flags) {
	/* TBD */
	return M2M_SUCCESS;
  }


sint8 m2m_periph_pullup_ctrl(uint32 pinmask, uint8 enable) {
	return pullup_ctrl(pinmask, enable);
  }


// m2m_wifi.c ***********************************************************************************************************

//#include "driver/include/m2m_wifi.h"
//#include "driver/source/m2m_hif.h"
//#include "driver/source/nmasic.h"

static volatile uint8 gu8ChNum;
static volatile uint8 gu8scanInProgress = 0;
static tpfAppWifiCb gpfAppWifiCb = NULL;


#ifdef ETH_MODE
static tpfAppEthCb  gpfAppEthCb  = NULL;
static uint8* 	        gau8ethRcvBuf=NULL;
static uint16 	        gu16ethRcvBufSize ;
#endif


//#define CONF_MGMT
#ifdef CONF_MGMT
static tpfAppMonCb  gpfAppMonCb  = NULL;
static struct _tstrMgmtCtrl
{
	uint8* pu8Buf;
	uint16 u16Offset;
	uint16 u16Sz;
}
gstrMgmtCtrl = {NULL, 0 , 0};
#endif
/**
*	@fn			m2m_wifi_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr, uint8 grp)
*	@brief		WiFi call back function
*	@param [in]	u8OpCode
*					HIF Opcode type.
*	@param [in]	u16DataSize
*					HIF data length.
*	@param [in]	u32Addr
*					HIF address.
*	@param [in]	grp
*					HIF group type.
*	@author
*	@date
*	@version	1.0
*/
static void m2m_wifi_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr) {
	uint8 rx_buf[8];
  
	switch(u8OpCode) {
    case M2M_WIFI_RESP_CON_STATE_CHANGED:
    {
      tstrM2mWifiStateChanged strState;
      if(hif_receive(u32Addr, (uint8*)&strState,sizeof(tstrM2mWifiStateChanged), 0) == M2M_SUCCESS)	{
        if(gpfAppWifiCb)
          gpfAppWifiCb(M2M_WIFI_RESP_CON_STATE_CHANGED, &strState);
        }
    }
      break;
  	case M2M_WIFI_RESP_GET_SYS_TIME:
    {
      tstrSystemTime strSysTime;
      if(hif_receive(u32Addr, (uint8*)&strSysTime,sizeof(tstrSystemTime), 0) == M2M_SUCCESS) {
        if(gpfAppWifiCb)
          gpfAppWifiCb(M2M_WIFI_RESP_GET_SYS_TIME, &strSysTime);
        }
    }
      break;
    case M2M_WIFI_RESP_CONN_INFO:
    {
      //https://microchipsupport.force.com/s/article/How-to-retrieve-the-Connection-Parameters-after-WINC1500-is-connected-to-an-Access-Point-AP
      tstrM2MConnInfo		strConnInfo;
      if(hif_receive(u32Addr, (uint8*)&strConnInfo, sizeof(tstrM2MConnInfo), 1) == M2M_SUCCESS)	{
        if(gpfAppWifiCb)
          gpfAppWifiCb(M2M_WIFI_RESP_CONN_INFO, &strConnInfo);
        }
    }
      break;
  	case M2M_WIFI_RESP_MEMORY_RECOVER:
      {
#if 0
		if(hif_receive(u32Addr, rx_buf, 4, 1) == M2M_SUCCESS) {
			tstrM2mWifiStateChanged strState;
			m2m_memcpy((uint8*) &strState, rx_buf,sizeof(tstrM2mWifiStateChanged));
			if (app_wifi_recover_cb)
				app_wifi_recover_cb(strState.u8CurrState);
      }
#endif
      }
      break;
    case M2M_WIFI_REQ_DHCP_CONF:
    {
      tstrM2MIPConfig strIpConfig;
      if(hif_receive(u32Addr, (uint8 *)&strIpConfig, sizeof(tstrM2MIPConfig), 0) == M2M_SUCCESS)	{
        if(gpfAppWifiCb)
          gpfAppWifiCb(M2M_WIFI_REQ_DHCP_CONF, (uint8 *)&strIpConfig);
        }
    }
      break;
    case M2M_WIFI_REQ_WPS:
    {
      tstrM2MWPSInfo strWps;
      m2m_memset((uint8*)&strWps,0,sizeof(tstrM2MWPSInfo));
      if(hif_receive(u32Addr, (uint8*)&strWps, sizeof(tstrM2MWPSInfo), 0) == M2M_SUCCESS)	{
        if(gpfAppWifiCb)
          gpfAppWifiCb(M2M_WIFI_REQ_WPS, &strWps);
        }
    }
      break;
  	case M2M_WIFI_RESP_IP_CONFLICT:
    {
      uint32  u32ConflictedIP;
      if(hif_receive(u32Addr, (uint8 *)&u32ConflictedIP, sizeof(u32ConflictedIP), 0) == M2M_SUCCESS) {
        M2M_INFO("Conflicted IP \" %u.%u.%u.%u \" \n", 
          BYTE_0(u32ConflictedIP),BYTE_1(u32ConflictedIP),BYTE_2(u32ConflictedIP),BYTE_3(u32ConflictedIP));
        if(gpfAppWifiCb)
          gpfAppWifiCb(M2M_WIFI_RESP_IP_CONFLICT, NULL);
        }
    }
      break;
    case M2M_WIFI_RESP_SCAN_DONE:
    {
      tstrM2mScanDone strState;
      gu8scanInProgress = 0;
      if(hif_receive(u32Addr, (uint8*)&strState, sizeof(tstrM2mScanDone), 0) == M2M_SUCCESS)	{
        gu8ChNum = strState.u8NumofCh;
        if(gpfAppWifiCb)
          gpfAppWifiCb(M2M_WIFI_RESP_SCAN_DONE, &strState);
        }
    }
      break;
    case M2M_WIFI_RESP_SCAN_RESULT:
    {
      tstrM2mWifiscanResult strScanResult;
      if(hif_receive(u32Addr, (uint8*)&strScanResult, sizeof(tstrM2mWifiscanResult), 0) == M2M_SUCCESS)	{
        if(gpfAppWifiCb)
          gpfAppWifiCb(M2M_WIFI_RESP_SCAN_RESULT, &strScanResult);
        }
    }
      break;
    case M2M_WIFI_RESP_CURRENT_RSSI:
      if(hif_receive(u32Addr, rx_buf, 4, 0) == M2M_SUCCESS) {
        if(gpfAppWifiCb)
          gpfAppWifiCb(M2M_WIFI_RESP_CURRENT_RSSI, rx_buf);
        }
      break;
    case M2M_WIFI_RESP_CLIENT_INFO:
      if(hif_receive(u32Addr, rx_buf, 4, 0) == M2M_SUCCESS)	{
        if(gpfAppWifiCb)
          gpfAppWifiCb(M2M_WIFI_RESP_CLIENT_INFO, rx_buf);
        }
      break;
    case M2M_WIFI_RESP_PROVISION_INFO:
    {
      tstrM2MProvisionInfo	strProvInfo;
      if(hif_receive(u32Addr, (uint8*)&strProvInfo, sizeof(tstrM2MProvisionInfo), 1) == M2M_SUCCESS) {
        if(gpfAppWifiCb)
          gpfAppWifiCb(M2M_WIFI_RESP_PROVISION_INFO, &strProvInfo);
        }
    }
      break;
    case M2M_WIFI_RESP_DEFAULT_CONNECT:
    {
      tstrM2MDefaultConnResp	strResp;
      if(hif_receive(u32Addr, (uint8*)&strResp, sizeof(tstrM2MDefaultConnResp), 1) == M2M_SUCCESS) {
        if(gpfAppWifiCb)
          gpfAppWifiCb(M2M_WIFI_RESP_DEFAULT_CONNECT, &strResp);
        }
    }
      break;
    case M2M_WIFI_RESP_GET_PRNG:
    {
      tstrPrng strPrng;
      if(hif_receive(u32Addr, (uint8*)&strPrng,sizeof(tstrPrng), 0) == M2M_SUCCESS)	{
        if(hif_receive(u32Addr + sizeof(tstrPrng),strPrng.pu8RngBuff,strPrng.u16PrngSize, 1) == M2M_SUCCESS) {
          if(gpfAppWifiCb)
            gpfAppWifiCb(M2M_WIFI_RESP_GET_PRNG,&strPrng);
          }
        }
    }
      break;
#ifdef ETH_MODE
    case M2M_WIFI_RESP_ETHERNET_RX_PACKET:
			uint8 u8SetRxDone;
			tstrM2mIpRsvdPkt strM2mRsvd;
			if(hif_receive(u32Addr, &strM2mRsvd ,sizeof(tstrM2mIpRsvdPkt), 0) == M2M_SUCCESS)	{
				tstrM2mIpCtrlBuf  strM2mIpCtrlBuf;
				uint16 u16Offset = strM2mRsvd.u16PktOffset;
				strM2mIpCtrlBuf.u16RemainigDataSize = strM2mRsvd.u16PktSz;
				if((gpfAppEthCb) && (gau8ethRcvBuf) && (gu16ethRcvBufSize > 0))	{
					do {
						u8SetRxDone = 1;
						if(strM2mIpCtrlBuf.u16RemainigDataSize > gu16ethRcvBufSize)	{
							u8SetRxDone = 0;
							strM2mIpCtrlBuf.u16DataSize = gu16ethRcvBufSize;
              }
						else {
							strM2mIpCtrlBuf.u16DataSize = strM2mIpCtrlBuf.u16RemainigDataSize;
              }

						if(hif_receive(u32Addr + u16Offset, gau8ethRcvBuf, strM2mIpCtrlBuf.u16DataSize, u8SetRxDone) == M2M_SUCCESS) {
							strM2mIpCtrlBuf.u16RemainigDataSize -= strM2mIpCtrlBuf.u16DataSize;							
							u16Offset += strM2mIpCtrlBuf.u16DataSize;
							gpfAppEthCb(M2M_WIFI_RESP_ETHERNET_RX_PACKET, gau8ethRcvBuf, &(strM2mIpCtrlBuf));
						}
						else {
							break;
						}
					} while (strM2mIpCtrlBuf.u16RemainigDataSize > 0);
				}
			}
      break;
#endif	/* ETH_MODE */
#ifdef CONF_MGMT
    case M2M_WIFI_RESP_WIFI_RX_PACKET:
      tstrM2MWifiRxPacketInfo		strRxPacketInfo;
      if(u16DataSize >= sizeof(tstrM2MWifiRxPacketInfo)) {
        if(hif_receive(u32Addr, (uint8*)&strRxPacketInfo, sizeof(tstrM2MWifiRxPacketInfo), 0) == M2M_SUCCESS)	{
          u16DataSize -= sizeof(tstrM2MWifiRxPacketInfo);
          if(u16DataSize > 0 && gstrMgmtCtrl.pu8Buf) {
            if(u16DataSize > (gstrMgmtCtrl.u16Sz + gstrMgmtCtrl.u16Offset))	{
              u16DataSize = gstrMgmtCtrl.u16Sz;
            }
            u32Addr += sizeof(tstrM2MWifiRxPacketInfo) + gstrMgmtCtrl.u16Offset;
            if(hif_receive(u32Addr , gstrMgmtCtrl.pu8Buf, u16DataSize, 1) != M2M_SUCCESS) return;
          }
          if(gpfAppMonCb)
            gpfAppMonCb(&strRxPacketInfo, gstrMgmtCtrl.pu8Buf,u16DataSize);
          }
        } 
      else {
        M2M_ERR("Incorrect mon data size %u\n", u16DataSize);
      }
    break;
#endif
    default:
      M2M_ERR("REQ Not defined %d\n",u8OpCode);
      break;
    }
  }

sint8 m2m_wifi_download_mode() {
	sint8 ret = M2M_SUCCESS;
	/* Apply device specific initialization. */
	ret = nm_drv_init_download_mode();
	if(ret != M2M_SUCCESS) 	
    goto _EXIT0;


	enable_interrupts();

_EXIT0:
	return ret;
  }

static sint8 m2m_validate_ap_parameters(CONST tstrM2MAPConfig* pstrM2MAPConfig) {
	sint8 s8Ret = M2M_SUCCESS;
  
	/* Check for incoming pointer */
	if(!pstrM2MAPConfig)	{
		M2M_ERR("INVALID POINTER\n");
		s8Ret = M2M_ERR_FAIL;
		goto ERR1;
    }
	/* Check for SSID */
	if((strlen((uint8 *)pstrM2MAPConfig->au8SSID) <= 0) || (strlen((uint8 *)pstrM2MAPConfig->au8SSID) >= M2M_MAX_SSID_LEN))	{
		M2M_ERR("INVALID SSID\n");
		s8Ret = M2M_ERR_FAIL;
		goto ERR1;
    }
	/* Check for Channel */
	if(pstrM2MAPConfig->u8ListenChannel > M2M_WIFI_CH_14 || pstrM2MAPConfig->u8ListenChannel < M2M_WIFI_CH_1)	{
		M2M_ERR("INVALID CH\n");
		s8Ret = M2M_ERR_FAIL;
		goto ERR1;
    }
	/* Check for DHCP Server IP address */
	if(!(pstrM2MAPConfig->au8DHCPServerIP[0] || pstrM2MAPConfig->au8DHCPServerIP[1]))	{
		if(!(pstrM2MAPConfig->au8DHCPServerIP[2])) {
			M2M_ERR("INVALID DHCP SERVER IP\n");
			s8Ret = M2M_ERR_FAIL;
			goto ERR1;
      }
    }
	/* Check for Security */
	if(pstrM2MAPConfig->u8SecType == M2M_WIFI_SEC_OPEN)	{
		goto ERR1;
    }
	else if(pstrM2MAPConfig->u8SecType == M2M_WIFI_SEC_WEP)	{
		/* Check for WEP Key index */
		if((pstrM2MAPConfig->u8KeyIndx <= 0) || (pstrM2MAPConfig->u8KeyIndx > WEP_KEY_MAX_INDEX))	{
			M2M_ERR("INVALID KEY INDEX\n");
			s8Ret = M2M_ERR_FAIL;
			goto ERR1;
      }
		/* Check for WEP Key size */
		if(	(pstrM2MAPConfig->u8KeySz != WEP_40_KEY_STRING_SIZE) &&
			(pstrM2MAPConfig->u8KeySz != WEP_104_KEY_STRING_SIZE)
      )	{
			M2M_ERR("INVALID KEY SIZE\n");
			s8Ret = M2M_ERR_FAIL;
			goto ERR1;
      }
		/* Check for WEP Key */
		if((!pstrM2MAPConfig->au8WepKey) || (strlen((uint8 *)pstrM2MAPConfig->au8WepKey) <= 0) || (strlen((uint8 *)pstrM2MAPConfig->au8WepKey) > WEP_104_KEY_STRING_SIZE)) {
			M2M_ERR("INVALID WEP KEY\n");
			s8Ret = M2M_ERR_FAIL;
			goto ERR1;
      }
    }
	else	{
		M2M_ERR("INVALID AUTHENTICATION MODE\n");
		s8Ret = M2M_ERR_FAIL;
		goto ERR1;
    }
	
ERR1:
	return s8Ret;
  }

static sint8 m2m_validate_scan_options(tstrM2MScanOption* ptstrM2MScanOption) {
	sint8 s8Ret = M2M_SUCCESS;
  
	/* Check for incoming pointer */
	if(!ptstrM2MScanOption) {
		M2M_ERR("INVALID POINTER\n");
		s8Ret = M2M_ERR_FAIL;
    }	
	/* Check for valid No of slots */
	 if(ptstrM2MScanOption->u8NumOfSlot < 1) {
		M2M_ERR("INVALID No of scan slots!\n");
		s8Ret = M2M_ERR_FAIL;
    }	
	/* Check for valid time of slots */
	 if(ptstrM2MScanOption->u8SlotTime < 1)	{
		M2M_ERR("INVALID scan slot time!\n");
		s8Ret = M2M_ERR_FAIL;
    }	
	/* Check for valid No of probe requests per slot */
	 if((ptstrM2MScanOption->u8ProbesPerSlot < 0)||(ptstrM2MScanOption->u8ProbesPerSlot > M2M_SCAN_DEFAULT_NUM_PROBE)) {
		M2M_ERR("INVALID No of probe requests per scan slot\n");
		s8Ret = M2M_ERR_FAIL;
    }	
	/* Check for valid RSSI threshold */
	 if((ptstrM2MScanOption->s8RssiThresh  < -99) || (ptstrM2MScanOption->s8RssiThresh >= 0))	{
		M2M_ERR("INVALID RSSI threshold %d \n",ptstrM2MScanOption->s8RssiThresh);
		s8Ret = M2M_ERR_FAIL;
    }	
	return s8Ret;
  }

sint8 m2m_wifi_init(tstrWifiInitParam *param) {
	tstrM2mRev strtmp;
	sint8 ret = M2M_SUCCESS;
	uint8 u8WifiMode = M2M_WIFI_MODE_NORMAL;
	
	if(!param) {
		ret = M2M_ERR_FAIL;
		goto _EXIT0;
    }
	
	gpfAppWifiCb = param->pfAppWifiCb;

#ifdef ETH_MODE
	gpfAppEthCb  	    = param->strEthInitParam.pfAppEthCb;
	gau8ethRcvBuf       = param->strEthInitParam.au8ethRcvBuf;
	gu16ethRcvBufSize	= param->strEthInitParam.u16ethRcvBufSize;
	u8WifiMode = param->strEthInitParam.u8EthernetEnable;
#endif /* ETH_MODE */

#ifdef CONF_MGMT
	gpfAppMonCb  = param->pfAppMonCb;
#endif
	gu8scanInProgress = 0;
	/* Apply device specific initialization. */
	ret = nm_drv_init(&u8WifiMode);
	if(ret != M2M_SUCCESS) 	
    goto _EXIT0;
  
	/* Initialize host interface module */
	ret = hif_init(NULL);
	if(ret != M2M_SUCCESS) 	
    goto _EXIT1;

	hif_register_cb(M2M_REQ_GROUP_WIFI,m2m_wifi_cb);

	ret = nm_get_firmware_info(&strtmp);

	M2M_INFO("Firmware ver   : %u.%u.%u\r\n", strtmp.u8FirmwareMajor, strtmp.u8FirmwareMinor, strtmp.u8FirmwarePatch);
	M2M_INFO("Min driver ver : %u.%u.%u\r\n", strtmp.u8DriverMajor, strtmp.u8DriverMinor, strtmp.u8DriverPatch);
	M2M_INFO("Curr driver ver: %u.%u.%u\r\n", M2M_DRIVER_VERSION_MAJOR_NO, M2M_DRIVER_VERSION_MINOR_NO, M2M_DRIVER_VERSION_PATCH_NO);
	if(M2M_ERR_FW_VER_MISMATCH == ret) {
		M2M_ERR("Mismatch Firmware Version\r\n");
    }

	goto _EXIT0;

_EXIT1:
	nm_drv_deinit(NULL);
_EXIT0:
	return ret;
  }

sint8 m2m_wifi_deinit(void *arg) {

	hif_deinit(NULL);
	nm_drv_deinit(NULL);
  
	return M2M_SUCCESS;
  }

sint8 m2m_wifi_handle_events(void *arg) {
	
  return hif_handle_isr();
  }

sint8 m2m_wifi_default_connect(void) {
	
  return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_DEFAULT_CONNECT, NULL, 0,NULL, 0,0);
  }

sint8 m2m_wifi_connect(char *pcSsid, uint8 u8SsidLen, uint8 u8SecType, void *pvAuthInfo, uint16 u16Ch) {
	
  return m2m_wifi_connect_sc(pcSsid, u8SsidLen, u8SecType, pvAuthInfo, u16Ch,0);
  }

sint8 m2m_wifi_connect_sc(char *pcSsid, uint8 u8SsidLen, uint8 u8SecType, void *pvAuthInfo, uint16 u16Ch, uint8 u8NoSaveCred) {
	sint8				ret = M2M_SUCCESS;
	tstrM2mWifiConnect	strConnect;
	tstrM2MWifiSecInfo	*pstrAuthInfo;

	if(u8SecType != M2M_WIFI_SEC_OPEN) {
		if(!pvAuthInfo) {
			M2M_ERR("Key is not valid\r\n");
			ret = M2M_ERR_FAIL;
			goto ERR1;
      }
		if((u8SecType == M2M_WIFI_SEC_WPA_PSK) && (strlen(pvAuthInfo) == (M2M_MAX_PSK_LEN-1))) {
			uint8 i = 0;
			uint8* pu8Psk = (uint8*)pvAuthInfo;
			while(i < (M2M_MAX_PSK_LEN-1)) {
				if(pu8Psk[i]<'0' || (pu8Psk[i]>'9' && pu8Psk[i] < 'A')|| (pu8Psk[i]>'F' && pu8Psk[i] < 'a') || pu8Psk[i] > 'f')	{
					M2M_ERR("Invalid Key\r\n");
					ret = M2M_ERR_FAIL;
					goto ERR1;
        	}
				i++;
        }
      }
    }
	if((u8SsidLen<=0) || (u8SsidLen>=M2M_MAX_SSID_LEN))	{
		M2M_ERR("SSID LEN INVALID\r\n");
		ret = M2M_ERR_FAIL;
		goto ERR1;
    }

	if(u16Ch>M2M_WIFI_CH_14) {
		if(u16Ch != M2M_WIFI_CH_ALL) {
			M2M_ERR("CH INVALID\r\n");
			ret = M2M_ERR_FAIL;
			goto ERR1;
      }
    }


	m2m_memcpy(strConnect.au8SSID, (uint8*)pcSsid, u8SsidLen);
	strConnect.au8SSID[u8SsidLen]	= 0;
	strConnect.u16Ch				= NM_BSP_B_L_16(u16Ch);
	/* Credentials will be Not be saved if u8NoSaveCred is set */ 
	strConnect.u8NoSaveCred 			= u8NoSaveCred ? 1:0;
	pstrAuthInfo = &strConnect.strSec;
	pstrAuthInfo->u8SecType		= u8SecType;

	if(u8SecType == M2M_WIFI_SEC_WEP)	{
		tstrM2mWifiWepParams *pstrWepParams = (tstrM2mWifiWepParams*)pvAuthInfo;
		tstrM2mWifiWepParams *pstrWep = &pstrAuthInfo->uniAuth.strWepInfo;
		pstrWep->u8KeyIndx =pstrWepParams->u8KeyIndx-1;

		if(pstrWep->u8KeyIndx >= WEP_KEY_MAX_INDEX)	{
			M2M_ERR("Invalid Wep key index %d\r\n", pstrWep->u8KeyIndx);
			ret = M2M_ERR_FAIL;
			goto ERR1;
		}
		pstrWep->u8KeySz = pstrWepParams->u8KeySz-1;
		if((pstrWep->u8KeySz != WEP_40_KEY_STRING_SIZE)&& (pstrWep->u8KeySz != WEP_104_KEY_STRING_SIZE))	{
			M2M_ERR("Invalid Wep key length %d\r\n", pstrWep->u8KeySz);
			ret = M2M_ERR_FAIL;
			goto ERR1;
      }
		m2m_memcpy((uint8*)pstrWep->au8WepKey,(uint8*)pstrWepParams->au8WepKey, pstrWepParams->u8KeySz);
		pstrWep->au8WepKey[pstrWepParams->u8KeySz] = 0;

    }

	else if(u8SecType == M2M_WIFI_SEC_WPA_PSK) {
		uint16	u16KeyLen = strlen((uint8*)pvAuthInfo);
		if((u16KeyLen <= 0)||(u16KeyLen >= M2M_MAX_PSK_LEN)) {
			M2M_ERR("Incorrect PSK key length\r\n");
			ret = M2M_ERR_FAIL;
			goto ERR1;
      }
		m2m_memcpy(pstrAuthInfo->uniAuth.au8PSK, (uint8*)pvAuthInfo, u16KeyLen + 1);
    }
	else if(u8SecType == M2M_WIFI_SEC_802_1X)	{
		m2m_memcpy((uint8*)&pstrAuthInfo->uniAuth.strCred1x, (uint8*)pvAuthInfo, sizeof(tstr1xAuthCredentials));
    }
	else if(u8SecType == M2M_WIFI_SEC_OPEN)	{

    }
	else	{
		M2M_ERR("undefined sec type\r\n");
		ret = M2M_ERR_FAIL;
		goto ERR1;
    }

	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_CONNECT, (uint8*)&strConnect, sizeof(tstrM2mWifiConnect),NULL, 0,0);

ERR1:
	return ret;
  }

sint8 m2m_wifi_disconnect(void) {
  
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_DISCONNECT, NULL, 0, NULL, 0,0);
  }

sint8 m2m_wifi_set_mac_address(uint8 au8MacAddress[6]) {
	tstrM2mSetMacAddress strTmp;
  
	m2m_memcpy((uint8*)strTmp.au8Mac, (uint8*)au8MacAddress, 6);
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_MAC_ADDRESS,
		(uint8*)&strTmp, sizeof(tstrM2mSetMacAddress), NULL, 0,0);
  }

sint8 m2m_wifi_set_static_ip(tstrM2MIPConfig *pstrStaticIPConf) {
  
	pstrStaticIPConf->u32DNS = NM_BSP_B_L_32(pstrStaticIPConf->u32DNS);
	pstrStaticIPConf->u32Gateway = NM_BSP_B_L_32(pstrStaticIPConf->u32Gateway);
	pstrStaticIPConf->u32StaticIP = NM_BSP_B_L_32(
		pstrStaticIPConf->u32StaticIP);
	pstrStaticIPConf->u32SubnetMask = NM_BSP_B_L_32(
		pstrStaticIPConf->u32SubnetMask);
  
	return hif_send(M2M_REQ_GROUP_IP, M2M_IP_REQ_STATIC_IP_CONF,
		(uint8*)pstrStaticIPConf, sizeof(tstrM2MIPConfig), NULL, 0,0);
  }

sint8 m2m_wifi_request_dhcp_client(void) {
	/*legacy API should be removed */
	return 0;
  }
sint8 m2m_wifi_request_dhcp_server(uint8* addr) {
    /*legacy API should be removed */
	return 0;
  }

/*!
@fn			NMI_API sint8 m2m_wifi_set_lsn_int(tstrM2mLsnInt *pstrM2mLsnInt);
@brief		Set the Wi-Fi listen interval for power save operation. It is represented in units
			of AP Beacon periods.
@param [in]	pstrM2mLsnInt
			Structure holding the listen interval configurations.
@return		The function SHALL return 0 for success and a negative value otherwise.
@sa			tstrM2mLsnInt , m2m_wifi_set_sleep_mode
@pre		m2m_wifi_set_sleep_mode shall be called first
@warning	The Function called once after initialization. 
*/
sint8 m2m_wifi_enable_dhcp(uint8 u8DhcpEn) {
	uint8	u8Req;
  
	u8Req = u8DhcpEn ? M2M_IP_REQ_ENABLE_DHCP : M2M_IP_REQ_DISABLE_DHCP;
	return hif_send(M2M_REQ_GROUP_IP, u8Req, NULL, 0, NULL, 0, 0);
  }

sint8 m2m_wifi_set_lsn_int(tstrM2mLsnInt* pstrM2mLsnInt) {
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_LSN_INT, (uint8*)pstrM2mLsnInt, sizeof(tstrM2mLsnInt), NULL, 0, 0);
  }

sint8 m2m_wifi_set_cust_InfoElement(uint8* pau8M2mCustInfoElement) {

	sint8  ret = M2M_ERR_FAIL;
	if(pau8M2mCustInfoElement) {
		if((pau8M2mCustInfoElement[0] + 1) < M2M_CUST_IE_LEN_MAX)	{
			ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_CUST_INFO_ELEMENT|M2M_REQ_DATA_PKT, (uint8*)pau8M2mCustInfoElement, pau8M2mCustInfoElement[0]+1, NULL, 0, 0);
      }
    }
	return ret;
  }

sint8 m2m_wifi_set_scan_options(tstrM2MScanOption* ptstrM2MScanOption) {
	sint8	s8Ret = M2M_ERR_FAIL;
  
	if(m2m_validate_scan_options (ptstrM2MScanOption) == M2M_SUCCESS)	{
		s8Ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_SCAN_OPTION, (uint8*)ptstrM2MScanOption, sizeof(tstrM2MScanOption),NULL, 0,0);
    }
	return s8Ret;
  }

sint8 m2m_wifi_set_scan_region(uint16  ScanRegion) {
	sint8	s8Ret = M2M_ERR_FAIL;
	tstrM2MScanRegion strScanRegion;
  
	strScanRegion.u16ScanRegion = ScanRegion;
  s8Ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_SCAN_REGION, (uint8*)&strScanRegion, sizeof(tstrM2MScanRegion),NULL, 0,0);
	return s8Ret;
  }

sint8 m2m_wifi_request_scan(uint8 ch) {
	sint8	s8Ret = M2M_SUCCESS;

	if(!gu8scanInProgress)	{
		if(((ch >= M2M_WIFI_CH_1) && (ch <= M2M_WIFI_CH_14)) || (ch == M2M_WIFI_CH_ALL))	{
			tstrM2MScan strtmp;
			strtmp.u8ChNum = ch;
			s8Ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SCAN, (uint8*)&strtmp, sizeof(tstrM2MScan),NULL, 0,0);
			if(s8Ret == M2M_SUCCESS) {
				gu8scanInProgress = 1;
        }
      }
		else {
			s8Ret = M2M_ERR_INVALID_ARG;
      }
    }
	else {
		s8Ret = M2M_ERR_SCAN_IN_PROGRESS;
    }
	return s8Ret;
  }

sint8 m2m_wifi_wps(uint8 u8TriggerType,const char  *pcPinNumber) {
	tstrM2MWPSConnect strtmp;

	/* Stop Scan if it is ongoing.
	*/
	gu8scanInProgress = 0;
	strtmp.u8TriggerType = u8TriggerType;
	/*If WPS is using PIN METHOD*/
	if(u8TriggerType == WPS_PIN_TRIGGER)
		m2m_memcpy((uint8*)strtmp.acPinNumber,(uint8*)pcPinNumber,8);
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_WPS, (uint8*)&strtmp,sizeof(tstrM2MWPSConnect), NULL, 0,0);
  }

sint8 m2m_wifi_wps_disable(void) {
	sint8 ret = M2M_SUCCESS;
  
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_DISABLE_WPS, NULL,0, NULL, 0, 0);
	return ret;
  }

/*!
@fn			NMI_API sint8 m2m_wifi_req_client_ctrl(uint8 cmd);
@brief		Send a command to the PS Client (An WINC1500 board running the ps_firmware), 
			if the PS client send any commands it will be received in wifi_cb M2M_WIFI_RESP_CLIENT_INFO
@param [in]	cmd
			Control command sent from PS Server to PS Client (command values defined by the application)
@return		The function SHALL return M2M_SUCCESS for success and a negative value otherwise.
@sa			m2m_wifi_req_server_init, M2M_WIFI_RESP_CLIENT_INFO
@pre		m2m_wifi_req_server_init should be called first
@warning	
*/
sint8 m2m_wifi_req_client_ctrl(uint8 u8Cmd) {

	sint8 ret = M2M_SUCCESS;
#ifdef _PS_SERVER_
	tstrM2Mservercmd	strCmd;
	strCmd.u8cmd = u8Cmd;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_CLIENT_CTRL, (uint8*)&strCmd, sizeof(tstrM2Mservercmd), NULL, 0, 0);
#else
	M2M_ERR("_PS_SERVER_ is not defined\r\n");
#endif
	return ret;
  }

/*!
@fn			NMI_API sint8 m2m_wifi_req_server_init(uint8 ch);
@brief		Initialize the PS Server, The WINC1500 support Non secure communication with another WINC1500, 
			(SERVER/CLIENT) through one byte command (probe request and probe response) without any connection setup
@param [in]	ch
			Server listening channel
@return		The function SHALL return M2M_SUCCESS for success and a negative value otherwise
@sa			m2m_wifi_req_client_ctrl
@warning	The server mode can't be used with any other modes (STA/P2P/AP)
*/
sint8 m2m_wifi_req_server_init(uint8 ch) {
	sint8 ret = M2M_SUCCESS;
  
#ifdef _PS_SERVER_
	tstrM2mServerInit strServer;
	strServer.u8Channel = ch;
	ret = hif_send(M2M_REQ_GROUP_WIFI,M2M_WIFI_REQ_SERVER_INIT, (uint8*)&strServer, sizeof(tstrM2mServerInit), NULL, 0, 0);
#else
	M2M_ERR("_PS_SERVER_ is not defined\r\n");
#endif
	return ret;
  }

sint8 m2m_wifi_p2p(uint8 u8Channel) {
	sint8 ret = M2M_SUCCESS;
  
	if((u8Channel == M2M_WIFI_CH_1) || (u8Channel == M2M_WIFI_CH_6) || (u8Channel == M2M_WIFI_CH_11))	{
		tstrM2MP2PConnect strtmp;
		strtmp.u8ListenChannel = u8Channel;
		ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_ENABLE_P2P, (uint8*)&strtmp, sizeof(tstrM2MP2PConnect), NULL, 0,0);
    }
	else {
		M2M_ERR("Listen channel should only be 1, 6 or 11\r\n");
		ret = M2M_ERR_FAIL;
    }
	return ret;
  }

sint8 m2m_wifi_p2p_disconnect(void) {
	sint8 ret = M2M_SUCCESS;
  
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_DISABLE_P2P, NULL, 0, NULL, 0, 0);
	return ret;
  }

sint8 m2m_wifi_enable_ap(CONST tstrM2MAPConfig* pstrM2MAPConfig) {
	sint8 ret = M2M_ERR_FAIL;
  
	if(M2M_SUCCESS == m2m_validate_ap_parameters(pstrM2MAPConfig))	{
		ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_ENABLE_AP, (uint8 *)pstrM2MAPConfig, sizeof(tstrM2MAPConfig), NULL, 0, 0);	
    }
	return ret;
  }

sint8 m2m_wifi_disable_ap(void) {
	sint8 ret = M2M_SUCCESS;
  
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_DISABLE_AP, NULL, 0, NULL, 0, 0);
	return ret;
  }

/*!
@fn          NMI_API sint8 m2m_wifi_req_curr_rssi(void);
@brief       Request the current RSSI for the current connected AP, 
			 the response received in wifi_cb M2M_WIFI_RESP_CURRENT_RSSI	
@sa          M2M_WIFI_RESP_CURRENT_RSSI              
@return      The function shall return M2M_SUCCESS for success and a negative value otherwise.
*/
sint8 m2m_wifi_req_curr_rssi(void) {
	sint8 ret = M2M_SUCCESS;
  
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_CURRENT_RSSI, NULL, 0, NULL,0, 0);
	return ret;
  }

sint8 m2m_wifi_send_ethernet_pkt(uint8* pu8Packet,uint16 u16PacketSize) {
	sint8	s8Ret = -1;
  
	if((pu8Packet) && (u16PacketSize>0)) {
		tstrM2MWifiTxPacketInfo		strTxPkt;

		strTxPkt.u16PacketSize		= u16PacketSize;
		strTxPkt.u16HeaderLength	= M2M_ETHERNET_HDR_LEN;
		s8Ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SEND_ETHERNET_PACKET | M2M_REQ_DATA_PKT,
		(uint8*)&strTxPkt, sizeof(tstrM2MWifiTxPacketInfo), pu8Packet, u16PacketSize,  M2M_ETHERNET_HDR_OFFSET - M2M_HIF_HDR_OFFSET);
    }
	return s8Ret;
  }

/*!
@fn          NMI_API sint8 m2m_wifi_get_otp_mac_address(uint8 *pu8MacAddr, uint8 *pu8IsValid);
@brief       Request the MAC address stored on the OTP (one time programmable) memory of the device.
			 (the function is Blocking until response received)	
@param [out] pu8MacAddr
			 Output MAC address buffer of 6 bytes size. Valid only if *pu8Valid=1.
@param [out] pu8IsValid
		     A output boolean value to indicate the validity of pu8MacAddr in OTP. 
		     Output zero if the OTP memory is not programmed, non-zero otherwise.	
@return      The function shall return M2M_SUCCESS for success and a negative value otherwise.
@sa          m2m_wifi_get_mac_address             
@pre         m2m_wifi_init required to call any WIFI/socket function
*/
sint8 m2m_wifi_get_otp_mac_address(uint8 *pu8MacAddr, uint8* pu8IsValid) {
	sint8 ret = M2M_SUCCESS;
  
	ret = hif_chip_wake();
	if(ret == M2M_SUCCESS) {
		ret = nmi_get_otp_mac_address(pu8MacAddr, pu8IsValid);
		if(ret == M2M_SUCCESS) {
			ret = hif_chip_sleep();
      }
    }
	return ret;
  }

/*!
@fn          NMI_API sint8 m2m_wifi_get_mac_address(uint8 *pu8MacAddr)
@brief       Request the current MAC address of the device (the working mac address).
			 (the function is Blocking until response received)	
@param [out] pu8MacAddr
			 Output MAC address buffer of 6 bytes size.	
@return      The function shall return M2M_SUCCESS for success and a negative value otherwise.
@sa          m2m_wifi_get_otp_mac_address             
@pre         m2m_wifi_init required to call any WIFI/socket function
*/
sint8 m2m_wifi_get_mac_address(uint8 *pu8MacAddr) {
	sint8 ret = M2M_SUCCESS;
  
	ret = hif_chip_wake();
	if(ret == M2M_SUCCESS) {
		ret = nmi_get_mac_address(pu8MacAddr);
		if(ret == M2M_SUCCESS) {
			ret = hif_chip_sleep();
      }
    }

	return ret;
  }

/*!
@fn          NMI_API sint8 m2m_wifi_req_scan_result(uint8 index);
@brief       Reads the AP information from the Scan Result list with the given index, 
			 the response received in wifi_cb M2M_WIFI_RESP_SCAN_RESULT, 
			 the response pointer should be casted with tstrM2mWifiscanResult structure 	
@param [in]  index 
			 Index for the requested result, the index range start from 0 till number of AP's found 
@sa          tstrM2mWifiscanResult,m2m_wifi_get_num_ap_found,m2m_wifi_request_scan             
@return      The function shall return M2M_SUCCESS for success and a negative value otherwise
@pre         m2m_wifi_request_scan need to be called first, then m2m_wifi_get_num_ap_found 
			 to get the number of AP's found
@warning     Function used only in STA mode only. the scan result updated only if scan request called,
			 else it will be cashed in firmware for the host scan request result, 
			 which mean if large delay occur between the scan request and the scan result request, 
			 the result will not be up-to-date
*/
sint8 m2m_wifi_req_scan_result(uint8 index) {
	sint8 ret = M2M_SUCCESS;
	tstrM2mReqScanResult strReqScanRlt;
  
	strReqScanRlt.u8Index = index;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SCAN_RESULT, (uint8*)&strReqScanRlt, sizeof(tstrM2mReqScanResult), NULL, 0, 0);
	return ret;
  }

/*!
@fn          NMI_API uint8 m2m_wifi_get_num_ap_found(void);
@brief       Reads the number of AP's found in the last Scan Request, 
			 The function read the number of AP's from global variable which updated in the 
			 wifi_cb in M2M_WIFI_RESP_SCAN_DONE.			 
@sa          m2m_wifi_request_scan               
@return      Return the number of AP's found in the last Scan Request.
@pre         m2m_wifi_request_scan need to be called first 
@warning     That function need to be called in the wifi_cb in M2M_WIFI_RESP_SCAN_DONE, 
			 calling that function in any other place will return undefined/undated numbers.
			 Function used only in STA mode only.
*/
uint8 m2m_wifi_get_num_ap_found(void) {
	return gu8ChNum;
  }

/*!
@fn		    NMI_API uint8 m2m_wifi_get_sleep_mode(void);
@brief	    Get the current Power save mode.
@return	    The current operating power saving mode.
@sa		    tenuPowerSaveModes , m2m_wifi_set_sleep_mode
*/
uint8 m2m_wifi_get_sleep_mode(void) {
	return hif_get_sleep_mode();
  }

/*!
@fn			NMI_API sint8 m2m_wifi_set_sleep_mode(uint8 PsTyp, uint8 BcastEn);
@brief      Set the power saving mode for the WINC1500. 
@param [in]	PsTyp
			Desired power saving mode. Supported types are defined in tenuPowerSaveModes.
@param [in]	BcastEn
			Broadcast reception enable flag. 
			If it is 1, the WINC1500 must be awake each DTIM Beacon for receiving Broadcast traffic.
			If it is 0, the WINC1500 will not wakeup at the DTIM Beacon, but its wakeup depends only 
			on the the configured Listen Interval. 
@return     The function SHALL return 0 for success and a negative value otherwise.
@sa			tenuPowerSaveModes
@warning    The function called once after initialization.  
*/
sint8 m2m_wifi_set_sleep_mode(uint8 PsTyp, uint8 BcastEn) {
	sint8 ret = M2M_SUCCESS;
	tstrM2mPsType strPs;
  
	strPs.u8PsType = PsTyp;
	strPs.u8BcastEn = BcastEn;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SLEEP, (uint8*)&strPs,sizeof(tstrM2mPsType), NULL, 0, 0);
	M2M_INFO("POWER SAVE %d\r\n",PsTyp);
	hif_set_sleep_mode(PsTyp);
	return ret;
  }

/*!
@fn	        NMI_API sint8 m2m_wifi_request_sleep(void)
@brief	    Request from WINC1500 device to Sleep for specific time in the M2M_PS_MANUAL Power save mode (only).
@param [in]	u32SlpReqTime
			Request Sleep in ms 
@return     The function SHALL return M2M_SUCCESS for success and a negative value otherwise.
@sa         tenuPowerSaveModes , m2m_wifi_set_sleep_mode
@warning	the Function should be called in M2M_PS_MANUAL power save only 
*/
sint8 m2m_wifi_request_sleep(uint32 u32SlpReqTime) {
	sint8 ret = M2M_SUCCESS;
	uint8 psType;
  
	psType = hif_get_sleep_mode();
	if(psType == M2M_PS_MANUAL)	{
		tstrM2mSlpReqTime strPs;
		strPs.u32SleepTime = u32SlpReqTime;
		ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_DOZE, (uint8*)&strPs,sizeof(tstrM2mSlpReqTime), NULL, 0, 0);
    }
	return ret;
  }

/*!
@fn			NMI_API sint8 m2m_wifi_set_device_name(uint8 *pu8DeviceName, uint8 u8DeviceNameLength);
@brief		Set the WINC1500 device name which is used as P2P device name.
@param [in]	pu8DeviceName
			Buffer holding the device name.
@param [in]	u8DeviceNameLength
			Length of the device name.
@return		The function SHALL return M2M_SUCCESS for success and a negative value otherwise.
@warning	The Function called once after initialization. 
*/
sint8 m2m_wifi_set_device_name(uint8 *pu8DeviceName, uint8 u8DeviceNameLength) {
	tstrM2MDeviceNameConfig strDeviceName;
  
	if(u8DeviceNameLength >= M2M_DEVICE_NAME_MAX)	{
		u8DeviceNameLength = M2M_DEVICE_NAME_MAX;
	}
	//pu8DeviceName[u8DeviceNameLength] = '\0';
	u8DeviceNameLength ++;
	m2m_memcpy(strDeviceName.au8DeviceName, pu8DeviceName, u8DeviceNameLength);
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_DEVICE_NAME,
		(uint8*)&strDeviceName, sizeof(tstrM2MDeviceNameConfig), NULL, 0,0);
  }

sint8 m2m_wifi_get_firmware_version(tstrM2mRev *pstrRev) {
	sint8 ret = M2M_SUCCESS;
  
	ret = hif_chip_wake();
	if(ret == M2M_SUCCESS) {
    	ret = nm_get_firmware_full_info(pstrRev);
		hif_chip_sleep();
    }
	return ret;
  }

#ifdef CONF_MGMT
sint8 m2m_wifi_enable_monitoring_mode(tstrM2MWifiMonitorModeCtrl *pstrMtrCtrl, uint8 *pu8PayloadBuffer,
								   uint16 u16BufferSize, uint16 u16DataOffset) {
	sint8	s8Ret = -1;
  
	if((pstrMtrCtrl->u8ChannelID >= M2M_WIFI_CH_1) && (pstrMtrCtrl->u8ChannelID <= M2M_WIFI_CH_14))	{
		gstrMgmtCtrl.pu8Buf		= pu8PayloadBuffer;
		gstrMgmtCtrl.u16Sz		= u16BufferSize;
		gstrMgmtCtrl.u16Offset	= u16DataOffset;
		s8Ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_ENABLE_MONITORING,
			(uint8*)pstrMtrCtrl, sizeof(tstrM2MWifiMonitorModeCtrl), NULL, 0,0);
    }
	return s8Ret;
  }

sint8 m2m_wifi_disable_monitoring_mode(void) {
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_DISABLE_MONITORING, NULL, 0, NULL, 0,0);
  }

sint8 m2m_wifi_send_wlan_pkt(uint8 *pu8WlanPacket, uint16 u16WlanHeaderLength, uint16 u16WlanPktSize) {
	sint8	s8Ret = -1;
  
	if(pu8WlanPacket)	{
		tstrM2MWifiTxPacketInfo		strTxPkt;

		strTxPkt.u16PacketSize		= u16WlanPktSize;
		strTxPkt.u16HeaderLength	= u16WlanHeaderLength;
		s8Ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SEND_WIFI_PACKET | M2M_REQ_DATA_PKT,
		(uint8*)&strTxPkt, sizeof(tstrM2MWifiTxPacketInfo), pu8WlanPacket, u16WlanPktSize, sizeof(tstrM2MWifiTxPacketInfo));
    }
	return s8Ret;
  }
#endif

sint8 m2m_wifi_start_provision_mode(tstrM2MAPConfig *pstrAPConfig, char *pcHttpServerDomainName, uint8 bEnableHttpRedirect) {
	sint8	s8Ret = M2M_ERR_FAIL;

	if((pstrAPConfig)) {
		tstrM2MProvisionModeConfig	strProvConfig;
		if(M2M_SUCCESS == m2m_validate_ap_parameters(pstrAPConfig))	{
			m2m_memcpy((uint8*)&strProvConfig.strApConfig, (uint8*)pstrAPConfig, sizeof(tstrM2MAPConfig));
			if((strlen((uint8 *)pcHttpServerDomainName) <= 0) || (NULL == pcHttpServerDomainName)) {
				M2M_ERR("INVALID DOMAIN NAME\r\n");
				goto ERR1;
    		}
			m2m_memcpy((uint8*)strProvConfig.acHttpServerDomainName, (uint8*)pcHttpServerDomainName, 64);
			strProvConfig.u8EnableRedirect = bEnableHttpRedirect;
		
			/* Stop Scan if it is ongoing.
			*/
			gu8scanInProgress = 0;
			s8Ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_START_PROVISION_MODE | M2M_REQ_DATA_PKT, 
						(uint8*)&strProvConfig, sizeof(tstrM2MProvisionModeConfig), NULL, 0, 0);
		}
		else {
			/*goto ERR1;*/
      }
    }
  
ERR1:
	return s8Ret;
  }

sint8 m2m_wifi_stop_provision_mode(void) {
  
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_STOP_PROVISION_MODE, NULL, 0, NULL, 0, 0);
  }

sint8 m2m_wifi_get_connection_info(void) {
  
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_GET_CONN_INFO, NULL, 0, NULL, 0, 0);
  }

sint8 m2m_wifi_set_sytem_time(uint32 u32UTCSeconds) {
	/* 
		The firmware accepts timestamps relative to 1900 like NTP Timestamp.
	*/
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_SYS_TIME, (uint8*)&u32UTCSeconds, sizeof(tstrSystemTime), NULL, 0, 0);
  }

/*!
 * @fn             NMI_API sint8 m2m_wifi_get_sytem_time(void);   
 * @see            m2m_wifi_enable_sntp
 			  		tstrSystemTime   
 * @note         get the system time from the sntp client
 *		         using the API \ref m2m_wifi_get_sytem_time.
 * @return        The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
sint8 m2m_wifi_get_sytem_time(void) {
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_GET_SYS_TIME, NULL,0, NULL, 0, 0);
  }

sint8 m2m_wifi_enable_sntp(uint8 bEnable) {
	uint8	u8Req;

	u8Req = bEnable ? M2M_WIFI_REQ_ENABLE_SNTP_CLIENT : M2M_WIFI_REQ_DISABLE_SNTP_CLIENT;
	return hif_send(M2M_REQ_GROUP_WIFI, u8Req, NULL, 0, NULL, 0, 0);
  }

/*!
@fn			NMI_API sint8 m2m_wifi_set_power_profile(uint8 u8PwrMode);
@brief		Change the power profile mode 
@param [in]	u8PwrMode
			Change the WINC power profile to different mode 
			PWR_LOW1/PWR_LOW2/PWR_HIGH/PWR_AUTO (tenuM2mPwrMode)
@return		The function SHALL return M2M_SUCCESS for success and a negative value otherwise.
@sa			tenuM2mPwrMode
@pre		m2m_wifi_init
@warning	must be called after the initializations and before any connection request and can't be changed in run time, 
*/
sint8 m2m_wifi_set_power_profile(uint8 u8PwrMode) {
	sint8 ret = M2M_SUCCESS;
	tstrM2mPwrMode strM2mPwrMode;
  
	strM2mPwrMode.u8PwrMode = u8PwrMode;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_POWER_PROFILE, (uint8*)&strM2mPwrMode,sizeof(tstrM2mPwrMode), NULL, 0, 0);
	return ret;
  }

/*!
@fn			NMI_API sint8 m2m_wifi_set_tx_power(uint8 u8TxPwrLevel);
@brief		set the TX power tenuM2mTxPwrLevel
@param [in]	u8TxPwrLevel
			change the TX power tenuM2mTxPwrLevel
@return		The function SHALL return M2M_SUCCESS for success and a negative value otherwise.
@sa			tenuM2mTxPwrLevel
@pre		m2m_wifi_init
@warning	
*/
sint8 m2m_wifi_set_tx_power(uint8 u8TxPwrLevel) {
	sint8 ret = M2M_SUCCESS;
	tstrM2mTxPwrLevel strM2mTxPwrLevel;
  
	strM2mTxPwrLevel.u8TxPwrLevel = u8TxPwrLevel;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_TX_POWER, (uint8*)&strM2mTxPwrLevel,sizeof(tstrM2mTxPwrLevel), NULL, 0, 0);
	return ret;
  }

/*!
@fn			NMI_API sint8 m2m_wifi_enable_firmware_logs(uint8 u8Enable);
@brief		Enable or Disable logs in run time (Disable Firmware logs will 
			enhance the firmware start-up time and performance)
@param [in]	u8Enable
			Set 1 to enable the logs 0 for disable
@return		The function SHALL return M2M_SUCCESS for success and a negative value otherwise.
@sa			__DISABLE_FIRMWARE_LOGS__ (build option to disable logs from initializations)
@pre		m2m_wifi_init
@warning	
*/
sint8 m2m_wifi_enable_firmware_logs(uint8 u8Enable) {
	sint8 ret = M2M_SUCCESS;
	tstrM2mEnableLogs strM2mEnableLogs;
  
	strM2mEnableLogs.u8Enable = u8Enable;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_ENABLE_LOGS, (uint8*)&strM2mEnableLogs,sizeof(tstrM2mEnableLogs), NULL, 0, 0);
	return ret;
  }

/*!
@fn			NMI_API sint8 m2m_wifi_set_battery_voltage(uint16 u16BattVoltx100);
@brief		Enable or Disable logs in run time (Disable Firmware logs will 
			enhance the firmware start-up time and performance)
@param [in]	u16BattVoltx100
			battery voltage multiplied by 100
@return		The function SHALL return M2M_SUCCESS for success and a negative value otherwise.
@sa			__DISABLE_FIRMWARE_LOGS__ (build option to disable logs from initializations)
@pre		m2m_wifi_init
@warning	
*/
sint8 m2m_wifi_set_battery_voltage(uint16 u16BattVoltx100) {
	sint8 ret = M2M_SUCCESS;
	tstrM2mBatteryVoltage strM2mBattVol = {0};
  
	strM2mBattVol.u16BattVolt = u16BattVoltx100;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_BATTERY_VOLTAGE, (uint8*)&strM2mBattVol,sizeof(tstrM2mBatteryVoltage), NULL, 0, 0);
	return ret;
  }

/*!
@fn        	 	 sint8 m2m_wifi_prng_get_random_bytes(uint8 *pu8PrngBuff,uint16 u16PrngSize)
@brief     	 Get random bytes using the PRNG bytes.	      
@param [in]    u16PrngSize
		  	 Size of the required random bytes to be generated.   	 
@param [in]    pu8PrngBuff
		        Pointer to user allocated buffer.  		            
@return           The function SHALL return M2M_SUCCESS for success and a negative value otherwise.
*/
sint8 m2m_wifi_prng_get_random_bytes(uint8 *pu8PrngBuff,uint16 u16PrngSize) {
	sint8 ret = M2M_ERR_FAIL;
  tstrPrng   strRng = {0};
  
	if((u16PrngSize < (M2M_BUFFER_MAX_SIZE - sizeof(tstrPrng))) && (pu8PrngBuff))	{
		strRng.u16PrngSize = u16PrngSize;
		strRng.pu8RngBuff  = pu8PrngBuff;
		ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_GET_PRNG|M2M_REQ_DATA_PKT,(uint8 *)&strRng, sizeof(tstrPrng),NULL,0, 0);
    }
	else {
		M2M_ERR("PRNG Buffer exceeded maximum size %d or NULL Buffer\r\n",u16PrngSize);
    }
  
	return ret;
  }

#ifdef ETH_MODE
/*!
@fn	\
	 NMI_API sint8 m2m_wifi_enable_mac_mcast(uint8* pu8MulticastMacAddress, uint8 u8AddRemove)

@brief
	Add MAC filter to receive Multicast packets.

@param [in]	pu8MulticastMacAddress
				Pointer to the MAC address.
@param [in] u8AddRemove
				Flag to Add/Remove MAC address.
@return
	The function SHALL return 0 for success and a negative value otherwise.
*/

NMI_API sint8 m2m_wifi_enable_mac_mcast(uint8* pu8MulticastMacAddress, uint8 u8AddRemove) {
	sint8 s8ret = M2M_ERR_FAIL;
	tstrM2MMulticastMac  strMulticastMac;

	if(pu8MulticastMacAddress )	{
		strMulticastMac.u8AddRemove = u8AddRemove;
		m2m_memcpy(strMulticastMac.au8macaddress,pu8MulticastMacAddress,M2M_MAC_ADDRES_LEN);
		M2M_DBG("mac multicast: %x:%x:%x:%x:%x:%x\r\n",strMulticastMac.au8macaddress[0],strMulticastMac.au8macaddress[1],strMulticastMac.au8macaddress[2],strMulticastMac.au8macaddress[3],strMulticastMac.au8macaddress[4],strMulticastMac.au8macaddress[5]);
		s8ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_MAC_MCAST, (uint8 *)&strMulticastMac,sizeof(tstrM2MMulticastMac),NULL,0,0);
	}

	return s8ret;
  }

/*!
@fn	\
	NMI_API sint8  m2m_wifi_set_receive_buffer(void* pvBuffer,uint16 u16BufferLen);

@brief
	set the ethernet receive buffer, should be called in the receive call back.

@param [in]	pvBuffer
				Pointer to the ethernet receive buffer.
@param [in] u16BufferLen
				Length of the buffer.

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8  m2m_wifi_set_receive_buffer(void* pvBuffer,uint16 u16BufferLen) {
	sint8 s8ret = M2M_SUCCESS;
  
	if(pvBuffer) {
		gau8ethRcvBuf = pvBuffer;
		gu16ethRcvBufSize= u16BufferLen;
    }
	else {
		s8ret = M2M_ERR_FAIL;
		M2M_ERR("Buffer NULL pointer\r\n");
    }
	return s8ret;
  }
#endif /* ETH_MODE */

// nm_asic.c ************************************************************************************************************

//#include "common/include/nm_common.h"
//#include "driver/source/nmbus.h"
//#include "bsp/include/nm_bsp.h"
//#include "driver/source/nmasic.h"
//#include "driver/include/m2m_types.h"

#define NMI_GLB_RESET_0				(NMI_PERIPH_REG_BASE + 0x400)
#define NMI_INTR_REG_BASE			(NMI_PERIPH_REG_BASE + 0xa00)
#define NMI_PIN_MUX_0				(NMI_PERIPH_REG_BASE + 0x408)
#define NMI_INTR_ENABLE				(NMI_INTR_REG_BASE)
#define GET_UINT32(X,Y)				(X[0+Y] + ((uint32)X[1+Y]<<8) + ((uint32)X[2+Y]<<16) +((uint32)X[3+Y]<<24))

#define TIMEOUT						(0xfffffffful)
#define M2M_DISABLE_PS				(0xd0ul)

static uint32 clk_status_reg_adr = 0xf; /* Assume initially it is B0 chip */

sint8 chip_apply_conf(uint32 u32Conf) {
	sint8 ret = M2M_SUCCESS;
	uint32 val32 = u32Conf;
	
#ifdef __ENABLE_PMU__
	val32 |= rHAVE_USE_PMU_BIT;
#endif
#ifdef __ENABLE_SLEEP_CLK_SRC_RTC__
	val32 |= rHAVE_SLEEP_CLK_SRC_RTC_BIT;
#elif defined __ENABLE_SLEEP_CLK_SRC_XO__
	val32 |= rHAVE_SLEEP_CLK_SRC_XO_BIT;
#endif
#ifdef __ENABLE_EXT_PA_INV_TX_RX__
	val32 |= rHAVE_EXT_PA_INV_TX_RX;
#endif
#ifdef __ENABLE_LEGACY_RF_SETTINGS__
	val32 |= rHAVE_LEGACY_RF_SETTINGS;
#endif
#ifdef __DISABLE_FIRMWARE_LOGS__
	val32 |= rHAVE_LOGS_DISABLED_BIT;
#endif
	do  {
		nm_write_reg(rNMI_GP_REG_1, val32);
		if(val32 != 0) {		
			uint32 reg = 0;
			ret = nm_read_reg_with_ret(rNMI_GP_REG_1, &reg);
			if(ret == M2M_SUCCESS) {
				if(reg == val32)
					break;
        }
      } 
    else {
			break;
      }
    } while(1);

	return M2M_SUCCESS;
  }

/**
*	@fn		nm_clkless_wake
*	@brief	Wakeup the chip using clockless registers
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	Samer Sarhan
*	@date	06 June 2014
*	@version	1.0
*/
sint8 nm_clkless_wake(void) {
	sint8 ret = M2M_SUCCESS;
	uint32 reg, clk_status_reg,trials = 0;
  
	/* wait 1ms, spi data read */
	nm_bsp_sleep(1);
	ret = nm_read_reg_with_ret(0x1, &reg);
	if(ret != M2M_SUCCESS) {
		M2M_ERR("Bus error (1). Wake up failed\r\n");
		goto _WAKE_EXIT;
    }
	/*
	 * At this point, I am not sure whether it is B0 or A0
	 * If B0, then clks_enabled bit exists in register 0xf
	 * If A0, then clks_enabled bit exists in register 0xe
	 */
	do	{
		/* Set bit 1 */
		nm_write_reg(0x1, reg | (1 << 1));
		/* wait 1ms, spi data read */
		nm_bsp_sleep(1);
		// Check the clock status
		ret = nm_read_reg_with_ret(clk_status_reg_adr, &clk_status_reg);
		if( (ret != M2M_SUCCESS) || ((ret == M2M_SUCCESS) && (clk_status_reg == 0)) ) {
			/* Register 0xf did not exist in A0.
			 * If register 0xf fails to read or if it reads 0,
			 * then the chip is A0.
			 */
			clk_status_reg_adr = 0xe;
			/* wait 1ms, spi data read */
			nm_bsp_sleep(1);
			ret = nm_read_reg_with_ret(clk_status_reg_adr, &clk_status_reg);
			
			/* Aelmeleh 24-08-2015*/
			/* Check for C3000 rev. D0 value */
			if( (ret != M2M_SUCCESS) || ((ret == M2M_SUCCESS) && (clk_status_reg == 0)) ) {
				 
				clk_status_reg_adr = 0x13;
				/* wait 1ms, spi data read */
				nm_bsp_sleep(1);
				ret = nm_read_reg_with_ret(clk_status_reg_adr, &clk_status_reg);
			
				if(ret != M2M_SUCCESS) {
					M2M_ERR("Bus error (2). Wake up failed\r\n");
					goto _WAKE_EXIT;
          }
        }
      }

		// in case of clocks off, wait 2ms, and check it again.
		// if still off, wait for another 2ms, for a total wait of 6ms.
		// If still off, redo the wake up sequence
		while( ((clk_status_reg & 0x4) == 0) && (((++trials) % 3) == 0)) {
			/* Wait for the chip to stabilize*/
			nm_bsp_sleep(2);

			// Make sure chip is awake. This is an extra step that can be removed
			// later to avoid the bus access overhead
			nm_read_reg_with_ret(clk_status_reg_adr, &clk_status_reg);

			if((clk_status_reg & 0x4) == 0)	{
				M2M_ERR("clocks still OFF. Wake up failed\r\n");
        }
      }
		// in case of failure, Reset the wakeup bit to introduce a new edge on the next loop
		if((clk_status_reg & 0x4) == 0)	{
			// Reset bit 0
			nm_write_reg(0x1, reg | (1 << 1));
    	}
    } while((clk_status_reg & 0x4) == 0);

_WAKE_EXIT:
	return ret;
  }

void chip_idle(void) {
	uint32 reg =0;
  
	nm_read_reg_with_ret(0x1, &reg);
	if(reg & 0x2)	{
		reg &=~(1 << 1);
		nm_write_reg(0x1, reg);
    }
  }

void enable_rf_blocks(void) {
  
	nm_write_reg(0x6, 0xdb);
	nm_write_reg(0x7, 0x6);
	nm_bsp_sleep(10);
	nm_write_reg(0x1480, 0);
	nm_write_reg(0x1484, 0);
	nm_bsp_sleep(10);

	nm_write_reg(0x6, 0x0);
	nm_write_reg(0x7, 0x0);
  }

sint8 enable_interrupts(void) {
	uint32 reg;
	sint8 ret;
  
	/**
	interrupt pin mux select
	**/
	ret = nm_read_reg_with_ret(NMI_PIN_MUX_0, &reg);
	if(M2M_SUCCESS != ret) {
		return M2M_ERR_BUS_FAIL;
    }
	reg |= ((uint32) 1 << 8);
	ret = nm_write_reg(NMI_PIN_MUX_0, reg);
	if(M2M_SUCCESS != ret) {
		return M2M_ERR_BUS_FAIL;
    }
	/**
	interrupt enable
	**/
	ret = nm_read_reg_with_ret(NMI_INTR_ENABLE, &reg);
	if(M2M_SUCCESS != ret) {
		return M2M_ERR_BUS_FAIL;
    }
	reg |= ((uint32) 1 << 16);
	ret = nm_write_reg(NMI_INTR_ENABLE, reg);
	if (M2M_SUCCESS != ret) {
		return M2M_ERR_BUS_FAIL;
    }
	return M2M_SUCCESS;
  }

sint8 cpu_start(void) {
	uint32 reg;
	sint8 ret;

	/**
	reset regs
	*/
	nm_write_reg(BOOTROM_REG,0);
	nm_write_reg(NMI_STATE_REG,0);
	nm_write_reg(NMI_REV_REG,0);

	/**
	Go...
	**/
	ret = nm_read_reg_with_ret(0x1118, &reg);
	if (M2M_SUCCESS != ret) {
		ret = M2M_ERR_BUS_FAIL;
		M2M_ERR("[nmi start]: fail read reg 0x1118 ...\r\n");
    }
	reg |= (1 << 0);
	ret = nm_write_reg(0x1118, reg);
	ret = nm_write_reg(0x150014, 0x1);
	ret += nm_read_reg_with_ret(NMI_GLB_RESET_0, &reg);
	if((reg & (1ul << 10)) == (1ul << 10)) {
		reg &= ~(1ul << 10);
		ret += nm_write_reg(NMI_GLB_RESET_0, reg);
    }

	reg |= (1ul << 10);
	ret += nm_write_reg(NMI_GLB_RESET_0, reg);
	nm_bsp_sleep(1); /* TODO: Why bus error if this delay is not here. */
	return ret;
  }

uint32 nmi_get_chipid(void) {
	static uint32 chipid = 0;

	if(chipid == 0) {
		uint32 rfrevid;

		if((nm_read_reg_with_ret(0x1000, &chipid)) != M2M_SUCCESS) {
			chipid = 0;
			return 0;
      }
		//if((ret = nm_read_reg_with_ret(0x11fc, &revid)) != M2M_SUCCESS) {
		//	return 0;
		//}
		if((nm_read_reg_with_ret(0x13f4, &rfrevid)) != M2M_SUCCESS) {
			chipid = 0;
			return 0;
  		}

		switch(chipid) {
      case 0x1002a0:
        if(rfrevid == 0x1) { /* 1002A0 */
          }
        else /* if (rfrevid == 0x2) */ { /* 1002A1 */
          chipid = 0x1002a1;
          }
        break;
      case 0x1002b0:
        if(rfrevid == 3) { /* 1002B0 */
          } 
        else if(rfrevid == 4) { /* 1002B1 */
          chipid = 0x1002b1;
          } 
        else /* if(rfrevid == 5) */ { /* 1002B2 */
          chipid = 0x1002b2;
          }
      break;
    case 0x1000F0:
			if((nm_read_reg_with_ret(0x3B0000, &chipid)) != M2M_SUCCESS) {
        chipid = 0;
        return 0;
        }
      break;
    case 0x1003A0:     // GD 18/4/21 boh...
      break;
    default:
      break;

      }
//#define PROBE_FLASH
#ifdef PROBE_FLASH
		if(chipid) {
			UWORD32 flashid;

			flashid = probe_spi_flash();
			if(flashid == 0x1230ef) {
				chipid &= ~(0x0f0000);
				chipid |= 0x050000;
			}
			if(flashid == 0xc21320c2) {
				chipid &= ~(0x0f0000);
				chipid |= 0x050000;
			}
		}
#else
		/*M2M is by default have SPI flash*/
		chipid &= ~(0x0f0000);
		chipid |= 0x050000;
#endif /* PROBE_FLASH */
    }
  
	return chipid;
  }

uint32 nmi_get_rfrevid(void) {
  uint32 rfrevid;

  if((nm_read_reg_with_ret(0x13f4, &rfrevid)) != M2M_SUCCESS) {
    rfrevid = 0;
    return 0;
    }
  return rfrevid;
  }

void restore_pmu_settings_after_global_reset(void) {
  
	/*
	* Must restore PMU register value after
	* global reset if PMU toggle is done at
	* least once since the last hard reset.
	*/
	if(REV(nmi_get_chipid()) >= REV_2B0) {
		nm_write_reg(0x1e48, 0xb78469ce);
    }
  }

void nmi_update_pll(void) {
	uint32 pll;

	pll = nm_read_reg(0x1428);
	pll &= ~0x1ul;
	nm_write_reg(0x1428, pll);
	pll |= 0x1ul;
	nm_write_reg(0x1428, pll);
  }

void nmi_set_sys_clk_src_to_xo(void) {
	uint32 val32;

	/* Switch system clock source to XO. This will take effect after nmi_update_pll(). */
	val32 = nm_read_reg(0x141c);
	val32 |= (1 << 2);
	nm_write_reg(0x141c, val32);

	/* Do PLL update */
	nmi_update_pll();
  }

static uint8 check_3000_id(void) {
  
	return ((nmi_get_chipid() & (0xf00000)) == 0x300000);
  }

sint8 chip_wake(void) {
	sint8 ret = M2M_SUCCESS;
  
	ret  = nm_clkless_wake();
	if(ret != M2M_SUCCESS) return ret;
	if(!check_3000_id()) {
		enable_rf_blocks();
  	}
	return ret;
  }

sint8 chip_reset_and_cpu_halt(void) {
	sint8 ret = M2M_SUCCESS;
	uint32 reg = 0;

	ret = chip_wake();
	if(ret != M2M_SUCCESS) {
		return ret;
  	}
	if(!check_3000_id()) {	
		chip_reset();
		ret = nm_read_reg_with_ret(0x1118, &reg);
		if(M2M_SUCCESS != ret) {
			ret = M2M_ERR_BUS_FAIL;
			M2M_ERR("[nmi start]: fail read reg 0x1118 ...\r\n");
      }
		reg |= (1 << 0);
		ret = nm_write_reg(0x1118, reg);
		ret += nm_read_reg_with_ret(NMI_GLB_RESET_0, &reg);
		if((reg & (1ul << 10)) == (1ul << 10)) {
			reg &= ~(1ul << 10);
			ret += nm_write_reg(NMI_GLB_RESET_0, reg);
			ret += nm_read_reg_with_ret(NMI_GLB_RESET_0, &reg);
      }
		nm_write_reg(BOOTROM_REG,0);
		nm_write_reg(NMI_STATE_REG,0);
		nm_write_reg(NMI_REV_REG,0);
		nm_write_reg(NMI_PIN_MUX_0, 0x11111000);
    }
	return ret;
  }

sint8 chip_reset(void) {
	sint8 ret = M2M_SUCCESS;
  
#ifndef CONF_WINC_USE_UART
	nmi_set_sys_clk_src_to_xo();
#endif
	ret += nm_write_reg(NMI_GLB_RESET_0, 0);
	nm_bsp_sleep(50);
#ifndef CONF_WINC_USE_UART
	restore_pmu_settings_after_global_reset();
#endif
	return ret;
  }

sint8 wait_for_bootrom(uint8 arg) {
	sint8 ret = M2M_SUCCESS;
	uint32 reg = 0, cnt = 0;
	uint32 u32GpReg1 = 0;

	reg = 0;
	while(1) {
		reg = nm_read_reg(0x1014);	/* wait for efuse loading done */
		if(reg & 0x80000000) {
			break;
      }
		nm_bsp_sleep(1); /* TODO: Why bus error if this delay is not here. */
    }
	reg = nm_read_reg(M2M_WAIT_FOR_HOST_REG);
	reg &= 0x1;

	/* check if waiting for the host will be skipped or not */
	if(reg == 0) {
		reg = 0;
		while(reg != M2M_FINISH_BOOT_ROM) {
			nm_bsp_sleep(1);
			reg = nm_read_reg(BOOTROM_REG);

			if(++cnt > TIMEOUT)	{
				M2M_DBG("failed to load firmware from flash.\r\n");
				ret = M2M_ERR_INIT;
				goto ERR2;
        }
      }
    }

	switch(arg) {
    case M2M_WIFI_MODE_ATE_HIGH:
      nm_write_reg(NMI_REV_REG, M2M_ATE_FW_START_VALUE);
      nm_write_reg(NMI_STATE_REG, NBIT20);
      break;
    case M2M_WIFI_MODE_ATE_LOW:
      nm_write_reg(NMI_REV_REG, M2M_ATE_FW_START_VALUE);
      nm_write_reg(NMI_STATE_REG, 0);
      break;
    case M2M_WIFI_MODE_ETHERNET:
      u32GpReg1 = rHAVE_ETHERNET_MODE_BIT;
      break;
    default:
		/*bypass this step*/
      break;
    }

	if(REV(nmi_get_chipid()) == REV_3A0) {
		chip_apply_conf(u32GpReg1 | rHAVE_USE_PMU_BIT);
    }
	else {
		chip_apply_conf(u32GpReg1);
    }
	
	nm_write_reg(BOOTROM_REG,M2M_START_FIRMWARE);

#ifdef __ROM_TEST__
	rom_test();
#endif /* __ROM_TEST__ */

ERR2:
	return ret;
  }

sint8 wait_for_firmware_start(uint8 arg) {
	sint8 ret=M2M_SUCCESS;
	uint32 reg=0, cnt=0;
	uint32 u32Timeout=TIMEOUT;
	volatile uint32 regAddress=NMI_STATE_REG;
	volatile uint32 checkValue=M2M_FINISH_INIT_STATE;
	
	if((M2M_WIFI_MODE_ATE_HIGH == arg) || (M2M_WIFI_MODE_ATE_LOW == arg)) {
		regAddress = NMI_REV_REG;
		checkValue = M2M_ATE_FW_IS_UP_VALUE;
    } 
  else {
		/*bypass this step*/
    }
		
	while(checkValue != reg) {
		nm_bsp_sleep(2); /* TODO: Why bus error if this delay is not here. */
		M2M_DBG("%x %x %x\r\n",(unsigned int)nm_read_reg(0x108c),(unsigned int)nm_read_reg(0x108c),(unsigned int)nm_read_reg(0x14A0));
		reg = nm_read_reg(regAddress);
		if(++cnt >= u32Timeout)	{
			M2M_DBG("Time out for wait firmware Run\r\n");
			ret = M2M_ERR_INIT;
			goto ERR;
      }
    }
	if(M2M_FINISH_INIT_STATE == checkValue)	{
		nm_write_reg(NMI_STATE_REG, 0);
    }
  
ERR:
	return ret;
  }

sint8 chip_deinit(void) {
	uint32 reg = 0;
	sint8 ret;
	uint8 timeout = 10;

	/**
	stop the firmware, need a re-download
	**/
	ret = nm_read_reg_with_ret(NMI_GLB_RESET_0, &reg);
	if(ret != M2M_SUCCESS) {
		M2M_ERR("failed to de-initialize\r\n");
  	}
	reg &= ~(1 << 10);
	ret = nm_write_reg(NMI_GLB_RESET_0, reg);

	if(ret != M2M_SUCCESS) {
		M2M_ERR("Error while writing reg\r\n");
		return ret;
    }

	do {
		ret = nm_read_reg_with_ret(NMI_GLB_RESET_0, &reg);
		if(ret != M2M_SUCCESS) {
			M2M_ERR("Error while reading reg\r\n");
			return ret;
      }
		/*Workaround to ensure that the chip is actually reset*/
		if((reg & (1 << 10))) {
			M2M_DBG("Bit 10 not reset retry %d\r\n", timeout);
			reg &= ~(1 << 10);
			ret = nm_write_reg(NMI_GLB_RESET_0, reg);
			timeout--;
      } 
    else {
			break;
      }

    } while(timeout);

	return ret;
  }

sint8 set_gpio_dir(uint8 gpio, uint8 dir) {
	uint32 val32;
	sint8 ret;

	ret = nm_read_reg_with_ret(0x20108, &val32);
	if(ret != M2M_SUCCESS)
    goto _EXIT;

	if(dir)
		val32 |= (1ul << gpio);
  else
		val32 &= ~(1ul << gpio);

	ret = nm_write_reg(0x20108, val32);

_EXIT:
	return ret;
  }

sint8 set_gpio_val(uint8 gpio, uint8 val) {
	uint32 val32;
	sint8 ret;

	ret = nm_read_reg_with_ret(0x20100, &val32);
	if(ret != M2M_SUCCESS) 
    goto _EXIT;

	if(val)
		val32 |= (1ul << gpio);
  else
		val32 &= ~(1ul << gpio);

	ret = nm_write_reg(0x20100, val32);

_EXIT:
	return ret;
  }

sint8 get_gpio_val(uint8 gpio, uint8* val) {
	uint32 val32;
	sint8 ret;

	ret = nm_read_reg_with_ret(0x20104, &val32);
	if(ret != M2M_SUCCESS) goto _EXIT;

	*val = (uint8)((val32 >> gpio) & 0x01);

_EXIT:
	return ret;
  }

sint8 pullup_ctrl(uint32 pinmask, uint8 enable) {
	sint8 s8Ret;
	uint32 val32;
  
	s8Ret = nm_read_reg_with_ret(0x142c, &val32);
	if(s8Ret != M2M_SUCCESS) {
		M2M_ERR("[pullup_ctrl]: failed to read\r\n");
		goto _EXIT;
    }
	if(enable)
		val32 &= ~pinmask;
  else
		val32 |= pinmask;
	s8Ret = nm_write_reg(0x142c, val32);
	if(s8Ret  != M2M_SUCCESS) {
		M2M_ERR("[pullup_ctrl]: failed to write\r\n");
		goto _EXIT;
    }
_EXIT:
	return s8Ret;
  }

sint8 nmi_get_otp_mac_address(uint8 *pu8MacAddr,  uint8 *pu8IsValid) {
	sint8 ret;
	uint32	u32RegValue;
	uint8	mac[6];
	tstrGpRegs strgp = {0};

	ret = nm_read_reg_with_ret(rNMI_GP_REG_2, &u32RegValue);
	if(ret != M2M_SUCCESS) goto _EXIT_ERR;

	ret = nm_read_block(u32RegValue|0x30000,(uint8*)&strgp,sizeof(tstrGpRegs));
	if(ret != M2M_SUCCESS) goto _EXIT_ERR;
	u32RegValue = strgp.u32Mac_efuse_mib;

	if(!EFUSED_MAC(u32RegValue)) {
		M2M_DBG("Default MAC\r\n");
		m2m_memset(pu8MacAddr, 0, 6);
		goto _EXIT_ERR;
    }

	M2M_DBG("OTP MAC\r\n");
	u32RegValue >>= 16;
	ret = nm_read_block(u32RegValue | 0x30000, mac, 6);
	m2m_memcpy(pu8MacAddr,mac,6);
	if(pu8IsValid) *pu8IsValid = 1;
	return ret;

_EXIT_ERR:
	if(pu8IsValid) 
    *pu8IsValid = 0;

	return ret;
  }

sint8 nmi_get_mac_address(uint8 *pu8MacAddr) {
	sint8 ret;
	uint32	u32RegValue;
	uint8	mac[6];
	tstrGpRegs strgp = {0};

	ret = nm_read_reg_with_ret(rNMI_GP_REG_2, &u32RegValue);
	if(ret != M2M_SUCCESS) 
    goto _EXIT_ERR;

	ret = nm_read_block(u32RegValue|0x30000,(uint8*)&strgp,sizeof(tstrGpRegs));
	if(ret != M2M_SUCCESS) 
    goto _EXIT_ERR;
	u32RegValue = strgp.u32Mac_efuse_mib;

	u32RegValue &=0x0000ffff;
	ret = nm_read_block(u32RegValue|0x30000, mac, 6);
	m2m_memcpy(pu8MacAddr, mac, 6);

	return ret;

_EXIT_ERR:
	return ret;
  }


// nm_bus.c ******************************************************************************************************
#ifndef CORTUS_APP

//#include "nmbus.h"
//#include "nmi2c.h"
//#include "nmspi.h"
//#include "nmuart.h"

#define MAX_TRX_CFG_SZ		8

/**
*	@fn		nm_bus_iface_init
*	@brief	Initialize bus interface
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
sint8 nm_bus_iface_init(void *pvInitVal) {
	sint8 ret;
  
	ret = nm_bus_init(pvInitVal);
	return ret;
  }

/**
*	@fn		nm_bus_iface_deinit
*	@brief	Deinitialize bus interface
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	Samer Sarhan
*	@date	07 April 2014
*	@version	1.0
*/
sint8 nm_bus_iface_deinit(void) {
	sint8 ret;
  
	ret = nm_bus_deinit();

	return ret;
  }

/**
*	@fn		nm_bus_iface_reconfigure
*	@brief	reconfigure bus interface
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	Viswanathan Murugesan
*	@date	22 Oct 2014
*	@version	1.0
*/
sint8 nm_bus_iface_reconfigure(void *ptr) {
	sint8 ret = M2M_SUCCESS;
#ifdef CONF_WINC_USE_UART
	ret = nm_uart_reconfigure(ptr);
#endif
	return ret;
  }

/*
*	@fn		nm_read_reg
*	@brief	Read register
*	@param [in]	u32Addr
*				Register address
*	@return	Register value
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
uint32 nm_read_reg(uint32 u32Addr) {
  
#ifdef CONF_WINC_USE_UART
	return nm_uart_read_reg(u32Addr);
#elif defined (CONF_WINC_USE_SPI)
	return nm_spi_read_reg(u32Addr);
#elif defined (CONF_WINC_USE_I2C)
	return nm_i2c_read_reg(u32Addr);
#else
#error "Plesae define bus usage"
#endif

  }

/*
*	@fn		nm_read_reg_with_ret
*	@brief	Read register with error code return
*	@param [in]	u32Addr
*				Register address
*	@param [out]	pu32RetVal
*				Pointer to u32 variable used to return the read value
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
sint8 nm_read_reg_with_ret(uint32 u32Addr, uint32* pu32RetVal) {
  
#ifdef CONF_WINC_USE_UART
	return nm_uart_read_reg_with_ret(u32Addr,pu32RetVal);
#elif defined (CONF_WINC_USE_SPI)
	return nm_spi_read_reg_with_ret(u32Addr,pu32RetVal);
#elif defined (CONF_WINC_USE_I2C)
	return nm_i2c_read_reg_with_ret(u32Addr,pu32RetVal);
#else
#error "Plesae define bus usage"
#endif
  }

/*
*	@fn		nm_write_reg
*	@brief	write register
*	@param [in]	u32Addr
*				Register address
*	@param [in]	u32Val
*				Value to be written to the register
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
sint8 nm_write_reg(uint32 u32Addr, uint32 u32Val) {
  
#ifdef CONF_WINC_USE_UART
	return nm_uart_write_reg(u32Addr,u32Val);
#elif defined (CONF_WINC_USE_SPI)
	return nm_spi_write_reg(u32Addr,u32Val);
#elif defined (CONF_WINC_USE_I2C)
	return nm_i2c_write_reg(u32Addr,u32Val);
#else
#error "Plesae define bus usage"
#endif
  }

static sint8 p_nm_read_block(uint32 u32Addr, uint8 *puBuf, uint16 u16Sz) {
  
#ifdef CONF_WINC_USE_UART
	return nm_uart_read_block(u32Addr,puBuf,u16Sz);
#elif defined (CONF_WINC_USE_SPI)
	return nm_spi_read_block(u32Addr,puBuf,u16Sz);
#elif defined (CONF_WINC_USE_I2C)
	return nm_i2c_read_block(u32Addr,puBuf,u16Sz);
#else
#error "Plesae define bus usage"
#endif
  }

/*
*	@fn		nm_read_block
*	@brief	Read block of data
*	@param [in]	u32Addr
*				Start address
*	@param [out]	puBuf
*				Pointer to a buffer used to return the read data
*	@param [in]	u32Sz
*				Number of bytes to read. The buffer size must be >= u32Sz
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
sint8 nm_read_block(uint32 u32Addr, uint8 *puBuf, uint32 u32Sz) {
	uint16 u16MaxTrxSz = egstrNmBusCapabilities.u16MaxTrxSz - MAX_TRX_CFG_SZ;
	uint32 off = 0;
	sint8 s8Ret = M2M_SUCCESS;

	for(;;)	{
		if(u32Sz <= u16MaxTrxSz) {
			s8Ret += p_nm_read_block(u32Addr, &puBuf[off], (uint16)u32Sz);
			break;
      }
		else {
			s8Ret += p_nm_read_block(u32Addr, &puBuf[off], u16MaxTrxSz);
			if(M2M_SUCCESS != s8Ret) break;
			u32Sz -= u16MaxTrxSz;
			off += u16MaxTrxSz;
			u32Addr += u16MaxTrxSz;
      }
    }

	return s8Ret;
  }

static sint8 p_nm_write_block(uint32 u32Addr, uint8 *puBuf, uint16 u16Sz) {
  
#ifdef CONF_WINC_USE_UART
	return nm_uart_write_block(u32Addr,puBuf,u16Sz);
#elif defined (CONF_WINC_USE_SPI)
	return nm_spi_write_block(u32Addr,puBuf,u16Sz);
#elif defined (CONF_WINC_USE_I2C)
	return nm_i2c_write_block(u32Addr,puBuf,u16Sz);
#else
#error "Plesae define bus usage"
#endif
  }

/**
*	@fn		nm_write_block
*	@brief	Write block of data
*	@param [in]	u32Addr
*				Start address
*	@param [in]	puBuf
*				Pointer to the buffer holding the data to be written
*	@param [in]	u32Sz
*				Number of bytes to write. The buffer size must be >= u32Sz
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
sint8 nm_write_block(uint32 u32Addr, uint8 *puBuf, uint32 u32Sz) {
	uint16 u16MaxTrxSz = egstrNmBusCapabilities.u16MaxTrxSz - MAX_TRX_CFG_SZ;
	uint32 off = 0;
	sint8 s8Ret = M2M_SUCCESS;

	for(;;) {
		if(u32Sz <= u16MaxTrxSz) {
			s8Ret += p_nm_write_block(u32Addr, &puBuf[off], (uint16)u32Sz);
			break;
      }
		else {
			s8Ret += p_nm_write_block(u32Addr, &puBuf[off], u16MaxTrxSz);
			if(M2M_SUCCESS != s8Ret) 
        break;
			u32Sz -= u16MaxTrxSz;
			off += u16MaxTrxSz;
			u32Addr += u16MaxTrxSz;
      }
    }

	return s8Ret;
  }

#endif


// nm_drv.c *************************************************************************************************************
//#include "common/include/nm_common.h"
//#include "driver/source/nmbus.h"
//#include "bsp/include/nm_bsp.h"
//#include "driver/source/nmdrv.h"
//#include "driver/source/nmasic.h"
//#include "driver/include/m2m_types.h"
//#include "spi_flash/include/spi_flash.h"

#ifdef CONF_WINC_USE_SPI
//#include "driver/source/nmspi.h"
#endif

/**
*	@fn		nm_get_firmware_info(tstrM2mRev* M2mRev)
*	@brief	Get Firmware version info
*	@param [out]	M2mRev
*			    pointer holds address of structure "tstrM2mRev" that contains the firmware version parameters
*	@version	1.0
*/
sint8 nm_get_firmware_info(tstrM2mRev* M2mRev) {
	uint16  curr_drv_ver, min_req_drv_ver,curr_firm_ver;
	uint32	reg = 0;
	sint8	ret = M2M_SUCCESS;

	ret = nm_read_reg_with_ret(NMI_REV_REG, &reg);
	//In case the Firmware running is ATE fw
	if(M2M_ATE_FW_IS_UP_VALUE == reg)	{
		//Read FW info again from the register specified for ATE
		ret = nm_read_reg_with_ret(NMI_REV_REG_ATE, &reg);
    }
	M2mRev->u8DriverMajor	= M2M_GET_DRV_MAJOR(reg);
	M2mRev->u8DriverMinor   = M2M_GET_DRV_MINOR(reg);
	M2mRev->u8DriverPatch	= M2M_GET_DRV_PATCH(reg);
	M2mRev->u8FirmwareMajor	= M2M_GET_FW_MAJOR(reg);
	M2mRev->u8FirmwareMinor = M2M_GET_FW_MINOR(reg);
	M2mRev->u8FirmwarePatch = M2M_GET_FW_PATCH(reg);
	M2mRev->u32Chipid	= nmi_get_chipid();
	
	curr_firm_ver   = M2M_MAKE_VERSION(M2mRev->u8FirmwareMajor, M2mRev->u8FirmwareMinor,M2mRev->u8FirmwarePatch);
	curr_drv_ver    = M2M_MAKE_VERSION(M2M_DRIVER_VERSION_MAJOR_NO, M2M_DRIVER_VERSION_MINOR_NO, M2M_DRIVER_VERSION_PATCH_NO);
	min_req_drv_ver = M2M_MAKE_VERSION(M2mRev->u8DriverMajor, M2mRev->u8DriverMinor,M2mRev->u8DriverPatch);
	if(curr_drv_ver <  min_req_drv_ver) {
		/*The current driver version should be larger or equal 
		than the min driver that the current firmware support  */
		ret = M2M_ERR_FW_VER_MISMATCH;
    }
	if(curr_drv_ver >  curr_firm_ver) {
		/*The current driver should be equal or less than the firmware version*/
		ret = M2M_ERR_FW_VER_MISMATCH;
    }
  
	return ret;
  }

/**
*	@fn		nm_get_firmware_info(tstrM2mRev* M2mRev)
*	@brief	Get Firmware version info
*	@param [out]	M2mRev
*			    pointer holds address of structure "tstrM2mRev" that contains the firmware version parameters
*	@version	1.0
*/
sint8 nm_get_firmware_full_info(tstrM2mRev* pstrRev) {
	uint16  curr_drv_ver, min_req_drv_ver,curr_firm_ver;
	uint32	reg = 0;
	sint8	ret = M2M_SUCCESS;
	tstrGpRegs strgp = {0};
  
	if(pstrRev) {
		m2m_memset((uint8*)pstrRev,0,sizeof(tstrM2mRev));
		ret = nm_read_reg_with_ret(rNMI_GP_REG_2, &reg);
		if(ret == M2M_SUCCESS) {
			if(reg != 0) {
				ret = nm_read_block(reg|0x30000,(uint8*)&strgp,sizeof(tstrGpRegs));
				if(ret == M2M_SUCCESS) {
					reg = strgp.u32Firmware_Ota_rev;
					reg &= 0x0000ffff;
					if(reg != 0) {
						ret = nm_read_block(reg | 0x30000,(uint8*)pstrRev,sizeof(tstrM2mRev));
						if(ret == M2M_SUCCESS) {
							curr_firm_ver   = M2M_MAKE_VERSION(pstrRev->u8FirmwareMajor, pstrRev->u8FirmwareMinor,pstrRev->u8FirmwarePatch);
							curr_drv_ver    = M2M_MAKE_VERSION(M2M_DRIVER_VERSION_MAJOR_NO, M2M_DRIVER_VERSION_MINOR_NO, M2M_DRIVER_VERSION_PATCH_NO);
							min_req_drv_ver = M2M_MAKE_VERSION(pstrRev->u8DriverMajor, pstrRev->u8DriverMinor,pstrRev->u8DriverPatch);
							if((curr_firm_ver == 0)||(min_req_drv_ver == 0)||(min_req_drv_ver == 0)){
								ret = M2M_ERR_FAIL;
								goto EXIT;
                }
							if(curr_drv_ver <  min_req_drv_ver) {
								/*The current driver version should be larger or equal 
								than the min driver that the current firmware support  */
								ret = M2M_ERR_FW_VER_MISMATCH;
								goto EXIT;
                }
							if(curr_drv_ver >  curr_firm_ver) {
								/*The current driver should be equal or less than the firmware version*/
								ret = M2M_ERR_FW_VER_MISMATCH;
								goto EXIT;
                }
						}
					}
          else {
						ret = M2M_ERR_FAIL;
					}
				}
			}else{
				ret = M2M_ERR_FAIL;
			}
		}
	}
EXIT:
	return ret;
  }

/**
*	@fn		nm_get_ota_firmware_info(tstrM2mRev* pstrRev)
*	@brief	Get Firmware version info
*	@param [out]	M2mRev
*			    pointer holds address of structure "tstrM2mRev" that contains the firmware version parameters
			
*	@version	1.0
*/
sint8 nm_get_ota_firmware_info(tstrM2mRev* pstrRev) {
	uint16  curr_drv_ver, min_req_drv_ver,curr_firm_ver;
	uint32	reg = 0;
	sint8	ret;
	tstrGpRegs strgp = {0};

	if(pstrRev)	{
		m2m_memset((uint8*)pstrRev,0,sizeof(tstrM2mRev));
		ret = nm_read_reg_with_ret(rNMI_GP_REG_2, &reg);
		if(ret == M2M_SUCCESS) {
			if(reg != 0) {
				ret = nm_read_block(reg | 0x30000,(uint8*)&strgp,sizeof(tstrGpRegs));
				if(ret == M2M_SUCCESS) {
					reg = strgp.u32Firmware_Ota_rev;
					reg >>= 16;
					if(reg != 0) {
						ret = nm_read_block(reg|0x30000,(uint8*)pstrRev,sizeof(tstrM2mRev));
						if(ret == M2M_SUCCESS) {
							curr_firm_ver   = M2M_MAKE_VERSION(pstrRev->u8FirmwareMajor, pstrRev->u8FirmwareMinor,pstrRev->u8FirmwarePatch);
							curr_drv_ver    = M2M_MAKE_VERSION(M2M_DRIVER_VERSION_MAJOR_NO, M2M_DRIVER_VERSION_MINOR_NO, M2M_DRIVER_VERSION_PATCH_NO);
							min_req_drv_ver = M2M_MAKE_VERSION(pstrRev->u8DriverMajor, pstrRev->u8DriverMinor,pstrRev->u8DriverPatch);
							if((curr_firm_ver == 0)||(min_req_drv_ver == 0)||(min_req_drv_ver == 0)){
								ret = M2M_ERR_FAIL;
								goto EXIT;
                }
							if(curr_drv_ver <  min_req_drv_ver) {
								/*The current driver version should be larger or equal 
								than the min driver that the current firmware support  */
								ret = M2M_ERR_FW_VER_MISMATCH;
                }
							if(curr_drv_ver >  curr_firm_ver) {
								/*The current driver should be equal or less than the firmware version*/
								ret = M2M_ERR_FW_VER_MISMATCH;
                }
              }
            }
          else {
						ret = M2M_ERR_FAIL;
          	}
        	}
      	}
      else {
				ret = M2M_ERR_FAIL;
        }
      }
    } 
  else {
		ret = M2M_ERR_INVALID_ARG;
    }
EXIT:
	return ret;
  }



/*
*	@fn		nm_drv_init_download_mode
*	@brief	Initialize NMC1000 driver
*	@return	M2M_SUCCESS in case of success and Negative error code in case of failure
*   @param [in]	arg
*				Generic argument
*	@author	Viswanathan Murugesan
*	@date	10 Oct 2014
*	@version	1.0
*/
sint8 nm_drv_init_download_mode(void) {
	sint8 ret = M2M_SUCCESS;

	ret = nm_bus_iface_init(NULL);
	if(M2M_SUCCESS != ret) {
		M2M_ERR("[nmi start]: fail init bus\r\n");
		goto ERR1;
    }

	/**
		reset the chip and halt the cpu in case of no wait efuse is set.
	*/
	chip_reset_and_cpu_halt();



#ifdef CONF_WINC_USE_SPI
	/* Must do this after global reset to set SPI data packet size. */
	nm_spi_init();
#endif

	M2M_INFO("Chip ID %lx\r\n", nmi_get_chipid());

	/*disable all interrupt in ROM (to disable uart) in 2b0 chip*/
	nm_write_reg(0x20300,0);

ERR1:
	return ret;
  }

/*
*	@fn		nm_drv_init
*	@brief	Initialize NMC1000 driver
*	@return	M2M_SUCCESS in case of success and Negative error code in case of failure
*   @param [in]	arg
*				Generic argument
*	@author	M. Abdelmawla
*	@date	15 July 2012
*	@version	1.0
*/
sint8 nm_drv_init(void *arg) {
	sint8 ret = M2M_SUCCESS;
	uint8 u8Mode;
	
	if(arg) {
		u8Mode = *((uint8 *)arg);
		if((u8Mode < M2M_WIFI_MODE_NORMAL) || (u8Mode >= M2M_WIFI_MODE_MAX)) {
			u8Mode = M2M_WIFI_MODE_NORMAL;
      }
    } 
  else {
		u8Mode = M2M_WIFI_MODE_NORMAL;
    }
	
	ret = nm_bus_iface_init(NULL);
	if(M2M_SUCCESS != ret) {
		M2M_ERR("[nmi start]: fail init bus\r\n");
		goto ERR1;
    }

#ifdef BUS_ONLY
	return;
#endif
	
	
#ifdef NO_HW_CHIP_EN
	ret = chip_wake();
	nm_bsp_sleep(10);
	if (M2M_SUCCESS != ret) {
		M2M_ERR("[nmi start]: fail chip_wakeup\r\n");
		goto ERR2;
	}
	/**
	Go...
	**/
	ret = chip_reset();
	if (M2M_SUCCESS != ret) {
		goto ERR2;
	}
#endif
	M2M_INFO("Chip ID %lx\r\n", nmi_get_chipid());
#ifdef CONF_WINC_USE_SPI
	/* Must do this after global reset to set SPI data packet size. */
	nm_spi_init();
#endif
#ifdef NO_HW_CHIP_EN
	/*return power save to default value*/
	chip_idle();

	ret = cpu_start();
	if (M2M_SUCCESS != ret) {
		goto ERR2;
	}
#endif
	ret = wait_for_bootrom(u8Mode);
	if(M2M_SUCCESS != ret) {
		goto ERR2;
    }
		
	ret = wait_for_firmware_start(u8Mode);
	if(M2M_SUCCESS != ret) {
		goto ERR2;
    }
	
	if((M2M_WIFI_MODE_ATE_HIGH == u8Mode) || (M2M_WIFI_MODE_ATE_LOW == u8Mode)) {
		goto ERR1;
    } 
  else {
		/*continue running*/
    }
	
	ret = enable_interrupts();
	if(M2M_SUCCESS != ret) {
		M2M_ERR("failed to enable interrupts..\r\n");
		goto ERR2;
    }
	
	return ret;
  
ERR2:
	nm_bus_iface_deinit();
ERR1:
	return ret;
  }

/*
*	@fn		nm_drv_deinit
*	@brief	Deinitialize NMC1000 driver
*	@author	M. Abdelmawla
*	@date	17 July 2012
*	@version	1.0
*/
sint8 nm_drv_deinit(void *arg) {
	sint8 ret;

	ret = chip_deinit();
	if(M2M_SUCCESS != ret) {
		M2M_ERR("[nmi stop]: chip_deinit fail\r\n");
		goto ERR1;
    }
	
	/* Disable SPI flash to save power when the chip is off */
	ret = spi_flash_enable(0);
	if(M2M_SUCCESS != ret) {
		M2M_ERR("[nmi stop]: SPI flash disable fail\r\n");
		goto ERR1;
    }

	ret = nm_bus_iface_deinit();
	if(M2M_SUCCESS != ret) {
		M2M_ERR("[nmi stop]: fail init bus\r\n");
		goto ERR1;
    }
  
#ifdef CONF_WINC_USE_SPI
	/* Must do this after global reset to set SPI data packet size. */
	nm_spi_deinit();
#endif

ERR1:
	return ret;
  }


// nm_i2c.c *********************************************************************************************************

//#include "common/include/nm_common.h"

#ifdef CONF_WINC_USE_I2C

#include "nmi2c.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"


/*
*	@fn		nm_i2c_read_reg_with_ret
*	@brief	Read register with error code return
*	@param [in]	u32Addr
*				Register address
*	@param [out]	pu32RetVal
*				Pointer to u32 variable used to return the read value
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
sint8 nm_i2c_read_reg_with_ret(uint32 u32Addr, uint32* pu32RetVal) {
	uint8 b[6];
	uint8 rsz;
	tstrNmI2cDefault strI2c;
	sint8 s8Ret = M2M_SUCCESS;

	if(u32Addr < 0xff) { /* clockless i2c */
		b[0] = 0x09;
		b[1] = (uint8)(u32Addr);
		rsz = 1;
		strI2c.u16Sz = 2;
    } 
  else {
		b[0] = 0x80;
		b[1] = (uint8)(u32Addr >> 24);
		b[2] = (uint8)(u32Addr >> 16);
		b[3] = (uint8)(u32Addr >> 8);
		b[4] = (uint8)(u32Addr);
		b[5] = 0x04;
		rsz = 4;
		strI2c.u16Sz = 6;
    }

	strI2c.pu8Buf = b;

	if(M2M_SUCCESS == nm_bus_ioctl(NM_BUS_IOCTL_W, &strI2c))	{
		strI2c.u16Sz = rsz;
		if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_R, &strI2c)) {
			//M2M_ERR("read error\r\n");
			s8Ret = M2M_ERR_BUS_FAIL;
      }
    }
	else	{
		M2M_ERR("failed to send cfg bytes\r\n");
		s8Ret = M2M_ERR_BUS_FAIL;
    }

	if (rsz == 1) {
		*pu32RetVal = b[0];
    }
  else {
		*pu32RetVal = b[0] | ((uint32)b[1] << 8) | ((uint32)b[2] << 16) | ((uint32)b[3] << 24);
    }
	return s8Ret;
  }

/*
*	@fn		nm_i2c_read_reg
*	@brief	Read register
*	@param [in]	u32Addr
*				Register address
*	@return	Register value
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
uint32 nm_i2c_read_reg(uint32 u32Addr) {
	uint32 val;
  
	nm_i2c_read_reg_with_ret(u32Addr, &val);
	return val;
  }

/*
*	@fn		nm_i2c_write_reg
*	@brief	write register
*	@param [in]	u32Addr
*				Register address
*	@param [in]	u32Val
*				Value to be written to the register
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
sint8 nm_i2c_write_reg(uint32 u32Addr, uint32 u32Val) {
	tstrNmI2cDefault strI2c;
	uint8 b[16];
	sint8 s8Ret = M2M_SUCCESS;

	if(u32Addr < 0xff) { /* clockless i2c */
		b[0] = 0x19;
		b[1] = (uint8)(u32Addr);
		b[2] = (uint8)(u32Val);
		strI2c.u16Sz = 3;
    } 
  else {
		b[0] = 0x90;
		b[1] = (uint8)(u32Addr >> 24);
		b[2] = (uint8)(u32Addr >> 16);
		b[3] = (uint8)(u32Addr >> 8);
		b[4] = (uint8)u32Addr;
		b[5] = 0x04;
		b[6] = (uint8)u32Val;
		b[7] = (uint8)(u32Val >> 8);
		b[8] = (uint8)(u32Val >> 16);
		b[9] = (uint8)(u32Val >> 24);
		strI2c.u16Sz = 10;
    }

	strI2c.pu8Buf = b;

	if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_W, &strI2c))	{
		M2M_ERR("write error\r\n");
		s8Ret = M2M_ERR_BUS_FAIL;
	}

	return s8Ret;
  }

/*
*	@fn		nm_i2c_read_block
*	@brief	Read block of data
*	@param [in]	u32Addr
*				Start address
*	@param [out]	puBuf
*				Pointer to a buffer used to return the read data
*	@param [in]	u16Sz
*				Number of bytes to read. The buffer size must be >= u16Sz
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
sint8 nm_i2c_read_block(uint32 u32Addr, uint8 *pu8Buf, uint16 u16Sz) {
	tstrNmI2cDefault strI2c;
	uint8 au8Buf[7];
	sint8 s8Ret = M2M_SUCCESS;

	au8Buf[0] = 0x02;
	au8Buf[1] = (uint8)(u32Addr >> 24);
	au8Buf[2] = (uint8)(u32Addr >> 16);
	au8Buf[3] = (uint8)(u32Addr >> 8);
	au8Buf[4] = (uint8)(u32Addr >> 0);
	au8Buf[5] = (uint8)(u16Sz >> 8);
	au8Buf[6] = (uint8)(u16Sz);

	strI2c.pu8Buf = au8Buf;
	strI2c.u16Sz = sizeof(au8Buf);

	if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_W, &strI2c))	{
		M2M_ERR("write error\r\n");
		s8Ret = M2M_ERR_BUS_FAIL;
	}
	else	{
		strI2c.pu8Buf = pu8Buf;
		strI2c.u16Sz = u16Sz;

		if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_R, &strI2c))	{
			M2M_ERR("read error\r\n");
			s8Ret = M2M_ERR_BUS_FAIL;
		}
	}

	return s8Ret;
}

/*
*	@fn		nm_i2c_write_block
*	@brief	Write block of data
*	@param [in]	u32Addr
*				Start address
*	@param [in]	puBuf
*				Pointer to the buffer holding the data to be written
*	@param [in]	u16Sz
*				Number of bytes to write. The buffer size must be >= u16Sz
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
sint8 nm_i2c_write_block(uint32 u32Addr, uint8 *pu8Buf, uint16 u16Sz) {
	uint8 au8Buf[7];
	tstrNmI2cSpecial strI2c;
	sint8 s8Ret = M2M_SUCCESS;

	au8Buf[0] = 0x12;
	au8Buf[1] = (uint8)(u32Addr >> 24);
	au8Buf[2] = (uint8)(u32Addr >> 16);
	au8Buf[3] = (uint8)(u32Addr >> 8);
	au8Buf[4] = (uint8)(u32Addr);
	au8Buf[5] = (uint8)(u16Sz >> 8);
	au8Buf[6] = (uint8)(u16Sz);

	strI2c.pu8Buf1 = au8Buf;
	strI2c.pu8Buf2 = pu8Buf;
	strI2c.u16Sz1 = sizeof(au8Buf);
	strI2c.u16Sz2 = u16Sz;

	if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_W_SPECIAL, &strI2c))
	{
		M2M_ERR("write error\r\n");
		s8Ret = M2M_ERR_BUS_FAIL;
	}

	return s8Ret;
}

#endif


// nm_spi.c **************************************************************************************************************

//#include "common/include/nm_common.h"

#ifdef CONF_WINC_USE_SPI

#define USE_OLD_SPI_SW

//#include "bus_wrapper/include/nm_bus_wrapper.h"
//#include "nmspi.h"

#define NMI_PERIPH_REG_BASE 0x1000
#define NMI_CHIPID	(NMI_PERIPH_REG_BASE)

#define NMI_SPI_REG_BASE 0xe800
#define NMI_SPI_CTL (NMI_SPI_REG_BASE)
#define NMI_SPI_MASTER_DMA_ADDR (NMI_SPI_REG_BASE+0x4)
#define NMI_SPI_MASTER_DMA_COUNT (NMI_SPI_REG_BASE+0x8)
#define NMI_SPI_SLAVE_DMA_ADDR (NMI_SPI_REG_BASE+0xc)
#define NMI_SPI_SLAVE_DMA_COUNT (NMI_SPI_REG_BASE+0x10)
#define NMI_SPI_TX_MODE (NMI_SPI_REG_BASE+0x20)
#define NMI_SPI_PROTOCOL_CONFIG (NMI_SPI_REG_BASE+0x24)
#define NMI_SPI_INTR_CTL (NMI_SPI_REG_BASE+0x2c)

#define NMI_SPI_PROTOCOL_OFFSET (NMI_SPI_PROTOCOL_CONFIG-NMI_SPI_REG_BASE)

#define SPI_BASE                NMI_SPI_REG_BASE

#define CMD_DMA_WRITE			0xc1
#define CMD_DMA_READ			0xc2
#define CMD_INTERNAL_WRITE		0xc3
#define CMD_INTERNAL_READ		0xc4
#define CMD_TERMINATE			0xc5
#define CMD_REPEAT				0xc6
#define CMD_DMA_EXT_WRITE		0xc7
#define CMD_DMA_EXT_READ		0xc8
#define CMD_SINGLE_WRITE		0xc9
#define CMD_SINGLE_READ			0xca
#define CMD_RESET				0xcf

#define N_OK					 1
#define N_FAIL					 0
#define N_RESET					-1
#define N_RETRY					-2

#define DATA_PKT_SZ_256 		256
#define DATA_PKT_SZ_512			512
#define DATA_PKT_SZ_1K			1024
#define DATA_PKT_SZ_4K			(4 * 1024)
#define DATA_PKT_SZ_8K			(8 * 1024)
#define DATA_PKT_SZ				DATA_PKT_SZ_8K

static uint8 	gu8Crc_off	=   0;

static sint8 nmi_spi_read(uint8* b, uint16 sz) {
	tstrNmSpiRw spi;
  
	spi.pu8InBuf = NULL;
	spi.pu8OutBuf = b;
	spi.u16Sz = sz;
	return nm_bus_ioctl(NM_BUS_IOCTL_RW, &spi);
  }

static sint8 nmi_spi_write(uint8* b, uint16 sz) {
	tstrNmSpiRw spi;
  
	spi.pu8InBuf = b;
	spi.pu8OutBuf = NULL;
	spi.u16Sz = sz;
	return nm_bus_ioctl(NM_BUS_IOCTL_RW, &spi);
  }

/********************************************

	Crc7

********************************************/

static const uint8 crc7_syndrome_table[256] = {
	0x00, 0x09, 0x12, 0x1b, 0x24, 0x2d, 0x36, 0x3f,
	0x48, 0x41, 0x5a, 0x53, 0x6c, 0x65, 0x7e, 0x77,
	0x19, 0x10, 0x0b, 0x02, 0x3d, 0x34, 0x2f, 0x26,
	0x51, 0x58, 0x43, 0x4a, 0x75, 0x7c, 0x67, 0x6e,
	0x32, 0x3b, 0x20, 0x29, 0x16, 0x1f, 0x04, 0x0d,
	0x7a, 0x73, 0x68, 0x61, 0x5e, 0x57, 0x4c, 0x45,
	0x2b, 0x22, 0x39, 0x30, 0x0f, 0x06, 0x1d, 0x14,
	0x63, 0x6a, 0x71, 0x78, 0x47, 0x4e, 0x55, 0x5c,
	0x64, 0x6d, 0x76, 0x7f, 0x40, 0x49, 0x52, 0x5b,
	0x2c, 0x25, 0x3e, 0x37, 0x08, 0x01, 0x1a, 0x13,
	0x7d, 0x74, 0x6f, 0x66, 0x59, 0x50, 0x4b, 0x42,
	0x35, 0x3c, 0x27, 0x2e, 0x11, 0x18, 0x03, 0x0a,
	0x56, 0x5f, 0x44, 0x4d, 0x72, 0x7b, 0x60, 0x69,
	0x1e, 0x17, 0x0c, 0x05, 0x3a, 0x33, 0x28, 0x21,
	0x4f, 0x46, 0x5d, 0x54, 0x6b, 0x62, 0x79, 0x70,
	0x07, 0x0e, 0x15, 0x1c, 0x23, 0x2a, 0x31, 0x38,
	0x41, 0x48, 0x53, 0x5a, 0x65, 0x6c, 0x77, 0x7e,
	0x09, 0x00, 0x1b, 0x12, 0x2d, 0x24, 0x3f, 0x36,
	0x58, 0x51, 0x4a, 0x43, 0x7c, 0x75, 0x6e, 0x67,
	0x10, 0x19, 0x02, 0x0b, 0x34, 0x3d, 0x26, 0x2f,
	0x73, 0x7a, 0x61, 0x68, 0x57, 0x5e, 0x45, 0x4c,
	0x3b, 0x32, 0x29, 0x20, 0x1f, 0x16, 0x0d, 0x04,
	0x6a, 0x63, 0x78, 0x71, 0x4e, 0x47, 0x5c, 0x55,
	0x22, 0x2b, 0x30, 0x39, 0x06, 0x0f, 0x14, 0x1d,
	0x25, 0x2c, 0x37, 0x3e, 0x01, 0x08, 0x13, 0x1a,
	0x6d, 0x64, 0x7f, 0x76, 0x49, 0x40, 0x5b, 0x52,
	0x3c, 0x35, 0x2e, 0x27, 0x18, 0x11, 0x0a, 0x03,
	0x74, 0x7d, 0x66, 0x6f, 0x50, 0x59, 0x42, 0x4b,
	0x17, 0x1e, 0x05, 0x0c, 0x33, 0x3a, 0x21, 0x28,
	0x5f, 0x56, 0x4d, 0x44, 0x7b, 0x72, 0x69, 0x60,
	0x0e, 0x07, 0x1c, 0x15, 0x2a, 0x23, 0x38, 0x31,
	0x46, 0x4f, 0x54, 0x5d, 0x62, 0x6b, 0x70, 0x79
  };


static uint8 crc7_byte(uint8 crc, uint8 data) {
  
	return crc7_syndrome_table[(crc << 1) ^ data];
  }

static uint8 crc7(uint8 crc, const uint8 *buffer, uint32 len) {
  
	while(len--)
		crc = crc7_byte(crc, *buffer++);
	return crc;
  }

/********************************************

	Spi protocol Function

********************************************/

static sint8 spi_cmd(uint8 cmd, uint32 adr, uint32 u32data, uint32 sz,uint8 clockless) {
	uint8 bc[9];
	uint8 len = 5;
	sint8 result = N_OK;

	bc[0] = cmd;
	switch(cmd) {
    case CMD_SINGLE_READ:				/* single word (4 bytes) read */
      bc[1] = (uint8)(adr >> 16);
      bc[2] = (uint8)(adr >> 8);
      bc[3] = (uint8)adr;
      len = 5;
      break;
    case CMD_INTERNAL_READ:			/* internal register read */
      bc[1] = (uint8)(adr >> 8);
      if(clockless)  bc[1] |= (1 << 7);
      bc[2] = (uint8)adr;
      bc[3] = 0x00;
      len = 5;
      break;
    case CMD_TERMINATE:					/* termination */
      bc[1] = 0x00;
      bc[2] = 0x00;
      bc[3] = 0x00;
      len = 5;
      break;
    case CMD_REPEAT:						/* repeat */
      bc[1] = 0x00;
      bc[2] = 0x00;
      bc[3] = 0x00;
      len = 5;
      break;
    case CMD_RESET:							/* reset */
      bc[1] = 0xff;
      bc[2] = 0xff;
      bc[3] = 0xff;
      len = 5;
      break;
    case CMD_DMA_WRITE:					/* dma write */
    case CMD_DMA_READ:					/* dma read */
      bc[1] = (uint8)(adr >> 16);
      bc[2] = (uint8)(adr >> 8);
      bc[3] = (uint8)adr;
      bc[4] = (uint8)(sz >> 8);
      bc[5] = (uint8)(sz);
      len = 7;
      break;
    case CMD_DMA_EXT_WRITE:		/* dma extended write */
    case CMD_DMA_EXT_READ:			/* dma extended read */
      bc[1] = (uint8)(adr >> 16);
      bc[2] = (uint8)(adr >> 8);
      bc[3] = (uint8)adr;
      bc[4] = (uint8)(sz >> 16);
      bc[5] = (uint8)(sz >> 8);
      bc[6] = (uint8)(sz);
      len = 8;
      break;
    case CMD_INTERNAL_WRITE:		/* internal register write */
      bc[1] = (uint8)(adr >> 8);
      if(clockless)  bc[1] |= (1 << 7);
      bc[2] = (uint8)(adr);
      bc[3] = (uint8)(u32data >> 24);
      bc[4] = (uint8)(u32data >> 16);
      bc[5] = (uint8)(u32data >> 8);
      bc[6] = (uint8)(u32data);
      len = 8;
      break;
    case CMD_SINGLE_WRITE:			/* single word write */
      bc[1] = (uint8)(adr >> 16);
      bc[2] = (uint8)(adr >> 8);
      bc[3] = (uint8)(adr);
      bc[4] = (uint8)(u32data >> 24);
      bc[5] = (uint8)(u32data >> 16);
      bc[6] = (uint8)(u32data >> 8);
      bc[7] = (uint8)(u32data);
      len = 9;
      break;
    default:
      result = N_FAIL;
      break;
    }

	if(result) {
		if(!gu8Crc_off)
			bc[len-1] = (crc7(0x7f, (const uint8 *)&bc[0], len-1)) << 1;
		else
			len-=1;

		if(M2M_SUCCESS != nmi_spi_write(bc, len)) {
			M2M_ERR("[nmi spi]: Failed cmd write, bus error...\r\n");
			result = N_FAIL;
      }
    }

	return result;
  }

static sint8 spi_cmd_rsp(uint8 cmd) {
	uint8 rsp;
	sint8 result = N_OK;
	sint8 s8RetryCnt;

	/**
		Command/Control response
	**/
	if((cmd == CMD_RESET) || (cmd == CMD_TERMINATE) || (cmd == CMD_REPEAT)) {
		if(M2M_SUCCESS != nmi_spi_read(&rsp, 1)) {
			result = N_FAIL;
			goto _fail_;
      }
    }

	/* wait for response */
	s8RetryCnt = 10;
	do {
		if(M2M_SUCCESS != nmi_spi_read(&rsp, 1)) {
			M2M_ERR("[nmi spi]: Failed cmd response read, bus error...\r\n");
			result = N_FAIL;
			goto _fail_;
      }
    } while((rsp != cmd) && (s8RetryCnt-- >0));

	/**
		State response
	**/
	/* wait for response */
	s8RetryCnt = 10;
	do {
		if(M2M_SUCCESS != nmi_spi_read(&rsp, 1)) {
			M2M_ERR("[nmi spi]: Failed cmd response read, bus error...\r\n");
			result = N_FAIL;
			goto _fail_;
      }
    } while((rsp != 0x00) && (s8RetryCnt-- >0));

_fail_:

	return result;
  }

static sint8 spi_data_read(uint8 *b, uint16 sz,uint8 clockless) {
	sint16 retry, ix, nbytes;
	sint8 result = N_OK;
	uint8 crc[2];
	uint8 rsp;

	/**
		Data
	**/
	ix = 0;
	do {
		if(sz <= DATA_PKT_SZ)
			nbytes = sz;
		else
			nbytes = DATA_PKT_SZ;

		/**
			Data Response header
		**/
		retry = 10;
		do {
			if(M2M_SUCCESS != nmi_spi_read(&rsp, 1)) {
				M2M_ERR("[nmi spi]: Failed data response read, bus error...\r\n");
				result = N_FAIL;
				break;
        }
//			if(((rsp >> 4) & 0xf) == 0b1100 /*0xf no, pag. 90 no no è ok*/)
      if((rsp & 0xf0) == 0xf0)
        break;
      } while(retry--);

		if(result == N_FAIL)
			break;

		if(retry <= 0) {
			M2M_ERR("[nmi spi]: Failed data response read...(%02x)\r\n", rsp);
			result = N_FAIL;
			break;
      }

		/**
			Read bytes
		**/
		if(M2M_SUCCESS != nmi_spi_read(&b[ix], nbytes)) {
			M2M_ERR("[nmi spi]: Failed data block read, bus error...\r\n");
			result = N_FAIL;
			break;
      }
		if(!clockless) {
			/**
			Read Crc
			**/
			if(!gu8Crc_off) {
				if(M2M_SUCCESS != nmi_spi_read(crc, 2)) {
					M2M_ERR("[nmi spi]: Failed data block crc read, bus error...\r\n");
					result = N_FAIL;
					break;
          }
        }
      }
		ix += nbytes;
		sz -= nbytes;
    } while(sz);

	return result;
  }

static sint8 spi_data_write(uint8 *b, uint16 sz) {
	sint16 ix;
	uint16 nbytes;
	sint8 result = 1;
	uint8 cmd, order, crc[2] = {0};
	//uint8 rsp;

	/**
		Data
	**/
	ix = 0;
	do {
		if (sz <= DATA_PKT_SZ)
			nbytes = sz;
		else
			nbytes = DATA_PKT_SZ;

		/**
			Write command
		**/
		cmd = 0xf0;
		if(ix == 0)  {
			if(sz <= DATA_PKT_SZ)
				order = 0x3;
			else
				order = 0x1;
      } 
    else {
			if(sz <= DATA_PKT_SZ)
				order = 0x3;
			else
				order = 0x2;
      }
		cmd |= order;
		if(M2M_SUCCESS != nmi_spi_write(&cmd, 1)) {
			M2M_ERR("[nmi spi]: Failed data block cmd write, bus error...\r\n");
			result = N_FAIL;
			break;
		}

		/**
			Write data
		**/
		if (M2M_SUCCESS != nmi_spi_write(&b[ix], nbytes)) {
			M2M_ERR("[nmi spi]: Failed data block write, bus error...\r\n");
			result = N_FAIL;
			break;
      }

		/**
			Write Crc
		**/
		if (!gu8Crc_off) {
			if (M2M_SUCCESS != nmi_spi_write(crc, 2)) {
				M2M_ERR("[nmi spi]: Failed data block crc write, bus error...\r\n");
				result = N_FAIL;
				break;
        }
      }

		ix += nbytes;
		sz -= nbytes;
    } while (sz);


	return result;
  }

/********************************************

	Spi Internal Read/Write Function

********************************************/

/********************************************

	Spi interfaces

********************************************/

static sint8 spi_write_reg(uint32 addr, uint32 u32data) {
	sint8 result = N_OK;
	uint8 cmd = CMD_SINGLE_WRITE;
	uint8 clockless = 0;
  
	if (addr <= 0x30)	{
		/**
		NMC1000 clockless registers.
		**/
		cmd = CMD_INTERNAL_WRITE;
		clockless = 1;
    }
	else	{
		cmd = CMD_SINGLE_WRITE;
		clockless = 0;
    }

#if defined USE_OLD_SPI_SW
	result = spi_cmd(cmd, addr, u32data, 4, clockless);
	if(result != N_OK) {
		M2M_ERR("[nmi spi]: Failed cmd, write reg (%08x)...\r\n", (unsigned int)addr);
		return N_FAIL;
    }

	result = spi_cmd_rsp(cmd);
	if(result != N_OK) {
		M2M_ERR("[nmi spi]: Failed cmd response, write reg (%08x)...\r\n", (unsigned int)addr);
		spi_cmd(CMD_RESET, 0, 0, 0, 0);
		return N_FAIL;
    }

	return N_OK;
#else

	result = spi_cmd_complete(cmd, addr, (uint8*)&u32data, 4, clockless);
	if (result != N_OK) {
		M2M_ERR( "[nmi spi]: Failed cmd, write reg (%08x)...\r\n", addr);
	}

	return result;

#endif
  }

static sint8 nm_spi_write(uint32 addr, uint8 *buf, uint16 size) {
	sint8 result;
	uint8 cmd = CMD_DMA_EXT_WRITE;


	/**
		Command
	**/
#if defined USE_OLD_SPI_SW
	result = spi_cmd(cmd, addr, 0, size,0);
	if (result != N_OK) {
		M2M_ERR("[nmi spi]: Failed cmd, write block (%08x)...\r\n", (unsigned int)addr);
		return N_FAIL;
    }

	result = spi_cmd_rsp(cmd);
	if (result != N_OK) {
		M2M_ERR("[nmi spi ]: Failed cmd response, write block (%08x)...\r\n", (unsigned int)addr);
		spi_cmd(CMD_RESET, 0, 0, 0, 0);
		return N_FAIL;
    }
#else
	result = spi_cmd_complete(cmd, addr, NULL, size, 0);
	if (result != N_OK) {
		M2M_ERR( "[nmi spi]: Failed cmd, write block (%08x)...\r\n", addr);
		return N_FAIL;
	}
#endif

	/**
		Data
	**/
	result = spi_data_write(buf, size);
	if (result != N_OK) {
		M2M_ERR("[nmi spi]: Failed block data write...\r\n");
		spi_cmd(CMD_RESET, 0, 0, 0, 0);
    }

	return N_OK;
  }

static sint8 spi_read_reg(uint32 addr, uint32 *u32data) {
	sint8 result = N_OK;
	uint8 cmd = CMD_SINGLE_READ;
	uint8 tmp[4];
	uint8 clockless = 0;

	if(addr <= 0xff)	{
		/**
		NMC1000 clockless registers.
		**/
		cmd = CMD_INTERNAL_READ;
		clockless = 1;
    }
	else {
		cmd = CMD_SINGLE_READ;
		clockless = 0;
    }

#if defined USE_OLD_SPI_SW
	result = spi_cmd(cmd, addr, 0, 4, clockless);
	if(result != N_OK) {
		M2M_ERR("[nmi spi]: Failed cmd, read reg (%08x)...\r\n", (unsigned int)addr);
		return N_FAIL;
    }

	result = spi_cmd_rsp(cmd);
	if(result != N_OK) {
		M2M_ERR("[nmi spi]: Failed cmd response, read reg (%08x)...\r\n", (unsigned int)addr);
		spi_cmd(CMD_RESET, 0, 0, 0, 0);
		return N_FAIL;
    }

	/* to avoid endianess issues */
	result = spi_data_read(&tmp[0], 4, clockless);
	if(result != N_OK) {
		M2M_ERR("[nmi spi]: Failed data read...\r\n");
		spi_cmd(CMD_RESET, 0, 0, 0, 0);
		return N_FAIL;
	}
#else
	result = spi_cmd_complete(cmd, addr, (uint8*)&tmp[0], 4, clockless);
	if (result != N_OK) {
		M2M_ERR( "[nmi spi]: Failed cmd, read reg (%08x)...\r\n", addr);
		return N_FAIL;
	}

#endif

	*u32data = tmp[0] |
		((uint32)tmp[1] << 8) |
		((uint32)tmp[2] << 16) |
		((uint32)tmp[3] << 24);

	return N_OK;
  }

static sint8 nm_spi_read(uint32 addr, uint8 *buf, uint16 size) {
	uint8 cmd = CMD_DMA_EXT_READ;
	sint8 result;

	/**
		Command
	**/
#if defined USE_OLD_SPI_SW
	result = spi_cmd(cmd, addr, 0, size,0);
	if (result != N_OK) {
		M2M_ERR("[nmi spi]: Failed cmd, read block (%08x)...\r\n", (unsigned int)addr);
		return N_FAIL;
    }

	result = spi_cmd_rsp(cmd);
	if (result != N_OK) {
		M2M_ERR("[nmi spi]: Failed cmd response, read block (%08x)...\r\n", (unsigned int)addr);
		spi_cmd(CMD_RESET, 0, 0, 0, 0);
		return N_FAIL;
    }

	/**
		Data
	**/
	result = spi_data_read(buf, size,0);
	if (result != N_OK) {
		M2M_ERR("[nmi spi]: Failed block data read...\r\n");
		spi_cmd(CMD_RESET, 0, 0, 0, 0);
		return N_FAIL;
    }
#else
		result = spi_cmd_complete(cmd, addr, buf, size, 0);
		if (result != N_OK) {
			M2M_ERR("[nmi spi]: Failed cmd, read block (%08x)...\r\n", addr);
			return N_FAIL;
		}
#endif

	return N_OK;
  }

/********************************************

	Bus interfaces

********************************************/

static void spi_init_pkt_sz(void) {
	uint32 val32;

	/* Make sure SPI max. packet size fits the defined DATA_PKT_SZ.  */
	val32 = nm_spi_read_reg(SPI_BASE+0x24);
	val32 &= ~(0x7 << 4);
	switch(DATA_PKT_SZ)	{
    case 256:  val32 |= (0 << 4); break;
    case 512:  val32 |= (1 << 4); break;
    case 1024: val32 |= (2 << 4); break;
    case 2048: val32 |= (3 << 4); break;
    case 4096: val32 |= (4 << 4); break;
    case 8192: val32 |= (5 << 4); break;

    }
	nm_spi_write_reg(SPI_BASE+0x24, val32);
  }

/*
*	@fn		nm_spi_init
*	@brief	Initialize the SPI
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
sint8 nm_spi_init(void) {
	uint32 chipid;
	uint32 reg =0;

	/**
		configure protocol
	**/
	gu8Crc_off = 0;

	// TODO: We can remove the CRC trials if there is a definite way to reset
	// the SPI to it's initial value.
	if(!spi_read_reg(NMI_SPI_PROTOCOL_CONFIG, &reg)) {
		/* Read failed. Try with CRC off. This might happen when module
		is removed but chip isn't reset*/
		gu8Crc_off = 1;
		M2M_ERR("[nmi spi]: Failed internal read protocol with CRC on, retrying with CRC off...\r\n");
		if(!spi_read_reg(NMI_SPI_PROTOCOL_CONFIG, &reg)){
			// Reaad failed with both CRC on and off, something went bad
			M2M_ERR( "[nmi spi]: Failed internal read protocol...\r\n");
			return 0;
    	}
    }
	if(gu8Crc_off == 0)	{
		reg &= ~0xc;	/* disable crc checking */
		reg &= ~0x70;
		reg |= (0x5 << 4);
		if (!spi_write_reg(NMI_SPI_PROTOCOL_CONFIG, reg)) {
			M2M_ERR( "[nmi spi]: Failed internal write protocol reg...\r\n");
			return 0;
      }
		gu8Crc_off = 1;
    }

	/**
		make sure can read back chip id correctly
	**/
	if(!spi_read_reg(0x1000, &chipid)) {
		M2M_ERR("[nmi spi]: Fail cmd read chip id...\r\n");
		return M2M_ERR_BUS_FAIL;
    }

	M2M_DBG("[nmi spi]: chipid (%08x)\r\n", (unsigned int)chipid);    // 001003A0 18/4/2021
	spi_init_pkt_sz();

	return M2M_SUCCESS;
  }

/*
*	@fn		nm_spi_init
*	@brief	DeInitialize the SPI
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	Samer Sarhan
*	@date	27 Feb 2015
*	@version	1.0
*/
sint8 nm_spi_deinit(void) {
  
	gu8Crc_off = 0;
	return M2M_SUCCESS;
  }

/*
*	@fn		nm_spi_read_reg
*	@brief	Read register
*	@param [in]	u32Addr
*				Register address
*	@return	Register value
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
uint32 nm_spi_read_reg(uint32 u32Addr) {
	uint32 u32Val;

	spi_read_reg(u32Addr, &u32Val);

	return u32Val;
  }

/*
*	@fn		nm_spi_read_reg_with_ret
*	@brief	Read register with error code return
*	@param [in]	u32Addr
*				Register address
*	@param [out]	pu32RetVal
*				Pointer to u32 variable used to return the read value
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
sint8 nm_spi_read_reg_with_ret(uint32 u32Addr, uint32* pu32RetVal) {
	sint8 s8Ret;

	s8Ret = spi_read_reg(u32Addr,pu32RetVal);

	if(N_OK == s8Ret) 
    s8Ret = M2M_SUCCESS;
	else 
    s8Ret = M2M_ERR_BUS_FAIL;

	return s8Ret;
  }

/*
*	@fn		nm_spi_write_reg
*	@brief	write register
*	@param [in]	u32Addr
*				Register address
*	@param [in]	u32Val
*				Value to be written to the register
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
sint8 nm_spi_write_reg(uint32 u32Addr, uint32 u32Val) {
	sint8 s8Ret;

	s8Ret = spi_write_reg(u32Addr, u32Val);

	if(N_OK == s8Ret) 
    s8Ret = M2M_SUCCESS;
	else 
    s8Ret = M2M_ERR_BUS_FAIL;

	return s8Ret;
  }

/*
*	@fn		nm_spi_read_block
*	@brief	Read block of data
*	@param [in]	u32Addr
*				Start address
*	@param [out]	puBuf
*				Pointer to a buffer used to return the read data
*	@param [in]	u16Sz
*				Number of bytes to read. The buffer size must be >= u16Sz
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
sint8 nm_spi_read_block(uint32 u32Addr, uint8 *puBuf, uint16 u16Sz) {
	sint8 s8Ret;

	s8Ret = nm_spi_read(u32Addr, puBuf, u16Sz);

	if(N_OK == s8Ret) 
    s8Ret = M2M_SUCCESS;
	else 
    s8Ret = M2M_ERR_BUS_FAIL;

	return s8Ret;
  }

/*
*	@fn		nm_spi_write_block
*	@brief	Write block of data
*	@param [in]	u32Addr
*				Start address
*	@param [in]	puBuf
*				Pointer to the buffer holding the data to be written
*	@param [in]	u16Sz
*				Number of bytes to write. The buffer size must be >= u16Sz
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	M. Abdelmawla
*	@date	11 July 2012
*	@version	1.0
*/
sint8 nm_spi_write_block(uint32 u32Addr, uint8 *puBuf, uint16 u16Sz) {
	sint8 s8Ret;

	s8Ret = nm_spi_write(u32Addr, puBuf, u16Sz);

	if(N_OK == s8Ret) 
    s8Ret = M2M_SUCCESS;
	else 
    s8Ret = M2M_ERR_BUS_FAIL;

	return s8Ret;
  }

#endif


// nm_uart.c ***********************************************************************************************************
//#include "common/include/nm_common.h"

#ifdef CONF_WINC_USE_UART

#include "driver/source/nmuart.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"

#define HDR_SZ  12

static uint8 get_cs(uint8* b, uint8 sz){
	int i;
	uint8 cs = 0;
  
	for(i=0; i < sz; i++)
		cs ^= b[i];
	return cs;
  }

/*
*	@fn			nm_uart_sync_cmd
*	@brief		Check COM Port
*	@return		M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author		Dina El Sissy
*	@date		13 AUG 2012
*	@version	1.0
*/
sint8 nm_uart_sync_cmd(void) {
	tstrNmUartDefault strUart;
	sint8 s8Ret = -1;
	uint8 b [HDR_SZ+1];
	uint8 rsz;
	uint8 onchip = 0;

	/*read reg*/
	b[0] = 0x12;

	rsz = 1;
	strUart.pu8Buf = b;
	strUart.u16Sz = 1;

	if(M2M_SUCCESS == nm_bus_ioctl(NM_BUS_IOCTL_W, &strUart)) {
		strUart.u16Sz = rsz;
		if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_R, &strUart))	{
			s8Ret = M2M_ERR_BUS_FAIL;
      }
    }
	else	{
		M2M_ERR("failed to send cfg bytes\r\n");
		s8Ret = M2M_ERR_BUS_FAIL;
    }
	if(b[0] == 0x5a)	{
		s8Ret = 0;
		onchip = 1;
		M2M_INFO("Built-in WINC1500 UART Found\r\n");
  	}
	else if(b[0] == 0x5b)	{
		s8Ret = 0;
		onchip = 0;
		M2M_INFO("WINC1500 Serial Bridge Found\r\n");
    }
	/*TODO: this should be the way we read the register since the cortus is little endian*/
	/**pu32RetVal = b[0] | ((uint32)b[1] << 8) | ((uint32)b[2] << 16) | ((uint32)b[3] << 24);*/
	if(s8Ret == M2M_SUCCESS)
		s8Ret = (sint8)onchip;
	return s8Ret;
  }

sint8 nm_uart_read_reg_with_ret(uint32 u32Addr, uint32* pu32RetVal) {
	tstrNmUartDefault strUart;
	sint8 s8Ret = M2M_SUCCESS;
	uint8 b [HDR_SZ+1];
	uint8 rsz;

	/*read reg*/
	b[0] = 0xa5;
	b[1] = 0;
	b[2] = 0;
	b[3] = 0;
	b[4] = 0;
	b[5] = (uint8)(u32Addr & 0x000000ff);
	b[6] = (uint8)((u32Addr & 0x0000ff00)>>8);
	b[7] = (uint8)((u32Addr & 0x00ff0000)>>16);
	b[8] = (uint8)((u32Addr & 0xff000000)>>24);
	b[9] = 0;
	b[10] = 0;
	b[11] = 0;
	b[12] = 0;

	b[2] = get_cs(&b[1],HDR_SZ);

	rsz = 4;
	strUart.pu8Buf = b;
	strUart.u16Sz = sizeof(b);

	if(M2M_SUCCESS == nm_bus_ioctl(NM_BUS_IOCTL_W, &strUart))	{
		if(!nm_bus_get_chip_type())	{
			strUart.u16Sz = 1;
			if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_R, &strUart))	{
				s8Ret = M2M_ERR_BUS_FAIL;
			}
			if(b[0] == 0xAC) {
				M2M_DBG("Successfully sent the command\r\n");
				strUart.u16Sz = rsz;
				if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_R, &strUart))	{
					s8Ret = M2M_ERR_BUS_FAIL;
				}
			}
			else {
				s8Ret = M2M_ERR_BUS_FAIL;
			}
		}
		else {
			strUart.u16Sz = rsz;
			if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_R, &strUart))	{
				s8Ret = M2M_ERR_BUS_FAIL;
        }
      }
    }
	else	{
		M2M_ERR("failed to send cfg bytes\r\n");
		s8Ret = M2M_ERR_BUS_FAIL;
    }
	/*TODO: this should be the way we read the register since the cortus is little endian*/
	/**pu32RetVal = b[0] | ((uint32)b[1] << 8) | ((uint32)b[2] << 16) | ((uint32)b[3] << 24);*/

	*pu32RetVal = ((uint32)b[0] << 24) | ((uint32)b[1] << 16) | ((uint32)b[2] << 8) | b[3];

	return s8Ret;
  }

/*
*	@fn			nm_uart_read_reg
*	@brief		Read register
*	@param [in]	u32Addr
*				Register address
*	@return		Register value
*	@author		Dina El Sissy
*	@date		13 AUG 2012
*	@version	1.0
*/
uint32 nm_uart_read_reg(uint32 u32Addr) {
	uint32 val;
	
  nm_uart_read_reg_with_ret(u32Addr , &val);
	return val;
  }

/*
*	@fn			nm_uart_write_reg
*	@brief		write register
*	@param [in]	u32Addr
*				Register address
*	@param [in]	u32Val
*				Value to be written to the register
*	@return		M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author		Dina El Sissy
*	@date		13 AUG 2012
*	@version	1.0
*/
sint8 nm_uart_write_reg(uint32 u32Addr, uint32 u32Val) {
	tstrNmUartDefault strUart;
	sint8 s8Ret = M2M_SUCCESS;
	uint8 b[HDR_SZ+1];

	/*write reg*/
	b[0] = 0xa5;
	b[1] = 1;
	b[2] = 0;
	b[3] = 0;
	b[4] = 0;
	b[5] = (uint8)(u32Addr & 0x000000ff);
	b[6] = (uint8)((u32Addr & 0x0000ff00)>>8);
	b[7] = (uint8)((u32Addr & 0x00ff0000)>>16);
	b[8] = (uint8)((u32Addr & 0xff000000)>>24);
	b[9] = (uint8)(u32Val & 0x000000ff);
	b[10] = (uint8)((u32Val & 0x0000ff00)>>8);
	b[11] = (uint8)((u32Val & 0x00ff0000)>>16);
	b[12] = (uint8)((u32Val & 0xff000000)>>24);

	b[2] = get_cs(&b[1],HDR_SZ);

	get_cs(&b[1],HDR_SZ);

	strUart.pu8Buf = b;
	strUart.u16Sz = sizeof(b);

	if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_W, &strUart))	{
		M2M_ERR("write error\r\n");
		s8Ret = M2M_ERR_BUS_FAIL;
  	}
	else	{
		if(!nm_bus_get_chip_type())	{
			//check for the ack from the SAMD21 for the packet reception.
			strUart.u16Sz = 1;
			if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_R, &strUart))	{
				s8Ret = M2M_ERR_BUS_FAIL;
			}
			if(b[0] == 0xAC) {
				M2M_DBG("Successfully sent the reg write command\r\n");
			}
			else {
				M2M_ERR("write error\r\n");
				s8Ret = M2M_ERR_BUS_FAIL;
        }
      }
    }

	return s8Ret;
  }


/**
*	@fn			nm_uart_read_block
*	@brief		Read block of data
*	@param [in]	u32Addr
*				Start address
*	@param [out]	puBuf
*				Pointer to a buffer used to return the read data
*	@param [in]	u16Sz
*				Number of bytes to read. The buffer size must be >= u16Sz
*	@return		M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author		Dina El Sissy
*	@date		13 AUG 2012
*	@version	1.0
*/
sint8 nm_uart_read_block(uint32 u32Addr, uint8 *pu8Buf, uint16 u16Sz) {
	tstrNmUartDefault strUart;
	sint8 s8Ret = M2M_SUCCESS;
	uint8 au8Buf[HDR_SZ+1];

	au8Buf[0] = 0xa5;
	au8Buf[1] = 2;
	au8Buf[2] = 0;
	au8Buf[3] = (uint8)(u16Sz & 0x00ff);
	au8Buf[4] = (uint8)((u16Sz & 0xff00)>>8);
	au8Buf[5] = (uint8)(u32Addr & 0x000000ff);
	au8Buf[6] = (uint8)((u32Addr & 0x0000ff00)>>8);
	au8Buf[7] = (uint8)((u32Addr & 0x00ff0000)>>16);
	au8Buf[8] = (uint8)((u32Addr & 0xff000000)>>24);
	au8Buf[9] = 0;
	au8Buf[10] = 0;
	au8Buf[11] = 0;
	au8Buf[12] = 0;

	au8Buf[2] = get_cs(&au8Buf[1],HDR_SZ);

	strUart.pu8Buf = au8Buf;
	strUart.u16Sz = sizeof(au8Buf);

	if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_W, &strUart))	{
		M2M_ERR("write error\r\n");
		s8Ret = M2M_ERR_BUS_FAIL;
    }
	else	{
		if(!nm_bus_get_chip_type())	{
			//check for the ack from the SAMD21 for the packet reception.
			strUart.u16Sz = 1;
			if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_R, &strUart))	{
				s8Ret = M2M_ERR_BUS_FAIL;
			}
			if(au8Buf[0] == 0xAC)	{
				M2M_DBG("Successfully sent the block read command\r\n");
				strUart.pu8Buf = pu8Buf;
				strUart.u16Sz = u16Sz;

				if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_R, &strUart))	{
					M2M_ERR("read error\r\n");
					s8Ret = M2M_ERR_BUS_FAIL;
          }
        }
			else {
				M2M_ERR("write error (Error sending the block read command)\r\n");
				s8Ret = M2M_ERR_BUS_FAIL;
        }
      }
		else	{
			strUart.pu8Buf = pu8Buf;
			strUart.u16Sz = u16Sz;

			if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_R, &strUart))	{
				M2M_ERR("read error\r\n");
				s8Ret = M2M_ERR_BUS_FAIL;
        }
      }
    }

	return s8Ret;
  }

/**
*	@fn			nm_uart_write_block
*	@brief		Write block of data
*	@param [in]	u32Addr
*				Start address
*	@param [in]	puBuf
*				Pointer to the buffer holding the data to be written
*	@param [in]	u16Sz
*				Number of bytes to write. The buffer size must be >= u16Sz
*	@return		M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author		Dina El Sissy
*	@date		13 AUG 2012
*	@version	1.0
*/
sint8 nm_uart_write_block(uint32 u32Addr, uint8 *puBuf, uint16 u16Sz) {
	tstrNmUartDefault strUart;
	sint8 s8Ret = M2M_SUCCESS;
	static uint8 au8Buf[HDR_SZ+1];

	au8Buf[0] = 0xa5;
	au8Buf[1] = 3;
	au8Buf[2] = 0;
	au8Buf[3] = (uint8)(u16Sz & 0x00ff);
	au8Buf[4] = (uint8)((u16Sz & 0xff00)>>8);
	au8Buf[5] = (uint8)(u32Addr & 0x000000ff);
	au8Buf[6] = (uint8)((u32Addr & 0x0000ff00)>>8);
	au8Buf[7] = (uint8)((u32Addr & 0x00ff0000)>>16);
	au8Buf[8] = (uint8)((u32Addr & 0xff000000)>>24);
	au8Buf[9] = 0;
	au8Buf[10] = 0;
	au8Buf[11] = 0;
	au8Buf[12] = 0;

	au8Buf[2] = get_cs(&au8Buf[1],HDR_SZ);

	strUart.pu8Buf = au8Buf;
	strUart.u16Sz = sizeof(au8Buf);

	if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_W, &strUart))	{
		M2M_ERR("write error\r\n");
		s8Ret = M2M_ERR_BUS_FAIL;
    }
	else	{
		if(!nm_bus_get_chip_type())	{
			//check for the ack from the SAMD21 for the packet reception.
			strUart.u16Sz = 1;
			if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_R, &strUart))	{
				s8Ret = M2M_ERR_BUS_FAIL;
      	}
			if(au8Buf[0] == 0xAC)	{
				M2M_DBG("Successfully sent the block Write command\r\n");
				strUart.pu8Buf = puBuf;
				strUart.u16Sz = u16Sz;

				if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_W, &strUart))	{
					M2M_ERR("write error\r\n");
					s8Ret = M2M_ERR_BUS_FAIL;
        	}
				else {
					//check for the ack from the SAMD21 for the payload reception.
					strUart.pu8Buf = au8Buf;
					strUart.u16Sz = 1;
					if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_R, &strUart))	{
						s8Ret = M2M_ERR_BUS_FAIL;
					}
					if(au8Buf[0] == 0xAC)	{
						M2M_DBG("Successfully sent the data payload\r\n");
					}
					else {
						M2M_ERR("write error\r\n");
						s8Ret = M2M_ERR_BUS_FAIL;
            }
          }
        }
			else {
				M2M_ERR("write error (Error sending the block write command)\r\n");
				s8Ret = M2M_ERR_BUS_FAIL;
        }
      }
		else {
			strUart.pu8Buf = puBuf;
			strUart.u16Sz = u16Sz;

			if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_W, &strUart))	{
				M2M_ERR("write error\r\n");
				s8Ret = M2M_ERR_BUS_FAIL;
        }
      }
    }
	return s8Ret;
  }

/**
*	@fn			nm_uart_reconfigure
*	@brief		Reconfigures the UART interface
*	@param [in]	ptr
*				Pointer to a DWORD containing baudrate at this moment.
*	@return		M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author		Viswanathan Murugesan
*	@date		22 OCT 2014
*	@version	1.0
*/
sint8 nm_uart_reconfigure(void *ptr) {
	tstrNmUartDefault strUart;
	sint8 s8Ret = M2M_SUCCESS;
	uint8 b[HDR_SZ+1];

	/*write reg*/
	b[0] = 0xa5;
	b[1] = 5;
	b[2] = 0;
	b[3] = 0;
	b[4] = 0;
	b[5] = 0;
	b[6] = 0;
	b[7] = 0;
	b[8] = 0;
	b[9] = (uint8)((*(unsigned long *)ptr) & 0x000000ff);
	b[10] = (uint8)(((*(unsigned long *)ptr) & 0x0000ff00)>>8);
	b[11] = (uint8)(((*(unsigned long *)ptr) & 0x00ff0000)>>16);
	b[12] = (uint8)(((*(unsigned long *)ptr) & 0xff000000)>>24);

	b[2] = get_cs(&b[1],HDR_SZ);

	get_cs(&b[1],HDR_SZ);

	strUart.pu8Buf = b;
	strUart.u16Sz = sizeof(b);

	if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_W, &strUart)) {
		M2M_ERR("write error\r\n");
		s8Ret = M2M_ERR_BUS_FAIL;
	}
	else {
		if(!nm_bus_get_chip_type())	{
			//check for the ack from the SAMD21 for the packet reception.
			strUart.u16Sz = 1;
			if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_R, &strUart))	{
				s8Ret = M2M_ERR_BUS_FAIL;
        }
			if(b[0] == 0xAC) {
				M2M_DBG("Successfully sent the UART reconfigure command\r\n");
        }
			else {
				M2M_ERR("write error\r\n");
				s8Ret = M2M_ERR_BUS_FAIL;
        }
      }
    }

	return s8Ret;
  }
#endif


// socket.c *********************************************************************************************************

//#include "bsp/include/nm_bsp.h"
//#include "socket/include/socket.h"
//#include "driver/source/m2m_hif.h"
//#include "socket/source/socket_internal.h"
//#include "driver/include/m2m_types.h"

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#define TLS_RECORD_HEADER_LENGTH			(5)
#define ETHERNET_HEADER_OFFSET				(34)
#define ETHERNET_HEADER_LENGTH				(14)
#define TCP_IP_HEADER_LENGTH				(40)
#define UDP_IP_HEADER_LENGTH				(28)

#define IP_PACKET_OFFSET					(ETHERNET_HEADER_LENGTH + ETHERNET_HEADER_OFFSET - M2M_HIF_HDR_OFFSET)

#define TCP_TX_PACKET_OFFSET				(IP_PACKET_OFFSET + TCP_IP_HEADER_LENGTH)
#define UDP_TX_PACKET_OFFSET				(IP_PACKET_OFFSET + UDP_IP_HEADER_LENGTH)
#define SSL_TX_PACKET_OFFSET				(TCP_TX_PACKET_OFFSET + TLS_RECORD_HEADER_LENGTH)

#define SOCKET_REQUEST(reqID, reqArgs, reqSize, reqPayload, reqPayloadSize, reqPayloadOffset)		\
	hif_send(M2M_REQ_GROUP_IP, reqID, reqArgs, reqSize, reqPayload, reqPayloadSize, reqPayloadOffset)


#define SSL_FLAGS_ACTIVE					NBIT0
#define SSL_FLAGS_BYPASS_X509				NBIT1
#define SSL_FLAGS_2_RESERVD					NBIT2
#define SSL_FLAGS_3_RESERVD					NBIT3
#define SSL_FLAGS_CACHE_SESSION				NBIT4
#define SSL_FLAGS_NO_TX_COPY				NBIT5


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
GLOBALS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

volatile sint8					gsockerrno;
volatile tstrSocket			gastrSockets[MAX_SOCKET];
volatile uint8					gu8OpCode;
volatile uint16					gu16BufferSize;
volatile uint16					gu16SessionID = 0;	

volatile tpfAppSocketCb		  gpfAppSocketCb;
volatile tpfAppResolveCb		gpfAppResolveCb;
volatile uint8					gbSocketInit = 0;
volatile tpfPingCb				gfpPingCb;

/*********************************************************************
Function
		Socket_ReadSocketData

Description
		Callback function used by the NMC1500 driver to deliver messages
		for socket layer.

Return
		None.

Author
		Ahmed Ezzat

Version
		1.0

Date
		17 July 2012
*********************************************************************/
NMI_API void Socket_ReadSocketData(SOCKET sock, tstrSocketRecvMsg *pstrRecv,uint8 u8SocketMsg,
								  uint32 u32StartAddress,uint16 u16ReadCount) {
  
	if((u16ReadCount > 0) && (gastrSockets[sock].pu8UserBuffer) && (gastrSockets[sock].u16UserBufferSize > 0) && (gastrSockets[sock].bIsUsed))	{
		uint32	u32Address = u32StartAddress;
		uint16	u16Read;
		sint16	s16Diff;
		uint8	u8SetRxDone;

		pstrRecv->u16RemainingSize = u16ReadCount;
		do {
			u8SetRxDone = 1;
			u16Read = u16ReadCount;
			s16Diff	= u16Read - gastrSockets[sock].u16UserBufferSize;
			if(s16Diff > 0)	{
				u8SetRxDone = 0;
				u16Read		= gastrSockets[sock].u16UserBufferSize;
        }
			if(hif_receive(u32Address, gastrSockets[sock].pu8UserBuffer, u16Read, u8SetRxDone) == M2M_SUCCESS) {
				pstrRecv->pu8Buffer			= gastrSockets[sock].pu8UserBuffer;
				pstrRecv->s16BufferSize		= u16Read;
				pstrRecv->u16RemainingSize	-= u16Read;

        gastrSockets[sock].statusEvents |= SOCKET_EVENT_RX_READY;
				if(gpfAppSocketCb)
					gpfAppSocketCb(sock,u8SocketMsg, pstrRecv);

				u16ReadCount -= u16Read;
				u32Address += u16Read;
        }
			else {
				M2M_INFO("(ERRR)Current <%d>\r\n", u16ReadCount);
				break;
        }
      } while(u16ReadCount != 0);
    }
  }

/*********************************************************************
Function
		m2m_ip_cb

Description
		Callback function used by the NMC1000 driver to deliver messages
		for socket layer.

Return
		None.

Author
		Ahmed Ezzat

Version
		1.0

Date
		17 July 2012
*********************************************************************/
static void m2m_ip_cb(uint8 u8OpCode, uint16 u16BufferSize,uint32 u32Address) {

	switch(u8OpCode) {
    case SOCKET_CMD_BIND:
    {
      tstrBindReply		strBindReply;
      tstrSocketBindMsg	strBind;

      if(hif_receive(u32Address, (uint8*)&strBindReply, sizeof(tstrBindReply), 0) == M2M_SUCCESS)	{
        strBind.status = strBindReply.s8Status;
        if(gpfAppSocketCb)
          gpfAppSocketCb(strBindReply.sock,SOCKET_MSG_BIND,&strBind);
        }
      }
      break;
    case SOCKET_CMD_LISTEN:
    {
      tstrListenReply			strListenReply;
      tstrSocketListenMsg		strListen;
      if(hif_receive(u32Address, (uint8*)&strListenReply, sizeof(tstrListenReply), 0) == M2M_SUCCESS)	{
        strListen.status = strListenReply.s8Status;
        if(gpfAppSocketCb)
          gpfAppSocketCb(strListenReply.sock,SOCKET_MSG_LISTEN, &strListen);
        }
      }
      break;
    case SOCKET_CMD_ACCEPT:
    {
      tstrAcceptReply			strAcceptReply;
      tstrSocketAcceptMsg		strAccept;
      if(hif_receive(u32Address, (uint8*)&strAcceptReply, sizeof(tstrAcceptReply), 0) == M2M_SUCCESS) {
        if(strAcceptReply.sConnectedSock >= 0) {
          gastrSockets[strAcceptReply.sConnectedSock].u8SSLFlags 	= 0;
          gastrSockets[strAcceptReply.sConnectedSock].bIsUsed	= 1;

          /* The session ID is used to distinguish different socket connections
            by comparing the assigned session ID to the one reported by the firmware*/
          ++gu16SessionID;
          if(gu16SessionID == 0)
            ++gu16SessionID;

          gastrSockets[strAcceptReply.sConnectedSock].u16SessionID = gu16SessionID;
          M2M_DBG("Socket %d session ID = %d\r\n",strAcceptReply.sConnectedSock , gu16SessionID );		
          }
        strAccept.sock = strAcceptReply.sConnectedSock;
        strAccept.strAddr.sin_family		= AF_INET;
        strAccept.strAddr.sin_port = strAcceptReply.strAddr.u16Port;
        strAccept.strAddr.sin_addr.s_addr = strAcceptReply.strAddr.u32IPAddr;
        if(gpfAppSocketCb)
          gpfAppSocketCb(strAcceptReply.sListenSock, SOCKET_MSG_ACCEPT, &strAccept);
//          gastrSockets[sock].statusEvents |= SOCKET_EVENT_ACCEPTED;// mettere "ACCEPTED"
        }
      }
      break;
    case SOCKET_CMD_CONNECT:
    case SOCKET_CMD_SSL_CONNECT:
    {
      tstrConnectReply	strConnectReply;
      tstrSocketConnectMsg	strConnMsg;
      if(hif_receive(u32Address, (uint8*)&strConnectReply, sizeof(tstrConnectReply), 0) == M2M_SUCCESS) {
        strConnMsg.sock	= strConnectReply.sock;
        strConnMsg.s8Error	= strConnectReply.s8Error;
        if(strConnectReply.s8Error == SOCK_ERR_NO_ERROR) {
          gastrSockets[strConnectReply.sock].u16DataOffset = strConnectReply.u16AppDataOffset - M2M_HIF_HDR_OFFSET;
          }
        if(gpfAppSocketCb)
          gpfAppSocketCb(strConnectReply.sock,SOCKET_MSG_CONNECT, &strConnMsg);
//          gastrSockets[sock].statusEvents |= SOCKET_EVENT_CONNECT_READY;// mettere "CONNECTED"?
        }
      }
      break;
    case SOCKET_CMD_DNS_RESOLVE:
    {
      tstrDnsReply	strDnsReply;
      if(hif_receive(u32Address, (uint8*)&strDnsReply, sizeof(tstrDnsReply), 0) == M2M_SUCCESS)	{
        strDnsReply.u32HostIP = strDnsReply.u32HostIP;
        if(gpfAppResolveCb)
          gpfAppResolveCb((uint8*)strDnsReply.acHostName, strDnsReply.u32HostIP);
        }
      }
      break;
    case SOCKET_CMD_RECV:
    case SOCKET_CMD_RECVFROM:
    case SOCKET_CMD_SSL_RECV:
    {
      SOCKET				sock;
      sint16				s16RecvStatus;
      tstrRecvReply		strRecvReply;
      uint16				u16ReadSize;
      tstrSocketRecvMsg	strRecvMsg;
      uint8         u8CallbackMsgID = SOCKET_MSG_RECV;
      uint16				u16DataOffset;

      if(u8OpCode == SOCKET_CMD_RECVFROM)
        u8CallbackMsgID = SOCKET_MSG_RECVFROM;

      /* Read RECV REPLY data structure.
      */
      u16ReadSize = sizeof(tstrRecvReply);
      if(hif_receive(u32Address, (uint8*)&strRecvReply, u16ReadSize, 0) == M2M_SUCCESS) {
        uint16 u16SessionID = 0;

        sock = strRecvReply.sock;
        u16SessionID = strRecvReply.u16SessionID;
        M2M_DBG("recv callback session ID = %d\r\n",u16SessionID);

        /* Reset the Socket RX Pending Flag.
        */
        gastrSockets[sock].bIsRecvPending = 0;

        s16RecvStatus	= NM_BSP_B_L_16(strRecvReply.s16RecvStatus);
        u16DataOffset	= NM_BSP_B_L_16(strRecvReply.u16DataOffset);
        strRecvMsg.strRemoteAddr.sin_port 			= strRecvReply.strRemoteAddr.u16Port;
        strRecvMsg.strRemoteAddr.sin_addr.s_addr 	= strRecvReply.strRemoteAddr.u32IPAddr;

        if(u16SessionID == gastrSockets[sock].u16SessionID)	{
          if((s16RecvStatus > 0) && (s16RecvStatus < u16BufferSize)) {
            /* Skip incoming bytes until reaching the Start of Application Data. 
            */
            u32Address += u16DataOffset;

            /* Read the Application data and deliver it to the application callback in
            the given application buffer. If the buffer is smaller than the received data,
            the data is passed to the application in chunks according to its buffer size.
            */
            u16ReadSize = (uint16)s16RecvStatus;
            Socket_ReadSocketData(sock, &strRecvMsg, u8CallbackMsgID, u32Address, u16ReadSize);
            }
          else if(s16RecvStatus == SOCK_ERR_TIMEOUT) {    /*GD */
            strRecvMsg.s16BufferSize = s16RecvStatus;
            strRecvMsg.pu8Buffer = NULL;
            if(gpfAppSocketCb)
              gpfAppSocketCb(sock,u8CallbackMsgID,&strRecvMsg);
            }
          else {
            strRecvMsg.s16BufferSize = s16RecvStatus;
            strRecvMsg.pu8Buffer = NULL;
            if(gpfAppSocketCb)
              gpfAppSocketCb(sock,SOCKET_MSG_CLOSE /*GD u8CallbackMsgID*/, &strRecvMsg);
            gastrSockets[sock].statusEvents |= SOCKET_EVENT_CLOSE;
            }
         }
        else {
          M2M_DBG("Discard recv callback %d %d \r\n",u16SessionID , gastrSockets[sock].u16SessionID);
          if(u16ReadSize < u16BufferSize)
            hif_receive(0, NULL, 0, 1);
          }
        }
      }
      break;
    case SOCKET_CMD_SEND:
    case SOCKET_CMD_SENDTO:
    case SOCKET_CMD_SSL_SEND:
    {
      SOCKET	sock;
      sint16	s16Rcvd;
      tstrSendReply	strReply;
      uint8		u8CallbackMsgID = SOCKET_MSG_SEND;

      if(u8OpCode == SOCKET_CMD_SENDTO)
        u8CallbackMsgID = SOCKET_MSG_SENDTO;

      if(hif_receive(u32Address, (uint8*)&strReply, sizeof(tstrSendReply), 0) == M2M_SUCCESS)	{
        uint16 u16SessionID=0;

        sock = strReply.sock;
        u16SessionID = strReply.u16SessionID;
        M2M_DBG("send callback session ID = %d\r\n",u16SessionID);

        s16Rcvd = NM_BSP_B_L_16(strReply.s16SentBytes);

        if(u16SessionID == gastrSockets[sock].u16SessionID)	{
          if(gpfAppSocketCb)
            gpfAppSocketCb(sock,u8CallbackMsgID, &s16Rcvd);
          gastrSockets[sock].statusEvents |= SOCKET_EVENT_TX_READY;// o ACKED? s
          }
        else {
          M2M_DBG("Discard send callback %d %d \r\n",u16SessionID , gastrSockets[sock].u16SessionID);
          }
        }
      }
      break;
    case SOCKET_CMD_PING:
    {
      tstrPingReply	strPingReply;
      if(hif_receive(u32Address, (uint8*)&strPingReply, sizeof(tstrPingReply), 1) == M2M_SUCCESS)	{
        gfpPingCb = (void (*)(uint32 , uint32 , uint8))strPingReply.u32CmdPrivate;
        if(gfpPingCb)
          gfpPingCb(strPingReply.u32IPAddr, strPingReply.u32RTT, strPingReply.u8ErrorCode);
        }
      }
      break;
    default:
      break;
    }
    
  }

/*********************************************************************
Function
		socketInit

Description

Return
		None.

Author
		Ahmed Ezzat

Version
		1.0

Date
		4 June 2012
*********************************************************************/
void socketInit(void) {
  
	if(gbSocketInit==0)	{
		m2m_memset((uint8*)gastrSockets, 0, MAX_SOCKET * sizeof(tstrSocket));
		hif_register_cb(M2M_REQ_GROUP_IP,m2m_ip_cb);
		gbSocketInit=1;
		gu16SessionID=0;
    }
  }

/*********************************************************************
Function
		socketDeinit

Description 

Return
		None.

Author
		Samer Sarhan

Version
		1.0

Date
		27 Feb 2015
*********************************************************************/
void socketDeinit(void) {	
  
	m2m_memset((uint8*)gastrSockets, 0, MAX_SOCKET * sizeof(tstrSocket));
	hif_register_cb(M2M_REQ_GROUP_IP, NULL);
	gpfAppSocketCb = NULL;
	gpfAppResolveCb = NULL;
	gbSocketInit=0;
  }

/*********************************************************************
Function
		registerSocketCallback

Description

Return
		None.

Author
		Ahmed Ezzat

Versio
		1.0

Date
		4 June 2012
*********************************************************************/
void registerSocketCallback(tpfAppSocketCb pfAppSocketCb, tpfAppResolveCb pfAppResolveCb) {
	
  gpfAppSocketCb = pfAppSocketCb;
	gpfAppResolveCb = pfAppResolveCb;
  }

/*********************************************************************
Function
		socket

Description
		Creates a socket.

Return
		- Negative value for error.
		- ZERO or positive value as a socket ID if successful.

Author
		Ahmed Ezzat

Version
		1.0

Date
		4 June 2012
*********************************************************************/
SOCKET socket(uint16 u16Domain, uint8 u8Type, uint8 u8Flags) {
	SOCKET  sock = INVALID_SOCKET;
	uint8		u8Count,u8SocketCount = MAX_SOCKET;
	volatile tstrSocket	*pstrSock;
	
	/* The only supported family is the AF_INET for UDP and TCP transport layer protocols. */
	switch(u16Domain) {
    case AF_INET:
      switch(u8Type) {
        case SOCK_STREAM:
          u8SocketCount = TCP_SOCK_MAX;
          u8Count = 0;
          break;
        case SOCK_DGRAM:
          /*--- UDP SOCKET ---*/
          u8SocketCount = MAX_SOCKET;
          u8Count = TCP_SOCK_MAX;
          break;
        default:
          return sock;
          break;
        }

      for(; u8Count < u8SocketCount; u8Count++)	{
        pstrSock = &gastrSockets[u8Count];
        if(pstrSock->bIsUsed == 0)	{
          m2m_memset((uint8*)pstrSock, 0, sizeof(tstrSocket));
          pstrSock->bIsUsed = 1;

          /* The session ID is used to distinguish different socket connections
            by comparing the assigned session ID to the one reported by the firmware*/
          ++gu16SessionID;
          if(gu16SessionID == 0)
            ++gu16SessionID;

          pstrSock->u16SessionID = gu16SessionID;
          M2M_DBG("1 Socket %d session ID = %d\r\n",u8Count, gu16SessionID );
          sock = (SOCKET)u8Count;

          if(u8Flags & SOCKET_FLAGS_SSL) {
            tstrSSLSocketCreateCmd	strSSLCreate;
            strSSLCreate.sslSock = sock;
            pstrSock->u8SSLFlags = SSL_FLAGS_ACTIVE | SSL_FLAGS_NO_TX_COPY;
            SOCKET_REQUEST(SOCKET_CMD_SSL_CREATE, (uint8*)&strSSLCreate, sizeof(tstrSSLSocketCreateCmd), 0, 0, 0);
            }
          break;
          }
        }
      break;
    default:
      break;
    }
  
	return sock;
  }

/*********************************************************************
Function
		bind

Description
		Request to bind a socket on a local address.

Return


Author
		Ahmed Ezzat

Version
		1.0

Date
		5 June 2012
*********************************************************************/
sint8 bind(SOCKET sock, struct sockaddr *pstrAddr, uint8 u8AddrLen) {
	sint8	s8Ret = SOCK_ERR_INVALID_ARG;
  
	if((pstrAddr) && (sock >= 0) && (gastrSockets[sock].bIsUsed) && (u8AddrLen != 0))	{
		tstrBindCmd	strBind;

		/* Build the bind request. */
		strBind.sock = sock;
		m2m_memcpy((uint8 *)&strBind.strAddr, (uint8 *)pstrAddr, sizeof(tstrSockAddr));

// che cazzo fa sto coglione??		strBind.strAddr.u16Family	= strBind.strAddr.u16Family;
//		strBind.strAddr.u16Port		= strBind.strAddr.u16Port;
//		strBind.strAddr.u32IPAddr	= strBind.strAddr.u32IPAddr;
		strBind.u16SessionID		= gastrSockets[sock].u16SessionID;
		
		/* Send the request. */
		s8Ret = SOCKET_REQUEST(SOCKET_CMD_BIND, (uint8*)&strBind, sizeof(tstrBindCmd), NULL, 0, 0);
		if(s8Ret != SOCK_ERR_NO_ERROR)	{
			s8Ret = SOCK_ERR_INVALID;
    	}
  	}
  
	return s8Ret;
  }

/*********************************************************************
Function
		listen

Description


Return


Author
		Ahmed Ezzat

Version
		1.0

Date
		5 June 2012
*********************************************************************/
sint8 listen(SOCKET sock, uint8 backlog) {
	sint8	s8Ret = SOCK_ERR_INVALID_ARG;
	
	if(sock >= 0 && gastrSockets[sock].bIsUsed)	{
		tstrListenCmd		strListen;

		strListen.sock = sock;
		strListen.u8BackLog = backlog;
		strListen.u16SessionID		= gastrSockets[sock].u16SessionID;

		s8Ret = SOCKET_REQUEST(SOCKET_CMD_LISTEN, (uint8*)&strListen, sizeof(tstrListenCmd), NULL, 0, 0);
		if(s8Ret != SOCK_ERR_NO_ERROR) {
     	s8Ret = SOCK_ERR_INVALID;
      }
    }
  
	return s8Ret;
  }

/*********************************************************************
Function
		accept

Description

Return


Author
		Ahmed Ezzat

Version
		1.0

Date
		5 June 2012
*********************************************************************/
sint8 accept(SOCKET sock, struct sockaddr *addr, uint8 *addrlen) {
	sint8	s8Ret = SOCK_ERR_INVALID_ARG;
	
	if(sock >= 0 && gastrSockets[sock].bIsUsed)	{
		s8Ret = SOCK_ERR_NO_ERROR;
    }
  
	return s8Ret;
  }

/*********************************************************************
Function
		connect

Description
		Connect to a remote TCP Server.

Return


Author
		Ahmed Ezzat

Version
		1.0

Date
		5 June 2012
*********************************************************************/
sint8 connect(SOCKET sock, struct sockaddr *pstrAddr, uint8 u8AddrLen) {
	sint8	s8Ret = SOCK_ERR_INVALID_ARG;
  
	if((sock >= 0) && (pstrAddr) && (gastrSockets[sock].bIsUsed) && (u8AddrLen != 0)) {
		tstrConnectCmd	strConnect;
		uint8  u8Cmd = SOCKET_CMD_CONNECT;
		if((gastrSockets[sock].u8SSLFlags) & SSL_FLAGS_ACTIVE) {
			u8Cmd = SOCKET_CMD_SSL_CONNECT;
			strConnect.u8SslFlags = gastrSockets[sock].u8SSLFlags;
    	}
		strConnect.sock = sock;
		m2m_memcpy((uint8 *)&strConnect.strAddr, (uint8 *)pstrAddr, sizeof(tstrSockAddr));

		strConnect.u16SessionID	= gastrSockets[sock].u16SessionID;
		s8Ret = SOCKET_REQUEST(u8Cmd, (uint8*)&strConnect,sizeof(tstrConnectCmd), NULL, 0, 0);
		if(s8Ret != SOCK_ERR_NO_ERROR) {
      
      // v. sotto lasciare errore!
      
			s8Ret = SOCK_ERR_INVALID;
    	}
  	}
  
	return s8Ret;
  }

/*********************************************************************
Function
		send

Description

Return

Author
		Ahmed Ezzat

Version
		1.0

Date
		5 June 2012
*********************************************************************/
sint16 send(SOCKET sock, void *pvSendBuffer, uint16 u16SendLength, uint16 flags) {
	sint16 s16Ret = SOCK_ERR_INVALID_ARG;
	
	if((sock >= 0) && (pvSendBuffer) && (u16SendLength <= SOCKET_BUFFER_MAX_LENGTH) && (gastrSockets[sock].bIsUsed))	{
		uint16 u16DataOffset;
		tstrSendCmd	strSend;
		uint8	u8Cmd;

		u8Cmd	= SOCKET_CMD_SEND;
		u16DataOffset	= TCP_TX_PACKET_OFFSET;

		strSend.sock = sock;
		strSend.u16DataSize		= NM_BSP_B_L_16(u16SendLength);
		strSend.u16SessionID	= gastrSockets[sock].u16SessionID;

		if(sock >= TCP_SOCK_MAX) {
			u16DataOffset = UDP_TX_PACKET_OFFSET;
    	}
		if(gastrSockets[sock].u8SSLFlags & SSL_FLAGS_ACTIVE) {
			u8Cmd = SOCKET_CMD_SSL_SEND;
			u16DataOffset	= gastrSockets[sock].u16DataOffset;
      }

		s16Ret = SOCKET_REQUEST(u8Cmd | M2M_REQ_DATA_PKT, (uint8*)&strSend, sizeof(tstrSendCmd), pvSendBuffer, u16SendLength, u16DataOffset);
//          printf("send: %d\r\n",s16Ret);
    if(s16Ret == -3) {      /// boh... cmq è l'unica cosa che ritorna...
      s16Ret=SOCK_ERR_BUFFER_FULL;
      }
// togliamo sta cagata! arabo di merda :D		if(s16Ret != SOCK_ERR_NO_ERROR)	{
    // https://microchipsupport.force.com/s/article/ATWINC1500--send---API-Returns-Negative-SendBytes
//			s16Ret = SOCK_ERR_BUFFER_FULL;
//      }
    }
	return s16Ret;
  }

/*********************************************************************
Function
		sendto

Description

Return

Author
		Ahmed Ezzat

Version
		1.0

Date
		4 June 2012
*********************************************************************/
sint16 sendto(SOCKET sock, void *pvSendBuffer, uint16 u16SendLength, uint16 flags, struct sockaddr *pstrDestAddr, uint8 u8AddrLen) {
	sint16	s16Ret = SOCK_ERR_INVALID_ARG;
	
	if((sock >= 0) && (pvSendBuffer) && (u16SendLength <= SOCKET_BUFFER_MAX_LENGTH)) {
		if(gastrSockets[sock].bIsUsed) {
			tstrSendCmd	strSendTo;

			m2m_memset((uint8*)&strSendTo, 0, sizeof(tstrSendCmd));

			strSendTo.sock			= sock;
			strSendTo.u16DataSize	= NM_BSP_B_L_16(u16SendLength);
			strSendTo.u16SessionID	= gastrSockets[sock].u16SessionID;
			
			if(pstrDestAddr) {
				struct sockaddr_in	*pstrAddr;
				pstrAddr = (void*)pstrDestAddr;

				strSendTo.strAddr.u16Family	= pstrAddr->sin_family;
				strSendTo.strAddr.u16Port	= pstrAddr->sin_port;
				strSendTo.strAddr.u32IPAddr	= pstrAddr->sin_addr.s_addr;
        }
			s16Ret = SOCKET_REQUEST(SOCKET_CMD_SENDTO | M2M_REQ_DATA_PKT, (uint8*)&strSendTo,  sizeof(tstrSendCmd),
				pvSendBuffer, u16SendLength, UDP_TX_PACKET_OFFSET);

			if(s16Ret != SOCK_ERR_NO_ERROR)	{   // ***** v. send... per return code
				s16Ret = SOCK_ERR_BUFFER_FULL;
        }
      }
    }
  
	return s16Ret;
  }

/*********************************************************************
Function
		recv

Description

Return


Author
		Ahmed Ezzat

Version
		1.0
		2.0  9 April 2013 --> Add timeout for recv operation.

Date
		5 June 2012
*********************************************************************/
sint16 recv(SOCKET sock, void *pvRecvBuf, uint16 u16BufLen, uint32 u32Timeoutmsec) {
	sint16	s16Ret = SOCK_ERR_INVALID_ARG;
	
	if((sock >= 0) && (pvRecvBuf) && (u16BufLen != 0) && (gastrSockets[sock].bIsUsed))	{
		s16Ret = SOCK_ERR_NO_ERROR;
		gastrSockets[sock].pu8UserBuffer 		= (uint8*)pvRecvBuf;
		gastrSockets[sock].u16UserBufferSize 	= u16BufLen;

		if(!gastrSockets[sock].bIsRecvPending) {
			tstrRecvCmd	strRecv;
			uint8	u8Cmd = SOCKET_CMD_RECV;

			gastrSockets[sock].bIsRecvPending = 1;
			if(gastrSockets[sock].u8SSLFlags & SSL_FLAGS_ACTIVE) {
				u8Cmd = SOCKET_CMD_SSL_RECV;
        }

			/* Check the timeout value. */
			if(u32Timeoutmsec == 0)
				strRecv.u32Timeoutmsec = 0xFFFFFFFF;
			else
				strRecv.u32Timeoutmsec = NM_BSP_B_L_32(u32Timeoutmsec);
			strRecv.sock = sock;
			strRecv.u16SessionID = gastrSockets[sock].u16SessionID;
		
			s16Ret = SOCKET_REQUEST(u8Cmd, (uint8*)&strRecv, sizeof(tstrRecvCmd), NULL , 0, 0);
			if(s16Ret != SOCK_ERR_NO_ERROR)	{
				s16Ret = SOCK_ERR_BUFFER_FULL;
      	}
    	}
  	}
  
 	return s16Ret;
  }

/*********************************************************************
Function
		recvfrom

Description

Return


Author
		Ahmed Ezzat

Version
		1.0
		2.0  9 April 2013 --> Add timeout for recv operation.

Date
		5 June 2012
*********************************************************************/
sint16 recvfrom(SOCKET sock, void *pvRecvBuf, uint16 u16BufLen, uint32 u32Timeoutmsec, struct sockaddr *pstrDestAddr, uint8 u8AddrLen /*gd...*/) {
	sint16	s16Ret = SOCK_ERR_NO_ERROR;
  
	if((sock >= 0) && (pvRecvBuf) && (u16BufLen != 0) && (gastrSockets[sock].bIsUsed))	{
//		if(gastrSockets[sock].bIsUsed) {
			s16Ret = SOCK_ERR_NO_ERROR;
			gastrSockets[sock].pu8UserBuffer = (uint8*)pvRecvBuf;
			gastrSockets[sock].u16UserBufferSize = u16BufLen;

			if(!gastrSockets[sock].bIsRecvPending) {
				tstrRecvCmd	strRecv;

				gastrSockets[sock].bIsRecvPending = 1;

				/* Check the timeout value. */
				if(u32Timeoutmsec == 0)
					strRecv.u32Timeoutmsec = 0xFFFFFFFF;
				else
					strRecv.u32Timeoutmsec = NM_BSP_B_L_32(u32Timeoutmsec);
				strRecv.sock = sock;
				strRecv.u16SessionID = gastrSockets[sock].u16SessionID;
				
				s16Ret = SOCKET_REQUEST(SOCKET_CMD_RECVFROM, (uint8*)&strRecv, sizeof(tstrRecvCmd), NULL, 0, 0);
				if(s16Ret != SOCK_ERR_NO_ERROR)	{
					s16Ret = SOCK_ERR_BUFFER_FULL;
          }
        }
//      }
    }
	else {
		s16Ret = SOCK_ERR_INVALID_ARG;
  	}
  
	return s16Ret;
  }

/*********************************************************************
Function
		close

Description

Return
		None.

Author
		Ahmed Ezzat

Version
		1.0

Date
		4 June 2012
*********************************************************************/
sint8 close(SOCKET sock) {
	sint8	s8Ret = SOCK_ERR_INVALID_ARG;
  
	if(sock >= 0 && gastrSockets[sock].bIsUsed) {
		uint8	u8Cmd = SOCKET_CMD_CLOSE;
		tstrCloseCmd strclose;
		strclose.sock = sock; 
		strclose.u16SessionID	= gastrSockets[sock].u16SessionID;
		
		gastrSockets[sock].bIsUsed = 0;
		gastrSockets[sock].u16SessionID =0;
		
		if(gastrSockets[sock].u8SSLFlags & SSL_FLAGS_ACTIVE) {
			u8Cmd = SOCKET_CMD_SSL_CLOSE;
      }
		s8Ret = SOCKET_REQUEST(u8Cmd, (uint8*)&strclose, sizeof(tstrCloseCmd), NULL,0, 0);
		if(s8Ret != SOCK_ERR_NO_ERROR) {
			s8Ret = SOCK_ERR_INVALID;
      }
		m2m_memset((uint8*)&gastrSockets[sock], 0, sizeof(tstrSocket));
    }
	return s8Ret;
  }

NMI_API sint8 shutdown(SOCKET s,unsigned char m) {
  uint32_t u32EnableCallbacks=0;
  
  
  switch(m) {
    case SD_RECEIVE:
// non c'è DIOCHEMMERDE!     	setsockopt(s, SOL_SOCKET, SO_SET_UDP_RECEIVE_CALLBACK, &u32EnableCallbacks, 4);
      break;
    case SD_SEND:
     	setsockopt(s, SOL_SOCKET, SO_SET_UDP_SEND_CALLBACK, &u32EnableCallbacks, 4);
      break;
    case SD_BOTH:
//     	setsockopt(s, SOL_SOCKET, SO_SET_UDP_RECEIVE_CALLBACK, &u32EnableCallbacks, 4);
     	setsockopt(s, SOL_SOCKET, SO_SET_UDP_SEND_CALLBACK, &u32EnableCallbacks, 4);
      break;
    }
  
  }

/*********************************************************************
Function
		nmi_inet_addr

Description

Return
		Unsigned 32-bit integer representing the IP address in Network
		byte order.

Author
		Ahmed Ezzat

Version
		1.0

Date
		4 June 2012
*********************************************************************/
uint32 nmi_inet_addr(const char *pcIpAddr) {
	uint8	tmp;
//	uint32	u32IP = 0;
	uint8 c;
	uint8	i, j;
#warning VERIFICARE Ipv4Addr!
  Ipv4Addr ipv4;
  
	tmp=0;

	for(i=0; i < 4; ++i)	{
		j=0;
		do {
			c=*pcIpAddr;
			++j;
			if(j > 4)	{
				return 0;
        }
			if(c == '.' || c == 0) {
				ipv4.addr[i] = tmp;
				tmp = 0;
        }
			else if(c >= '0' && c <= '9') {
				tmp = (tmp * 10) + (c - '0');
        }
			else {
				return 0;
        }
     	++pcIpAddr;
      } while(c != '.' && c != 0);
    }
//	m2m_memcpy((uint8*)&u32IP, ipv4.addr, 4);
  
	return ipv4.ip;
  }

/*********************************************************************
Function
		gethostbyname

Description

Return
		None.

Author
		Ahmed Ezzat

Version
		1.0

Date
		4 June 2012
*********************************************************************/
sint8 gethostbyname(uint8 *pcHostName) {
	sint8	s8Err = SOCK_ERR_INVALID_ARG;
	uint8	u8HostNameSize = (uint8)strlen(pcHostName);
  
	if(u8HostNameSize <= HOSTNAME_MAX_SIZE)	{
		s8Err = SOCKET_REQUEST(SOCKET_CMD_DNS_RESOLVE | M2M_REQ_DATA_PKT, (uint8*)pcHostName, u8HostNameSize + 1, NULL,0, 0);
		if(s8Err != SOCK_ERR_NO_ERROR) {
			s8Err = SOCK_ERR_INVALID;
      }
    }
	return s8Err;
  }

/*********************************************************************
Function
		setsockopt

Description

Return
		None.

Author
		Abdelrahman Diab

Version
		1.0

Date
		9 September 2014
*********************************************************************/
static sint8 sslSetSockOpt(SOCKET sock, uint8  u8Opt, const void *pvOptVal, uint16 u16OptLen) {
	sint8	s8Ret = SOCK_ERR_INVALID_ARG;
  
	if(sock < TCP_SOCK_MAX)	{
		if(gastrSockets[sock].u8SSLFlags & SSL_FLAGS_ACTIVE)	{
			switch(u8Opt) {
        case SO_SSL_BYPASS_X509_VERIF:
        {
          int	optVal = *((int*)pvOptVal);
          if(optVal) {
            gastrSockets[sock].u8SSLFlags |= SSL_FLAGS_BYPASS_X509;
            }
          else {
            gastrSockets[sock].u8SSLFlags &= ~SSL_FLAGS_BYPASS_X509;
            }
          s8Ret = SOCK_ERR_NO_ERROR;
        }
          break;
        case SO_SSL_ENABLE_SESSION_CACHING:
        {
          int	optVal = *((int*)pvOptVal);
          if(optVal)	{
            gastrSockets[sock].u8SSLFlags |= SSL_FLAGS_CACHE_SESSION;
            }
          else {
            gastrSockets[sock].u8SSLFlags &= ~SSL_FLAGS_CACHE_SESSION;
            }
          s8Ret = SOCK_ERR_NO_ERROR;
        }
          break;
        case SO_SSL_SNI:
          if(u16OptLen < HOSTNAME_MAX_SIZE)	{
            uint8					*pu8SNI = (uint8*)pvOptVal;
            tstrSSLSetSockOptCmd	strCmd;

            strCmd.sock			= sock;
            strCmd.u16SessionID	= gastrSockets[sock].u16SessionID;
            strCmd.u8Option		= u8Opt;
            strCmd.u32OptLen	= u16OptLen;
            m2m_memcpy(strCmd.au8OptVal, pu8SNI, HOSTNAME_MAX_SIZE);

            if(SOCKET_REQUEST(SOCKET_CMD_SSL_SET_SOCK_OPT, (uint8*)&strCmd, sizeof(tstrSSLSetSockOptCmd),
              0, 0, 0) == M2M_ERR_MEM_ALLOC) {
              s8Ret = SOCKET_REQUEST(SOCKET_CMD_SSL_SET_SOCK_OPT | M2M_REQ_DATA_PKT, 
                (uint8*)&strCmd, sizeof(tstrSSLSetSockOptCmd), 0, 0, 0);
              }
            s8Ret = SOCK_ERR_NO_ERROR;
            }
          else {
            M2M_ERR("SNI Exceeds Max Length\n");
            }
          break;
        default:
          M2M_ERR("Unknown SSL Socket Option %d\n",u8Opt);
          break;
        }
      }
		else	{
			M2M_ERR("Not SSL Socket\n");
      }
    }
  
	return s8Ret;
  }

/*********************************************************************
Function
		setsockopt

Description

Return
		None.

Author
		Abdelrahman Diab

Version
		1.0

Date
		9 September 2014
*********************************************************************/
sint8 setsockopt(SOCKET sock, uint8 u8Level, uint8 option_name,
       const void *option_value, uint16 u16OptionLen) {
	sint8	s8Ret = SOCK_ERR_INVALID_ARG;
  
	if((sock >= 0) && (option_value) && (gastrSockets[sock].bIsUsed))	{
		if(u8Level == SOL_SSL_SOCKET)	{
			s8Ret = sslSetSockOpt(sock, option_name, option_value, u16OptionLen);
  		}
		else	{
			uint8	u8Cmd = SOCKET_CMD_SET_SOCKET_OPTION;
			tstrSetSocketOptCmd strSetSockOpt;
			strSetSockOpt.u8Option=option_name;
			strSetSockOpt.sock = sock; 
			strSetSockOpt.u32OptionValue = *(uint32*)option_value;
			strSetSockOpt.u16SessionID		= gastrSockets[sock].u16SessionID;

			s8Ret = SOCKET_REQUEST(u8Cmd, (uint8*)&strSetSockOpt, sizeof(tstrSetSocketOptCmd), NULL,0, 0);
			if(s8Ret != SOCK_ERR_NO_ERROR)	{
				s8Ret = SOCK_ERR_INVALID;
        }
      }
    }
  
	return s8Ret;	
  }

/*********************************************************************
Function
		getsockopt

Description

Return
		None.

Author
		Ahmed Ezzat

Version
		1.0

Date
		24 August 2014
*********************************************************************/
sint8 getsockopt(SOCKET sock, uint8 u8Level, uint8 u8OptName, const void *pvOptValue, uint8* pu8OptLen) {
  
	/* TBD */
	return M2M_SUCCESS;
  }

/*********************************************************************
Function
	m2m_ping_req

Description
	Send Ping request.

Return
	
Author
	Ahmed Ezzat

Version
	1.0

Date
	4 June 2015
*********************************************************************/
sint8 m2m_ping_req(uint32 u32DstIP, uint8 u8TTL, tpfPingCb fpPingCb) {
	sint8	s8Ret = M2M_ERR_INVALID_ARG;

	if((u32DstIP != 0) && (fpPingCb))	{
		tstrPingCmd	strPingCmd;

		strPingCmd.u16PingCount		= 1;
		strPingCmd.u32DestIPAddr	= u32DstIP;
		strPingCmd.u32CmdPrivate	= (uint32)fpPingCb;
		strPingCmd.u8TTL			= u8TTL;

		s8Ret = SOCKET_REQUEST(SOCKET_CMD_PING, (uint8*)&strPingCmd, sizeof(tstrPingCmd), NULL, 0, 0);
    }
	return s8Ret;
  }

/*********************************************************************
Function
	sslSetActiveCipherSuites

Description
	Send Ping request.

Return
	
Author
	Ahmed Ezzat

Version
	1.0

Date
	4 June 2015
*********************************************************************/
sint8 sslSetActiveCipherSuites(uint32 u32SslCsBMP) {
	sint8	s8Ret = SOCK_ERR_INVALID_ARG;
  
	if(u32SslCsBMP != 0)	{
		tstrSslSetActiveCsList	strCsList;
	
		strCsList.u32CsBMP = u32SslCsBMP;
		s8Ret = SOCKET_REQUEST(SOCKET_CMD_SSL_SET_CS_LIST, (uint8*)&strCsList, sizeof(tstrSslSetActiveCsList), NULL, 0, 0);
    }
	return s8Ret;
  }


// spi_flash.c ********************************************************************************************************
#ifdef PROFILING
#include "windows.h"
#endif
//#include "spi_flash/include/spi_flash.h"
#define DUMMY_REGISTER	(0x1084)

//#define TIMEOUT (-1) /*MS*/

//#define DISABLE_UNSED_FLASH_FUNCTIONS

#define HOST_SHARE_MEM_BASE		(0xd0000UL)
#define CORTUS_SHARE_MEM_BASE	(0x60000000UL)
#define NMI_SPI_FLASH_ADDR		(0x111c)
/***********************************************************
SPI Flash DMA 
***********************************************************/
#define GET_UINT32(X,Y)			(X[0+Y] + ((uint32)X[1+Y]<<8) + ((uint32)X[2+Y]<<16) +((uint32)X[3+Y]<<24))
#define SPI_FLASH_BASE			(0x10200)
#define SPI_FLASH_MODE			(SPI_FLASH_BASE + 0x00)
#define SPI_FLASH_CMD_CNT		(SPI_FLASH_BASE + 0x04)
#define SPI_FLASH_DATA_CNT		(SPI_FLASH_BASE + 0x08)
#define SPI_FLASH_BUF1			(SPI_FLASH_BASE + 0x0c)
#define SPI_FLASH_BUF2			(SPI_FLASH_BASE + 0x10)
#define SPI_FLASH_BUF_DIR		(SPI_FLASH_BASE + 0x14)
#define SPI_FLASH_TR_DONE		(SPI_FLASH_BASE + 0x18)
#define SPI_FLASH_DMA_ADDR		(SPI_FLASH_BASE + 0x1c)
#define SPI_FLASH_MSB_CTL		(SPI_FLASH_BASE + 0x20)
#define SPI_FLASH_TX_CTL		(SPI_FLASH_BASE + 0x24)

/*********************************************/
/* STATIC FUNCTIONS							 */
/*********************************************/

/**
*	@fn			spi_flash_read_status_reg
*	@brief		Read status register
*	@param[OUT]	val
					value of status reg
*	@return		Status of execution
*/ 
static sint8 spi_flash_read_status_reg(uint8 *val) {
	sint8 ret = M2M_SUCCESS;
	uint8 cmd[1];
	uint32 reg;

	cmd[0] = 0x05;

	ret += nm_write_reg(SPI_FLASH_DATA_CNT, 4);
	ret += nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	ret += nm_write_reg(SPI_FLASH_BUF_DIR, 0x01);
	ret += nm_write_reg(SPI_FLASH_DMA_ADDR, DUMMY_REGISTER);
	ret += nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1<<7));
	do	{
		ret += nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&reg);
		if(M2M_SUCCESS != ret) 
      break;
    }	while(reg != 1);

	reg = (M2M_SUCCESS == ret)?(nm_read_reg(DUMMY_REGISTER)):(0);
	*val = (uint8)(reg & 0xff);
	return ret;
  }

#ifdef DISABLE_UNSED_FLASH_FUNCTIONS
/**
*	@fn			spi_flash_read_security_reg
*	@brief		Read security register
*	@return		Security register value
*/ 
static uint8 spi_flash_read_security_reg(void) {
	uint8	cmd[1];
	uint32	reg;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x2b;

	ret += nm_write_reg(SPI_FLASH_DATA_CNT, 1);
	ret += nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	ret += nm_write_reg(SPI_FLASH_BUF_DIR, 0x01);
	ret += nm_write_reg(SPI_FLASH_DMA_ADDR, DUMMY_REGISTER);
	ret += nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1<<7));
	do	{
		ret += nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&reg);
		if(M2M_SUCCESS != ret) break;
    }	while(reg != 1);
	reg = (M2M_SUCCESS == ret)?(nm_read_reg(DUMMY_REGISTER)):(0);

	return (sint8)reg & 0xff;
  }

/**
*	@fn			spi_flash_gang_unblock
*	@brief		Unblock all flash area
*/ 
static sint8 spi_flash_gang_unblock(void) {
	uint8	cmd[1];
	uint32	val	= 0;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x98;

	ret += nm_write_reg(SPI_FLASH_DATA_CNT, 0);
	ret += nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	ret += nm_write_reg(SPI_FLASH_BUF_DIR, 0x01);
	ret += nm_write_reg(SPI_FLASH_DMA_ADDR, 0);
	ret += nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1<<7));
	do {
		ret += nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&val);
		if(M2M_SUCCESS != ret) break;
    }	while(val != 1);

	return ret;
  }

/**
*	@fn			spi_flash_clear_security_flags
*	@brief		Clear all security flags
*/ 
static sint8 spi_flash_clear_security_flags(void) {
	uint8 cmd[1];
	uint32	val	= 0;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x30;

	ret += nm_write_reg(SPI_FLASH_DATA_CNT, 0);
	ret += nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	ret += nm_write_reg(SPI_FLASH_BUF_DIR, 0x01);
	ret += nm_write_reg(SPI_FLASH_DMA_ADDR, 0);
	ret += nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1<<7));
	do {
		ret += nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&val);
		if(M2M_SUCCESS != ret) 
      break;
    }	while(val != 1);

	return ret;
  }
#endif

/**
*	@fn			spi_flash_load_to_cortus_mem
*	@brief		Load data from SPI flash into cortus memory
*	@param[IN]	u32MemAdr
*					Cortus load address. It must be set to its AHB access address
*	@param[IN]	u32FlashAdr
*					Address to read from at the SPI flash
*	@param[IN]	u32Sz
*					Data size
*	@return		Status of execution
*/ 
static sint8 spi_flash_load_to_cortus_mem(uint32 u32MemAdr, uint32 u32FlashAdr, uint32 u32Sz) {
	uint8 cmd[5];
	uint32	val	= 0;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x0b;
	cmd[1] = (uint8)(u32FlashAdr >> 16);
	cmd[2] = (uint8)(u32FlashAdr >> 8);
	cmd[3] = (uint8)(u32FlashAdr);
	cmd[4] = 0xA5;

	ret += nm_write_reg(SPI_FLASH_DATA_CNT, u32Sz);
	ret += nm_write_reg(SPI_FLASH_BUF1, cmd[0]|(cmd[1]<<8)|(cmd[2]<<16)|(cmd[3]<<24));
	ret += nm_write_reg(SPI_FLASH_BUF2, cmd[4]);
	ret += nm_write_reg(SPI_FLASH_BUF_DIR, 0x1f);
	ret += nm_write_reg(SPI_FLASH_DMA_ADDR, u32MemAdr);
	ret += nm_write_reg(SPI_FLASH_CMD_CNT, 5 | (1<<7));
	do	{
		ret += nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&val);
		if(M2M_SUCCESS != ret) 
      break;
    }	while(val != 1);

	return ret;
  }

/**
*	@fn			spi_flash_sector_erase
*	@brief		Erase sector (4KB)
*	@param[IN]	u32FlashAdr
*					Any memory address within the sector
*	@return		Status of execution
*/ 
static sint8 spi_flash_sector_erase(uint32 u32FlashAdr) {
	uint8 cmd[4];
	uint32	val	= 0;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x20;
	cmd[1] = (uint8)(u32FlashAdr >> 16);
	cmd[2] = (uint8)(u32FlashAdr >> 8);
	cmd[3] = (uint8)(u32FlashAdr);

	ret += nm_write_reg(SPI_FLASH_DATA_CNT, 0);
	ret += nm_write_reg(SPI_FLASH_BUF1, cmd[0]|(cmd[1]<<8)|(cmd[2]<<16)|(cmd[3]<<24));
	ret += nm_write_reg(SPI_FLASH_BUF_DIR, 0x0f);
	ret += nm_write_reg(SPI_FLASH_DMA_ADDR, 0);
	ret += nm_write_reg(SPI_FLASH_CMD_CNT, 4 | (1<<7));
	do	{
		ret += nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&val);
		if(M2M_SUCCESS != ret) 
      break;
    }	while(val != 1);

	return ret;
  }

/**
*	@fn			spi_flash_write_enable
*	@brief		Send write enable command to SPI flash
*	@return		Status of execution
*/ 
static sint8 spi_flash_write_enable(void) {
	uint8 cmd[1];
	uint32	val	= 0;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x06;

	ret += nm_write_reg(SPI_FLASH_DATA_CNT, 0);
	ret += nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	ret += nm_write_reg(SPI_FLASH_BUF_DIR, 0x01);
	ret += nm_write_reg(SPI_FLASH_DMA_ADDR, 0);
	ret += nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1<<7));
	do	{
		ret += nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&val);
		if(M2M_SUCCESS != ret) break;
    }	while(val != 1);

	return ret;
  }

/**
*	@fn			spi_flash_write_disable
*	@brief		Send write disable command to SPI flash
*/
static sint8 spi_flash_write_disable(void) {
	uint8 cmd[1];
	uint32	val	= 0;
	sint8	ret = M2M_SUCCESS;
	cmd[0] = 0x04;

	ret += nm_write_reg(SPI_FLASH_DATA_CNT, 0);
	ret += nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	ret += nm_write_reg(SPI_FLASH_BUF_DIR, 0x01);
	ret += nm_write_reg(SPI_FLASH_DMA_ADDR, 0);
	ret += nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1<<7));
	do {
		ret += nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&val);
		if(M2M_SUCCESS != ret) 
      break;
    }	while(val != 1);

	return ret;
  }

/**
*	@fn			spi_flash_page_program
*	@brief		Write data (less than page size) from cortus memory to SPI flash
*	@param[IN]	u32MemAdr
*					Cortus data address. It must be set to its AHB access address
*	@param[IN]	u32FlashAdr
*					Address to write to at the SPI flash
*	@param[IN]	u32Sz
*					Data size
*/ 
static sint8 spi_flash_page_program(uint32 u32MemAdr, uint32 u32FlashAdr, uint32 u32Sz) {
	uint8 cmd[4];
	uint32	val	= 0;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x02;
	cmd[1] = (uint8)(u32FlashAdr >> 16);
	cmd[2] = (uint8)(u32FlashAdr >> 8);
	cmd[3] = (uint8)(u32FlashAdr);

	ret += nm_write_reg(SPI_FLASH_DATA_CNT, 0);
	ret += nm_write_reg(SPI_FLASH_BUF1, cmd[0]|(cmd[1]<<8)|(cmd[2]<<16)|(cmd[3]<<24));
	ret += nm_write_reg(SPI_FLASH_BUF_DIR, 0x0f);
	ret += nm_write_reg(SPI_FLASH_DMA_ADDR, u32MemAdr);
	ret += nm_write_reg(SPI_FLASH_CMD_CNT, 4 | (1<<7) | ((u32Sz & 0xfffff) << 8));
	do {
		ret += nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&val);
		if(M2M_SUCCESS != ret) 
      break;
    }	while(val != 1);

	return ret;
  }

/**
*	@fn			spi_flash_read_internal
*	@brief		Read from data from SPI flash
*	@param[OUT]	pu8Buf
*					Pointer to data buffer
*	@param[IN]	u32Addr
*					Address to read from at the SPI flash
*	@param[IN]	u32Sz
*					Data size
*/ 
static sint8 spi_flash_read_internal(uint8 *pu8Buf, uint32 u32Addr, uint32 u32Sz) {
	sint8 ret = M2M_SUCCESS;
  
	/* read size must be < 64KB */
	ret = spi_flash_load_to_cortus_mem(HOST_SHARE_MEM_BASE, u32Addr, u32Sz);
	if(M2M_SUCCESS != ret) 
    goto ERR;
	ret = nm_read_block(HOST_SHARE_MEM_BASE, pu8Buf, u32Sz);
ERR:
	return ret;
  }

/**
*	@fn			spi_flash_pp
*	@brief		Program data of size less than a page (256 bytes) at the SPI flash
*	@param[IN]	u32Offset
*					Address to write to at the SPI flash
*	@param[IN]	pu8Buf
*					Pointer to data buffer
*	@param[IN]	u32Sz
*					Data size
*	@return		Status of execution
*/
static sint8 spi_flash_pp(uint32 u32Offset, uint8 *pu8Buf, uint16 u16Sz) {
	sint8 ret = M2M_SUCCESS;
	uint8 tmp;
  
	spi_flash_write_enable();
	/* use shared packet memory as temp mem */
	ret += nm_write_block(HOST_SHARE_MEM_BASE, pu8Buf, u16Sz);
	ret += spi_flash_page_program(HOST_SHARE_MEM_BASE, u32Offset, u16Sz);
	ret += spi_flash_read_status_reg(&tmp);
	do {
		if(ret != M2M_SUCCESS) 
      goto ERR;
		ret += spi_flash_read_status_reg(&tmp);
    } while(tmp & 0x01);
	ret += spi_flash_write_disable();
  
ERR:
	return ret;
  }

/**
*	@fn			spi_flash_rdid
*	@brief		Read SPI Flash ID
*	@return		SPI FLash ID
*/
static uint32 spi_flash_rdid(void) {
	unsigned char cmd[1];
	uint32 reg = 0;
	uint32 cnt = 0;
	sint8	ret = M2M_SUCCESS;

	cmd[0] = 0x9f;

	ret += nm_write_reg(SPI_FLASH_DATA_CNT, 4);
	ret += nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	ret += nm_write_reg(SPI_FLASH_BUF_DIR, 0x1);
	ret += nm_write_reg(SPI_FLASH_DMA_ADDR, DUMMY_REGISTER);
	ret += nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1<<7));
	do {
		ret += nm_read_reg_with_ret(SPI_FLASH_TR_DONE, (uint32 *)&reg);
		if(M2M_SUCCESS != ret) break;
		if(++cnt > 500)	{
			ret = M2M_ERR_INIT;
			break;
      }
    }	while(reg != 1);
	reg = (M2M_SUCCESS == ret)?(nm_read_reg(DUMMY_REGISTER)):(0);
	M2M_PRINT("Flash ID %x \n",(unsigned int)reg);
  
	return reg;
  }

/**
*	@fn			spi_flash_unlock
*	@brief		Unlock SPI Flash
*/
#if 0
static void spi_flash_unlock(void) {
	uint8 tmp;
  
	tmp = spi_flash_read_security_reg();
	spi_flash_clear_security_flags();
	if(tmp & 0x80) {
		spi_flash_write_enable();
		spi_flash_gang_unblock();
	}
}
#endif
static void spi_flash_enter_low_power_mode(void) {
	volatile unsigned long tmp;
	unsigned char* cmd = (unsigned char*) &tmp;

	cmd[0] = 0xb9;

	nm_write_reg(SPI_FLASH_DATA_CNT, 0);
	nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	nm_write_reg(SPI_FLASH_BUF_DIR, 0x1);
	nm_write_reg(SPI_FLASH_DMA_ADDR, 0);
	nm_write_reg(SPI_FLASH_CMD_CNT, 1 | (1 << 7));
	while(nm_read_reg(SPI_FLASH_TR_DONE) != 1)
    CLRWDT();
  }


static void spi_flash_leave_low_power_mode(void) {
	volatile unsigned long tmp;
	unsigned char* cmd = (unsigned char*) &tmp;

	cmd[0] = 0xab;

	nm_write_reg(SPI_FLASH_DATA_CNT, 0);
	nm_write_reg(SPI_FLASH_BUF1, cmd[0]);
	nm_write_reg(SPI_FLASH_BUF_DIR, 0x1);
	nm_write_reg(SPI_FLASH_DMA_ADDR, 0);
	nm_write_reg(SPI_FLASH_CMD_CNT,  1 | (1 << 7));
	while(nm_read_reg(SPI_FLASH_TR_DONE) != 1)
    CLRWDT();
  }

/*********************************************/
/* GLOBAL FUNCTIONS							 */
/*********************************************/
/**
 *	@fn		spi_flash_enable
 *	@brief	Enable spi flash operations
 */
sint8 spi_flash_enable(uint8 enable) {
	sint8 s8Ret = M2M_SUCCESS;
  
	if(REV(nmi_get_chipid()) >= REV_3A0) {		
		uint32 u32Val;
		
		/* Enable pinmux to SPI flash. */
		s8Ret = nm_read_reg_with_ret(0x1410, &u32Val);
		if(s8Ret != M2M_SUCCESS) {
			goto ERR1;
		}
		/* GPIO15/16/17/18 */
		u32Val &= ~((0x7777ul) << 12);
		u32Val |= ((0x1111ul) << 12);
		nm_write_reg(0x1410, u32Val);
		if(enable) {
			spi_flash_leave_low_power_mode();
    	} 
    else {
			spi_flash_enter_low_power_mode();
      }
		/* Disable pinmux to SPI flash to minimize leakage. */
		u32Val &= ~((0x7777ul) << 12);
		u32Val |= ((0x0010ul) << 12);
		nm_write_reg(0x1410, u32Val);
  	}
ERR1:
	return s8Ret;
  }

/**
*	@fn			spi_flash_read
*	@brief		Read from data from SPI flash
*	@param[OUT]	pu8Buf
*					Pointer to data buffer
*	@param[IN]	u32offset
*					Address to read from at the SPI flash
*	@param[IN]	u32Sz
*					Data size
*	@return		Status of execution
*	@note		Data size is limited by the SPI flash size only
*/ 
sint8 spi_flash_read(uint8 *pu8Buf, uint32 u32offset, uint32 u32Sz) {
	sint8 ret = M2M_SUCCESS;
  
	if(u32Sz > FLASH_BLOCK_SIZE) {
		do {
			ret = spi_flash_read_internal(pu8Buf, u32offset, FLASH_BLOCK_SIZE);
			if(M2M_SUCCESS != ret) goto ERR;
			u32Sz -= FLASH_BLOCK_SIZE;
			u32offset += FLASH_BLOCK_SIZE;
			pu8Buf += FLASH_BLOCK_SIZE;
      } while(u32Sz > FLASH_BLOCK_SIZE);
    }
	
	ret = spi_flash_read_internal(pu8Buf, u32offset, u32Sz);

ERR:
	return ret;
  }

/**
*	@fn			spi_flash_write
*	@brief		Proram SPI flash
*	@param[IN]	pu8Buf
*					Pointer to data buffer
*	@param[IN]	u32Offset
*					Address to write to at the SPI flash
*	@param[IN]	u32Sz
*					Data size
*	@return		Status of execution
*/ 
sint8 spi_flash_write(uint8* pu8Buf, uint32 u32Offset, uint32 u32Sz) {
#ifdef PROFILING
	uint32 t1 = 0;
	uint32 percent =0;
	uint32 tpercent =0;
#endif
	sint8 ret = M2M_SUCCESS;
	uint32 u32wsz;
	uint32 u32off;
	uint32 u32Blksz;
	u32Blksz = FLASH_PAGE_SZ;
	u32off = u32Offset % u32Blksz;
#ifdef PROFILING
	tpercent = (u32Sz/u32Blksz)+((u32Sz%u32Blksz)>0);
	t1 = GetTickCount();
	M2M_PRINT(">Start programming...\r\n");
#endif
  
	if(u32Sz<=0)	{
		M2M_ERR("Data size = %d",(int)u32Sz);
		ret = M2M_ERR_FAIL;
		goto ERR;
    }

	if (u32off)/*first part of data in the address page*/
	{
		u32wsz = u32Blksz - u32off;
		if(spi_flash_pp(u32Offset, pu8Buf, (uint16)BSP_MIN(u32Sz, u32wsz)) != M2M_SUCCESS) {
			ret = M2M_ERR_FAIL;
			goto ERR;
		}
		if(u32Sz < u32wsz) 
      goto EXIT;
		pu8Buf += u32wsz;
		u32Offset += u32wsz;
		u32Sz -= u32wsz;
    }
	while(u32Sz > 0)	{
		u32wsz = BSP_MIN(u32Sz, u32Blksz);

		/*write complete page or the remaining data*/
		if(spi_flash_pp(u32Offset, pu8Buf, (uint16)u32wsz) != M2M_SUCCESS) {
			ret = M2M_ERR_FAIL;
			goto ERR;
      }
		pu8Buf += u32wsz;
		u32Offset += u32wsz;
		u32Sz -= u32wsz;
#ifdef PROFILING
		percent++;
		printf("\r>Complete Percentage = %d%%.\r",((percent*100)/tpercent));
#endif
    }
EXIT:
#ifdef PROFILING
	M2M_PRINT("\rDone\t\t\t\t\t\t");
	M2M_PRINT("\n#Programming time = %f sec\n\r",(GetTickCount() - t1)/1000.0);
#endif
ERR:
	return ret;
  }

/**
*	@fn			spi_flash_erase
*	@brief		Erase from data from SPI flash
*	@param[IN]	u32Offset
*					Address to write to at the SPI flash
*	@param[IN]	u32Sz
*					Data size
*	@return		Status of execution
*	@note		Data size is limited by the SPI flash size only
*/ 
sint8 spi_flash_erase(uint32 u32Offset, uint32 u32Sz) {
	uint32 i = 0;
	sint8 ret = M2M_SUCCESS;
	uint8  tmp = 0;
  
#ifdef PROFILING
	uint32 t;
	t = GetTickCount();
#endif
	M2M_PRINT("\r\n>Start erasing...\r\n");
	for(i=u32Offset; i < (u32Sz +u32Offset); i += (16*FLASH_PAGE_SZ))	{
		ret += spi_flash_write_enable();
		ret += spi_flash_read_status_reg(&tmp);
		ret += spi_flash_sector_erase(i + 10);
		ret += spi_flash_read_status_reg(&tmp);
		do {
			if(ret != M2M_SUCCESS) 
        goto ERR;
			ret += spi_flash_read_status_reg(&tmp);
      } while(tmp & 0x01);
    }
	M2M_PRINT("Done\r\n");
#ifdef PROFILING
	M2M_PRINT("#Erase time = %f sec\n", (GetTickCount()-t)/1000.0);
#endif
  
ERR:
	return ret;
  }

/**
*	@fn			spi_flash_get_size
*	@brief		Get size of SPI Flash
*	@return		Size of Flash
*/
uint32 spi_flash_get_size(void) {
	uint32 u32FlashId = 0, u32FlashPwr = 0;
	static uint32 gu32InternalFlashSize= 0;
	
	if(!gu32InternalFlashSize)	{
		u32FlashId = spi_flash_rdid();//spi_flash_probe();
		if(u32FlashId != 0xffffffff) {
			/*flash size is the third byte from the FLASH RDID*/
			u32FlashPwr = ((u32FlashId>>16) & 0xff) - 0x11; /*2MBIT is the min*/
			/*That number power 2 to get the flash size*/
			gu32InternalFlashSize = 1 << u32FlashPwr;
			M2M_INFO("Flash Size %lu Mb\n",gu32InternalFlashSize);
    	}
		else	{
			M2M_ERR("Can't Detect Flash size\n");
      } 
    }

	return gu32InternalFlashSize;
  }


// nm_bus_wrapper_saml21.c
//https://github.com/MicrochipTech/Wireless-Sensor-Network/blob/master/MiWi/ECC%20Provisioning%20SAMR30%20XPRO/ECC608_provisioning/src/ASF/common/components/wifi/winc1500/bus_wrapper/source/nm_bus_wrapper_saml21.c

//#include "bsp/include/nm_bsp.h"
//#include "common/include/nm_common.h"
//#include "bus_wrapper/include/nm_bus_wrapper.h"
//#include "asf.h"
//#include "conf_winc.h"

#define NM_BUS_MAX_TRX_SZ	256

tstrNmBusCapabilities egstrNmBusCapabilities = {
	NM_BUS_MAX_TRX_SZ
  };

#ifdef CONF_WINC_USE_I2C

struct i2c_master_module i2c_master_instance;
#define SLAVE_ADDRESS 0x60

/** Number of times to try to send packet if failed. */
#define I2C_TIMEOUT 100

static sint8 nm_i2c_write(uint8 *b, uint16 sz) {
	sint8 result = M2M_SUCCESS;
	uint16_t timeout = 0;

	struct i2c_master_packet packet = {
		.address     = SLAVE_ADDRESS,
		.data_length = sz,
		.data        = b,
    };

	/* Write buffer to slave until success. */
	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK) {
		/* Increment timeout counter and check if timed out. */
		if (timeout++ == I2C_TIMEOUT) {
			break;
      }
    }

	return result;
  }

static sint8 nm_i2c_read(uint8 *rb, uint16 sz) {
	uint16_t timeout = 0;
	sint8 result = M2M_SUCCESS;
	struct i2c_master_packet packet = {
		.address     = SLAVE_ADDRESS,
		.data_length = sz,
		.data        = rb,
    };

	/* Write buffer to slave until success. */
	while (i2c_master_read_packet_wait(&i2c_master_instance, &packet) != STATUS_OK) {
		/* Increment timeout counter and check if timed out. */
		if(timeout++ == I2C_TIMEOUT) {
			break;
      }
    }

	return result;
  }

static sint8 nm_i2c_write_special(uint8 *wb1, uint16 sz1, uint8 *wb2, uint16 sz2) {
	static uint8 tmp[NM_BUS_MAX_TRX_SZ];
  
	m2m_memcpy(tmp, wb1, sz1);
	m2m_memcpy(&tmp[sz1], wb2, sz2);
	return nm_i2c_write(tmp, sz1+sz2);
  }
#endif

#ifdef CONF_WINC_USE_SPI

//struct spi_module master;
//struct spi_slave_inst slave_inst;

static sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz) {
	uint8 u8Dummy=0;
	uint8 u8SkipMosi=0, u8SkipMiso=0;
	uint16_t txd_data=0;
	uint16_t rxd_data=0;

	if(((!pu8Miso) && (!pu8Mosi)) || (u16Sz == 0)) {
		return M2M_ERR_INVALID_ARG;
    }

	if(!pu8Mosi) {
		pu8Mosi = &u8Dummy;
		u8SkipMosi = 1;
    }
	if(!pu8Miso) {
		pu8Miso = &u8Dummy;
		u8SkipMiso = 1;
    }

	//spi_select_slave(&master, &slave_inst, true);
  m_SPICSBit=0;

	while(u16Sz) {
		txd_data = *pu8Mosi;
    SPI1STATbits.SPIROV = 0;  // Reset overflow bit
//		while (!spi_is_ready_to_write(&master))			;
//		while(spi_write(&master, txd_data) != STATUS_OK)			;
        while(SPI1STATbits.SPITBF)
          CLRWDT();
#warning VERIFICARE ! su HWREMOTE raddoppiava i byte, metteva uno 0...
    SPI1BUF = txd_data;
//    Nop(); Nop();
//    while(SPISTATbits.SPIBUSY == 1 /*(SPISTATLbits.SPITBF == 1)*/ /* || (SPISTATLbits.SPIRBF == 0) */ )
    while(!SPI1STATbits.SPIRBF)
      CLRWDT();
  //  Nop();    // boh sì

//    __delay_us(5);
    
    
		/* Read SPI master data register. */
//		while (!spi_is_ready_to_read(&master))			;
//		while (spi_read(&master, &rxd_data) != STATUS_OK)			;
		*pu8Miso = SPI1BUF;

		u16Sz--;
		if(!u8SkipMiso)
			pu8Miso++;
		if(!u8SkipMosi)
			pu8Mosi++;
      }

//	while (!spi_is_write_complete(&master))		;

//	spi_select_slave(&master, &slave_inst, false);
  m_SPICSBit=1;

	return M2M_SUCCESS;
  }
#endif

/*
*	@fn		nm_bus_init
*	@brief	Initialize the bus wrapper
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_bus_init(void *pvinit) {
	sint8 result = M2M_SUCCESS;
  uint8 spiMode=0;

#ifdef CONF_WINC_USE_I2C
	/* Initialize config structure and software module. */
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);

	/* Change buffer timeout to something longer. */
	config_i2c_master.buffer_timeout = 1000;

	/* Initialize and enable device with config. */
	i2c_master_init(&i2c_master_instance, SERCOM2, &config_i2c_master);

	i2c_master_enable(&i2c_master_instance);

#elif defined CONF_WINC_USE_SPI
	/* Structure for SPI configuration. */
//	struct spi_config config;
//	struct spi_slave_inst_config slave_config;

	/* Select SPI slave CS pin. */
	/* This step will set the CS high */
//	spi_slave_inst_get_config_defaults(&slave_config);
//	slave_config.ss_pin = CONF_WINC_SPI_CS_PIN;
//	spi_attach_slave(&slave_inst, &slave_config);

	/* Configure the SPI master. */
//	spi_get_config_defaults(&config);
//	config.mux_setting = CONF_WINC_SPI_SERCOM_MUX;
//	config.pinmux_pad0 = CONF_WINC_SPI_PINMUX_PAD0;
//	config.pinmux_pad1 = CONF_WINC_SPI_PINMUX_PAD1;
//	config.pinmux_pad2 = CONF_WINC_SPI_PINMUX_PAD2;
//	config.pinmux_pad3 = CONF_WINC_SPI_PINMUX_PAD3;
//	config.master_slave_select_enable = false;

//	config.mode_specific.master.baudrate = CONF_WINC_SPI_CLOCK;
//	if(spi_init(&master, CONF_WINC_SPI_MODULE, &config) != STATUS_OK) {
//		return M2M_ERR_BUS_FAIL;
//   }

	SPI1CONbits.ON=0;
	SPISDITris=1;				// SDI è input
	SPISDOTris=0;				// SDO è output
	SPISCKTris=0;			// SCK è output (qua è fisso cmq)
  m_WCS=1;		//
  SPIWCSTris=0;		// CS è output

  CNPUDbits.CNPUD2=1; // pullup su MISO... può servire
  
//  SPI1CON= 0b00000000000000000000000000000000;    // no special length; no audio; SPITUREN irq per dma?
  
  switch(spiMode) {		// CKP=b6=CPOL=0 se idle basso/1 se idle alto; CKE=b8=0 se DOut cambia quando idle->active, 1 se viceversa
/*Mode CPOL CPHA
    0   0   0
    1   0   1
    2   1   0
    3   1   1 */
		// ci sarebbe pure il b9 che è sample a metà o alla fine...
    case 0:
      SPI1CON= 0b00000000000000000000000100100000;    // master, mode 0 (AT WINC 1500 vuole questo, https://www.avrfreaks.net/sites/default/files/forum_attachments/Atmel-XXXXX-WINC1500-SPI_Porting_Guide.pdf )
      // parrebbero invertiti, v. SD spi.. ma invece no, boh 2021
      break;
    case 1:
      SPI1CON= 0b00000000000000000000000000100000;    //no fancy stuff
      break;
    case 2:
      SPI1CON= 0b00000000000000000000000101100000;    //
      break;
    case 3:
      SPI1CON= 0b00000000000000000000000001100000;    // CKP = 1, CKE = 0
      break;
    }
  SPI1CONbits.STXISEL=0b00;     // tx irq
  
  SPI1STAT= 0b00000000000000000000000000000000;    // 
	SPI1BRG = ((GetPeripheralClock()/10000000UL)/2)-1;			// 12=3.3MHz; 27/9/21 (100 : (2*(12+1)))
    
	/* Enable the SPI master. */
	SPI1CONbits.ON=1;

	nm_bsp_reset();
	nm_bsp_sleep(1);
#endif
  
	return result;
  }

/*
*	@fn		nm_bus_ioctl
*	@brief	send/receive from the bus
*	@param[IN]	u8Cmd
*					IOCTL command for the operation
*	@param[IN]	pvParameter
*					Arbitrary parameter depenging on IOCTL
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@note	For SPI only, it's important to be able to send/receive at the same time
*/
sint8 nm_bus_ioctl(uint8 u8Cmd, void* pvParameter) {
	sint8 s8Ret = 0;
  
	switch(u8Cmd)	{
#ifdef CONF_WINC_USE_I2C
		case NM_BUS_IOCTL_R: 
    {
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *)pvParameter;
			s8Ret = nm_i2c_read(pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
		case NM_BUS_IOCTL_W: 
    {
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *)pvParameter;
			s8Ret = nm_i2c_write(pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
		case NM_BUS_IOCTL_W_SPECIAL: 
    {
			tstrNmI2cSpecial *pstrParam = (tstrNmI2cSpecial *)pvParameter;
			s8Ret = nm_i2c_write_special(pstrParam->pu8Buf1, pstrParam->u16Sz1, pstrParam->pu8Buf2, pstrParam->u16Sz2);
		}
		break;
#elif defined CONF_WINC_USE_SPI
		case NM_BUS_IOCTL_RW: 
    {
			tstrNmSpiRw *pstrParam = (tstrNmSpiRw *)pvParameter;
			s8Ret = spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
		}
		break;
#endif
		default:
			s8Ret = -1;
			M2M_ERR("invalid ioctl cmd\n");
			break;
    }

	return s8Ret;
  }

/*
*	@fn		nm_bus_deinit
*	@brief	De-initialize the bus wrapper
*/
sint8 nm_bus_deinit(void) {
	sint8 result = M2M_SUCCESS;
//	struct port_config pin_conf;
		
//	port_get_config_defaults(&pin_conf);
	/* Configure control pins as input no pull up. */
//	pin_conf.direction  = PORT_PIN_DIR_INPUT;
//	pin_conf.input_pull = PORT_PIN_PULL_NONE;
  CNPUBbits.CNPUB14=0; // pullup su MISO

#ifdef CONF_WINC_USE_I2C
	i2c_master_disable(&i2c_master_instance);
#endif /* CONF_WINC_USE_I2C */
#ifdef CONF_WINC_USE_SPI
//	spi_disable(&master);
  SPI1CONbits.ON=0;
#endif /* CONF_WINC_USE_SPI */
  
	return result;
  }

/*
*	@fn			nm_bus_reinit
*	@brief		re-initialize the bus wrapper
*	@param [in]	void *config
*					re-init configuration data
*	@return		M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author		Dina El Sissy
*	@date		19 Sept 2012
*	@version	1.0
*/
sint8 nm_bus_reinit(void* config) {
  
	return M2M_SUCCESS;
  }

nm_warm_reset(void) {
  
  spi_cmd(CMD_RESET, 0, 0, 0, 0);
  }
