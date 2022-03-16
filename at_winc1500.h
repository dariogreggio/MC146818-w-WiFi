// AT WINC 1500 includes by G.Dar, 
// * based upon shit/crap from harmony github and more MERDAAAAA ;) #cancroagliumani #morteaibambini 2021
// https://documentation.help/WINC1500/group___wlan_defines.html
// https://asf.microchip.com/docs/latest/common.components.wifi.winc1500.simple_udp_server_example.saml21_xplained_pro_b/html/nmasic_8h.html
// GD extensions may 21 sempre cancro all'itaglia ;)

#include <xc.h>
//#include "compiler.h"
//#include "io_cfg.h"


#ifndef _ATWINC1500_H_
#define _ATWINC1500_H_


#ifdef __cplusplus
    extern "C" {
#endif

         
#define INADDR_ANY "0.0.0.0"
#define INADDR_BROADCAST MAKELONG(MAKEWORD(192,168),MAKEWORD(1,255))
#define INVALID_SOCKET (-1)
#define DNS_TIMEOUT 2000

    
enum SOCKET_EVENT {
    SOCKET_EVENT_RX_READY=1,
    SOCKET_EVENT_TX_READY=2,
    SOCKET_EVENT_TX_ACKED=4,
    SOCKET_EVENT_TX_SHUTDOWN=8,
    SOCKET_EVENT_RX_SHUTDOWN=16,
    SOCKET_EVENT_CLOSE=32
    };

#define SOCKET_SD_RECEIVE 1
#define SOCKET_SD_SEND 2
         
/**@defgroup  DataT  DataTypes
 * @ingroup nm_bsp
 * @{
 */

  /*!
 * @ingroup DataTypes
 * @typedef      unsigned char uint8;
 * @brief        Range of values between 0 to 255
 */
typedef unsigned char	uint8;

 /*!
 * @ingroup DataTypes
 * @typedef      unsigned short	uint16;
 * @brief        Range of values between 0 to 65535
 */
typedef unsigned short	uint16;

 /*!
 * @ingroup Data Types
 * @typedef      unsigned long	uint32;
 * @brief        Range of values between 0 to 4294967295
 */
typedef unsigned long	uint32;


  /*!
 * @ingroup Data Types
 * @typedef      signed char		sint8;
 * @brief        Range of values between -128 to 127
 */
typedef signed char		sint8;

 /*!
 * @ingroup DataTypes
 * @typedef      signed short	sint16;
 * @brief        Range of values between -32768 to 32767
 */
typedef signed short	sint16;

  /*!
 * @ingroup DataTypes
 * @typedef      signed long		sint32;
 * @brief        Range of values between -2147483648 to 2147483647
 */

typedef signed long		sint32;
 //@}

#define true 1
#define false 0
#define STATUS_OK 1
#warning VERIFICARE quanto vale STATUS_OK

typedef union {
    uint8 addr[4];
    uint32 ip;
    } Ipv4Addr;
typedef uint16 ipv4port;


enum { 
    SD_RECEIVE=0,
    SD_SEND,
    SD_BOTH
    };

    
    
// nmasic.h *************************************************************************************************


//#include "common/include/nm_common.h"

#define NMI_PERIPH_REG_BASE     0x1000
#define NMI_CHIPID	            (NMI_PERIPH_REG_BASE)
#define rNMI_GP_REG_0			(0x149c)
#define rNMI_GP_REG_1			(0x14A0)
#define rNMI_GP_REG_2			(0xc0008)
#define rNMI_GLB_RESET			(0x1400)
#define rNMI_BOOT_RESET_MUX		(0x1118)
#define NMI_STATE_REG			(0x108c)
#define BOOTROM_REG				(0xc000c)
#define NMI_REV_REG  			(0x207ac)	/*Also, Used to load ATE firmware from SPI Flash and to ensure that it is running too*/
#define NMI_REV_REG_ATE			(0x1048) 	/*Revision info register in case of ATE FW*/
#define M2M_WAIT_FOR_HOST_REG 	(0x207bc)
#define M2M_FINISH_INIT_STATE 	0x02532636UL
#define M2M_FINISH_BOOT_ROM   	 0x10add09eUL
#define M2M_START_FIRMWARE   	 0xef522f61UL
#define M2M_START_PS_FIRMWARE    0x94992610UL

#define M2M_ATE_FW_START_VALUE	(0x3C1CD57D)	/*Also, Change this value in boot_firmware if it will be changed here*/
#define M2M_ATE_FW_IS_UP_VALUE	(0xD75DC1C3)	/*Also, Change this value in ATE (Burst) firmware if it will be changed here*/

#define REV_2B0        (0x2B0)
#define REV_B0         (0x2B0)
#define REV_3A0        (0x3A0)
#define GET_CHIPID()	nmi_get_chipid()
#define ISNMC1000(id)   (((id & 0xfffff000) == 0x100000) ? 1 : 0)
#define ISNMC1500(id)   (((id & 0xfffff000) == 0x150000) ? 1 : 0)
#define REV(id)         ( ((id) & 0x00000fff ) )
#define EFUSED_MAC(value) (value & 0xffff0000)

#define rHAVE_SDIO_IRQ_GPIO_BIT     (NBIT0)
#define rHAVE_USE_PMU_BIT           (NBIT1)
#define rHAVE_SLEEP_CLK_SRC_RTC_BIT (NBIT2)
#define rHAVE_SLEEP_CLK_SRC_XO_BIT  (NBIT3)
#define rHAVE_EXT_PA_INV_TX_RX      (NBIT4)
#define rHAVE_LEGACY_RF_SETTINGS    (NBIT5)
#define rHAVE_LOGS_DISABLED_BIT		(NBIT6)
#define rHAVE_ETHERNET_MODE_BIT		(NBIT7)

typedef struct{
	uint32 u32Mac_efuse_mib;
	uint32 u32Firmware_Ota_rev;
    } tstrGpRegs;

/**
*	@fn		nm_clkless_wake
*	@brief	Wakeup the chip using clockless registers
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author	Samer Sarhan
*/
sint8 nm_clkless_wake(void);
sint8 chip_wake(void);
void chip_idle(void);
void enable_rf_blocks(void);
sint8 enable_interrupts(void);
sint8 cpu_start(void);
uint32 nmi_get_chipid(void);
uint32 nmi_get_rfrevid(void);
void restore_pmu_settings_after_global_reset(void);
void nmi_update_pll(void);
void nmi_set_sys_clk_src_to_xo(void);
sint8 chip_reset(void);
sint8 wait_for_bootrom(uint8);
sint8 wait_for_firmware_start(uint8);
sint8 chip_deinit(void);
sint8 chip_reset_and_cpu_halt(void);
sint8 set_gpio_dir(uint8 gpio, uint8 dir);
sint8 set_gpio_val(uint8 gpio, uint8 val);
sint8 get_gpio_val(uint8 gpio, uint8* val);
sint8 pullup_ctrl(uint32 pinmask, uint8 enable);
sint8 nmi_get_otp_mac_address(uint8 *pu8MacAddr, uint8 * pu8IsValid);
sint8 nmi_get_mac_address(uint8 *pu8MacAddr);
sint8 chip_apply_conf(uint32 u32conf);


#endif	/*_NMASIC_H_*/


// nm_bsp_internal.h *************************************************************************************************

#ifndef _NM_BSP_INTERNAL_H_
#define _NM_BSP_INTERNAL_H_



#ifdef WIN32
#include "nm_bsp_win32.h"
#endif

#ifdef __K20D50M__
#include "nm_bsp_k20d50m.h"
#endif

#ifdef __MSP430FR5739__
#include "bsp_msp430fr5739.h"
#endif

#ifdef _FREESCALE_MCF51CN128_
#include "bsp/include/nm_bsp_mcf51cn128.h"
#endif

#ifdef __MCF964548__
#include "bsp/include/nm_bsp_mc96f4548.h"
#endif

#ifdef __APP_APS3_CORTUS__
#include "nm_bsp_aps3_cortus.h"
#endif

#if (defined __SAML21J18A__) || (defined __SAML21J18B__)
#include "bsp/include/nm_bsp_saml21.h"
#endif

#if (defined __SAMD21J18A__) || (defined __SAMD21G18A__)
#include "bsp/include/nm_bsp_samd21.h"
#endif

#if (defined __SAM4S16C__) || (defined __SAM4SD32C__)
#include "bsp/include/nm_bsp_sam4s.h"
#endif

#ifdef __SAMG53N19__
#include "bsp/include/nm_bsp_samg53.h"
#endif

#ifdef __SAMG55J19__
#include "bsp/include/nm_bsp_samg55.h"
#endif

#ifdef CORTUS_APP
#include "crt_iface.h"
#endif

#ifdef NRF51
#include "nm_bsp_nrf51822.h"
#endif

#ifdef _ARDUINO_UNO_
#include "bsp/include/nm_bsp_arduino_uno.h"
#endif


#endif //_NM_BSP_INTERNAL_H_



// nm_bsp.h *************************************************************************************************

#ifndef _NM_BSP_H_
#define _NM_BSP_H_

#define NMI_API
/*!< 
*        Attribute used to define memory section to map Functions in host memory.
*/
#define CONST const

/*!< 
*     Used for code portability.
*/

/*!
 * @typedef      void (*tpfNmBspIsr) (void);
 * @brief           Pointer to function.\n
 *                     Used as a data type of ISR function registered by \ref nm_bsp_register_isr
 * @return         None
 */
typedef void (*tpfNmBspIsr)(void);



#ifndef NULL
#define NULL ((void*)0)
#endif
/*!<
*    Void Pointer to '0' in case of NULL is not defined.
*/


#define BSP_MIN(x,y) ((x)>(y)?(y):(x))
/*!<
*     Computes the minimum of \b x and \b y.
*/

 //@}


#ifndef CORTUS_APP


/** \defgroup BSPAPI Function
 *   @ingroup nm_bsp
 */


/** @defgroup NmBspInitFn nm_bsp_init
 *  @ingroup BSPAPI
 *  Initialization for BSP such as Reset and Chip Enable Pins for WINC, delays, register ISR, enable/disable IRQ for WINC, ...etc. You must use this function in the head of your application to
 *  enable WINC and Host Driver communicate each other.
 */
 /**@{*/
/*!
 * @fn           sint8 nm_bsp_init(void);
 * @note         Implementation of this function is host dependent.
 * @warning      Missing use will lead to unavailability of host communication.\n
 *
 * @return       The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.

 */
sint8 nm_bsp_init(void);
 /**@}*/


 /** @defgroup NmBspDeinitFn nm_bsp_deinit
 *    @ingroup BSPAPI
 *   	 De-initialization for BSP (\e Board \e Support \e Package)
 */
 /**@{*/
/*!
 * @fn           sint8 nm_bsp_deinit(void);
 * @pre          Initialize \ref nm_bsp_init first
 * @note         Implementation of this function is host dependent.
 * @warning      Missing use may lead to unknown behavior in case of soft reset.\n
 * @see          nm_bsp_init
 * @return      The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.

 */
sint8 nm_bsp_deinit(void);
 /**@}*/


/** @defgroup NmBspResetFn  nm_bsp_reset
*     @ingroup BSPAPI
*      Resetting NMC1500 SoC by setting CHIP_EN and RESET_N signals low, then after specific delay the function will put CHIP_EN high then RESET_N high,
*      for the timing between signals please review the WINC data-sheet
*/
/**@{*/
 /*!
 * @fn           void nm_bsp_reset(void);
 * @param [in]   None
 * @pre          Initialize \ref nm_bsp_init first
 * @note         Implementation of this function is host dependent and called by HIF layer.
 * @see          nm_bsp_init
 * @return       None

 */
void nm_bsp_reset(void);
 /**@}*/


/** @defgroup NmBspSleepFn nm_bsp_sleep
*     @ingroup BSPAPI
*     Sleep in units of milliseconds.\n
*    This function used by HIF Layer according to different situations.
*/
/**@{*/
/*!
 * @fn           void nm_bsp_sleep(uint32);
 * @brief
 * @param [in]   u32TimeMsec
 *               Time unit in milliseconds
 * @pre          Initialize \ref nm_bsp_init first
 * @warning      Maximum value must nor exceed 4294967295 milliseconds which is equal to 4294967.295 seconds.\n
 * @note         Implementation of this function is host dependent.
 * @see           nm_bsp_init
 * @return       None
 */
void nm_bsp_sleep(uint32 u32TimeMsec);
/**@}*/


/** @defgroup NmBspRegisterFn nm_bsp_register_isr
*     @ingroup BSPAPI
*   Register ISR (Interrupt Service Routine) in the initialization of HIF (Host Interface) Layer.
*   When the interrupt trigger the BSP layer should call the pfisr function once inside the interrupt.
*/
/**@{*/
/*!
 * @fn           void nm_bsp_register_isr(tpfNmBspIsr);
 * @param [in]   tpfNmBspIsr  pfIsr
 *               Pointer to ISR handler in HIF
 * @warning      Make sure that ISR for IRQ pin for WINC is disabled by default in your implementation.
 * @note         Implementation of this function is host dependent and called by HIF layer.
 * @see          tpfNmBspIsr
 * @return       None

 */
void nm_bsp_register_isr(tpfNmBspIsr pfIsr);
/**@}*/


/** @defgroup NmBspInterruptCtrl nm_bsp_interrupt_ctrl
*     @ingroup BSPAPI
*    Synchronous enable/disable interrupts function
*/
/**@{*/
/*!
 * @fn           void nm_bsp_interrupt_ctrl(uint8);
 * @brief        Enable/Disable interrupts
 * @param [in]   u8Enable
 *               '0' disable interrupts. '1' enable interrupts 
 * @see          tpfNmBspIsr           
 * @note         Implementation of this function is host dependent and called by HIF layer.
 * @return       None

 */
void nm_bsp_interrupt_ctrl(uint8 u8Enable);
  /**@}*/


#endif

#ifdef _NM_BSP_BIG_END
#define NM_BSP_B_L_32(x) \
((((x) & 0x000000FF) << 24) + \
(((x) & 0x0000FF00) << 8)  + \
(((x) & 0x00FF0000) >> 8)   + \
(((x) & 0xFF000000) >> 24))
#define NM_BSP_B_L_16(x) \
((((x) & 0x00FF) << 8) + \
(((x)  & 0xFF00) >> 8))
#else
#define NM_BSP_B_L_32(x)  (x)
#define NM_BSP_B_L_16(x)  (x)
#endif


#endif	/*_NM_BSP_H_*/


// nm_bsp_samg55.h *************************************************************************************************

#ifndef _NM_BSP_SAMG55_H_
#define _NM_BSP_SAMG55_H_

//#include "config/conf_winc.h"

#define NM_EDGE_INTERRUPT		(1)

#define NM_DEBUG				CONF_WINC_DEBUG
#define NM_BSP_PRINTF			CONF_WINC_PRINTF

#endif /* _NM_BSP_SAMG55_H_ */


// nm_bus_wrapper.h *************************************************************************************************

#ifndef _NM_BUS_WRAPPER_H_
#define _NM_BUS_WRAPPER_H_

//#include "common/include/nm_common.h"

/**
	BUS Type
**/
#define  NM_BUS_TYPE_I2C	((uint8)0)
#define  NM_BUS_TYPE_SPI	((uint8)1)
#define  NM_BUS_TYPE_UART	((uint8)2)
/**
	IOCTL commands
**/
#define NM_BUS_IOCTL_R			((uint8)0)	/*!< Read only ==> I2C/UART. Parameter:tstrNmI2cDefault/tstrNmUartDefault */
#define NM_BUS_IOCTL_W			((uint8)1)	/*!< Write only ==> I2C/UART. Parameter type tstrNmI2cDefault/tstrNmUartDefault*/
#define NM_BUS_IOCTL_W_SPECIAL	((uint8)2)	/*!< Write two buffers within the same transaction
												(same start/stop conditions) ==> I2C only. Parameter:tstrNmI2cSpecial */
#define NM_BUS_IOCTL_RW			((uint8)3)	/*!< Read/Write at the same time ==> SPI only. Parameter:tstrNmSpiRw */

#define NM_BUS_IOCTL_WR_RESTART	((uint8)4)				/*!< Write buffer then made restart condition then read ==> I2C only. parameter:tstrNmI2cSpecial */
/**
*	@struct	tstrNmBusCapabilities
*	@brief	Structure holding bus capabilities information
*	@sa	NM_BUS_TYPE_I2C, NM_BUS_TYPE_SPI
*/
typedef struct {
	uint16	u16MaxTrxSz;	/*!< Maximum transfer size. Must be >= 16 bytes*/
    } tstrNmBusCapabilities;

/**
*	@struct	tstrNmI2cDefault
*	@brief	Structure holding I2C default operation parameters
*	@sa		NM_BUS_IOCTL_R, NM_BUS_IOCTL_W
*/
typedef struct {
	uint8 u8SlaveAdr;
	uint8	*pu8Buf;	/*!< Operation buffer */
	uint16	u16Sz;		/*!< Operation size */
    } tstrNmI2cDefault;

/**
*	@struct	tstrNmI2cSpecial
*	@brief	Structure holding I2C special operation parameters
*	@sa		NM_BUS_IOCTL_W_SPECIAL
*/
typedef struct {
	uint8 u8SlaveAdr;
	uint8	*pu8Buf1;	/*!< pointer to the 1st buffer */
	uint8	*pu8Buf2;	/*!< pointer to the 2nd buffer */
	uint16	u16Sz1;		/*!< 1st buffer size */
	uint16	u16Sz2;		/*!< 2nd buffer size */
    } tstrNmI2cSpecial;

/**
*	@struct	tstrNmSpiRw
*	@brief	Structure holding SPI R/W parameters
*	@sa		NM_BUS_IOCTL_RW
*/
typedef struct {
	uint8	*pu8InBuf;		/*!< pointer to input buffer.
							Can be set to null and in this case zeros should be sent at MOSI */
	uint8	*pu8OutBuf;		/*!< pointer to output buffer.
							Can be set to null and in this case data from MISO can be ignored  */
	uint16	u16Sz;			/*!< Transfere size */
    } tstrNmSpiRw;


/**
*	@struct	tstrNmUartDefault
*	@brief	Structure holding UART default operation parameters
*	@sa		NM_BUS_IOCTL_R, NM_BUS_IOCTL_W
*/
typedef struct {
	uint8	*pu8Buf;	/*!< Operation buffer */
	uint16	u16Sz;		/*!< Operation size */
    } tstrNmUartDefault;
/*!< Bus capabilities. This structure must be declared at platform specific bus wrapper */
extern tstrNmBusCapabilities egstrNmBusCapabilities;


/**
*	@fn		nm_bus_init
*	@brief	Initialize the bus wrapper
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_bus_init(void *);

/**
*	@fn		nm_bus_ioctl
*	@brief	send/receive from the bus
*	@param [in]	u8Cmd
*					IOCTL command for the operation
*	@param [in]	pvParameter
*					Arbitrary parameter depending on IOCTL
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@note	For SPI only, it's important to be able to send/receive at the same time
*/
sint8 nm_bus_ioctl(uint8 u8Cmd, void* pvParameter);

/**
*	@fn		nm_bus_deinit
*	@brief	De-initialize the bus wrapper
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_bus_deinit(void);

/*
*	@fn			nm_bus_reinit
*	@brief		re-initialize the bus wrapper
*	@param [in]	void *config
*					re-init configuration data
*	@return		ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_bus_reinit(void *);
/*
*	@fn			nm_bus_get_chip_type
*	@brief		get chip type
*	@return		ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
#ifdef CONF_WINC_USE_UART
uint8 nm_bus_get_chip_type(void);
#endif

#endif	/*_NM_BUS_WRAPPER_H_*/

// nm_common.h *************************************************************************************************

#ifndef _NM_COMMON_H_
#define _NM_COMMON_H_

//#include "bsp/include/nm_bsp.h"
//#include "common/include/nm_debug.h"

/**@defgroup  CommonDefines CommonDefines
 * @ingroup WlanDefines
 */
/**@{*/
#define M2M_TIME_OUT_DELAY 10000

/*states*/
#define M2M_SUCCESS         ((sint8)0)
#define M2M_ERR_SEND		((sint8)-1)
#define M2M_ERR_RCV			((sint8)-2)
#define M2M_ERR_MEM_ALLOC	((sint8)-3)
#define M2M_ERR_TIME_OUT	((sint8)-4)
#define M2M_ERR_INIT        ((sint8)-5)
#define M2M_ERR_BUS_FAIL    ((sint8)-6)
#define M2M_NOT_YET			((sint8)-7)
#define M2M_ERR_FIRMWARE	((sint8)-8)
#define M2M_SPI_FAIL		((sint8)-9)
#define M2M_ERR_FIRMWARE_bURN	 ((sint8)-10)
#define M2M_ACK				((sint8)-11)
#define M2M_ERR_FAIL		((sint8)-12)
#define M2M_ERR_FW_VER_MISMATCH         ((sint8)-13)
#define M2M_ERR_SCAN_IN_PROGRESS         ((sint8)-14)
/*
Invalid argument
*/
#define M2M_ERR_INVALID_ARG				 ((sint8)-15)

/*i2c MAASTER ERR*/
#define I2C_ERR_LARGE_ADDRESS 	  0xE1UL	/*the address exceed the max addressing mode in i2c flash*/
#define I2C_ERR_TX_ABRT 		  0xE2UL	/*NO ACK from slave*/
#define I2C_ERR_OVER_SIZE 		  0xE3UL	/**/
#define ERR_PREFIX_NMIS		      0xE4UL	/*wrong first four byte in flash NMIS*/
#define ERR_FIRMEWARE_EXCEED_SIZE 0xE5UL	/*Total size of firmware exceed the max size 256k*/
/**/
#define PROGRAM_START		0x26961735UL
#define BOOT_SUCCESS		0x10add09eUL
#define BOOT_START		    0x12345678UL


#define NBIT31				(0x80000000)
#define NBIT30				(0x40000000)
#define NBIT29				(0x20000000)
#define NBIT28				(0x10000000)
#define NBIT27				(0x08000000)
#define NBIT26				(0x04000000)
#define NBIT25				(0x02000000)
#define NBIT24				(0x01000000)
#define NBIT23				(0x00800000)
#define NBIT22				(0x00400000)
#define NBIT21				(0x00200000)
#define NBIT20				(0x00100000)
#define NBIT19				(0x00080000)
#define NBIT18				(0x00040000)
#define NBIT17				(0x00020000)
#define NBIT16				(0x00010000)
#define NBIT15				(0x00008000)
#define NBIT14				(0x00004000)
#define NBIT13				(0x00002000)
#define NBIT12				(0x00001000)
#define NBIT11				(0x00000800)
#define NBIT10				(0x00000400)
#define NBIT9				(0x00000200)
#define NBIT8				(0x00000100)
#define NBIT7				(0x00000080)
#define NBIT6				(0x00000040)
#define NBIT5				(0x00000020)
#define NBIT4				(0x00000010)
#define NBIT3				(0x00000008)
#define NBIT2				(0x00000004)
#define NBIT1				(0x00000002)
#define NBIT0				(0x00000001)

#define M2M_MAX(A,B)					((A) > (B) ? (A) : (B))
#define M2M_SEL(x,m1,m2,m3)				((x>1)?((x>2)?(m3):(m2)):(m1))
#define WORD_ALIGN(val) 				(((val) & 0x03) ? ((val) + 4 - ((val) & 0x03)) : (val))



#define DATA_PKT_OFFSET	4

#ifndef BIG_ENDIAN
#define BYTE_0(word)   					((uint8)(((word) >> 0 	) & 0x000000FFUL))
#define BYTE_1(word)  	 				((uint8)(((word) >> 8 	) & 0x000000FFUL))
#define BYTE_2(word)   					((uint8)(((word) >> 16) & 0x000000FFUL))
#define BYTE_3(word)   					((uint8)(((word) >> 24) & 0x000000FFUL))
#else
#define BYTE_0(word)   					((uint8)(((word) >> 24) & 0x000000FFUL))
#define BYTE_1(word)  	 				((uint8)(((word) >> 16) & 0x000000FFUL))
#define BYTE_2(word)   					((uint8)(((word) >> 8 	) & 0x000000FFUL))
#define BYTE_3(word)   					((uint8)(((word) >> 0 	) & 0x000000FFUL))
#endif

/**@}*/
NMI_API void m2m_memcpy(uint8* pDst,uint8* pSrc,uint32 sz);
NMI_API void m2m_memset(uint8* pBuf,uint8 val,uint32 sz);
NMI_API uint16 m2m_strlen(uint8 * pcStr);
NMI_API sint8 m2m_memcmp(uint8 *pu8Buff1,uint8 *pu8Buff2 ,uint32 u32Size);
NMI_API uint8 m2m_strncmp(uint8 *pcS1, uint8 *pcS2, uint16 u16Len);
NMI_API uint8 *m2m_strstr(uint8 *pcIn, uint8 *pcStr);
NMI_API uint8 m2m_checksum(uint8* buf, int sz);

#endif	/*_NM_COMMON_H_*/


// nm_drv.h *************************************************************************************************

#ifndef _NMDRV_H_
#define _NMDRV_H_

//#include "common/include/nm_common.h"

/**
*  @struct		tstrM2mRev
*  @brief		Structure holding firmware version parameters and build date/time
*/
typedef struct {
	uint32 u32Chipid; /* HW revision which will be basically the chip ID */
	uint8 u8FirmwareMajor; /* Version Major Number which represents the official release base */
	uint8 u8FirmwareMinor; /* Version Minor Number which represents the engineering release base */
	uint8 u8FirmwarePatch;	/* Version patch Number which represents the patches release base */
	uint8 u8DriverMajor; /* Version Major Number which represents the official release base */
	uint8 u8DriverMinor; /* Version Minor Number which represents the engineering release base */
	uint8 u8DriverPatch; /* Version Patch Number which represents the patches release base */
	uint8 BuildDate[sizeof(__DATE__)];
	uint8 BuildTime[sizeof(__TIME__)];
	uint8 _PAD8_;
    } tstrM2mRev;

/**
*	@fn		nm_get_firmware_info(tstrM2mRev* M2mRev)
*	@brief	Get Firmware version info
*	@param [out]	M2mRev
*			    pointer holds address of structure "tstrM2mRev" that contains the firmware version parameters
*	@version	1.0
*/
sint8 nm_get_firmware_info(tstrM2mRev* M2mRev);
/**
*	@fn		nm_get_firmware_full_info(tstrM2mRev* pstrRev)
*	@brief	Get Firmware version info
*	@param [out]	M2mRev
*			    pointer holds address of structure "tstrM2mRev" that contains the firmware version parameters
*	@version	1.0
*/
sint8 nm_get_firmware_full_info(tstrM2mRev* pstrRev);
/**
*	@fn		nm_get_ota_firmware_info(tstrM2mRev* pstrRev)
*	@brief	Get Firmware version info
*	@param [out]	M2mRev
*			    pointer holds address of structure "tstrM2mRev" that contains the firmware version parameters
			
*	@version	1.0
*/
sint8 nm_get_ota_firmware_info(tstrM2mRev* pstrRev);
/*
*	@fn		nm_drv_init
*	@brief	Initialize NMC1000 driver
*	@return	ZERO in case of success and Negative error code in case of failure
*/
sint8 nm_drv_init_download_mode(void);

/*
*	@fn		nm_drv_init
*	@brief	Initialize NMC1000 driver
*	@return	M2M_SUCCESS in case of success and Negative error code in case of failure
*   @param [in]	arg
*				Generic argument TBD
*	@return	ZERO in case of success and Negative error code in case of failure

*/
sint8 nm_drv_init(void * arg);

/**
*	@fn		nm_drv_deinit
*	@brief	Deinitialize NMC1000 driver
*	@author	M. Abdelmawla
*   @param [in]	arg
*				Generic argument TBD
*	@return	ZERO in case of success and Negative error code in case of failure
*/
sint8 nm_drv_deinit(void * arg);


#endif	/*_NMDRV_H_*/



// nm_debug.h *************************************************************************************************

#ifndef _NM_DEBUG_H_
#define _NM_DEBUG_H_

//#include "bsp/include/nm_bsp.h"
//#include "bsp/include/nm_bsp_internal.h"

/**@defgroup  DebugDefines DebugDefines
 * @ingroup WlanDefines
 */
/**@{*/


#define M2M_LOG_NONE									0
#define M2M_LOG_ERROR									1
#define M2M_LOG_INFO									2
#define M2M_LOG_REQ										3
#define M2M_LOG_DBG										4

#if (defined __APS3_CORTUS__)
#define M2M_LOG_LEVEL									M2M_LOG_ERROR
#else
#define M2M_LOG_LEVEL									M2M_LOG_REQ
#endif


//#define M2M_ERR(a...) //{ printf(a); }
//#define M2M_INFO(a...) //{ printf(a); }
//#define M2M_REQ(...)
//#define M2M_DBG(...)
//#define M2M_PRINT(...)

#if (CONF_WINC_DEBUG == 1)
#undef M2M_PRINT
#define M2M_PRINT(...)							do{CONF_WINC_PRINTF(__VA_ARGS__);CONF_WINC_PRINTF("\r");}while(0)
#if (M2M_LOG_LEVEL >= M2M_LOG_ERROR)
#undef M2M_ERR
#define M2M_ERR(...)							do{CONF_WINC_PRINTF("(APP)(ERR)[%s][%d]",__FUNCTION__,__LINE__); CONF_WINC_PRINTF(__VA_ARGS__);CONF_WINC_PRINTF("\r");}while(0)
#if (M2M_LOG_LEVEL >= M2M_LOG_INFO)
#undef M2M_INFO
#define M2M_INFO(...)							do{CONF_WINC_PRINTF("(APP)(INFO)"); CONF_WINC_PRINTF(__VA_ARGS__);CONF_WINC_PRINTF("\r");}while(0)
#if (M2M_LOG_LEVEL >= M2M_LOG_REQ)
#undef M2M_REQ
#define M2M_REQ(...)							do{CONF_WINC_PRINTF("(APP)(R)"); CONF_WINC_PRINTF(__VA_ARGS__);CONF_WINC_PRINTF("\r");}while(0)
#if (M2M_LOG_LEVEL >= M2M_LOG_DBG)
#undef M2M_DBG
#define M2M_DBG(...)							do{CONF_WINC_PRINTF("(APP)(DBG)[%s][%d]",__FUNCTION__,__LINE__); CONF_WINC_PRINTF(__VA_ARGS__);CONF_WINC_PRINTF("\r");}while(0)
#endif /*M2M_LOG_DBG*/
#endif /*M2M_LOG_REQ*/
#endif /*M2M_LOG_INFO*/
#endif /*M2M_LOG_ERROR*/
#endif /*CONF_WINC_DEBUG */

/**@}*/
#endif /* _NM_DEBUG_H_ */


// nm_conf_winc.h *************************************************************************************************

#ifndef CONF_WINC_H_INCLUDED
#define CONF_WINC_H_INCLUDED

// #define __CONF_ASF__ //for atmel studio
#define __CONF_MBED__ //for mbed os

#ifdef __CONF_ASF__
#include "board.h"
#elif defined (__CONF_MBED__)
//#include <delay.h>
//#include <ioport.h>
//#include <pio.h>
//#include "flexcom.h"
//#include "spi_driver.h"
//#include "pdc.h"
//#include "board.h"
//#include "samg55.h"
#endif

/*
   ---------------------------------
   ---------- PIN settings ---------
   ---------------------------------
*/ 
#define CONF_WINC_PIN_RESET				//IOPORT_CREATE_PIN(PIOA, 26)
#define CONF_WINC_PIN_CHIP_ENABLE		//IOPORT_CREATE_PIN(PIOA, 29)
#define CONF_WINC_PIN_WAKE				//IOPORT_CREATE_PIN(PIOA, 25)

/*
   ---------------------------------
   ---------- SPI settings ---------
   ---------------------------------
*/

#define CONF_WINC_USE_SPI				1

/** SPI pin and instance settings. */
#define CONF_WINC_SPI					EXT1_SPI_MODULE
#define CONF_WINC_SPI_ID				ID_SPI5
#define CONF_WINC_SPI_MISO_GPIO			SPI_MISO_GPIO
#define CONF_WINC_SPI_MISO_FLAGS		SPI_MISO_FLAGS
#define CONF_WINC_SPI_MOSI_GPIO			SPI_MOSI_GPIO
#define CONF_WINC_SPI_MOSI_FLAGS		SPI_MOSI_FLAGS
#define CONF_WINC_SPI_CLK_GPIO			SPI_SPCK_GPIO
#define CONF_WINC_SPI_CLK_FLAGS			SPI_SPCK_FLAGS
#define CONF_WINC_SPI_CS_GPIO			SPI_NPCS0_GPIO
#define CONF_WINC_SPI_CS_FLAGS			SPI_NPCS0_FLAGS
#define CONF_WINC_SPI_NPCS				(0)

/** SPI delay before SPCK and between consecutive transfer. */
#define CONF_WINC_SPI_DLYBS				(0)
#define CONF_WINC_SPI_DLYBCT 			(0)

/** SPI interrupt pin. */
#define CONF_WINC_SPI_INT_PIN			//IOPORT_CREATE_PIN(PIOA, 24)
#define CONF_WINC_SPI_INT_PIO			//PIOA
#define CONF_WINC_SPI_INT_PIO_ID		//ID_PIOA
#define CONF_WINC_SPI_INT_MASK			//PIO_PA24
#define CONF_WINC_SPI_INT_PRIORITY		//(0)

/** Clock polarity & phase. */
#define CONF_WINC_SPI_POL				(0)
#define CONF_WINC_SPI_PHA				(1)

/** SPI clock. */
#define CONF_WINC_SPI_CLOCK				(48000000)

/*
   ---------------------------------
   --------- Debug Options ---------
   ---------------------------------
*/

#define CONF_WINC_DEBUG					(1)
#define CONF_WINC_PRINTF				printf


#define CONF_MBED_SAMG55_BTN 	(0)


#endif /* CONF_WINC_H_INCLUDED */


// mtm_ate_mode.h *************************************************************************************************

#ifdef _M2M_ATE_FW_

#ifndef _M2M_ATE_MODE_H_
#define _M2M_ATE_MODE_H_

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include "common/include/nm_common.h"
#include "driver/include/m2m_types.h"

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#define M2M_ATE_MAX_NUM_OF_RATES		(20)
/*!< Maximum number of all rates (b,g and n)
 */
#define M2M_ATE_MAX_FRAME_LENGTH		(1024)
/*!< Maximum number of length for each frame
 */
#define M2M_ATE_MIN_FRAME_LENGTH		(1)
/*!< Minimum number of length for each frame
 */ 


#define M2M_ATE_SUCCESS					(M2M_SUCCESS)
/*!< No Error and operation has been completed successfully.
*/
#define M2M_ATE_ERR_VALIDATE			(M2M_ERR_FAIL)	
/*!< Error in parameters passed to functions.
 */
#define M2M_ATE_ERR_TX_ALREADY_RUNNING	(-1)
/*!< This means that TX case is already running and RX or even TX can't start without stopping it first.
 */
#define M2M_ATE_ERR_RX_ALREADY_RUNNING	(-2)			
/*!< This means that RX case is already running and TX or even RX can't start without stopping it first.
 */
#define M2M_ATE_ERR_UNHANDLED_CASE		(-3)	
/*!< Invalid case.
 */
#define M2M_ATE_RX_DISABLE_DA          		0x0
#define M2M_ATE_RX_ENABLE_DA          		0x1

#define M2M_ATE_RX_DISABLE_SA          		0x0
#define M2M_ATE_RX_ENABLE_SA          		0x1

#define M2M_ATE_DISABLE_SELF_MACADDR       	0x0
#define M2M_ATE_SET_SELF_MACADDR         	0x1
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
DATA TYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/*!
 *@enum		tenuM2mAteFwState	
 *@brief	Enumeration used for change ATE firmware state
 */
typedef enum {
	M2M_ATE_FW_STATE_STOP			= 0x00,
	/*!< State to stop ATE firmware
	 */
	M2M_ATE_FW_STATE_RUN			= 0x01,
	/*!< State to run ATE firmware
	 */
}tenuM2mAteFwState;

/*!
 *@enum		tenuM2mAteTxRates	
 *@brief	Used to get value of rate referenced by this index
 */
typedef enum {
	M2M_ATE_TX_RATE_1_Mbps_INDEX	= 0x00,
	M2M_ATE_TX_RATE_2_Mbps_INDEX	= 0x01,
	M2M_ATE_TX_RATE_55_Mbps_INDEX	= 0x02,
	M2M_ATE_TX_RATE_11_Mbps_INDEX	= 0x03,
	/*!< B-Rates
	 */
	M2M_ATE_TX_RATE_6_Mbps_INDEX	= 0x04,
	M2M_ATE_TX_RATE_9_Mbps_INDEX	= 0x05,
	M2M_ATE_TX_RATE_12_Mbps_INDEX	= 0x06,
	M2M_ATE_TX_RATE_18_Mbps_INDEX	= 0x07,
	M2M_ATE_TX_RATE_24_Mbps_INDEX	= 0x08,
	M2M_ATE_TX_RATE_36_Mbps_INDEX	= 0x09,
	M2M_ATE_TX_RATE_48_Mbps_INDEX	= 0x0A,
	M2M_ATE_TX_RATE_54_Mbps_INDEX	= 0x0B,
	/*!< G-Rates
	 */
	M2M_ATE_TX_RATE_MCS_0_INDEX		= 0x0C,
	M2M_ATE_TX_RATE_MCS_1_INDEX		= 0x0D,
	M2M_ATE_TX_RATE_MCS_2_INDEX		= 0x0E,
	M2M_ATE_TX_RATE_MCS_3_INDEX		= 0x0F,
	M2M_ATE_TX_RATE_MCS_4_INDEX		= 0x10,
	M2M_ATE_TX_RATE_MCS_5_INDEX		= 0x11,
	M2M_ATE_TX_RATE_MCS_6_INDEX		= 0x12,
	M2M_ATE_TX_RATE_MCS_7_INDEX		= 0x13,
	/*!< N-Rates
	 */
}tenuM2mAteTxIndexOfRates;

/*!
 *@enum		tenuM2mAteTxDutyCycle	
 *@brief	Values of duty cycle
 */
typedef enum {
	M2M_ATE_TX_DUTY_1				= 0x01,
	M2M_ATE_TX_DUTY_2				= 0x02,
	M2M_ATE_TX_DUTY_3				= 0x03,
	M2M_ATE_TX_DUTY_4				= 0x04,
	M2M_ATE_TX_DUTY_5				= 0x05,
	M2M_ATE_TX_DUTY_6				= 0x06,
	M2M_ATE_TX_DUTY_7				= 0x07,
	M2M_ATE_TX_DUTY_8				= 0x08,
	M2M_ATE_TX_DUTY_9				= 0x09,
	M2M_ATE_TX_DUTY_10				= 0xA0,
    } tenuM2mAteTxDutyCycle;


#define M2M_ATE_TX_DUTY_MAX_VALUE	M2M_ATE_TX_DUTY_1
/*!< The maximum value of duty cycle
*/
#define M2M_ATE_TX_DUTY_MIN_VALUE	M2M_ATE_TX_DUTY_10
/*!< The minimum value of duty cycle
*/

/*!
 *@enum		tenuM2mAteTxDpdControl	
 *@brief	Allowed values for DPD control 
 */
typedef enum {
	M2M_ATE_TX_DPD_DYNAMIC	= 0x00,
	M2M_ATE_TX_DPD_BYPASS	= 0x01,
	M2M_ATE_TX_DPD_ENABLED	= 0x02,
    } tenuM2mAteTxDpdControl;

/*!
 *@enum		tenuM2mAteTxGainSetting	
 *@brief	Options for TX gain selection mode
 */
typedef enum {
	M2M_ATE_TX_GAIN_DYNAMIC	= 0x00,
	M2M_ATE_TX_GAIN_BYPASS	= 0x01,
	M2M_ATE_TX_GAIN_FCC		= 0x02,
	M2M_ATE_TX_GAIN_TELEC	= 0x03,
    } tenuM2mAteTxGainSetting;

/*!
 *@enum		tenuM2mAtePMUSetting	
 *@brief	Used to Enable PMU or disable it
 */
typedef enum {
	M2M_ATE_PMU_DISBLE	= 0x00,
	M2M_ATE_PMU_ENABLE	= 0x01,
    } tenuM2mAtePMUSetting;

/*!
 *@enum		tenuM2mAteTxSource	
 *@brief	Used to define if enable PHY continues mode or MAC
 */
typedef enum {
	M2M_ATE_TX_SRC_MAC	= 0x00,
	M2M_ATE_TX_SRC_PHY	= 0x01,
    } tenuM2mAteTxSource;

/*!
 *@enum		tenuM2mAteTxMode	
 *@brief	Used to define type of TX mode either normal or CW(Continuous Wave) TX sequence
 */
typedef enum {
	M2M_ATE_TX_MODE_NORM	= 0x00,
	M2M_ATE_TX_MODE_CW		= 0x01,
    } tenuM2mAteTxMode;

/*!
 *@enum		tenuM2mAteRxPwrMode	
 *@brief	Used to define type of RX mode either high power or low power
 */
typedef enum {
	M2M_ATE_RX_PWR_HIGH	= 0x00,
	M2M_ATE_RX_PWR_LOW	= 0x01,
    } tenuM2mAteRxPwrMode;

/*!
 *@enum		tenuM2mAteChannels	
 *@brief	Available channels for TX and RX 
 */
typedef enum {
	M2M_ATE_CHANNEL_1	= 0x01,
	M2M_ATE_CHANNEL_2	= 0x02,
	M2M_ATE_CHANNEL_3	= 0x03,
	M2M_ATE_CHANNEL_4	= 0x04,
	M2M_ATE_CHANNEL_5	= 0x05,
	M2M_ATE_CHANNEL_6	= 0x06,
	M2M_ATE_CHANNEL_7	= 0x07,
	M2M_ATE_CHANNEL_8	= 0x08,
	M2M_ATE_CHANNEL_9	= 0x09,
	M2M_ATE_CHANNEL_10	= 0x0A,
	M2M_ATE_CHANNEL_11	= 0x0B,
	M2M_ATE_CHANNEL_12	= 0x0C,
	M2M_ATE_CHANNEL_13	= 0x0D,
	M2M_ATE_CHANNEL_14	= 0x0E,
    } tenuM2mAteChannels;

/*!
 *@struct	tstrM2mAteRxStatus
 *@brief	Used to save statistics of RX case
 */
typedef struct {
	uint32 num_rx_pkts;
	/*!< Number of total RX packet
	 */
	uint32 num_err_pkts;
	/*!< Number of RX failed packets
	 */
	 uint32 num_good_pkts;
	/*!< Number of RX packets actually received
	 */
    } tstrM2mAteRxStatus;

/*!
 *@struct	tstrM2mAteRxStatus
 *@brief	Used to save statistics of RX case
 */
typedef struct {
	uint8 u8RxPwrMode;
	/*!< RX power mode review tenuM2mAteRxPwrMode
	 */
    } tstrM2mAteInit;

/*!
 *@struct	tstrM2mAteTx
 *@brief	Used as data source in case of enabling TX test case
 */
typedef struct {
	uint32	num_frames;	
	/*!< Number of frames to be sent where maximum number allowed is 4294967295 ul, and ZERO means infinite number of frames
	 */
	uint32	data_rate;
	/*!< Rate to sent packets over to select rate use value of \ref tenuM2mAteTxIndexOfRates and pass it to \ref m2m_ate_get_tx_rate
	 */
	uint8		channel_num;
	/*!< Channel number \ref tenuM2mAteChannels
	 */
	uint8    duty_cycle; 
	/*!< Duty cycle value between from 1 to 10, where maximum = 1, minimum = 10 \ref tenuM2mAteTxDutyCycle
	 */
	uint16    frame_len;
    /*!< Use \ref M2M_ATE_MAX_FRAME_LENGTH (1024) as the maximum value while \ref M2M_ATE_MIN_FRAME_LENGTH (1) is the minimum value
	 */
	uint8     tx_gain_sel;
	/*!< TX gain mode selection value \ref tenuM2mAteTxGainSetting
	 */
	uint8     dpd_ctrl;
	/*!< DPD mode value\ref tenuM2mAteTxDpdControl
	 */
	uint8     use_pmu;
	/*!< This is 0 if PMU is not used otherwise it must be be 1 \ref tenuM2mAtePMUSetting
	 */
	uint8     phy_burst_tx; 
	/*!< Source of Burst TX either PHY or MAC\ref tenuM2mAteTxSource
	 */
	uint8     cw_tx; 
	/*!< Mode of Burst TX either normal TX sequence or CW(Continuous Wave) TX sequence \ref tenuM2mAteTxMode
	 */
	uint32     xo_offset_x1000; 
	/*!< Signed XO offset value in PPM (Part Per Million) multiplied by 1000.
	 */
	uint8     use_efuse_xo_offset;
	/*!< Set to 0 to use the XO offset provided in xo_offset_x1000. Set to 1 to use XO offset programmed on WINC efuse. 
	*/
	uint8 peer_mac_addr[6];
	/*!< Set peer address to send directed frames to a certain address.
	*/
    } tstrM2mAteTx;

/*!
 *@struct	tstrM2mAteRx
 *@brief	Used as data source in case of enabling RX test case
 */
typedef struct {
	uint8		channel_num;
	/*!< Channel number \ref tenuM2mAteChannels
	 */
	uint8     use_pmu;
	/*!< This is 0 if PMU is not used otherwise it must be be 1 \ref tenuM2mAtePMUSetting
	 */
	uint32     xo_offset_x1000; 
	/*!< Signed XO offset value in PPM (Part Per Million) multiplied by 1000.
	 */
	uint8     use_efuse_xo_offset;
	/*!< Set to 0 to use the XO offset provided in xo_offset_x1000. Set to 1 to use XO offset programmed on WINC efuse. 
	*/
	uint8    self_mac_addr[6];
	/*!< Set to the self mac address required to be overriden. 
	*/
	uint8    peer_mac_addr[6];
	/*!< Set to the source mac address expected to filter frames from. 
	*/
	uint8    mac_filter_en_da;
	/*!< Flag set to enable or disable reception with destination address as a filter. 
	*/
	uint8    mac_filter_en_sa;
	/*!< Flag set to enable or disable reception with source address as a filter. 
	*/
	uint8   override_self_mac_addr;
	/*!< Flag set to enable or disable self mac address feature. 
	*/
    } tstrM2mAteRx;

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION PROTOTYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/*!
@fn	\
	sint8 m2m_ate_init(void);

@brief
	This function used to download ATE firmware from flash and start it

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
sint8 m2m_ate_init(void);


/*!
@fn	\
	sint8 m2m_ate_init(tstrM2mAteInit *pstrInit);

@brief
	This function used to download ATE firmware from flash and start it

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
sint8 m2m_ate_init_param(tstrM2mAteInit *pstrInit);

/*!
@fn	\
	sint8 m2m_ate_deinit(void);

@brief
	De-Initialization of ATE firmware mode 

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
sint8 m2m_ate_deinit(void);

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
sint8 m2m_ate_set_fw_state(uint8);

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
sint8 m2m_ate_get_fw_state(void);

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
uint32 m2m_ate_get_tx_rate(uint8);

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
sint8 m2m_ate_get_tx_status(void);

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
sint8 m2m_ate_start_tx(tstrM2mAteTx *);

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
sint8 m2m_ate_stop_tx(void);

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
sint8 m2m_ate_get_rx_status(void);

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
sint8 m2m_ate_start_rx(tstrM2mAteRx *);

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
sint8 m2m_ate_stop_rx(void);

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
sint8 m2m_ate_read_rx_status(tstrM2mAteRxStatus *);

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
sint8 m2m_ate_set_dig_gain(double dGaindB);

/*!
@fn	\
	sint8 m2m_ate_get_dig_gain(double * dGaindB)

@brief
	This function is used to get the digital gain

@param [out]	double * dGaindB
		The retrieved digital gain value obtained from HW registers in dB.
@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
sint8 m2m_ate_get_dig_gain(double * dGaindB);

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
sint8 m2m_ate_get_pa_gain(double *paGaindB);

/*!
@fn	\
	sint8 m2m_ate_get_ppa_gain(double * ppaGaindB)

@brief
	This function is used to get the PPA gain

@param [out]	uint32 * ppaGaindB
		The retrieved PPA gain value obtained from HW registers in dB.
@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
sint8 m2m_ate_get_ppa_gain(double * ppaGaindB);

/*!
@fn	\
	sint8 m2m_ate_get_tot_gain(double * totGaindB)

@brief
	This function is used to calculate the total gain

@param [out]	double * totGaindB
		The retrieved total gain value obtained from calculations made based on the digital gain, PA and PPA gain values.
@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
sint8 m2m_ate_get_tot_gain(double * totGaindB);

	
#endif /* _M2M_CONFIG_MODE_H_ */

#endif //_M2M_ATE_FW_


// mtm_hif.h *************************************************************************************************
     
#ifndef _M2M_HIF_
#define _M2M_HIF_

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

//#include "common/include/nm_common.h"
/*!< Include depends on UNO Board is used or not*/
#ifdef ENABLE_UNO_BOARD
#include "m2m_uno_hif.h"
#endif

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#define M2M_HIF_MAX_PACKET_SIZE      (1600 - 4)
/*!< Maximum size of the buffer could be transferred between Host and Firmware.
*/

#define M2M_HIF_HDR_OFFSET (sizeof(tstrHifHdr) + 4)

/**
*	@struct		tstrHifHdr
*	@brief		Structure to hold HIF header
*/
typedef struct {
    uint8   u8Gid;		/*!< Group ID */
    uint8   u8Opcode;	/*!< OP code */
    uint16  u16Length;	/*!< Payload length */
    } tstrHifHdr;


/*!
@typedef typedef void (*tpfHifCallBack)(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr);
@brief	used to point to Wi-Fi call back function depend on Arduino project or other projects.
@param [in]	u8OpCode
				HIF Opcode type.
@param [in]	u16DataSize
				HIF data length.
@param [in]	u32Addr
				HIF address.
@param [in]	grp
				HIF group type.
*/
typedef void (*tpfHifCallBack)(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr);
/**
*   @fn			NMI_API sint8 hif_init(void * arg);
*   @brief
				To initialize HIF layer.
*   @param [in]	arg
*				Pointer to the arguments.
*   @return
				The function shall return ZERO for successful operation and a negative value otherwise.
*/
NMI_API sint8 hif_init(void * arg);
/**
*	@fn			NMI_API sint8 hif_deinit(void * arg);
*	@brief
				To Deinitialize HIF layer.
*   @param [in]	arg
*				Pointer to the arguments.
*    @return
				The function shall return ZERO for successful operation and a negative value otherwise.
*/
NMI_API sint8 hif_deinit(void * arg);
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
*    @return	The function shall return ZERO for successful operation and a negative value otherwise.
*/
NMI_API sint8 hif_send(uint8 u8Gid,uint8 u8Opcode,uint8 *pu8CtrlBuf,uint16 u16CtrlBufSize,
					   uint8 *pu8DataBuf,uint16 u16DataSize, uint16 u16DataOffset);
/*
*	@fn		hif_receive
*	@brief	Host interface interrupt serviece routine
*	@param [in]	u32Addr
*				Receive start address
*	@param [out] pu8Buf
*				Pointer to receive buffer. Allocated by the caller
*	@param [in]	 u16Sz
*				Receive buffer size
*	@param [in]	isDone
*				If you don't need any more packets send True otherwise send false
*   @return
				The function shall return ZERO for successful operation and a negative value otherwise.
*/

NMI_API sint8 hif_receive(uint32 u32Addr, uint8 *pu8Buf, uint16 u16Sz, uint8 isDone);
/**
*	@fn			hif_register_cb
*	@brief
				To set Callback function for every  Component.

*	@param [in]	u8Grp
*				Group to which the Callback function should be set.

*	@param [in]	fn
*				function to be set to the specified group.
*   @return
				The function shall return ZERO for successful operation and a negative value otherwise.
*/
NMI_API sint8 hif_register_cb(uint8 u8Grp,tpfHifCallBack fn);
/**
*	@fn		NMI_API sint8 hif_chip_sleep(void);
*	@brief
				To make the chip sleep.
*   @return
				The function shall return ZERO for successful operation and a negative value otherwise.
*/
NMI_API sint8 hif_chip_sleep(void);
/**
*	@fn		NMI_API sint8 hif_chip_wake(void);
*	@brief
			To Wakeup the chip.
*   @return
			The function shall return ZERO for successful operation and a negative value otherwise.
*/

NMI_API sint8 hif_chip_wake(void);
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

NMI_API void hif_set_sleep_mode(uint8 u8Pstype);
/*!
@fn	\
	NMI_API uint8 hif_get_sleep_mode(void);

@brief
	Get the sleep mode of the HIF layer.

@return
	The function SHALL return the sleep mode of the HIF layer.
*/

NMI_API uint8 hif_get_sleep_mode(void);

#ifdef CORTUS_APP
/**
*	@fn		hif_Resp_handler(uint8 *pu8Buffer, uint16 u16BufferSize)
*	@brief
				Response handler for HIF layer.

*	@param [in]	pu8Buffer
				Pointer to the buffer.

*	@param [in]	u16BufferSize
				Buffer size.

*   @return
			    The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 hif_Resp_handler(uint8 *pu8Buffer, uint16 u16BufferSize);
#endif

/**
*	@fn		hif_handle_isr(void)
*	@brief
			Handle interrupt received from NMC1500 firmware.
*   @return
			The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 hif_handle_isr(void);

#endif


// mtm_crypto.h *************************************************************************************************

#ifndef __M2M_CRYPTO_H__
#define __M2M_CRYPTO_H__


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/


//#include "common/include/nm_common.h"
//#include "driver/include/m2m_types.h"
//#include "driver/source/m2m_hif.h"

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#define M2M_MAX_RSA_LEN					(256)
#define M2M_SHA256_DIGEST_LEN			32
#define M2M_SHA256_CONTEXT_BUFF_LEN (128)
#define M2M_SHA256_MAX_DATA				(M2M_BUFFER_MAX_SIZE - M2M_SHA256_CONTEXT_BUFF_LEN - M2M_HIF_HDR_OFFSET)
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
DATA TYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/*!
@struct	\
	tstrM2mSha256Ctxt

@brief
	SHA256 context data
*/
typedef struct sha256ctxt{
	uint32	au32Sha256CtxtBuff[M2M_SHA256_CONTEXT_BUFF_LEN/sizeof(uint32)];
    } tstrM2mSha256Ctxt;



/*!
@enum	\
	tenuRsaSignStatus

@brief
	RSA Signature status: pass or fail.
*/
typedef enum {
	M2M_RSA_SIGN_OK,
	M2M_RSA_SIGN_FAIL
    } tenuRsaSignStatus;

/*!
@typedef \
	tpfAppCryproCb

@brief	
@param [in]	u8MsgType
				
@param [in]	pvMsg
				A pointer to a buffer containing the notification parameters (if any). It should be
				Casted to the correct data type corresponding to the notification type.


*/
typedef void (*tpfAppCryproCb) (uint8 u8MsgType,void * pvResp, void * pvMsg);

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION PROTOTYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/


/*!
@fn	\
	sint8 m2m_crypto_init();
	
@brief	crypto initialization

@param[in]	pfAppCryproCb

*/
sint8 m2m_crypto_init(tpfAppCryproCb pfAppCryproCb);
/*!
@fn	\
	sint8 m2m_sha256_hash_init(tstrM2mSha256Ctxt *psha256Ctxt);
	
@brief	SHA256 hash initialization

@param[in]	psha256Ctxt
				Pointer to a sha256 context allocated by the caller.
*/
sint8 m2m_crypto_sha256_hash_init(tstrM2mSha256Ctxt *psha256Ctxt);


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
sint8 m2m_crypto_sha256_hash_update(tstrM2mSha256Ctxt *psha256Ctxt, uint8 *pu8Data, uint16 u16DataLength);


/*!
@fn	\
	sint8 m2m_sha256_hash_finish(tstrM2mSha256Ctxt *psha256Ctxt, uint8 *pu8Sha256Digest);
	
@brief	SHA256 hash finalization

@param[in]	psha256Ctxt
				Pointer to a sha256 context allocated by the caller.
				
@param [in] pu8Sha256Digest
				Buffer allocated by the caller which will hold the resultant SHA256 Digest. It must be allocated no less than M2M_SHA256_DIGEST_LEN.
*/
sint8 m2m_crypto_sha256_hash_finish(tstrM2mSha256Ctxt *psha256Ctxt, uint8 *pu8Sha256Digest);




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
						  uint16 u16HashLength, uint8 *pu8RsaSignature);


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
					   uint16 u16HashLength, uint8 *pu8RsaSignature);


#endif /* __M2M_CRYPTO_H__ */


// mtm_ota.h *************************************************************************************************

#ifndef __M2M_OTA_H__
#define __M2M_OTA_H__

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

//#include "common/include/nm_common.h"
//#include "driver/include/m2m_types.h"
//#include "driver/source/nmdrv.h"
/**@addtogroup WlanEnums Enumerations and Typedefs
 * @ingroup m2m_wifi
 */
 /* @{ */

/*!
@struct	\
	tstrOtaUpdateInfo

@brief
	OTA Update Information

@sa
	tenuWPSTrigger
*/
typedef struct {
	uint32	u8NcfUpgradeVersion;
	/*!< NCF OTA Upgrade Version
	*/
	uint32	u8NcfCurrentVersion;
	/*!< NCF OTA Current firmware version
	*/
	uint32	u8NcdUpgradeVersion;
	/*!< NCD (host) upgraded version (if the u8NcdRequiredUpgrade == true)
	*/
	uint8	u8NcdRequiredUpgrade;
	/*!< NCD Required upgrade to the above version
	*/
	uint8 	u8DownloadUrlOffset;
	/*!< Download URL offset in the received packet
	*/
	uint8 	u8DownloadUrlSize;
	/*!< Download URL size in the received packet
	*/
	uint8	__PAD8__;
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrOtaUpdateInfo;

/*!
@typedef void (*tpfOtaNotifCb) (tstrOtaUpdateInfo *);

@brief A callback to get notification about an potential OTA update.

@param[in] pstrOtaUpdateInfo A structure to provide notification payload.

@sa
	tstrOtaUpdateInfo
@warning
		The notification is not supported (Not implemented yet)

*/
typedef void (*tpfOtaNotifCb) (tstrOtaUpdateInfo *pstrOtaUpdateInfo);


/*!
@typedef void (*tpfOtaUpdateCb) (uint8 u8OtaUpdateStatusType ,uint8 u8OtaUpdateStatus);

@brief
   A callback to get OTA status update, the callback provide the status type and its status the OTA callback provides the download status,
   the switch to the downloaded firmware status and roll-back status.

@param[in] u8OtaUpdateStatusType Possible values are listed in tenuOtaUpdateStatusType. Possible types are:
- [DL_STATUS](@ref DL_STATUS)
- [SW_STATUS](@ref SW_STATUS)
- [RB_STATUS](@ref RB_STATUS)

@param[in] u8OtaUpdateStatus Possible values are listed in tenuOtaUpdateStatus.

@see
	tenuOtaUpdateStatusType
	tenuOtaUpdateStatus
 */
typedef void (*tpfOtaUpdateCb) (uint8 u8OtaUpdateStatusType ,uint8 u8OtaUpdateStatus);
 /**@}*/
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION PROTOTYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/


/** @defgroup OtaInitFn m2m_ota_init
 *  @ingroup WLANAPI
* Synchronous initialization function for the OTA layer by registering the update callback.
* The notification callback is not supported at the current version. Calling this API is a
* MUST for all the OTA API's.

 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8  m2m_ota_init(tpfOtaUpdateCb  pfOtaUpdateCb,tpfOtaNotifCb  pfOtaNotifCb)

@param [in]	pfOtaUpdateCb
				OTA Update callback function

@param [in]	pfOtaNotifCb
				OTA notify callback function

@return
	The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
*/
NMI_API sint8  m2m_ota_init(tpfOtaUpdateCb  pfOtaUpdateCb,tpfOtaNotifCb  pfOtaNotifCb);
 /**@}*/

 /** @defgroup OtaNotifStFn m2m_ota_notif_set_url
 *  @ingroup WLANAPI
 * Set the OTA notification server URL, the functions need to be called before any check for update
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8  m2m_ota_notif_set_url(uint8 * u8Url);

@param [in]	u8Url
			 Set the OTA notification server URL, the functions need to be called before any check for update.
@warning
			Calling m2m_ota_init is required
			Notification Server is not supported in the current version (function is not implemented)
@see
			m2m_ota_init
@return
	The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
*/
NMI_API sint8  m2m_ota_notif_set_url(uint8 * u8Url);
 /**@}*/
 /** @defgroup OtaNotifCheckFn m2m_ota_notif_check_for_update
 *  @ingroup WLANAPI
 * Synchronous function to check for the OTA update using the Notification Server
 * URL. Function is not implemented (not supported at the current version)
 *
 */
  /**@{*/
/*!
@fn	\
	NMI_API sint8  m2m_ota_notif_check_for_update(void);

@warning
		Function is not implemented (not supported at the current version)

@sa
			m2m_ota_init
			m2m_ota_notif_set_url
@return
	The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
*/
NMI_API sint8  m2m_ota_notif_check_for_update(void);
 /**@}*/
 /** @defgroup OtaSched m2m_ota_notif_sched
*  @ingroup WLANAPI
* Schedule OTA notification Server check for update request after specific number of days
*/
  /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_ota_notif_sched(uint32 u32Period);


@param [in]	u32Period
			Period in days

@sa
		m2m_ota_init
		m2m_ota_notif_check_for_update
		m2m_ota_notif_set_url
@return
	The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
*/
NMI_API sint8 m2m_ota_notif_sched(uint32 u32Period);
  /**@}*/
/** @defgroup OtaStartUpdatefn m2m_ota_start_update
*  @ingroup WLANAPI
*	Request OTA start update using the downloaded URL, the OTA module will download the OTA image and ensure integrity of the image,
*   and update the validity of the image in control structure. Switching to that image requires calling @ref m2m_ota_switch_firmware API.
*   As a prerequisite @ref m2m_ota_init should be called before using @ref m2m_ota_start().
*/
  /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_ota_start_update(uint8 * u8DownloadUrl);

@param [in]	u8DownloadUrl
		The download firmware URL, you get it from device info according to the application server

@warning
	Calling this API does not guarantee OTA WINC image update, It depends on the connection with the download server and the validity of the image.
	If the API response is failure this may invalidate the roll-back image if it was previously valid, since the WINC does not have any internal memory
	except the flash roll-back image location to validate the downloaded image from

@see
		m2m_ota_init
		tpfOtaUpdateCb

@return
	The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
\section Example
   The example shows an example of how the OTA image update is carried out.
@code
static void OtaUpdateCb(uint8 u8OtaUpdateStatusType ,uint8 u8OtaUpdateStatus){
	if(u8OtaUpdateStatusType == DL_STATUS) {
		if(u8OtaUpdateStatus == OTA_STATUS_SUCSESS) {
			//switch to the upgraded firmware
			m2m_ota_switch_firmware();
		}
	}
	else if(u8OtaUpdateStatusType == SW_STATUS) {
		if(u8OtaUpdateStatus == OTA_STATUS_SUCSESS) {
			M2M_INFO("Now OTA successfully done");
			//start the host SW upgrade then system reset is required (Reinitialize the driver)
		}
	}
}
void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg){
	case M2M_WIFI_REQ_DHCP_CONF:
	{
		//after successfully connection, start the over air upgrade
		m2m_ota_start_update(OTA_URL);
	}
	break;
	default:
	break;
}
int main (void){
	tstrWifiInitParam param;
	tstr1xAuthCredentials gstrCred1x    = AUTH_CREDENTIALS;
	nm_bsp_init();

	m2m_memset((uint8*)&param, 0, sizeof(param));
	param.pfAppWifiCb = wifi_event_cb;

	//Initialize the WINC Driver
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret)	{
		M2M_ERR("Driver Init Failed <%d>\n",ret);
		while(1);
	}
	//Initialize the OTA module
	m2m_ota_init(OtaUpdateCb,NULL);
	//connect to AP that provide connection to the OTA server
	m2m_wifi_default_connect();

	while(1)
	{

		//Handle the app state machine plus the WINC event handler
		while(m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {

		}

	}
}
@endcode

*/
NMI_API sint8 m2m_ota_start_update(uint8 * u8DownloadUrl);
  /**@}*/
/** @defgroup OtaRollbackfn m2m_ota_rollback
*  @ingroup WLANAPI
	Request OTA Roll-back to the old (other) WINC image, the WINC firmware will check the validation of the Roll-back image
	and switch to it if it is valid.
	If the API response is success, system restart is required (re-initialize the driver with hardware rest) update the host driver version may
	be required if it is did not match the minimum version supported by the  WINC firmware.

*/
  /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_ota_rollback(void);

@sa
	m2m_ota_init
	m2m_ota_start_update

@return
	The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
*/
NMI_API sint8 m2m_ota_rollback(void);
  /**@}*/
  /**@}*/
/** @defgroup OtaSwitchFirmware m2m_ota_switch_firmware
*  @ingroup WLANAPI
* Switch to the upgraded Firmware, that API will update the control structure working image to the upgraded image
  take effect will be on the next system restart
*/
  /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_ota_switch_firmware(void);

@warning
   It is important to note that if the API succeeds, system restart is required (re-initializing the driver with hardware reset) updating the host driver version may be required
   if it does not match the minimum driver version supported by the WINC's firmware.
@sa
	m2m_ota_init
	m2m_ota_start_update

@return
	The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
*/
NMI_API sint8 m2m_ota_switch_firmware(void);
/*!
@fn	\
	NMI_API sint8 m2m_ota_get_firmware_version(void);

@brief
	Get the OTA Firmware version.

@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_ota_get_firmware_version(tstrM2mRev *pstrRev);
  /**@}*/
NMI_API sint8 m2m_ota_test(void);

#endif /* __M2M_OTA_H__ */


// mtm_periph.h *************************************************************************************************

#ifndef _M2M_PERIPH_H_
#define _M2M_PERIPH_H_


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/


//#include "common/include/nm_common.h"
//#include "driver/include/m2m_types.h"

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
DATA TYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/*!
@struct	\
	tstrPerphInitParam

@brief
	Peripheral module initialization parameters.
*/
typedef struct {
	void * arg;
    } tstrPerphInitParam;


/*!
@enum	\
	tenuGpioNum

@brief
	A list of GPIO numbers configurable through the m2m_periph module.
*/
typedef enum {
#if 0
	M2M_PERIPH_GPIO3, /*!< GPIO15 pad	*/
	M2M_PERIPH_GPIO4, /*!< GPIO16 pad	*/
	M2M_PERIPH_GPIO5, /*!< GPIO18 pad	*/
	M2M_PERIPH_GPIO6, /*!< GPIO18 pad	*/
	M2M_PERIPH_GPIO15, /*!< GPIO15 pad	*/
	M2M_PERIPH_GPIO16, /*!< GPIO16 pad	*/
	M2M_PERIPH_GPIO18, /*!< GPIO18 pad	*/
#endif
	M2M_PERIPH_GPIO15, /*!< GPIO15 pad	*/
	M2M_PERIPH_GPIO16, /*!< GPIO16 pad	*/
	M2M_PERIPH_GPIO18, /*!< GPIO18 pad	*/
	M2M_PERIPH_GPIO_MAX
    } tenuGpioNum;


/*!
@enum	\
	tenuI2cMasterSclMuxOpt

@brief
	Allowed pin multiplexing options for I2C master SCL signal.
*/
typedef enum {
	M2M_PERIPH_I2C_MASTER_SCL_MUX_OPT_HOST_WAKEUP, /*!< I2C master SCL is avaiable on HOST_WAKEUP. */
	M2M_PERIPH_I2C_MASTER_SCL_MUX_OPT_SD_DAT3,     /*!< I2C master SCL is avaiable on SD_DAT3 (GPIO 7). */
	M2M_PERIPH_I2C_MASTER_SCL_MUX_OPT_GPIO13,      /*!< I2C master SCL is avaiable on GPIO 13. */
	M2M_PERIPH_I2C_MASTER_SCL_MUX_OPT_GPIO4,       /*!< I2C master SCL is avaiable on GPIO 4.*/
	M2M_PERIPH_I2C_MASTER_SCL_MUX_OPT_I2C_SCL,     /*!< I2C master SCL is avaiable on I2C slave SCL. */
	M2M_PERIPH_I2C_MASTER_SCL_MUX_OPT_NUM
    } tenuI2cMasterSclMuxOpt;

/*!
@enum	\
	tenuI2cMasterSdaMuxOpt

@brief
	Allowed pin multiplexing options for I2C master SDA signal.
*/
typedef enum {
	M2M_PERIPH_I2C_MASTER_SDA_MUX_OPT_RTC_CLK , /*!< I2C master SDA is avaiable on RTC_CLK. */
	M2M_PERIPH_I2C_MASTER_SDA_MUX_OPT_SD_CLK,   /*!< I2C master SDA is avaiable on SD_CLK (GPIO 8). */
	M2M_PERIPH_I2C_MASTER_SDA_MUX_OPT_GPIO14,   /*!< I2C master SDA is avaiable on GPIO 14. */
	M2M_PERIPH_I2C_MASTER_SDA_MUX_OPT_GPIO6,    /*!< I2C master SDA is avaiable on GPIO 6.*/
	M2M_PERIPH_I2C_MASTER_SDA_MUX_OPT_I2C_SDA,  /*!< I2C master SDA is avaiable on I2C slave SDA. */
	M2M_PERIPH_I2C_MASTER_SDA_MUX_OPT_NUM
    } tenuI2cMasterSdaMuxOpt;


/*!
@struct	\
	tstrI2cMasterInitParam

@brief
	I2C master configuration parameters.
@sa
	tenuI2cMasterSclMuxOpt
	tenuI2cMasterSdaMuxOpt
*/
typedef struct {
	uint8 enuSclMuxOpt; /*!< SCL multiplexing option. Allowed value are defined in tenuI2cMasterSclMuxOpt  */
	uint8 enuSdaMuxOpt; /*!< SDA multiplexing option. Allowed value are defined in tenuI2cMasterSdaMuxOpt  */
	uint8 u8ClkSpeedKHz; /*!< I2C master clock speed in KHz. */
    } tstrI2cMasterInitParam;

/*!
@enum	\
	tenuI2cMasterFlags

@brief
	Bitwise-ORed flags for use in m2m_periph_i2c_master_write and m2m_periph_i2c_master_read
@sa
	m2m_periph_i2c_master_write
	m2m_periph_i2c_master_read
*/
typedef enum  {
    I2C_MASTER_NO_FLAGS          = 0x00,
	/*!< No flags.  */
    I2C_MASTER_NO_STOP           = 0x01,
	/*!< No stop bit after this transaction. Useful for scattered buffer read/write operations. */
	I2C_MASTER_NO_START          = 0x02,
	/*!< No start bit at the beginning of this transaction. Useful for scattered buffer read/write operations.*/
    } tenuI2cMasterFlags;

/*!
@enum	\
	tenuPullupMask

@brief
	Bitwise-ORed flags for use in m2m_perph_pullup_ctrl.
@sa
	m2m_periph_pullup_ctrl

*/
typedef enum {
	M2M_PERIPH_PULLUP_DIS_HOST_WAKEUP     = (1ul << 0),
	M2M_PERIPH_PULLUP_DIS_RTC_CLK         = (1ul << 1),
	M2M_PERIPH_PULLUP_DIS_IRQN            = (1ul << 2),
	M2M_PERIPH_PULLUP_DIS_GPIO_3          = (1ul << 3),
	M2M_PERIPH_PULLUP_DIS_GPIO_4          = (1ul << 4),
	M2M_PERIPH_PULLUP_DIS_GPIO_5          = (1ul << 5),
	M2M_PERIPH_PULLUP_DIS_SD_DAT3         = (1ul << 6),
	M2M_PERIPH_PULLUP_DIS_SD_DAT2_SPI_RXD = (1ul << 7),
	M2M_PERIPH_PULLUP_DIS_SD_DAT1_SPI_SSN = (1ul << 9),
	M2M_PERIPH_PULLUP_DIS_SD_CMD_SPI_SCK  = (1ul << 10),
	M2M_PERIPH_PULLUP_DIS_SD_DAT0_SPI_TXD = (1ul << 11),
	M2M_PERIPH_PULLUP_DIS_GPIO_6          = (1ul << 12),
	M2M_PERIPH_PULLUP_DIS_SD_CLK          = (1ul << 13),
	M2M_PERIPH_PULLUP_DIS_I2C_SCL         = (1ul << 14),
	M2M_PERIPH_PULLUP_DIS_I2C_SDA         = (1ul << 15),
	M2M_PERIPH_PULLUP_DIS_GPIO_11         = (1ul << 16),
	M2M_PERIPH_PULLUP_DIS_GPIO_12         = (1ul << 17),
	M2M_PERIPH_PULLUP_DIS_GPIO_13         = (1ul << 18),
	M2M_PERIPH_PULLUP_DIS_GPIO_14         = (1ul << 19),
	M2M_PERIPH_PULLUP_DIS_GPIO_15         = (1ul << 20),
	M2M_PERIPH_PULLUP_DIS_GPIO_16         = (1ul << 21),
	M2M_PERIPH_PULLUP_DIS_GPIO_17         = (1ul << 22),
	M2M_PERIPH_PULLUP_DIS_GPIO_18         = (1ul << 23),
	M2M_PERIPH_PULLUP_DIS_GPIO_19         = (1ul << 24),
	M2M_PERIPH_PULLUP_DIS_GPIO_20         = (1ul << 25),
	M2M_PERIPH_PULLUP_DIS_GPIO_21         = (1ul << 26),
	M2M_PERIPH_PULLUP_DIS_GPIO_22         = (1ul << 27),
	M2M_PERIPH_PULLUP_DIS_GPIO_23         = (1ul << 28),
	M2M_PERIPH_PULLUP_DIS_GPIO_24         = (1ul << 29),
    } tenuPullupMask;

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION PROTOTYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/


/*!
@fn	\
	NMI_API sint8 m2m_periph_init(tstrPerphInitParam * param);

@brief
	Initialize the NMC1500 peripheral driver module.

@param [in]	param
				Peripheral module initialization structure. See members of tstrPerphInitParam.

@return
	The function SHALL return 0 for success and a negative value otherwise.

@sa
	tstrPerphInitParam
*/
NMI_API sint8 m2m_periph_init(tstrPerphInitParam * param);

/*!
@fn	\
	NMI_API sint8 m2m_periph_gpio_set_dir(uint8 u8GpioNum, uint8 u8GpioDir);

@brief
	Configure a specific NMC1500 pad as a GPIO and sets its direction (input or output).

@param [in]	u8GpioNum
				GPIO number. Allowed values are defined in tenuGpioNum.

@param [in]	u8GpioDir
				GPIO direction: Zero = input. Non-zero = output.

@return
	The function SHALL return 0 for success and a negative value otherwise.

@sa
	tenuGpioNum
*/
NMI_API sint8 m2m_periph_gpio_set_dir(uint8 u8GpioNum, uint8 u8GpioDir);

/*!
@fn	\
	NMI_API sint8 m2m_periph_gpio_set_val(uint8 u8GpioNum, uint8 u8GpioVal);

@brief
	Set an NMC1500 GPIO output level high or low.

@param [in]	u8GpioNum
				GPIO number. Allowed values are defined in tenuGpioNum.

@param [in]	u8GpioVal
				GPIO output value. Zero = low, non-zero = high.

@return
	The function SHALL return 0 for success and a negative value otherwise.

@sa
	tenuGpioNum
*/
NMI_API sint8 m2m_periph_gpio_set_val(uint8 u8GpioNum, uint8 u8GpioVal);

/*!
@fn	\
	NMI_API sint8 m2m_periph_gpio_get_val(uint8 u8GpioNum, uint8 * pu8GpioVal);

@brief
	Read an NMC1500 GPIO input level.

@param [in]	u8GpioNum
				GPIO number. Allowed values are defined in tenuGpioNum.

@param [out] pu8GpioVal
				GPIO input value. Zero = low, non-zero = high.

@return
	The function SHALL return 0 for success and a negative value otherwise.

@sa
	tenuGpioNum
*/
NMI_API sint8 m2m_periph_gpio_get_val(uint8 u8GpioNum, uint8 * pu8GpioVal);

/*!
@fn	\
	NMI_API sint8 m2m_periph_gpio_pullup_ctrl(uint8 u8GpioNum, uint8 u8PullupEn);

@brief
	Set an NMC1500 GPIO pullup resisitor enable or disable.

@param [in]	u8GpioNum
				GPIO number. Allowed values are defined in tenuGpioNum.

@param [in] u8PullupEn
				Zero: pullup disabled. Non-zero: pullup enabled.

@return
	The function SHALL return 0 for success and a negative value otherwise.

@sa
	tenuGpioNum
*/
NMI_API sint8 m2m_periph_gpio_pullup_ctrl(uint8 u8GpioNum, uint8 u8PullupEn);

/*!
@fn	\
	NMI_API sint8 m2m_periph_i2c_master_init(tstrI2cMasterInitParam * param);

@brief
	Initialize and configure the NMC1500 I2C master peripheral.

@param [in]	param
				I2C master initialization structure. See members of tstrI2cMasterInitParam.

@return
	The function SHALL return 0 for success and a negative value otherwise.

@sa
	tstrI2cMasterInitParam
*/
NMI_API sint8 m2m_periph_i2c_master_init(tstrI2cMasterInitParam * param);

/*!
@fn	\
	NMI_API sint8 m2m_periph_i2c_master_write(uint8 u8SlaveAddr, uint8 * pu8Buf, uint16 u16BufLen, uint8 flags);

@brief
	Write a stream of bytes to the I2C slave device.

@param [in]	u8SlaveAddr
				7-bit I2C slave address.
@param [in]	pu8Buf
				A pointer to an input buffer which contains a stream of bytes.
@param [in]	u16BufLen
				Input buffer length in bytes.
@param [in]	flags
				Write operation bitwise-ORed flags. See tenuI2cMasterFlags.

@return
	The function SHALL return 0 for success and a negative value otherwise.

@sa
	tenuI2cMasterFlags
*/
NMI_API sint8 m2m_periph_i2c_master_write(uint8 u8SlaveAddr, uint8 * pu8Buf, uint16 u16BufLen, uint8 flags);


/*!
@fn	\
	NMI_API sint8 m2m_periph_i2c_master_read(uint8 u8SlaveAddr, uint8 * pu8Buf, uint16 u16BufLen, uint16 * pu16ReadLen, uint8 flags);

@brief
	Write a stream of bytes to the I2C slave device.

@param [in]	u8SlaveAddr
				7-bit I2C slave address.
@param [out] pu8Buf
				A pointer to an output buffer in which a stream of bytes are received.
@param [in]	u16BufLen
				Max output buffer length in bytes.
@param [out] pu16ReadLen
				Actual number of bytes received.
@param [in]	flags
				Write operation bitwise-ORed flags. See tenuI2cMasterFlags.

@return
	The function SHALL return 0 for success and a negative value otherwise.

@sa
	tenuI2cMasterFlags
*/
NMI_API sint8 m2m_periph_i2c_master_read(uint8 u8SlaveAddr, uint8 * pu8Buf, uint16 u16BufLen, uint16 * pu16ReadLen, uint8 flags);


/*!
@fn	\
	NMI_API sint8 m2m_periph_pullup_ctrl(uint32 pinmask, uint8 enable);

@brief
	Control the programmable pull-up resistor on the chip pads .


@param [in]	pinmask
				Write operation bitwise-ORed mask for which pads to control. Allowed values are defined in tenuPullupMask.

@param [in]	enable
				Set to 0 to disable pull-up resistor. Non-zero will enable the pull-up.

@return
	The function SHALL return 0 for success and a negative value otherwise.

@sa
	tenuPullupMask
*/
NMI_API sint8 m2m_periph_pullup_ctrl(uint32 pinmask, uint8 enable);



#endif /* _M2M_PERIPH_H_ */



// mtm_types.h *************************************************************************************************

#ifndef __M2M_WIFI_TYPES_H__
#define __M2M_WIFI_TYPES_H__


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#ifndef	_BOOT_
#ifndef _FIRMWARE_
//#include "common/include/nm_common.h"
#else
#include "m2m_common.h"
#endif
#endif


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/**@defgroup  WlanDefines Defines
 * @ingroup m2m_wifi
 */
/**@{*/
#define M2M_MAJOR_SHIFT (8)
#define M2M_MINOR_SHIFT (4)
#define M2M_PATCH_SHIFT (0)

#define M2M_DRV_VERSION_SHIFT (16)
#define M2M_FW_VERSION_SHIFT (0)

#define M2M_GET_MAJOR(ver_info_hword) ((uint8)((ver_info_hword) >> M2M_MAJOR_SHIFT) & 0xff)
#define M2M_GET_MINOR(ver_info_hword) ((uint8)((ver_info_hword) >> M2M_MINOR_SHIFT) & 0x0f)
#define M2M_GET_PATCH(ver_info_hword) ((uint8)((ver_info_hword) >> M2M_PATCH_SHIFT) & 0x0f)

#define M2M_GET_FW_VER(ver_info_word)  ((uint16) ((ver_info_word) >> M2M_FW_VERSION_SHIFT))
#define M2M_GET_DRV_VER(ver_info_word) ((uint16) ((ver_info_word) >> M2M_DRV_VERSION_SHIFT))

#define M2M_GET_DRV_MAJOR(ver_info_word) M2M_GET_MAJOR(M2M_GET_DRV_VER(ver_info_word))
#define M2M_GET_DRV_MINOR(ver_info_word) M2M_GET_MINOR(M2M_GET_DRV_VER(ver_info_word))
#define M2M_GET_DRV_PATCH(ver_info_word) M2M_GET_PATCH(M2M_GET_DRV_VER(ver_info_word))

#define M2M_GET_FW_MAJOR(ver_info_word) M2M_GET_MAJOR(M2M_GET_FW_VER(ver_info_word))
#define M2M_GET_FW_MINOR(ver_info_word) M2M_GET_MINOR(M2M_GET_FW_VER(ver_info_word))
#define M2M_GET_FW_PATCH(ver_info_word) M2M_GET_PATCH(M2M_GET_FW_VER(ver_info_word))

#define M2M_MAKE_VERSION(major, minor, patch) ( \
	((uint16)((major)  & 0xff)  << M2M_MAJOR_SHIFT) | \
	((uint16)((minor)  & 0x0f)  << M2M_MINOR_SHIFT) | \
	((uint16)((patch)  & 0x0f)  << M2M_PATCH_SHIFT))

#define M2M_MAKE_VERSION_INFO(fw_major, fw_minor, fw_patch, drv_major, drv_minor, drv_patch) \
	( \
	( ((uint32)M2M_MAKE_VERSION((fw_major),  (fw_minor),  (fw_patch)))  << M2M_FW_VERSION_SHIFT) | \
	( ((uint32)M2M_MAKE_VERSION((drv_major), (drv_minor), (drv_patch))) << M2M_DRV_VERSION_SHIFT))

#define REL_19_4_1_VER			M2M_MAKE_VERSION_INFO(19,4,1,19,3,0)
#define REL_19_4_0_VER			M2M_MAKE_VERSION_INFO(19,4,0,19,3,0)
#define REL_19_3_1_VER			M2M_MAKE_VERSION_INFO(19,3,1,19,3,0)
#define REL_19_3_0_VER			M2M_MAKE_VERSION_INFO(19,3,0,19,3,0)
#define REL_19_2_2_VER			M2M_MAKE_VERSION_INFO(19,2,2,19,2,0)
#define REL_19_2_1_VER			M2M_MAKE_VERSION_INFO(19,2,1,19,2,0)
#define REL_19_2_0_VER			M2M_MAKE_VERSION_INFO(19,2,0,19,2,0)
#define REL_19_1_0_VER			M2M_MAKE_VERSION_INFO(19,1,0,18,2,0)
#define REL_19_0_0_VER			M2M_MAKE_VERSION_INFO(19,0,0,18,1,1)

/*======*======*======*======*
		FIRMWARE VERSION NO INFO
 *======*======*======*======*/

#define M2M_FIRMWARE_VERSION_MAJOR_NO 					(19)
/*!< Firmware Major release version number.
*/


#define M2M_FIRMWARE_VERSION_MINOR_NO					(4)
/*!< Firmware Minor release version number.
*/

#define M2M_FIRMWARE_VERSION_PATCH_NO					(4)
/*!< Firmware patch release version number.
*/

/*======*======*======*======*
  SUPPORTED DRIVER VERSION NO INFO
 *======*======*======*======*/

#define M2M_DRIVER_VERSION_MAJOR_NO 					(19)
/*!< Driver Major release version number.
*/


#define M2M_DRIVER_VERSION_MINOR_NO						(3)
/*!< Driver Minor release version number.
*/

#define M2M_DRIVER_VERSION_PATCH_NO						(0)
/*!< Driver patch release version number.
*/


#if !defined(M2M_FIRMWARE_VERSION_MAJOR_NO) || !defined(M2M_FIRMWARE_VERSION_MINOR_NO)
#error Undefined version number
#endif

#define M2M_BUFFER_MAX_SIZE								(1600UL - 4)
/*!< Maximum size for the shared packet buffer.
 */


#define M2M_MAC_ADDRES_LEN                               6
/*!< The size fo 802 MAC address.
 */

#define M2M_ETHERNET_HDR_OFFSET							34
/*!< The offset of the Ethernet header within the WLAN Tx Buffer.
 */


#define M2M_ETHERNET_HDR_LEN							14
/*!< Length of the Etherenet header in bytes.
*/


#define M2M_MAX_SSID_LEN 								33
/*!< Maximum size for the Wi-Fi SSID including the NULL termination.
 */


#define M2M_MAX_PSK_LEN           						65
/*!< Maximum size for the WPA PSK including the NULL termination.
 */


#define M2M_DEVICE_NAME_MAX								48
/*!< Maximum Size for the device name including the NULL termination.
 */


#define M2M_LISTEN_INTERVAL 							1
/*!< The STA uses the Listen Interval parameter to indicate to the AP how
	many beacon intervals it shall sleep before it retrieves the queued frames
	from the AP. 
*/


#define M2M_1X_USR_NAME_MAX								21
/*!< The maximum size of the user name including the NULL termination.
	It is used for RADIUS authentication in case of connecting the device to
	an AP secured with WPA-Enterprise.
*/


#define M2M_1X_PWD_MAX									41
/*!< The maximum size of the password including the NULL termination.
	It is used for RADIUS authentication in case of connecting the device to
	an AP secured with WPA-Enterprise.
*/

#define M2M_CUST_IE_LEN_MAX								252
/*!< The maximum size of IE (Information Element).
*/
/*********************
 *
 * WIFI GROUP requests
 */

#define M2M_CONFIG_CMD_BASE									1
/*!< The base value of all the host configuration commands opcodes.
*/
#define M2M_STA_CMD_BASE									40
/*!< The base value of all the station mode host commands opcodes.
*/
#define M2M_AP_CMD_BASE										70
/*!< The base value of all the Access Point mode host commands opcodes.
*/
#define M2M_P2P_CMD_BASE									90
/*!< The base value of all the P2P mode host commands opcodes.
*/
#define M2M_SERVER_CMD_BASE									100
/*!< The base value of all the power save mode host commands codes.
*/
/**********************
 * OTA GROUP requests
 */
#define M2M_OTA_CMD_BASE									100
/*!< The base value of all the OTA mode host commands opcodes.
 * The OTA Have special group so can extended from 1-M2M_MAX_GRP_NUM_REQ
*/
/***********************
 *
 * CRYPTO group requests
 */
#define M2M_CRYPTO_CMD_BASE									1
/*!< The base value of all the crypto mode host commands opcodes.
 * The crypto Have special group so can extended from 1-M2M_MAX_GRP_NUM_REQ
*/

#define M2M_MAX_GRP_NUM_REQ									(127)
/*!< max number of request in one group equal to 127 as the last bit reserved for config or data pkt
*/

#define WEP_40_KEY_STRING_SIZE 							((uint8)10)
/*!< Indicate the wep key size in bytes for 40 bit string passphrase.
*/

#define WEP_104_KEY_STRING_SIZE 						((uint8)26)
/*!< Indicate the wep key size in bytes for 104 bit string passphrase.
*/
#define WEP_KEY_MAX_INDEX								((uint8)4)
/*!< Indicate the max key index value for WEP authentication
*/
#define M2M_SHA256_CONTEXT_BUFF_LEN							(128)
/*!< sha256 context size
*/
#define M2M_SCAN_DEFAULT_NUM_SLOTS							(2)
/*!< The default. number of scan slots performed by the WINC board.
*/
#define M2M_SCAN_DEFAULT_SLOT_TIME							(30)
/*!< The default. duration in miliseconds of a scan slots performed by the WINC board.
*/
#define M2M_SCAN_DEFAULT_NUM_PROBE							(2)
/*!< The default. number of scan slots performed by the WINC board.
*/


/*======*======*======*======*
	CONNECTION ERROR DEFINITIONS
 *======*======*======*======*/
typedef enum { 		
	M2M_DEFAULT_CONN_INPROGRESS = ((sint8)-23),  		
	/*!<
	A failure that indicates that a default connection or forced connection is in progress
	*/
	M2M_DEFAULT_CONN_FAIL,				
	/*!<
	A failure response that indicates that the winc failed to connect to the cached network
	*/
	 M2M_DEFAULT_CONN_SCAN_MISMATCH,	 													
	/*!<
	A failure response that indicates that no one of the cached networks 
	was found in the scan results, as a result to the function call m2m_default_connect.
	*/
	M2M_DEFAULT_CONN_EMPTY_LIST
	/*!<
	A failure response that indicates an empty network list as 
	a result to the function call m2m_default_connect.
	*/

    } tenuM2mDefaultConnErrcode;




/*======*======*======*======*
	OTA DEFINITIONS
 *======*======*======*======*/
 
#define OTA_STATUS_VALID					(0x12526285)
/*!< 
	Magic value updated in the Control structure in case of ROLLACK image Valid
*/
#define OTA_STATUS_INVALID					(0x23987718)
/*!< 
	Magic value updated in the Control structure in case of ROLLACK image InValid
*/
#define OTA_MAGIC_VALUE						(0x1ABCDEF9)
/*!< 
	Magic value set at the beginning of the OTA image header
*/

#define OTA_FORMAT_VER_0					(0)	/*Till 19.2.2 format*/
#define OTA_FORMAT_VER_1					(1) /*starting from 19.3.0 CRC is used and sequence number is used*/
/*!<
	Control structure format version
*/
#define OTA_SHA256_DIGEST_SIZE 				(32)
/*!< 
	Sha256 digest size in the OTA image, 
	the sha256 digest is set at the beginning of image before the OTA header 
*/

#define OTA_SUCCESS 						(0)
/*!<
	OTA Success status 
*/
#define OTA_ERR_WORKING_IMAGE_LOAD_FAIL		((sint8)-1)
/*!<
	Failure to load the firmware image
*/
#define OTA_ERR_INVAILD_CONTROL_SEC			((sint8)-2)
/*!<
	Control structure is being corrupted   
*/
#define M2M_ERR_OTA_SWITCH_FAIL     		((sint8)-3)
/*!<
	switching to the updated image failed as may be the image is invalid 
*/
#define M2M_ERR_OTA_START_UPDATE_FAIL     	((sint8)-4)
/*!<
	OTA update fail due to multiple reasons 
	- Connection failure
	- Image integrity fail  
	
*/
#define M2M_ERR_OTA_ROLLBACK_FAIL     		((sint8)-5)
/*!<
	Roll-back failed due to Roll-back image is not valid 
*/
#define M2M_ERR_OTA_INVAILD_FLASH_SIZE     	((sint8)-6)
/*!<
	The OTA Support at least 4MB flash size, if the above error will appear if the current flash is less than 4M
*/
#define M2M_ERR_OTA_INVAILD_ARG		     	((sint8)-7)
/*!<
	Invalid argument in any OTA Function
*/
/**@}*/

/**
* @addtogroup WlanEnums Enumerations and Typedefs
* @ingroup m2m_wifi
*/
 /**@{*/ 

/*!
@enum	\
	tenuM2mConnChangedErrcode
	
@brief
	
*/
typedef enum {
	 M2M_ERR_SCAN_FAIL = ((uint8)1),
	/*!< Indicate that the WINC board has failed to perform the scan operation.
	*/
	 M2M_ERR_JOIN_FAIL,	 								
	/*!< Indicate that the WINC board has failed to join the BSS .
	*/
	 M2M_ERR_AUTH_FAIL, 									
	/*!< Indicate that the WINC board has failed to authenticate with the AP.
	*/
	 M2M_ERR_ASSOC_FAIL,
	/*!< Indicate that the WINC board has failed to associate with the AP.
	*/
	 M2M_ERR_CONN_INPROGRESS,
	 /*!< Indicate that the WINC board has another connection request in progress.
	*/
    } tenuM2mConnChangedErrcode;
    
/*!
@enum	\
	tenuM2mWepKeyIndex
	
@brief
	
*/
typedef enum {
	M2M_WIFI_WEP_KEY_INDEX_1 = ((uint8) 1),
	M2M_WIFI_WEP_KEY_INDEX_2,
	M2M_WIFI_WEP_KEY_INDEX_3,
	M2M_WIFI_WEP_KEY_INDEX_4,
	/*!< Index for WEP key Authentication
	*/
    } tenuM2mWepKeyIndex;

/*!
@enum	\
	tenuM2mPwrMode
	
@brief
	
*/
typedef enum {
	PWR_AUTO = ((uint8) 1),
	/*!< FW will decide the best power mode to use internally. */
	PWR_LOW1,
	/*low power mode #1*/
	PWR_LOW2,
	/*low power mode #2*/
	PWR_HIGH,
	/* high power mode*/
    } tenuM2mPwrMode;

/*!
@struct	\	
	tstrM2mPwrState

@brief
	Power Mode
*/
typedef struct {
	uint8	u8PwrMode; 
	/*!< power Save Mode
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2mPwrMode;
    
/*!
@enum	\
	tenuM2mTxPwrLevel
	
@brief
	
*/
typedef enum {
	TX_PWR_HIGH = ((uint8) 1),
	/*!< PPA Gain 6dbm	PA Gain 18dbm */
	TX_PWR_MED,
	/*!< PPA Gain 6dbm	PA Gain 12dbm */
	TX_PWR_LOW,
	/*!< PPA Gain 6dbm	PA Gain 6dbm */
    } tenuM2mTxPwrLevel;

/*!
@struct	\	
	tstrM2mTxPwrLevel

@brief
	Tx power level 
*/
typedef struct {
	uint8	u8TxPwrLevel; 
	/*!< Tx power level
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2mTxPwrLevel;

/*!
@struct	\	
	tstrM2mEnableLogs

@brief
	Enable Firmware logs
*/
typedef struct {
	uint8	u8Enable; 
	/*!< Enable/Disable firmware logs
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2mEnableLogs;

/*!
@struct	\	
	tstrM2mBatteryVoltage

@brief
	Battery Voltage
*/
typedef struct {
	//Note: on SAMD D21 the size of double is 8 Bytes
	uint16	u16BattVolt; 
	/*!< Battery Voltage
	*/
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2mBatteryVoltage;

/*!
@enum	\
	tenuM2mReqGroup

@brief
*/
typedef enum{
	M2M_REQ_GROUP_MAIN = 0,
	M2M_REQ_GROUP_WIFI,
	M2M_REQ_GROUP_IP,
	M2M_REQ_GROUP_HIF,
	M2M_REQ_GROUP_OTA,
	M2M_REQ_GROUP_SSL,
	M2M_REQ_GROUP_CRYPTO,
	M2M_REQ_GROUP_SIGMA,
    } tenuM2mReqGroup;

/*!
@enum	\
	tenuM2mReqpkt

@brief
*/
typedef enum{
	M2M_REQ_CONFIG_PKT,
	M2M_REQ_DATA_PKT = 0x80 /*BIT7*/
    } tenuM2mReqpkt;
    
/*!
@enum	\
	tenuM2mConfigCmd

@brief
	This enum contains all the host commands used to configure the WINC board.

*/
typedef enum {
	M2M_WIFI_REQ_RESTART = M2M_CONFIG_CMD_BASE,
	/*!<
		Restart the WINC MAC layer, it's doesn't restart the IP layer.
	*/
	M2M_WIFI_REQ_SET_MAC_ADDRESS,
	/*!<
		Set the WINC mac address (not possible for production effused boards).
	*/
	M2M_WIFI_REQ_CURRENT_RSSI,
	/*!<
		Request the current connected AP RSSI.
	*/
	M2M_WIFI_RESP_CURRENT_RSSI,
	/*!<
		Response to M2M_WIFI_REQ_CURRENT_RSSI with the RSSI value.
	*/
	M2M_WIFI_REQ_GET_CONN_INFO,
	/*!< Request connection information command.
	*/
	M2M_WIFI_RESP_CONN_INFO,

	/*!< Connect with default AP response.
	*/
	M2M_WIFI_REQ_SET_DEVICE_NAME,
	/*!<
		Set the WINC device name property.
	*/
	M2M_WIFI_REQ_START_PROVISION_MODE,
	/*!<
		Start the provisioning mode for the M2M Device.
	*/
	M2M_WIFI_RESP_PROVISION_INFO,
	/*!<
		Send the provisioning information to the host.
	*/
	M2M_WIFI_REQ_STOP_PROVISION_MODE,
	/*!<
		Stop the current running provision mode.
	*/
	M2M_WIFI_REQ_SET_SYS_TIME,
	/*!<
		Set time of day from host.
	*/
	M2M_WIFI_REQ_ENABLE_SNTP_CLIENT,
	/*!<
		Enable the simple network time protocol to get the
		time from the Internet. this is required for security purposes.
	*/
	M2M_WIFI_REQ_DISABLE_SNTP_CLIENT,
	/*!<
		Disable the simple network time protocol for applications that
		do not need it.
	*/
	M2M_WIFI_RESP_MEMORY_RECOVER,
	/*!<
	 * Reserved for debuging
	 * */
	M2M_WIFI_REQ_CUST_INFO_ELEMENT,
	/*!< Add Custom ELement to Beacon Managament Frame.
	*/
	M2M_WIFI_REQ_SCAN,
	/*!< Request scan command.
	*/
	M2M_WIFI_RESP_SCAN_DONE,
	/*!< Scan complete notification response.
	*/
	M2M_WIFI_REQ_SCAN_RESULT,
	/*!< Request Scan results command.
	*/
	M2M_WIFI_RESP_SCAN_RESULT,
	/*!< Request Scan results resopnse.
	*/
	M2M_WIFI_REQ_SET_SCAN_OPTION,
	/*!< Set Scan options "slot time, slot number .. etc" .
	*/
	M2M_WIFI_REQ_SET_SCAN_REGION,
	/*!< Set scan region.
	*/
	M2M_WIFI_REQ_SET_POWER_PROFILE,
	/*!< The API shall set power mode to one of 3 modes
	*/
	M2M_WIFI_REQ_SET_TX_POWER,
	/*!<  API to set TX power. 
	*/
	M2M_WIFI_REQ_SET_BATTERY_VOLTAGE,
	/*!<  API to set Battery Voltage. 
	*/
	M2M_WIFI_REQ_SET_ENABLE_LOGS,
	/*!<  API to set Battery Voltage. 
	*/
	M2M_WIFI_REQ_GET_SYS_TIME,
	/*!<
		REQ GET time of day from WINC.
	*/
	M2M_WIFI_RESP_GET_SYS_TIME,
	/*!<
		RESP time of day from host.
	*/
	M2M_WIFI_REQ_SEND_ETHERNET_PACKET,
	/*!< Send Ethernet packet in bypass mode.
	*/
	M2M_WIFI_RESP_ETHERNET_RX_PACKET,
	/*!< Receive Ethernet packet in bypass mode.
	*/	
	M2M_WIFI_REQ_SET_MAC_MCAST,
	/*!< Set the WINC multicast filters.
	*/
	M2M_WIFI_REQ_GET_PRNG,
	/*!< Request PRNG.
	*/
	M2M_WIFI_RESP_GET_PRNG,
	/*!< Response for PRNG.
	*/
	M2M_WIFI_MAX_CONFIG_ALL,
    } tenuM2mConfigCmd;

/*!
@enum	\
	tenuM2mStaCmd
	
@brief
	This enum contains all the WINC commands while in Station mode.
*/
typedef enum {
	M2M_WIFI_REQ_CONNECT = M2M_STA_CMD_BASE,
	/*!< Connect with AP command.
	*/
	M2M_WIFI_REQ_DEFAULT_CONNECT,
	/*!< Connect with default AP command.
	*/
	M2M_WIFI_RESP_DEFAULT_CONNECT,
	/*!< Request connection information response.
	*/
	M2M_WIFI_REQ_DISCONNECT,
	/*!< Request to disconnect from AP command.
	*/
	M2M_WIFI_RESP_CON_STATE_CHANGED,
	/*!< Connection state changed response.
	*/
	M2M_WIFI_REQ_SLEEP,
	/*!< Set PS mode command.
	*/
	M2M_WIFI_REQ_WPS_SCAN,
	/*!< Request WPS scan command.
	*/
	M2M_WIFI_REQ_WPS,
	/*!< Request WPS start command.
	*/
	M2M_WIFI_REQ_START_WPS,
	/*!< This command is for internal use by the WINC and 
		should not be used by the host driver.
	*/
	M2M_WIFI_REQ_DISABLE_WPS,
	/*!< Request to disable WPS command.
	*/
	M2M_WIFI_REQ_DHCP_CONF,
	/*!< Response indicating that IP address was obtained.
	*/
	M2M_WIFI_RESP_IP_CONFIGURED,
	/*!< This command is for internal use by the WINC and 
		should not be used by the host driver.
	*/
	M2M_WIFI_RESP_IP_CONFLICT,
	/*!< Response indicating a conflict in obtained IP address.
		The user should re attempt the DHCP request.
	*/
	M2M_WIFI_REQ_ENABLE_MONITORING,
	/*!< Request to enable monitor mode  command.
	*/
	M2M_WIFI_REQ_DISABLE_MONITORING,
	/*!< Request to disable monitor mode  command.
	*/
	M2M_WIFI_RESP_WIFI_RX_PACKET,
	/*!< Indicate that a packet was received in monitor mode.
	*/
	M2M_WIFI_REQ_SEND_WIFI_PACKET,
	/*!< Send packet in monitor mode.
	*/
	M2M_WIFI_REQ_LSN_INT,
	/*!< Set WiFi listen interval.
	*/
	M2M_WIFI_REQ_DOZE,
	/*!< Used to force the WINC to sleep in manual PS mode.
	*/
	M2M_WIFI_MAX_STA_ALL,
    } tenuM2mStaCmd;

/*!
@enum	\
	tenuM2mApCmd

@brief
	This enum contains all the WINC commands while in AP mode.
*/
typedef enum {
	M2M_WIFI_REQ_ENABLE_AP = M2M_AP_CMD_BASE,
	/*!< Enable AP mode command.
	*/
	M2M_WIFI_REQ_DISABLE_AP,
	/*!< Disable AP mode command.
	*/
	M2M_WIFI_MAX_AP_ALL,
    } tenuM2mApCmd;

/*!
@enum	\
	tenuM2mP2pCmd

@brief
	This enum contains all the WINC commands while in P2P mode.
*/
typedef enum {
	M2M_WIFI_REQ_P2P_INT_CONNECT = M2M_P2P_CMD_BASE,
	/*!< This command is for internal use by the WINC and 
		should not be used by the host driver.
	*/
	M2M_WIFI_REQ_ENABLE_P2P,
	/*!< Enable P2P mode command.
	*/
	M2M_WIFI_REQ_DISABLE_P2P,
	/*!< Disable P2P mode command.
	*/
	M2M_WIFI_REQ_P2P_REPOST,
	/*!< This command is for internal use by the WINC and 
		should not be used by the host driver.
	*/
	M2M_WIFI_MAX_P2P_ALL,
    } tenuM2mP2pCmd;



/*!
@enum	\
	tenuM2mServerCmd

@brief
	This enum contains all the WINC commands while in PS mode.
	These command are currently not supported.
*/
typedef enum {
	M2M_WIFI_REQ_CLIENT_CTRL = M2M_SERVER_CMD_BASE,
	M2M_WIFI_RESP_CLIENT_INFO,
	M2M_WIFI_REQ_SERVER_INIT,
	M2M_WIFI_MAX_SERVER_ALL
}tenuM2mServerCmd;



/*!
@enum	\
	tenuM2mOtaCmd
	
@brief

*/
typedef enum {
	M2M_OTA_REQ_NOTIF_SET_URL = M2M_OTA_CMD_BASE,
	M2M_OTA_REQ_NOTIF_CHECK_FOR_UPDATE,
	M2M_OTA_REQ_NOTIF_SCHED,
	M2M_OTA_REQ_START_UPDATE,
	M2M_OTA_REQ_SWITCH_FIRMWARE,
	M2M_OTA_REQ_ROLLBACK,
	M2M_OTA_RESP_NOTIF_UPDATE_INFO,
	M2M_OTA_RESP_UPDATE_STATUS,
	M2M_OTA_REQ_TEST,
	M2M_OTA_MAX_ALL,
    } tenuM2mOtaCmd;

/*!
@enum	\
	tenuM2mCryptoCmd

@brief

*/
typedef enum {
	M2M_CRYPTO_REQ_SHA256_INIT = M2M_CRYPTO_CMD_BASE,
	M2M_CRYPTO_RESP_SHA256_INIT,
	M2M_CRYPTO_REQ_SHA256_UPDATE,
	M2M_CRYPTO_RESP_SHA256_UPDATE,
	M2M_CRYPTO_REQ_SHA256_FINSIH,
	M2M_CRYPTO_RESP_SHA256_FINSIH,
	M2M_CRYPTO_REQ_RSA_SIGN_GEN,
	M2M_CRYPTO_RESP_RSA_SIGN_GEN,
	M2M_CRYPTO_REQ_RSA_SIGN_VERIFY,
	M2M_CRYPTO_RESP_RSA_SIGN_VERIFY,
	M2M_CRYPTO_MAX_ALL,
    } tenuM2mCryptoCmd;

/*!
@enum	\
	tenuM2mIpCmd

@brief

*/
typedef enum {
	/* Request IDs corresponding to the IP GROUP. */
	M2M_IP_REQ_STATIC_IP_CONF = ((uint8) 10),
	M2M_IP_REQ_ENABLE_DHCP,
	M2M_IP_REQ_DISABLE_DHCP
    } tenuM2mIpCmd;

/*!
@enum	\
	tenuM2mSigmaCmd
	
@brief

*/
typedef enum {
	/* Request IDs corresponding to the IP GROUP. */
	M2M_SIGMA_ENABLE = ((uint8) 3),
	M2M_SIGMA_TA_START,
	M2M_SIGMA_TA_STATS,
	M2M_SIGMA_TA_RECEIVE_STOP,
	M2M_SIGMA_ICMP_ARP,
	M2M_SIGMA_ICMP_RX,
	M2M_SIGMA_ICMP_TX,
	M2M_SIGMA_UDP_TX,
	M2M_SIGMA_UDP_TX_DEFER,
	M2M_SIGMA_SECURITY_POLICY,
	M2M_SIGMA_SET_SYSTIME
    } tenuM2mSigmaCmd;


/*!
@enum	\
	tenuM2mConnState

@brief
	Wi-Fi Connection State.
*/
typedef enum {
	M2M_WIFI_DISCONNECTED = 0,
	/*!< Wi-Fi state is disconnected.
	*/
	M2M_WIFI_CONNECTED,
	/*!< Wi-Fi state is connected.
	*/
	M2M_WIFI_ROAMED,
	/*!< Wi-Fi state is roamed to new AP. https://asf.microchip.com/docs/latest/saml21/html/group___wlan_enums.html#ga460c311f58f4c40c3d20eec9e99db586
	*/
	M2M_WIFI_UNDEF = 0xff
	/*!< Undefined Wi-Fi State.
	*/
    } tenuM2mConnState;

/*!
@enum	\
	tenuM2mSecType

@brief
	Wi-Fi Supported Security types.
*/
typedef enum {
	M2M_WIFI_SEC_INVALID = 0,
	/*!< Invalid security type.
	*/
	M2M_WIFI_SEC_OPEN,
	/*!< Wi-Fi network is not secured.
	*/
	M2M_WIFI_SEC_WPA_PSK,
	/*!< Wi-Fi network is secured with WPA/WPA2 personal(PSK).
	*/
	M2M_WIFI_SEC_WEP,
	/*!< Security type WEP (40 or 104) OPEN OR SHARED.
	*/
	M2M_WIFI_SEC_802_1X
	/*!< Wi-Fi network is secured with WPA/WPA2 Enterprise.IEEE802.1x user-name/password authentication.
	 */
    } tenuM2mSecType;


/*!
@enum	\
	tenuM2mSecType

@brief
	Wi-Fi Supported SSID types.
*/
typedef enum {
	SSID_MODE_VISIBLE = 0,
	/*!< SSID is visible to others.
	*/
	SSID_MODE_HIDDEN
	/*!< SSID is hidden.
	*/
    } tenuM2mSsidMode;

/*!
@enum	\
	tenuM2mScanCh

@brief
	Wi-Fi RF Channels.
*/
typedef enum {
	M2M_WIFI_CH_1 = ((uint8) 0),
	M2M_WIFI_CH_2,
	M2M_WIFI_CH_3,
	M2M_WIFI_CH_4,
	M2M_WIFI_CH_5,
	M2M_WIFI_CH_6,
	M2M_WIFI_CH_7,
	M2M_WIFI_CH_8,
	M2M_WIFI_CH_9,
	M2M_WIFI_CH_10,
	M2M_WIFI_CH_11,
	M2M_WIFI_CH_12,
	M2M_WIFI_CH_13,
	M2M_WIFI_CH_14,
	M2M_WIFI_CH_ALL = ((uint8) 255)
    } tenuM2mScanCh;

/*!
@enum	\
	tenuM2mScanRegion

@brief
	Wi-Fi RF Channels.
*/
typedef enum {

	REG_CH_1 = ((uint16) 1 << 0),
	REG_CH_2 = ((uint16) 1 << 1),
	REG_CH_3 = ((uint16) 1 << 2),
	REG_CH_4 = ((uint16) 1 << 3),
	REG_CH_5 = ((uint16) 1 << 4),
	REG_CH_6 = ((uint16) 1 << 5),
	REG_CH_7 = ((uint16) 1 << 6),
	REG_CH_8 = ((uint16) 1 << 7),
	REG_CH_9 = ((uint16) 1 << 8),
	REG_CH_10 = ((uint16) 1 << 9),
	REG_CH_11 = ((uint16) 1 << 10),
	REG_CH_12 = ((uint16) 1 << 11),
	REG_CH_13 = ((uint16) 1 << 12),
	REG_CH_14 = ((uint16) 1 << 13),
	REG_CH_ALL = ((uint16) 0x3FFF),
	NORTH_AMERICA = ((uint16) 0x7FF),
	/** 11 channel
	*/
	EUROPE		=   ((uint16) 0x1FFF),
	/** 13 channel
	*/
	ASIA		=   ((uint16) 0x3FFF)
	/* 14 channel
	*/
    } tenuM2mScanRegion;


/*!
@enum	\
	tenuPowerSaveModes

@brief
	Power Save Modes.
*/
typedef enum {
	M2M_NO_PS,
	/*!< Power save is disabled.
	*/
	M2M_PS_AUTOMATIC,
	/*!< Power save is done automatically by the WINC.
		This mode doesn't disable all of the WINC modules and 
		use higher amount of power than the H_AUTOMATIC and 
		the DEEP_AUTOMATIC modes..
	*/
	M2M_PS_H_AUTOMATIC,
	/*!< Power save is done automatically by the WINC.
		Achieve higher power save than the AUTOMATIC mode
		by shutting down more parts of the WINC board.
	*/
	M2M_PS_DEEP_AUTOMATIC,
	/*!< Power save is done automatically by the WINC.
		Achieve the highest possible power save.
	*/
	M2M_PS_MANUAL
	/*!< Power save is done manually by the user.
	*/
    } tenuPowerSaveModes;

/*!
@enum	\
	tenuM2mWifiMode
	
@brief
	Wi-Fi Operation Mode.
*/
typedef enum {
	M2M_WIFI_MODE_NORMAL = ((uint8) 1),
	/*!< Normal Mode means to run customer firmware version.
	 */
	M2M_WIFI_MODE_ATE_HIGH,
	/*!< Config Mode in HIGH POWER means to run production test firmware version which is known as ATE (Burst) firmware.
	 */
	M2M_WIFI_MODE_ATE_LOW,
	/*!< Config Mode in LOW POWER means to run production test firmware version which is known as ATE (Burst) firmware.
	 */
	M2M_WIFI_MODE_ETHERNET,
	/*!< etherent Mode
	 */
	M2M_WIFI_MODE_MAX,
    } tenuM2mWifiMode;

/*!
@enum	\
	tenuWPSTrigger

@brief
	WPS Triggering Methods.
*/
typedef enum{
	WPS_PIN_TRIGGER = 0,
	/*!< WPS is triggered in PIN method.
	*/
	WPS_PBC_TRIGGER = 4
	/*!< WPS is triggered via push button.
	*/
    } tenuWPSTrigger;


/*!
@struct	\
	tstrM2mWifiWepParams

@brief
	WEP security key parameters.
*/
typedef struct{
	uint8	u8KeyIndx;
	/*!< Wep key Index.
	*/
	uint8	u8KeySz;
	/*!< Wep key Size.
	*/
	uint8	au8WepKey[WEP_104_KEY_STRING_SIZE + 1];
	/*!< WEP Key represented as a NULL terminated ASCII string.
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes to keep the structure word alligned.
	*/
    } tstrM2mWifiWepParams;


/*!
@struct	\
	tstr1xAuthCredentials

@brief
	Credentials for the user to authenticate with the AAA server (WPA-Enterprise Mode IEEE802.1x).
*/
typedef struct{
	uint8	au8UserName[M2M_1X_USR_NAME_MAX];
	/*!< User Name. It must be Null terminated string.
	*/
	uint8	au8Passwd[M2M_1X_PWD_MAX];
	/*!< Password corresponding to the user name. It must be Null terminated string.
	*/
    } tstr1xAuthCredentials;


/*!
@union	\
	tuniM2MWifiAuth

@brief
	Wi-Fi Security Parameters for all supported security modes.
*/
typedef union{
	uint8				au8PSK[M2M_MAX_PSK_LEN];
	/*!< Pre-Shared Key in case of WPA-Personal security.
	*/
	tstr1xAuthCredentials	strCred1x;
	/*!< Credentials for RADIUS server authentication in case of WPA-Enterprise security.
	*/
	tstrM2mWifiWepParams	strWepInfo;
	/*!< WEP key parameters in case of WEP security.
	*/
    } tuniM2MWifiAuth;


/*!
@struct	\
	tstrM2MWifiSecInfo

@brief
	Authentication credentials to connect to a Wi-Fi network.
*/
typedef struct{
	tuniM2MWifiAuth		uniAuth;
	/*!< Union holding all possible authentication parameters corresponding the current security types.
	*/
	uint8				u8SecType;
	/*!< Wi-Fi network security type. See tenuM2mSecType for supported security types.
	*/
#define __PADDING__		(4 - ((sizeof(tuniM2MWifiAuth) + 1) % 4))
	uint8				__PAD__[__PADDING__];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2MWifiSecInfo;


/*!
@struct	\
	tstrM2mWifiConnect

@brief
	Wi-Fi Connect Request
*/
typedef struct{
	tstrM2MWifiSecInfo		strSec;
	/*!< Security parameters for authenticating with the AP.
	*/
	uint16				u16Ch;
	/*!< RF Channel for the target SSID.
	*/
	uint8				au8SSID[M2M_MAX_SSID_LEN];
	/*!< SSID of the desired AP. It must be NULL terminated string.
	*/
	uint8 				u8NoSaveCred;
#define __CONN_PAD_SIZE__		(4 - ((sizeof(tstrM2MWifiSecInfo) + M2M_MAX_SSID_LEN + 3) % 4))
	uint8				__PAD__[__CONN_PAD_SIZE__];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2mWifiConnect;


/*!
@struct	\
	tstrM2MWPSConnect

@brief
	WPS Configuration parameters

@sa
	tenuWPSTrigger
*/
typedef struct {
	uint8 	u8TriggerType;
	/*!< WPS triggering method (Push button or PIN)
	*/
	char         acPinNumber[8];
	/*!< WPS PIN No (for PIN method)
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2MWPSConnect;


/*!
@struct	\
	tstrM2MWPSInfo

@brief	WPS Result

	This structure is passed to the application in response to a WPS request. If the WPS session is completed successfully, the
	structure will have Non-ZERO authentication type. If the WPS Session fails (due to error or timeout) the authentication type
	is set to ZERO.

@sa
	tenuM2mSecType
*/
typedef struct{
	uint8	u8AuthType;
	/*!< Network authentication type.
	*/
	uint8   	u8Ch;
	/*!< RF Channel for the AP.
	*/
	uint8	au8SSID[M2M_MAX_SSID_LEN];
	/*!< SSID obtained from WPS.
	*/
	uint8	au8PSK[M2M_MAX_PSK_LEN];
	/*!< PSK for the network obtained from WPS.
	*/
    } tstrM2MWPSInfo;


/*!
@struct	\
	tstrM2MDefaultConnResp

@brief
	Response error of the m2m_default_connect

@sa
	M2M_DEFAULT_CONN_SCAN_MISMATCH
	M2M_DEFAULT_CONN_EMPTY_LIST
*/
typedef struct{
	sint8		s8ErrorCode;
	/*!<
		Default connect error code. possible values are:
		- M2M_DEFAULT_CONN_EMPTY_LIST
		- M2M_DEFAULT_CONN_SCAN_MISMATCH
	*/
	uint8	__PAD24__[3];
    } tstrM2MDefaultConnResp;

/*!
@struct	\
	tstrM2MScan

@brief
	Wi-Fi Scan Request

@sa
	tenuM2mScanCh
*/
typedef struct {
	uint8   u8NumOfSlot;
	/*|< The min number of slots is 2 for every channel,
	every slot the soc will send Probe Request on air, and wait/listen for PROBE RESP/BEACONS for the u16slotTime
	*/
	uint8   u8SlotTime;
	/*|< the time that the Soc will wait on every channel listening to the frames on air
		when that time increaseed number of AP will increased in the scan results
		min time is 10 ms and the max is 250 ms
	*/
	uint8  u8ProbesPerSlot;
	/*!< Number of probe requests to be sent per channel scan slot.
	*/
	sint8   s8RssiThresh;
	/*! < The RSSI threshold of the AP which will be connected to directly.
	*/

    } tstrM2MScanOption;

/*!
@struct	\
	tstrM2MScanRegion

@brief
	Wi-Fi channel regulation region information.

@sa
	tenuM2mScanRegion
*/
typedef struct {
	uint16   u16ScanRegion;
	/*|< Specifies the number of channels allowed in the region (e.g. North America = 11 ... etc.).
	*/
	uint8 __PAD16__[2];

    } tstrM2MScanRegion;

/*!
@struct	\
	tstrM2MScan

@brief
	Wi-Fi Scan Request

@sa
	tenuM2mScanCh
*/
typedef struct {
	uint8 	u8ChNum;
	/*!< The Wi-Fi RF Channel number
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment
	*/

    } tstrM2MScan;

/*!
@struct	\
	tstrCyptoResp

@brief
	crypto response
*/
typedef struct {
	sint8 s8Resp;
	/***/
	uint8 __PAD24__[3];
	/*
	*/
    } tstrCyptoResp;


/*!
@struct	\
	tstrM2mScanDone

@brief
	Wi-Fi Scan Result
*/
typedef struct{
	uint8 	u8NumofCh;
	/*!< Number of found APs
	*/
	sint8 	s8ScanState;
	/*!< Scan status
	*/
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2mScanDone;


/*!
@struct	\
	tstrM2mReqScanResult

@brief	Scan Result Request

	The Wi-Fi Scan results list is stored in Firmware. The application can request a certain scan result by its index.
*/
typedef struct {
	uint8 	u8Index;
	/*!< Index of the desired scan result
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2mReqScanResult;


/*!
@struct	\
	tstrM2mWifiscanResult

@brief	Wi-Fi Scan Result

	Information corresponding to an AP in the Scan Result list identified by its order (index) in the list.
*/
typedef struct {
	uint8 	u8index;
	/*!< AP index in the scan result list.
	*/
	sint8 	s8rssi;
	/*!< AP signal strength.
	*/
	uint8 	u8AuthType;
	/*!< AP authentication type.
	*/
	uint8 	u8ch;
	/*!< AP RF channel.
	*/
	uint8	au8BSSID[6];
	/*!< BSSID of the AP.
	*/
	uint8 	au8SSID[M2M_MAX_SSID_LEN];
	/*!< AP ssid.
	*/
	uint8 	_PAD8_;
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2mWifiscanResult;


/*!
@struct	\
	tstrM2mWifiStateChanged

@brief
	Wi-Fi Connection State

@sa
	M2M_WIFI_DISCONNECTED, M2M_WIFI_CONNECTED, M2M_WIFI_REQ_CON_STATE_CHANGED,tenuM2mConnChangedErrcode
*/
typedef struct {
	uint8	u8CurrState;
	/*!< Current Wi-Fi connection state
	*/
	uint8  u8ErrCode;
	/*!< Error type review tenuM2mConnChangedErrcode
	*/
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2mWifiStateChanged;


/*!
@struct	\
	tstrM2mPsType

@brief
	Power Save Configuration

@sa
	tenuPowerSaveModes
*/
typedef struct{
	uint8 	u8PsType;
	/*!< Power save operating mode
	*/
	uint8 	u8BcastEn;
	/*!<
	*/
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2mPsType;

/*!
@struct	\
	tstrM2mSlpReqTime

@brief
	Manual power save request sleep time

*/
typedef struct {
	/*!< Sleep time in ms
	*/
	uint32 u32SleepTime;

    } tstrM2mSlpReqTime;

/*!
@struct	\
	tstrM2mLsnInt

@brief	Listen interval

	It is the value of the Wi-Fi STA listen interval for power saving. It is given in units of Beacon period. 
	Periodically after the listen interval fires, the WINC is wakeup and listen to the beacon and check for any buffered frames for it from the AP.
*/
typedef struct {
	uint16 	u16LsnInt;
	/*!< Listen interval in Beacon period count.
	*/
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
}tstrM2mLsnInt;


/*!
@struct	\
	tstrM2MWifiMonitorModeCtrl

@brief	Wi-Fi Monitor Mode Filter

	This structure sets the filtering criteria for WLAN packets when monitoring mode is enable. 
	The received packets matching the filtering parameters, are passed directly to the application.
*/
typedef struct{
	uint8	u8ChannelID;
	/* !< RF Channel ID. It must use values from tenuM2mScanCh
	*/
	uint8	u8FrameType;
	/*!< It must use values from tenuWifiFrameType.
	*/
	uint8	u8FrameSubtype;
	/*!< It must use values from tenuSubTypes.
	*/
	uint8	au8SrcMacAddress[6];
	/* ZERO means DO NOT FILTER Source address.
	*/
	uint8	au8DstMacAddress[6];
	/* ZERO means DO NOT FILTER Destination address.
	*/
	uint8	au8BSSID[6];
	/* ZERO means DO NOT FILTER BSSID.
	*/
	uint8 u8EnRecvHdr;
	/*
	 Enable recv the full hder before the payload	
	*/
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2MWifiMonitorModeCtrl;


/*!
@struct	\
	tstrM2MWifiRxPacketInfo

@brief	Wi-Fi RX Frame Header

	The M2M application has the ability to allow Wi-Fi monitoring mode for receiving all Wi-Fi Raw frames matching a well defined filtering criteria.
	When a target Wi-Fi packet is received, the header information are extracted and assigned in this structure. 
*/
typedef struct {
	uint8	u8FrameType;
	/*!< It must use values from tenuWifiFrameType.
	*/
	uint8	u8FrameSubtype;
	/*!< It must use values from tenuSubTypes.
	*/
	uint8	u8ServiceClass;
	/*!< Service class from Wi-Fi header.
	*/
	uint8	u8Priority;
	/*!< Priority from Wi-Fi header.
	*/
	uint8	u8HeaderLength;
	/*!< Frame Header length.
	*/
	uint8	u8CipherType;
	/*!< Encryption type for the rx packet.
	*/
	uint8	au8SrcMacAddress[6];
	/* ZERO means DO NOT FILTER Source address.
	*/
	uint8	au8DstMacAddress[6];
	/* ZERO means DO NOT FILTER Destination address.
	*/
	uint8	au8BSSID[6];
	/* ZERO means DO NOT FILTER BSSID.
	*/
	uint16	u16DataLength;
	/*!< Data payload length (Header excluded).
	*/
	uint16	u16FrameLength;
	/*!< Total frame length (Header + Data).
	*/
	uint32	u32DataRateKbps;
	/*!< Data Rate in Kbps.
	*/
	sint8		s8RSSI;
	/*!< RSSI.
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2MWifiRxPacketInfo;


/*!
@struct	\
	tstrM2MWifiTxPacketInfo

@brief	Wi-Fi TX Packet Info

	The M2M Application has the ability to compose a RAW Wi-Fi frames (under the application responsibility).
	When transmitting a Wi-Fi packet, the application must supply the firmware with this structure for sending the target frame.
*/
typedef struct {
	uint16	u16PacketSize;
	/*!< Wlan frame length.
	*/
	uint16	u16HeaderLength;
	/*!< Wlan frame header length.
	*/
    } tstrM2MWifiTxPacketInfo;


/*!
 @struct	\
 	tstrM2MP2PConnect

 @brief
 	Set the device to operate in the Wi-Fi Direct (P2P) mode.
*/
typedef struct {
	uint8 	u8ListenChannel;
	/*!< P2P Listen Channel (1, 6 or 11)
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2MP2PConnect;

/*!
@struct	\
	tstrM2MAPConfig

@brief	AP Configuration

	This structure holds the configuration parameters for the M2M AP mode. It should be set by the application when
	it requests to enable the M2M AP operation mode. The M2M AP mode currently supports only WEP security (with
	the NO Security option available of course).
*/
typedef struct {
	/*!<
		Configuration parameters for the WiFi AP.
	*/
	uint8 	au8SSID[M2M_MAX_SSID_LEN];
	/*!< AP SSID
	*/
	uint8 	u8ListenChannel;
	/*!< Wi-Fi RF Channel which the AP will operate on
	*/
	uint8	u8KeyIndx;
	/*!< Wep key Index
	*/
	uint8	u8KeySz;
	/*!< Wep key Size
	*/
	uint8	au8WepKey[WEP_104_KEY_STRING_SIZE + 1];
	/*!< Wep key
	*/
	uint8 	u8SecType;
	/*!< Security type: Open or WEP only in the current implementation
	*/
	uint8 	u8SsidHide;
	/*!< SSID Status "Hidden(1)/Visible(0)"
	*/
	uint8	au8DHCPServerIP[4];
	/*!< Ap IP server address
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing alignment
	*/
    } tstrM2MAPConfig;


/*!
@struct	\
	tstrM2mServerInit

@brief
	PS Server initialization.
*/
typedef struct {
	uint8 	u8Channel;
	/*!< Server Listen channel
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2mServerInit;


/*!
@struct	\
	tstrM2mClientState

@brief
	PS Client State.
*/
typedef struct {
	uint8 	u8State;
	/*!< PS Client State
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2mClientState;


/*!
@struct	\
	tstrM2Mservercmd

@brief
	PS Server CMD
*/
typedef struct {
	uint8	u8cmd;
	/*!< PS Server Cmd
	*/
	uint8	__PAD24__[3];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2Mservercmd;


/*!
@struct	\
	tstrM2mSetMacAddress

@brief
	Sets the MAC address from application. The WINC load the mac address from the effuse by default to the WINC configuration memory, 
	but that function is used to let the application overwrite the configuration memory with the mac address from the host.

@note
	It's recommended to call this only once before calling connect request and after the m2m_wifi_init
*/
typedef struct {
	uint8 	au8Mac[6];
	/*!< MAC address array
	*/
	uint8	__PAD16__[2];
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2mSetMacAddress;


/*!
@struct	\
 	tstrM2MDeviceNameConfig

@brief	Device name

	It is assigned by the application. It is used mainly for Wi-Fi Direct device
	discovery and WPS device information.
*/
typedef struct {
	uint8 	au8DeviceName[M2M_DEVICE_NAME_MAX];
	/*!< NULL terminated device name
	*/
    } tstrM2MDeviceNameConfig;


/*!
@struct	\
 	tstrM2MIPConfig

@brief
 	Static IP configuration.

@note
 	All member IP addresses are expressed in Network Byte Order (eg. "192.168.10.1" will be expressed as 0x010AA8C0).
 */
typedef struct {
	uint32 	u32StaticIP;
	/*!< The static IP assigned to the device.
	*/
	uint32 	u32Gateway;
	/*!< IP of the Default internet gateway.
	*/
	uint32 	u32DNS;
	/*!< IP for the DNS server.
	*/
	uint32 	u32SubnetMask;
	/*!< Subnet mask for the local area network.
	*/
    } tstrM2MIPConfig;

/*!
@struct	\
 	tstrM2mIpRsvdPkt

@brief
 	Received Packet Size and Data Offset

 */
typedef struct{
	uint16	u16PktSz;
	uint16	u16PktOffset;
    } tstrM2mIpRsvdPkt;


/*!
@struct	\
 	tstrM2MProvisionModeConfig

@brief
 	M2M Provisioning Mode Configuration
 */

typedef struct {
	tstrM2MAPConfig	strApConfig;
	/*!<
		Configuration parameters for the WiFi AP.
	*/
	char				acHttpServerDomainName[64];
	/*!<
		The device domain name for HTTP provisioning.
	*/
	uint8			u8EnableRedirect;
	/*!<
		A flag to enable/disable HTTP redirect feature for the HTTP Provisioning server. If the Redirect is enabled,
		all HTTP traffic (http://URL) from the device associated with WINC AP will be redirected to the HTTP Provisioning Web page.
		- 0 : Disable HTTP Redirect.
		- 1 : Enable HTTP Redirect.
	*/
	uint8			__PAD24__[3];
    } tstrM2MProvisionModeConfig;


/*!
@struct	\
 	tstrM2MProvisionInfo

@brief
 	M2M Provisioning Information obtained from the HTTP Provisioning server.
 */
typedef struct{
	uint8	au8SSID[M2M_MAX_SSID_LEN];
	/*!<
		Provisioned SSID.
	*/
	uint8	au8Password[M2M_MAX_PSK_LEN];
	/*!<
		Provisioned Password.
	*/
	uint8	u8SecType;
	/*!<
		Wifi Security type.
	*/
	uint8	u8Status;
	/*!<
		Provisioning status. It must be checked before reading the provisioning information. It may be
		- M2M_SUCCESS 	: Provision successful.
		- M2M_FAIL		: Provision Failed.
	*/
    } tstrM2MProvisionInfo;


/*!
@struct	\
 	tstrM2MConnInfo

@brief
 	M2M Provisioning Information obtained from the HTTP Provisioning server.
 */
typedef struct {
	char	acSSID[M2M_MAX_SSID_LEN];
	/*!< AP connection SSID name  */
	uint8	u8SecType;
	/*!< Security type */
	uint8	au8IPAddr[4];
	/*!< Connection IP address */
	uint8	au8MACAddress[6];
	/*!< MAC address of the peer Wi-Fi station */ 
	sint8	s8RSSI;
	/*!< Connection RSSI signal */
    uint8 	u8CurrChannel;
//    uint8 	u8SecType;  //https://asf.microchip.com/docs/latest/saml21/html/structtstr_m2_m_conn_info.html
	uint8	__PAD24__[1];
	/*!< Padding bytes for forcing 4-byte alignment */
    } tstrM2MConnInfo;

/*!
@struct	\
 	tstrOtaInitHdr

@brief
 	OTA Image Header 
 */

typedef struct {
	uint32 u32OtaMagicValue;
	/*!< Magic value kept in the OTA image after the 
	sha256 Digest buffer to define the Start of OTA Header */
	uint32 u32OtaPayloadSzie;
	/*!<
	The Total OTA image payload size, include the sha256 key size
	*/

    } tstrOtaInitHdr;

/*!
@struct	\
 	tstrOtaControlSec

@brief
 	Control section structure is used to define the working image and 
	the validity of the roll-back image and its offset, also both firmware versions is kept in that structure.
 */

typedef struct {
	uint32 u32OtaMagicValue;
/*!<
	Magic value used to ensure the structure is valid or not 
*/
	uint32 u32OtaFormatVersion;
/*!<
		NA   NA   NA   Flash version   cs struct version
		00   00   00   00              00 
	Control structure format version, the value will be incremented in case of structure changed or updated
*/
	uint32 u32OtaSequenceNumber;
/*!<
	Sequence number is used while update the control structure to keep track of how many times that section updated 
*/
	uint32 u32OtaLastCheckTime;
/*!<
	Last time OTA check for update
*/
	uint32 u32OtaCurrentworkingImagOffset;
/*!<
	Current working offset in flash 
*/
	uint32 u32OtaCurrentworkingImagFirmwareVer;
/*!<
	current working image version ex 18.0.1
*/
	uint32 u32OtaRollbackImageOffset;
/*!<
	Roll-back image offset in flash 
*/
	uint32 u32OtaRollbackImageValidStatus;
/*!<
	roll-back image valid status 
*/
	uint32 u32OtaRollbackImagFirmwareVer;
/*!<
	Roll-back image version (ex 18.0.3)
*/
	uint32 u32OtaCortusAppWorkingOffset;
/*!<
	cortus app working offset in flash 
*/
	uint32 u32OtaCortusAppWorkingValidSts;
/*!<
	Working Cortus app valid status 
*/
	uint32 u32OtaCortusAppWorkingVer;
/*!<
	Working cortus app version (ex 18.0.3)
*/
	uint32 u32OtaCortusAppRollbackOffset;
/*!<
	cortus app rollback offset in flash 
*/
	uint32 u32OtaCortusAppRollbackValidSts;
/*!<
	roll-back cortus app valid status 
*/
	uint32 u32OtaCortusAppRollbackVer;
/*!<
	Roll-back cortus app version (ex 18.0.3)
*/
	uint32 u32OtaControlSecCrc;
/*!<
	CRC for the control structure to ensure validity 
*/
    } tstrOtaControlSec;

/*!
@enum	\
	tenuOtaUpdateStatus

@brief
	OTA return status
*/
typedef enum {
	OTA_STATUS_SUCSESS        = 0,
	/*!< OTA Success with not errors. */
	OTA_STATUS_FAIL           = 1,
	/*!< OTA generic fail. */
	OTA_STATUS_INVAILD_ARG    = 2,
	/*!< Invalid or malformed download URL. */
	OTA_STATUS_INVAILD_RB_IMAGE    = 3,
	/*!< Invalid rollback image. */
	OTA_STATUS_INVAILD_FLASH_SIZE    = 4,
	/*!< Flash size on device is not enough for OTA. */
	OTA_STATUS_AlREADY_ENABLED    = 5,
	/*!< An OTA operation is already enabled. */
	OTA_STATUS_UPDATE_INPROGRESS    = 6,
	/*!< An OTA operation update is in progress */
	OTA_STATUS_IMAGE_VERIF_FAILED = 7,
	/*!<  OTA Verfication failed */
	OTA_STATUS_CONNECTION_ERROR = 8,
	/*!< OTA connection error */
	OTA_STATUS_SERVER_ERROR = 9,
	/*!< OTA server Error (file not found or else ...) */
    } tenuOtaUpdateStatus;
/*!
@enum	\
	tenuOtaUpdateStatusType

@brief
	OTA update Status type
*/
typedef enum {

	DL_STATUS        = 1,
	/*!< Download OTA file status
	*/
	SW_STATUS        = 2,
	/*!< Switching to the upgrade firmware status
	*/
	RB_STATUS        = 3,
	/*!< Roll-back status
	*/
    } tenuOtaUpdateStatusType;


/*!
@struct	\
	tstrOtaUpdateStatusResp

@brief
	OTA Update Information

@sa
	tenuWPSTrigger
*/
typedef struct {
	uint8	u8OtaUpdateStatusType;
	/*!<
		Status type tenuOtaUpdateStatusType
	*/
	uint8	u8OtaUpdateStatus;
	/*!<
	OTA_SUCCESS 						
	OTA_ERR_WORKING_IMAGE_LOAD_FAIL		
	OTA_ERR_INVAILD_CONTROL_SEC			
	M2M_ERR_OTA_SWITCH_FAIL     		
	M2M_ERR_OTA_START_UPDATE_FAIL     	
	M2M_ERR_OTA_ROLLBACK_FAIL     		
	M2M_ERR_OTA_INVAILD_FLASH_SIZE     	
	M2M_ERR_OTA_INVAILD_ARG		     
	*/
	uint8 _PAD16_[2];
    } tstrOtaUpdateStatusResp;


/*!
@struct	\
	tstrSystemTime

@brief
	Used for time storage.
*/
typedef struct{
	uint16	u16Year;
	uint8	u8Month;
	uint8	u8Day;
	uint8	u8Hour;
	uint8	u8Minute;
	uint8	u8Second;
    } tstrSystemTime;

/*!
@struct	\
 	tstrM2MMulticastMac

@brief
 	M2M add/remove multi-cast mac address
 */
 typedef struct {
	uint8 au8macaddress[M2M_MAC_ADDRES_LEN];
	/*!<
		Mac address needed to be added or removed from filter.
	*/
	uint8 u8AddRemove;
	/*!<
		set by 1 to add or 0 to remove from filter.
	*/
	uint8	__PAD8__;
	/*!< Padding bytes for forcing 4-byte alignment
	*/
    } tstrM2MMulticastMac;

/*!
@struct	\
 	tstrPrng

@brief
 	M2M Request PRNG
 */
 typedef struct {
	 /*!<
		return buffer address
	*/
	uint8 *pu8RngBuff;
	 /*!<
		PRNG size requested
	*/
	uint16 	u16PrngSize;
	/*!<
		PRNG pads
	*/
	uint8 __PAD16__[2];
    } tstrPrng;


 /**@}*/

#endif



// mtm_wifi.h *************************************************************************************************

#ifndef __M2M_WIFI_H__
#define __M2M_WIFI_H__

/** \defgroup m2m_wifi WLAN
 *
 */

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

//#include "common/include/nm_common.h"
//#include "driver/include/m2m_types.h"
//#include "driver/source/nmdrv.h"

#ifdef CONF_MGMT


/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/**@defgroup  WlanEnums Enumerations and Typedefs
 * @ingroup m2m_wifi
 * @{*/
/*!
@enum	\
	tenuWifiFrameType

@brief
	Enumeration for Wi-Fi MAC frame type codes (2-bit) 
	The following types are used to identify the type of frame sent or received.
	Each frame type constitutes a number of frame subtypes as defined in @ref tenuSubTypes to specify the exact type of frame.
	Values are defined as per the IEEE 802.11 standard.
	
@remarks
	The following frame types are useful for advanced user usage when @ref CONF_MGMT is defined
	and the user application requires to monitor the frame transmission and reception.
@see
	tenuSubTypes
*/
typedef enum {
	MANAGEMENT            = 0x00,
	/*!< Wi-Fi Management frame (Probe Req/Resp, Beacon, Association Req/Resp ...etc).
	*/
	CONTROL               = 0x04,
	/*!< Wi-Fi Control frame (eg. ACK frame).
	*/
	DATA_BASICTYPE        = 0x08,
	/*!< Wi-Fi Data frame.
	*/
	RESERVED              = 0x0C,

	M2M_WIFI_FRAME_TYPE_ANY	= 0xFF
/*!< Set monitor mode to receive any of the frames types
*/
}tenuWifiFrameType;


/*!
@enum	\
	tenuSubTypes

@brief
	Enumeration for Wi-Fi MAC Frame subtype code (6-bit).
	The frame subtypes fall into one of the three frame type groups as defined in @ref tenuWifiFrameType
	(MANAGEMENT, CONTROL & DATA).
	Values are defined as per the IEEE 802.11 standard.
@remarks
	The following sub-frame types are useful for advanced user usage when @ref CONF_MGMT is defined
	and the application developer requires to monitor the frame transmission and reception.
@see
    	tenuWifiFrameType
*/
typedef enum {
	/*!< Sub-Types related to Management Sub-Types */
	ASSOC_REQ             = 0x00,
	ASSOC_RSP             = 0x10,
	REASSOC_REQ           = 0x20,
	REASSOC_RSP           = 0x30,
	PROBE_REQ             = 0x40,
	PROBE_RSP             = 0x50,
	BEACON                = 0x80,
	ATIM                  = 0x90,
	DISASOC               = 0xA0,
	AUTH                  = 0xB0,
	DEAUTH                = 0xC0,
	ACTION                = 0xD0,
/**@{*/ 
	/* Sub-Types related to Control */
	PS_POLL               = 0xA4,
	RTS                   = 0xB4,
	CTS                   = 0xC4,
	ACK                   = 0xD4,
	CFEND                 = 0xE4,
	CFEND_ACK             = 0xF4,
	BLOCKACK_REQ          = 0x84,
	BLOCKACK              = 0x94,
/**@{*/ 
	/* Sub-Types related to Data */
	DATA                  = 0x08,
	DATA_ACK              = 0x18,
	DATA_POLL             = 0x28,
	DATA_POLL_ACK         = 0x38,
	NULL_FRAME            = 0x48,
	CFACK                 = 0x58,
	CFPOLL                = 0x68,
	CFPOLL_ACK            = 0x78,
	QOS_DATA              = 0x88,
	QOS_DATA_ACK          = 0x98,
	QOS_DATA_POLL         = 0xA8,
	QOS_DATA_POLL_ACK     = 0xB8,
	QOS_NULL_FRAME        = 0xC8,
	QOS_CFPOLL            = 0xE8,
	QOS_CFPOLL_ACK        = 0xF8,
	M2M_WIFI_FRAME_SUB_TYPE_ANY = 0xFF
	/*!< Set monitor mode to receive any of the frames types
	*/
}tenuSubTypes;


/*!
@enum	\
	tenuInfoElementId

@brief
	Enumeration for the Wi-Fi Information Element(IE) IDs, which indicates the specific type of IEs.
	IEs are management frame information included in management frames.
	Values are defined as per the IEEE 802.11 standard.

@details	Available IDs are:-
	
	ISSID   :	 Service Set Identifier (SSID)
	
	ISUPRATES   :	 Supported Rates
	
	IFHPARMS     :	 FH parameter set
	
	IDSPARMS      :       DS parameter set
	
	ICFPARMS      :        CF parameter set
	
	ITIM            :    	 Traffic Information Map

	IIBPARMS        :    	 IBSS parameter set
	
	ICOUNTRY        :  	 Country element.
	
	IEDCAPARAMS     :      EDCA parameter set
	
	ITSPEC              :  	 Traffic Specification

	ITCLAS             :  	 Traffic Classification
	
	ISCHED             :     Schedule.
	
	ICTEXT              :  	 Challenge Text
	
	IPOWERCONSTRAINT   :   	 Power Constraint.
	
	IPOWERCAPABILITY    :  	 Power Capability
	
	ITPCREQUEST         :   	 TPC Request                    
	
	ITPCREPORT          :   	 TPC Report                     
	
	ISUPCHANNEL         :   Supported channel list  
	
	ICHSWANNOUNC         :  Channel Switch Announcement    
	
	IMEASUREMENTREQUEST 	:	 Measurement request            
	
	IMEASUREMENTREPORT   :  Measurement report             
	
	IQUIET               :  Quiet element Info             
	
	IIBSSDFS           :  	 IBSS DFS                       
	
	IERPINFO           :  	 ERP Information                
	
	ITSDELAY           :    	 TS Delay                       
	
	ITCLASPROCESS      :	 TCLAS Processing               
	
	IHTCAP               :  	 HT Capabilities                
	
	IQOSCAP             :  	 QoS Capability                 
	
	IRSNELEMENT         :   RSN Information Element        
	
	IEXSUPRATES         :   Extended Supported Rates       
	
	IEXCHSWANNOUNC      :  Extended Ch Switch Announcement
	
	IHTOPERATION        :  	 HT Information                 
	
	ISECCHOFF          :   Secondary Channel Offset      
	
	I2040COEX           :   Coexistence IE           
	
	I2040INTOLCHREPORT  :   Intolerant channel report
	
	IOBSSSCAN           :  	 OBSS Scan parameters           
	
	IEXTCAP             :  	 Extended capability          
	
	IWMM                :  	 WMM parameters                 
	
	IWPAELEMENT         :  WPA Information Element
	
*/
typedef enum {
	ISSID               = 0,
	/*!< Service Set Identifier (SSID)
	*/
	ISUPRATES           = 1,
	/*!< Supported Rates
	*/
	IFHPARMS            = 2,
	/*!< FH parameter set
	*/
	IDSPARMS            = 3,
	/*!< DS parameter set
	*/
	ICFPARMS            = 4,
	/*!< CF parameter set
	*/
	ITIM                = 5,
	/*!< Traffic Information Map
	*/
	IIBPARMS            = 6,
	/*!< IBSS parameter set
	*/
	ICOUNTRY            = 7,
	/*!< Country element.
	*/
	IEDCAPARAMS         = 12,
	/*!< EDCA parameter set
	*/
	ITSPEC              = 13,
	/*!< Traffic Specification
	*/
	ITCLAS              = 14,
	/*!< Traffic Classification
	*/
	ISCHED              = 15,
	/*!< Schedule.
	*/
	ICTEXT              = 16,
	/*!< Challenge Text
	*/
	IPOWERCONSTRAINT    = 32,
	/*!< Power Constraint.
	*/
	IPOWERCAPABILITY    = 33,
	/*!< Power Capability
	*/
	ITPCREQUEST         = 34,
	/*!< TPC Request
	*/
	ITPCREPORT          = 35,
	/*!< TPC Report
	*/
	ISUPCHANNEL         = 36,
	/* Supported channel list
	*/
	ICHSWANNOUNC        = 37,
	/*!< Channel Switch Announcement
	*/
	IMEASUREMENTREQUEST = 38,
	/*!< Measurement request
	*/
	IMEASUREMENTREPORT  = 39,
	/*!< Measurement report
	*/
	IQUIET              = 40,
	/*!< Quiet element Info
	*/
	IIBSSDFS            = 41,
	/*!< IBSS DFS
	*/
	IERPINFO            = 42,
	/*!< ERP Information
	*/
	ITSDELAY            = 43,
	/*!< TS Delay
	*/
	ITCLASPROCESS       = 44,
	/*!< TCLAS Processing
	*/
	IHTCAP              = 45,
	/*!< HT Capabilities
	*/
	IQOSCAP             = 46,
	/*!< QoS Capability
	*/
	IRSNELEMENT         = 48,
	/*!< RSN Information Element
	*/
	IEXSUPRATES         = 50,
	/*!< Extended Supported Rates
	*/
	IEXCHSWANNOUNC      = 60,
	/*!< Extended Ch Switch Announcement
	*/
	IHTOPERATION        = 61,
	/*!< HT Information
	*/
	ISECCHOFF           = 62,
	/*!< Secondary Channel Offset
	*/
	I2040COEX           = 72,
	/*!< 20/40 Coexistence IE
	*/
	I2040INTOLCHREPORT  = 73,
	/*!< 20/40 Intolerant channel report
	*/
	IOBSSSCAN           = 74,
	/*!< OBSS Scan parameters
	*/
	IEXTCAP             = 127,
	/*!< Extended capability
	*/
	IWMM                = 221,
	/*!< WMM parameters
	*/
	IWPAELEMENT         = 221
	/*!< WPA Information Element
	*/
}tenuInfoElementId;
 //@}

/*!
@struct	\
	tenuWifiCapability

@brief
	Enumeration for capability Information field bit. 
	The value of the capability information field from the 802.11 management frames received by the wireless LAN interface. 
	Defining the capabilities of the Wi-Fi system. Values are defined as per the IEEE 802.11 standard.

@details
	Capabilities:-
	ESS/IBSS             : Defines whether a frame is coming from an AP or not.              
	POLLABLE    		: CF Poll-able                  
	POLLREQ       		: Request to be polled         
	PRIVACY      		: WEP encryption supported     
	SHORTPREAMBLE   : Short Preamble is supported  
	SHORTSLOT           : Short Slot is supported      
	PBCC       	        :PBCC                         
	CHANNELAGILITY :Channel Agility              
	SPECTRUM_MGMT  :Spectrum Management          
	DSSS_OFDM      : DSSS-OFDM                    
*/
typedef enum{
	ESS            = 0x01,
	/*!< ESS capability
	*/
	IBSS           = 0x02,
	/*!< IBSS mode
	*/
	POLLABLE       = 0x04,
	/*!< CF Pollable
	*/
	POLLREQ        = 0x08,
	/*!< Request to be polled
	*/
	PRIVACY        = 0x10,
	/*!< WEP encryption supported
	*/
	SHORTPREAMBLE  = 0x20,
	/*!< Short Preamble is supported
	*/
	SHORTSLOT      = 0x400,
	/*!< Short Slot is supported
	*/
	PBCC           = 0x40,
	/*!< PBCC
	*/
	CHANNELAGILITY = 0x80,
	/*!< Channel Agility
	*/
	SPECTRUM_MGMT  = 0x100,
	/*!< Spectrum Management
	*/
	DSSS_OFDM      = 0x2000
	/*!< DSSS-OFDM
	*/
    } tenuWifiCapability;

#endif

/*!
@typedef \
	tpfAppWifiCb

@brief	
				Wi-Fi's main callback function handler, for handling the M2M_WIFI events received on the Wi-Fi interface. 
			       Such notifications are received in response to Wi-Fi/P2P operations such as @ref m2m_wifi_request_scan,
				@ref m2m_wifi_connect. 
				Wi-Fi/P2P operations are implemented in an asynchronous mode, and all incoming information/status
				are to be handled through this callback function when the corresponding notification is received.
				Applications are expected to assign this wi-fi callback function by calling @ref m2m_wifi_init
@param [in]	u8MsgType
				Type of notification. Possible types are:
				/ref M2M_WIFI_RESP_CON_STATE_CHANGED
				/ref M2M_WIFI_RESP_CONN_INFO
				/ref M2M_WIFI_REQ_DHCP_CONF
				/ref M2M_WIFI_REQ_WPS
				/ref M2M_WIFI_RESP_IP_CONFLICT 
				/ref M2M_WIFI_RESP_SCAN_DONE
				/ref M2M_WIFI_RESP_SCAN_RESULT
				/ref M2M_WIFI_RESP_CURRENT_RSSI
				/ref M2M_WIFI_RESP_CLIENT_INFO
				/ref M2M_WIFI_RESP_PROVISION_INFO
				/ref M2M_WIFI_RESP_DEFAULT_CONNECT
		
			In case Bypass mode is defined :
				@ref M2M_WIFI_RESP_ETHERNET_RX_PACKET
		
			In case Monitoring mode is used:
				@ref M2M_WIFI_RESP_WIFI_RX_PACKET
				
@param [in]	pvMsg
				A pointer to a buffer containing the notification parameters (if any). It should be
				Casted to the correct data type corresponding to the notification type.

@see
	tstrM2mWifiStateChanged
	tstrM2MWPSInfo
	tstrM2mScanDone
	tstrM2mWifiscanResult
*/
typedef void (*tpfAppWifiCb) (uint8 u8MsgType, void * pvMsg);

/*!
@typedef \
	tpfAppEthCb

@brief	
	Ethernet (Bypass mode) notification callback function receiving Bypass mode events as defined in
	the Wi-Fi responses enumeration @ref tenuM2mStaCmd. 

@param [in]	u8MsgType
	Type of notification. Possible types are:
	- [M2M_WIFI_RESP_ETHERNET_RX_PACKET](@ref M2M_WIFI_RESP_ETHERNET_RX_PACKET)

@param [in]	pvMsg
	A pointer to a buffer containing the notification parameters (if any). It should be
	casted to the correct data type corresponding to the notification type.
	For example, it could be a pointer to the buffer holding the received frame in case of @ref M2M_WIFI_RESP_ETHERNET_RX_PACKET
	event.
	
@param [in]	pvControlBuf
	A pointer to control buffer describing the accompanied message.
	To be casted to @ref tstrM2mIpCtrlBuf in case of @ref M2M_WIFI_RESP_ETHERNET_RX_PACKET event.

@warning
	Make sure that the bypass mode is defined before using @ref tpfAppEthCb. 

@see
	m2m_wifi_init

*/
typedef void (*tpfAppEthCb) (uint8 u8MsgType, void * pvMsg,void * pvCtrlBuf);

/*!
@typedef	\
	tpfAppMonCb

@brief	
	Wi-Fi monitoring mode callback function. This function delivers all received wi-Fi packets through the Wi-Fi interface.
       Applications requiring to operate in the monitoring should call the asynchronous function m2m_wifi_enable_monitoring_mode
       and expect to receive the Wi-Fi packets through this callback function, when the event ....is received.
	To disable the monitoring mode a call to @ref m2m_wifi_disable_monitoring_mode should be made.
@param [in]	pstrWifiRxPacket
				Pointer to a structure holding the Wi-Fi packet header parameters.

@param [in]	pu8Payload
				Pointer to the buffer holding the Wi-Fi packet payload information required by the application starting from the
				defined OFFSET by the application (when calling m2m_wifi_enable_monitoring_mode).
				Could hold a value of NULL, if the application does not need any data from the payload.

@param [in]	u16PayloadSize
				The size of the payload in bytes.
				
@see
	m2m_wifi_enable_monitoring_mode		
	
@warning
	u16PayloadSize should not exceed the buffer size given through m2m_wifi_enable_monitoring_mode.
	
*/
typedef void (*tpfAppMonCb) (tstrM2MWifiRxPacketInfo *pstrWifiRxPacket, uint8 * pu8Payload, uint16 u16PayloadSize);

/**
@struct 	\
	tstrEthInitParam
	
@brief		
	Structure to hold Ethernet interface parameters. 
	Structure is to be defined and have its attributes set,based on the application's functionality before 
	a call is made to the initialize the wi-fi operations by calling the  @ref m2m_wifi_init function.
	Part of the wi-fi configuration structure @ref tstrWifiInitParam.
	Applications shouldn't need to define this structure, if the bypass mode is not defined.
	
@see
	tpfAppEthCb
	tpfAppWifiCb
	m2m_wifi_init

@warning
	Make sure that bypass mode is defined before using @ref tstrEthInitParam. 

*/
typedef struct {
	tpfAppWifiCb pfAppWifiCb;
	/*!<
		Callback for wifi notifications.
	*/
	tpfAppEthCb  pfAppEthCb;
	/*!<
		Callback for Ethernet interface.
	*/
	uint8 * au8ethRcvBuf;
	/*!<
		Pointer to Receive Buffer of Ethernet Packet
	*/
	uint16	u16ethRcvBufSize;
	/*!<
		Size of Receive Buffer for Ethernet Packet
	*/
	uint8 u8EthernetEnable;
	/*!<
		Enable Ethernet mode flag
	*/
	uint8 __PAD8__;
	
    } tstrEthInitParam;
    
/*!
@struct	\
 	tstrM2mIpCtrlBuf
 	
@brief		
 	Structure holding the incoming buffer's data size information, indicating the data size of the buffer and the remaining buffer's data size .
	The data of the buffer which holds the packet sent to the host when in the bypass mode, is placed in the @ref tstrEthInitParam structure in the 
	@ref au8ethRcvBuf attribute. This following information is retrieved in the host when an event @ref M2M_WIFI_RESP_ETHERNET_RX_PACKET is received in 
	the Wi-Fi callback function @ref tpfAppWifiCb. 

	The application is expected to use this structure's information to determine if there is still incoming data to be received from the firmware.

 	
 @see
	 tpfAppEthCb
	 tstrEthInitParam
 
 @warning
	 Make sure that bypass mode is defined before using @ref tstrM2mIpCtrlBuf

 */
typedef struct{
	uint16	u16DataSize;
	/*!<
		Size of the received data in bytes.
	*/
	uint16	u16RemainigDataSize;
	/*!<
		Size of the remaining data bytes to be delivered to host.
	*/
    } tstrM2mIpCtrlBuf;


/**
@struct		\
	tstrWifiInitParam

@brief		
	Structure, holding the Wi-fi configuration attributes such as the wi-fi callback , monitoring mode callback and Ethernet parameter initialization structure.
	Such configuration parameters are required to be set before calling the wi-fi initialization function @ref m2m_wifi_init.
	@ref pfAppWifiCb attribute must be set to handle the wi-fi callback operations.
	@ref pfAppMonCb attribute, is optional based on whether the application requires the monitoring mode configuration, and can there not
	be set before the initialization.
	@ref strEthInitParam structure, is another optional configuration based on whether the bypass mode is set.
	
*/
typedef struct {
	tpfAppWifiCb pfAppWifiCb;
	/*!<
		Callback for Wi-Fi notifications.
	*/
	tpfAppMonCb  pfAppMonCb;
	/*!<
		Callback for monitoring interface.
	*/
	tstrEthInitParam strEthInitParam ;
	/*!<
	Structure to hold Ethernet interface parameters.
	*/

    } tstrWifiInitParam;

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION PROTOTYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/** \defgroup WLANAPI Function
 *   @ingroup m2m_wifi
 */
 /** @defgroup WiFiDownloadFn m2m_wifi_download_mode
 *  @ingroup WLANAPI
 *   	Synchronous download mode entry function that prepares the WINC board to enter the download mode, ready for the firmware or certificate download.
*	The WINC board is prepared for download, through initializations for the WINC driver including bus initializations and interrupt enabling, it also halts the chip, to allow for the firmware downloads.
*	Firmware can be downloaded through a number of interfaces, UART, I2C and SPI.
 */
 /**@{*/
/*!
@fn	\
	NMI_API void  m2m_wifi_download_mode(void);

@return		
	The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
*/
NMI_API sint8  m2m_wifi_download_mode(void);
 /**@}*/
 /** @defgroup WifiInitFn m2m_wifi_init
 *  @ingroup WLANAPI
 *  Synchronous initialization function for the WINC driver. This function initializes the driver by, registering the call back function for M2M_WIFI layer(also the call back function for bypass mode/monitoring mode if defined), 
 *  initializing the host interface layer and the bus interfaces. 
 *  Wi-Fi callback registering is essential to allow the handling of the events received, in response to the asynchronous Wi-Fi operations. 

Following are the possible Wi-Fi events that are expected to be received through the call back function(provided by the application) to the M2M_WIFI layer are : 

		@ref M2M_WIFI_RESP_CON_STATE_CHANGED \n
		@ref M2M_WIFI_RESP_CONN_INFO \n
		@ref M2M_WIFI_REQ_DHCP_CONF \n
		@ref M2M_WIFI_REQ_WPS \n
		@ref M2M_WIFI_RESP_IP_CONFLICT \n 
		@ref M2M_WIFI_RESP_SCAN_DONE \n
		@ref M2M_WIFI_RESP_SCAN_RESULT \n
		@ref M2M_WIFI_RESP_CURRENT_RSSI \n
		@ref M2M_WIFI_RESP_CLIENT_INFO \n
		@ref M2M_WIFI_RESP_PROVISION_INFO  \n
		@ref M2M_WIFI_RESP_DEFAULT_CONNECT \n
	Example: \n
	In case Bypass mode is defined : \n
		@ref M2M_WIFI_RESP_ETHERNET_RX_PACKET
		
	In case Monitoring mode is used: \n
		@ref M2M_WIFI_RESP_WIFI_RX_PACKET
		
	Any application using the WINC driver must call this function at the start of its main function.
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8  m2m_wifi_init(tstrWifiInitParam * pWifiInitParam);

@param [in]	pWifiInitParam
	This is a pointer to the @ref tstrWifiInitParam structure which holds the pointer to the application WIFI layer call back function,
	monitoring mode call back and @ref tstrEthInitParam structure containing bypass mode parameters.
	
@pre 
	Prior to this function call, application users must provide a call back function responsible for receiving all the wi-fi events that are received on the M2M_WIFI layer.
	
@warning
	Failure to successfully complete function indicates that the driver couldn't be initialized and a fatal error will prevent the application from proceeding. 
	
@see
	m2m_wifi_deinit
	tenuM2mStaCmd

@return		
	The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
*/
NMI_API sint8  m2m_wifi_init(tstrWifiInitParam * pWifiInitParam);
 /**@}*/
 /** @defgroup WifiDeinitFn m2m_wifi_deinit
 *  @ingroup WLANAPI
 *   Synchronous de-initialization function to the WINC1500 driver. De-initializes the host interface and frees any resources used by the M2M_WIFI layer. 
 *   This function must be called in the application closing phase,to ensure that all resources have been correctly released. No arguments are expected to be passed in. 
 */
/**@{*/
/*!
@fn	\
	NMI_API sint8  m2m_wifi_deinit(void * arg);
	
@param [in]	arg
		Generic argument. Not used in current implementation.
@return		
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
*/
NMI_API sint8  m2m_wifi_deinit(void * arg);
 /**@}*/
/** @defgroup WifiHandleEventsFn m2m_wifi_handle_events
*  @ingroup WLANAPI
* 	Synchronous M2M event handler function, responsible for handling interrupts received from the WINC firmware.
*     Application developers should call this function periodically in-order to receive the events that are to be handled by the
*     callback functions implemented by the application.

 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_handle_events(void * arg);

@pre
	Prior to receiving  wi-fi interrupts, the WINC driver should have been successfully initialized by calling the @ref m2m_wifi_init function.
	 
@warning
	Failure to successfully complete this function indicates bus errors and hence a fatal error that will prevent the application from proceeding.

@return		
	The function returns @ref M2M_SUCCESS for successful interrupt handling and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_handle_events(void * arg);
 /**@}*/
/** @defgroup WifiDefaultConnectFn m2m_wifi_default_connect
 *  @ingroup WLANAPI
 *   Asynchronous Wi-Fi connection function. An application calling this function will cause the firmware to correspondingly connect to the last successfully connected AP from the cached connections. 
 *   A failure to connect will result in a response of @ref M2M_WIFI_RESP_DEFAULT_CONNECT indicating the connection error as defined in the structure @ref tstrM2MDefaultConnResp.
 *   Possible errors are: 
 *   The connection list is empty @ref M2M_DEFAULT_CONN_EMPTY_LIST or a mismatch for the saved AP name @ref M2M_DEFAULT_CONN_SCAN_MISMATCH.
 *    only difference between this function and @ref m2m_wifi_connect, is the connection parameters. 
 *   Connection using this function is expected to connect to cached connection parameters. 

 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_default_connect(void);

@pre 
	Prior to connecting, the WINC driver should have been successfully initialized by calling the @ref m2m_wifi_init function.
  
@warning
 This function must be called in station mode only.
 It's important to note that successful completion of a call to m2m_wifi_default_connect() does not guarantee success of the WIFI connection, 
 and a negative return value indicates only locally-detected errors.
	
@see
	m2m_wifi_connect
	
@return		
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.

*/
NMI_API sint8 m2m_wifi_default_connect(void);
 /**@}*/
/** @defgroup WifiConnectFn m2m_wifi_connect
 *   @ingroup WLANAPI
 *   Asynchronous wi-fi connection function to a specific AP. Prior to a successful connection, the application developers must know the SSID of the AP, the security type,
 *   the authentication information parameters and the channel number to which the connection will be established.
 *  The connection status is known when a response of @ref M2M_WIFI_RESP_CON_STATE_CHANGED is received based on the states defined in @ref tenuM2mConnState,
 *  successful connection is defined by @ref M2M_WIFI_CONNECTED
*   
 *   The only difference between this function and @ref m2m_wifi_default_connect, is the connection parameters. 
 *   Connection using this function is expected to be made to a specific AP and to a specified channel.
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_connect(char *pcSsid, uint8 u8SsidLen, uint8 u8SecType, void *pvAuthInfo, uint16 u16Ch);

@param [in]	pcSsid
				A buffer holding the SSID corresponding to the requested AP.
				
@param [in]	u8SsidLen
				Length of the given SSID (not including the NULL termination).
				A length less than ZERO or greater than the maximum defined SSID @ref M2M_MAX_SSID_LEN will result in a negative error 
				@ref M2M_ERR_FAIL.
				
@param [in]	u8SecType
				Wi-Fi security type security for the network. It can be one of the following types:
				-@ref M2M_WIFI_SEC_OPEN
				-@ref M2M_WIFI_SEC_WEP
				-@ref M2M_WIFI_SEC_WPA_PSK
				-@ref M2M_WIFI_SEC_802_1X
				A value outside these possible values will result in a negative return error @ref M2M_ERR_FAIL.

@param [in]	pvAuthInfo
				Authentication parameters required for completing the connection. It is type is based on the Security type.
				If the authentication parameters are NULL or are greater than the maximum length of the authentication parameters length as defined by
				@ref M2M_MAX_PSK_LEN a negative error will return @ref M2M_ERR_FAIL(-12) indicating connection failure.

@param [in]	u16Ch
				Wi-Fi channel number as defined in @ref tenuM2mScanCh enumeration.
				Channel number greater than @ref M2M_WIFI_CH_14 returns a negative error @ref M2M_ERR_FAIL(-12).
				Except if the value is M2M_WIFI_CH_ALL(255), since this indicates that the firmware should scan all channels to find the SSID requested to connect to.
				Failure to find the connection match will return a negative error @ref M2M_DEFAULT_CONN_SCAN_MISMATCH.
@pre
  		Prior to a successful connection request, the wi-fi driver must have been successfully initialized through the call of the @ref @m2m_wifi_init function
@see
	tuniM2MWifiAuth
	tstr1xAuthCredentials
	tstrM2mWifiWepParams
	
@warning
	-This function must be called in station mode only.
	-Successful completion of this function does not guarantee success of the WIFI connection, and
	a negative return value indicates only locally-detected errors.
	
@return	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
	
*/
NMI_API sint8 m2m_wifi_connect(char *pcSsid, uint8 u8SsidLen, uint8 u8SecType, void *pvAuthInfo, uint16 u16Ch);
 /**@}*/
/** @defgroup WifiConnectFn m2m_wifi_connect_sc
 *   @ingroup WLANAPI
 *   Asynchronous wi-fi connection function to a specific AP. Prior to a successful connection, the application developers must know the SSID of the AP, the security type,
 *   the authentication information parameters and the channel number to which the connection will be established.this API allows the user to choose
 *   whether to
 *  The connection status is known when a response of @ref M2M_WIFI_RESP_CON_STATE_CHANGED is received based on the states defined in @ref tenuM2mConnState,
 *  successful connection is defined by @ref M2M_WIFI_CONNECTED
 *   The only difference between this function and @ref m2m_wifi_connect, is the option to save the acess point info ( SSID, password...etc) or not.
 *   Connection using this function is expected to be made to a specific AP and to a specified channel.
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_connect_sc(char *pcSsid, uint8 u8SsidLen, uint8 u8SecType, void *pvAuthInfo, uint16 u16Ch,uint8 u8SaveCred);

@param [in]	pcSsid
				A buffer holding the SSID corresponding to the requested AP.
				
@param [in]	u8SsidLen
				Length of the given SSID (not including the NULL termination).
				A length less than ZERO or greater than the maximum defined SSID @ref M2M_MAX_SSID_LEN will result in a negative error 
				@ref M2M_ERR_FAIL.
				
@param [in]	u8SecType
				Wi-Fi security type security for the network. It can be one of the following types:
				-@ref M2M_WIFI_SEC_OPEN
				-@ref M2M_WIFI_SEC_WEP
				-@ref M2M_WIFI_SEC_WPA_PSK
				-@ref M2M_WIFI_SEC_802_1X
				A value outside these possible values will result in a negative return error @ref M2M_ERR_FAIL.

@param [in]	pvAuthInfo
				Authentication parameters required for completing the connection. It is type is based on the Security type.
				If the authentication parameters are NULL or are greater than the maximum length of the authentication parameters length as defined by
				@ref M2M_MAX_PSK_LEN a negative error will return @ref M2M_ERR_FAIL(-12) indicating connection failure.

@param [in]	u16Ch
				Wi-Fi channel number as defined in @ref tenuM2mScanCh enumeration.
				Channel number greater than @ref M2M_WIFI_CH_14 returns a negative error @ref M2M_ERR_FAIL(-12).
				Except if the value is M2M_WIFI_CH_ALL(255), since this indicates that the firmware should scan all channels to find the SSID requested to connect to.
				Failure to find the connection match will return a negative error @ref M2M_DEFAULT_CONN_SCAN_MISMATCH.
				
@param [in] u8NoSaveCred
				Option to store the acess point SSID and password into the WINC flash memory or not.
				
@pre
  		Prior to a successful connection request, the wi-fi driver must have been successfully initialized through the call of the @ref @m2m_wifi_init function
@see
	tuniM2MWifiAuth
	tstr1xAuthCredentials
	tstrM2mWifiWepParams
	
@warning
	-This function must be called in station mode only.
	-Successful completion of this function does not guarantee success of the WIFI connection, and
	a negative return value indicates only locally-detected errors.
	
@return	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
	
*/
 NMI_API sint8 m2m_wifi_connect_sc(char *pcSsid, uint8 u8SsidLen, uint8 u8SecType, void *pvAuthInfo, uint16 u16Ch, uint8 u8SaveCred);
 /**@}*/
/** @defgroup WifiDisconnectFn m2m_wifi_disconnect
 *   @ingroup WLANAPI
 *   Synchronous wi-fi disconnection function, requesting a Wi-Fi disconnect from the currently connected AP.
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_disconnect(void);
	
@pre 
	Disconnection must be made to a successfully connected AP. If the WINC is not in the connected state, a call to this function will hold insignificant.

@warning
	This function must be called in station mode only.
	
@see
	m2m_wifi_connect
	m2m_wifi_default_connect
	
@return		
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_disconnect(void);
 /**@}*/
/** @defgroup StartProvisionModeFn m2m_wifi_start_provision_mode
 *   @ingroup WLANAPI
 *    Asynchronous wi-fi provisioning function, which starts the WINC HTTP PROVISIONING mode.
	The function triggers the WINC to activate the Wi-Fi AP (HOTSPOT) mode with the passed configuration parameters and then starts the
	HTTP Provision WEB Server. 
	The provisioning status is returned in an event @ref M2M_WIFI_RESP_PROVISION_INFO
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_start_provision_mode(tstrM2MAPConfig *pstrAPConfig, char *pcHttpServerDomainName, uint8 bEnableHttpRedirect);
	
@param [in]	pstrAPConfig
				AP configuration parameters as defined in @ref tstrM2MAPConfig configuration structure.
				A NULL value passed in, will result in a negative error @ref M2M_ERR_FAIL.
				
@param [in]	pcHttpServerDomainName
				Domain name of the HTTP Provision WEB server which others will use to load the provisioning Home page.
				For example "wincconf.net".

@param [in]	bEnableHttpRedirect
				A flag to enable/disable the HTTP redirect feature. Possible values are:
				- ZERO  				DO NOT Use HTTP Redirect. In this case the associated device could open the provisioning page ONLY when
									the HTTP Provision URL of the WINC HTTP Server is correctly written on the browser.
				- Non-Zero value	       Use HTTP Redirect. In this case, all http traffic (http://URL) from the associated
									device (Phone, PC, ...etc) will be redirected to the WINC HTTP Provisioning Home page.

@pre	
	- A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered at startup. Registering the callback
	  is done through passing it to the initialization @ref m2m_wifi_init function.
	- The event @ref M2M_WIFI_RESP_CONN_INFO must be handled in the callback to receive the requested connection info.
	
@see
	tpfAppWifiCb
	m2m_wifi_init
	M2M_WIFI_RESP_PROVISION_INFO
	m2m_wifi_stop_provision_mode
	tstrM2MAPConfig

@warning
		DO Not use ".local" in the pcHttpServerDomainName.
		
@return
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.

\section Example
  The example demonstrates a code snippet for how provisioning is triggered and the response event received accordingly. 
@code
	#include "m2m_wifi.h"
	#include "m2m_types.h"


	void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)
	{
		switch(u8WiFiEvent)
		{
		case M2M_WIFI_RESP_PROVISION_INFO:
			{
				tstrM2MProvisionInfo	*pstrProvInfo = (tstrM2MProvisionInfo*)pvMsg;
				if(pstrProvInfo->u8Status == M2M_SUCCESS)				{
					m2m_wifi_connect((char*)pstrProvInfo->au8SSID, (uint8)strlen(pstrProvInfo->au8SSID), pstrProvInfo->u8SecType, 
							pstrProvInfo->au8Password, M2M_WIFI_CH_ALL);

					printf("PROV SSID : %s\n",pstrProvInfo->au8SSID);
					printf("PROV PSK  : %s\n",pstrProvInfo->au8Password);
				}
				else				{
					printf("(ERR) Provisioning Failed\n");
				}
			}
			break;

			default:
			break;
		}
	}

	int main()	{
		tstrWifiInitParam 	param;
		
		param.pfAppWifiCb	= wifi_event_cb;
		if(!m2m_wifi_init(&param))		{
			tstrM2MAPConfig		apConfig;
			uint8				bEnableRedirect = 1;
			
			strcpy(apConfig.au8SSID, "WINC_SSID");
			apConfig.u8ListenChannel 	= 1;
			apConfig.u8SecType			= M2M_WIFI_SEC_OPEN;
			apConfig.u8SsidHide			= 0;
			
			// IP Address
			apConfig.au8DHCPServerIP[0]	= 192;
			apConfig.au8DHCPServerIP[1]	= 168;
			apConfig.au8DHCPServerIP[2]	= 1;
			apConfig.au8DHCPServerIP[0]	= 1;

			m2m_wifi_start_provision_mode(&apConfig, "atmelwincconf.com", bEnableRedirect);
						
			while(1)			{
				m2m_wifi_handle_events(NULL);
			}
		}
	}
	
@endcode
*/
NMI_API sint8 m2m_wifi_start_provision_mode(tstrM2MAPConfig *pstrAPConfig, char *pcHttpServerDomainName, uint8 bEnableHttpRedirect);
 /**@}*/
/** @defgroup StopProvisioningModeFn m2m_wifi_stop_provision_mode
 *   @ingroup WLANAPI
 *  Synchronous provision termination function which stops the provision mode if it is active.
 */
 /**@{*/
/*!
@fn	\
	sint8 m2m_wifi_stop_provision_mode(void);

@pre
	An active provisioning session must be active before it is terminated through this function.
@see
	m2m_wifi_start_provision_mode
	
@return
	The function returns ZERO for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_stop_provision_mode(void);
 /**@}*/
/** @defgroup GetConnectionInfoFn m2m_wifi_get_connection_info
 *   @ingroup WLANAPI
 *  Asynchronous connection status retrieval function, retrieves the status information of the currently connected AP. The result is passed to the Wi-Fi notification callback
*    through the event @ref M2M_WIFI_RESP_CONN_INFO. Connection information is retrieved from the structure @ref tstrM2MConnInfo.
 */
 /**@{*/
/*!
@fn	\
	sint8 m2m_wifi_get_connection_info(void);

@brief
	Retrieve the current Connection information. The result is passed to the Wi-Fi notification callback
	with [M2M_WIFI_RESP_CONN_INFO](@ref M2M_WIFI_RESP_CONN_INFO).
@pre	
	- A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered at startup. Registering the callback
	is done through passing it to the initialization @ref m2m_wifi_init function.
	- The event @ref M2M_WIFI_RESP_CONN_INFO must be handled in the callback to receive the requested connection info.
	
	Connection Information retrieved:

	
	-Connection Security
	-Connection RSSI
	-Remote MAC address
	-Remote IP address

	and in case of WINC station mode the SSID of the AP is also retrieved.
@warning
	-In case of WINC AP mode or P2P mode, ignore the SSID field (NULL string).
@sa
	M2M_WIFI_RESP_CONN_INFO,
	tstrM2MConnInfo
@return
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
\section Example
  The code snippet shows an example of how wi-fi connection information is retrieved .
@code

	#include "m2m_wifi.h"
	#include "m2m_types.h"


	void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)	{
		switch(u8WiFiEvent)		{
		case M2M_WIFI_RESP_CONN_INFO:
			{
				tstrM2MConnInfo		*pstrConnInfo = (tstrM2MConnInfo*)pvMsg;
				
				printf("CONNECTED AP INFO\n");
				printf("SSID     			: %s\n",pstrConnInfo->acSSID);
				printf("SEC TYPE 			: %d\n",pstrConnInfo->u8SecType);
				printf("Signal Strength		: %d\n", pstrConnInfo->s8RSSI); 
				printf("Local IP Address	: %d.%d.%d.%d\n", 
					pstrConnInfo->au8IPAddr[0] , pstrConnInfo->au8IPAddr[1], pstrConnInfo->au8IPAddr[2], pstrConnInfo->au8IPAddr[3]);
			}
			break;

		case M2M_WIFI_REQ_DHCP_CONF:
			{
				// Get the current AP information.
				m2m_wifi_get_connection_info();
			}
			break;
		default:
			break;
		}
	}

	int main()	{
		tstrWifiInitParam 	param;
		
		param.pfAppWifiCb	= wifi_event_cb;
		if(!m2m_wifi_init(&param))
		{
			// connect to the default AP
			m2m_wifi_default_connect();
						
			while(1)
			{
				m2m_wifi_handle_events(NULL);
			}
		}
	}
	
@endcode
*/
NMI_API sint8 m2m_wifi_get_connection_info(void);
 /**@}*/
/** @defgroup WifiSetMacAddFn m2m_wifi_set_mac_address
 *   @ingroup WLANAPI
 *  Synchronous MAC address assigning to the NMC1500. It is used for non-production SW. Assign MAC address to the WINC device. 
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_set_mac_address(uint8 au8MacAddress[6]);



@param [in]	au8MacAddress
				MAC Address to be provisioned to the WINC.

@return		
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_set_mac_address(uint8 au8MacAddress[6]);
 /**@}*/
/** @defgroup WifiWpsFn m2m_wifi_wps
 *   @ingroup WLANAPI
 *   Asynchronous WPS triggering function.
 *   This function is called for the WINC to enter the WPS (Wi-Fi Protected Setup) mode. The result is passed to the Wi-Fi notification callback
*	with the event @ref M2M_WIFI_REQ_WPS.
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_wps(uint8 u8TriggerType,const char * pcPinNumber);

@param [in]	u8TriggerType
				WPS Trigger method. Could be:
				- [WPS_PIN_TRIGGER](@ref WPS_PIN_TRIGGER)   Push button method
				- [WPS_PBC_TRIGGER](@ref WPS_PBC_TRIGGER)	Pin method
				
@param [in]	pcPinNumber
				PIN number for WPS PIN method. It is not used if the trigger type is WPS_PBC_TRIGGER. It must follow the rules
				stated by the WPS Standard.

@warning
	This function is not allowed in AP or P2P modes.
	
@pre	
	- A Wi-Fi notification callback of type (@ref tpfAppWifiCb MUST be implemented and registered at startup. Registering the callback
	  is done through passing it to the [m2m_wifi_init](@ref m2m_wifi_init).
	- The event [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS) must be handled in the callback to receive the WPS status.
	- The WINC device MUST be in IDLE or STA mode. If AP or P2P mode is active, the WPS will not be performed. 
	- The [m2m_wifi_handle_events](@ref m2m_wifi_handle_events) MUST be called to receive the responses in the callback.
@see
	tpfAppWifiCb
	m2m_wifi_init
	M2M_WIFI_REQ_WPS
	tenuWPSTrigger
	tstrM2MWPSInfo

@return
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
\section Example
  The code snippet shows an example of how wifi WPS is triggered .
@code

	#include "m2m_wifi.h"
	#include "m2m_types.h"

	void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)	{
		switch(u8WiFiEvent)		{
		case M2M_WIFI_REQ_WPS:
			{
				tstrM2MWPSInfo	*pstrWPS = (tstrM2MWPSInfo*)pvMsg;
				if(pstrWPS->u8AuthType != 0)
				{
					printf("WPS SSID           : %s\n",pstrWPS->au8SSID);
					printf("WPS PSK            : %s\n",pstrWPS->au8PSK);
					printf("WPS SSID Auth Type : %s\n",pstrWPS->u8AuthType == M2M_WIFI_SEC_OPEN ? "OPEN" : "WPA/WPA2");
					printf("WPS Channel        : %d\n",pstrWPS->u8Ch + 1);
					
					// establish Wi-Fi connection
					m2m_wifi_connect((char*)pstrWPS->au8SSID, (uint8)m2m_strlen(pstrWPS->au8SSID),
						pstrWPS->u8AuthType, pstrWPS->au8PSK, pstrWPS->u8Ch);
				}
				else				{
					printf("(ERR) WPS Is not enabled OR Timed out\n");
				}
			}
			break;
			
		default:
			break;
		}
	}

	int main()	{
		tstrWifiInitParam 	param;
		
		param.pfAppWifiCb	= wifi_event_cb;
		if(!m2m_wifi_init(&param))		{
			// Trigger WPS in Push button mode.
			m2m_wifi_wps(WPS_PBC_TRIGGER, NULL);
			
			while(1)			{
				m2m_wifi_handle_events(NULL);
			}
		}
	}
	
@endcode
*/
NMI_API sint8 m2m_wifi_wps(uint8 u8TriggerType,const char  *pcPinNumber);
 /**@}*/
/** @defgroup WifiWpsDisableFn m2m_wifi_wps_disable
 *   @ingroup WLANAPI
 * Disable the NMC1500 WPS operation.
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_wps_disable(void);


@return
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_wps_disable(void);
 /**@}*/
/** @defgroup WifiP2PFn m2m_wifi_p2p
 *   @ingroup WLANAPI
 *    Asynchronous  Wi-Fi direct (P2P) enabling mode function.
	The WINC supports P2P in device listening mode ONLY (intent is ZERO).
	The WINC P2P implementation does not support P2P GO (Group Owner) mode.
	Active P2P devices (e.g. phones) could find the WINC in the search list. When a device is connected to WINC, a Wi-Fi notification event
	@ref M2M_WIFI_RESP_CON_STATE_CHANGED is triggered. After a short while, the DHCP IP Address is obtained 
	and an event @ref M2M_WIFI_REQ_DHCP_CONF is triggered. Refer to the code examples for a more illustrative example.
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_p2p(uint8 u8Channel);

@param [in]	u8Channel
    P2P Listen RF channel. According to the P2P standard It must hold only one of the following values 1, 6 or 11.

@pre
	- A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered at initialization. Registering the callback
	  is done through passing it to the @ref m2m_wifi_init.
	- The events @ref M2M_WIFI_RESP_CON_STATE_CHANGED and @ref M2M_WIFI_REQ_DHCP_CONF 
	  must be handled in the callback.
	- The @ref m2m_wifi_handle_events MUST be called to receive the responses in the callback.

@warning
	This function is not allowed in AP or STA modes.

@see
	tpfAppWifiCb
	m2m_wifi_init
	M2M_WIFI_RESP_CON_STATE_CHANGED
	M2M_WIFI_REQ_DHCP_CONF
	tstrM2mWifiStateChanged

@return
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
\section Example
  The code snippet shown an example of how the p2p mode operates.
@code
	#include "m2m_wifi.h"
	#include "m2m_types.h"
	
	void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)	{
		switch(u8WiFiEvent)		{
		case M2M_WIFI_RESP_CON_STATE_CHANGED:
			{
				tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged*)pvMsg;
				M2M_INFO("Wifi State :: %s :: ErrCode %d\n", pstrWifiState->u8CurrState? "CONNECTED":"DISCONNECTED",pstrWifiState->u8ErrCode);
				
				// Do something
			}
			break;
			
		case M2M_WIFI_REQ_DHCP_CONF:
			{
				uint8	*pu8IPAddress = (uint8*)pvMsg;

				printf("P2P IP Address \"%u.%u.%u.%u\"\n",pu8IPAddress[0],pu8IPAddress[1],pu8IPAddress[2],pu8IPAddress[3]);
			}
			break;
			
		default:
			break;
		}
	}
	
	int main()	{
		tstrWifiInitParam 	param;
		
		param.pfAppWifiCb	= wifi_event_cb;
		if(!m2m_wifi_init(&param))		{
			// Trigger P2P
			m2m_wifi_p2p(M2M_WIFI_CH_1);
			
			while(1)			{
				m2m_wifi_handle_events(NULL);
			}
		}
	}
	
@endcode

*/
NMI_API sint8 m2m_wifi_p2p(uint8 u8Channel);
 /**@}*/
/** @defgroup WifiP2PDisconnectFn m2m_wifi_p2p_disconnect
 *   @ingroup WLANAPI
 * Disable the NMC1500 device Wi-Fi direct mode (P2P). 
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_p2p_disconnect(void);
@pre 
	The p2p mode must have be enabled and active before a disconnect can be called.
@see
         m2m_wifi_p2p
@return
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_p2p_disconnect(void);
 /**@}*/
/** @defgroup WifiEnableApFn m2m_wifi_enable_ap
 *   @ingroup WLANAPI
 * 	Asynchronous wi-fi hot-spot enabling function. 
 *    The WINC supports AP mode operation with the following limitations:
	- Only 1 STA could be associated at a time.
	- Open and WEP are the only supported security types
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_enable_ap(CONST tstrM2MAPConfig* pstrM2MAPConfig);

@param [in]	pstrM2MAPConfig
				A structure holding the AP configurations.

@warning
	This function is not allowed in P2P or STA modes.
	
@pre
	- A Wi-Fi notification callback of type @ref tpfAppWifiCb  MUST be implemented and registered at initialization. Registering the callback
	  is done through passing it to the [m2m_wifi_init](@ref m2m_wifi_init).
	- The event @ref M2M_WIFI_REQ_DHCP_CONF must be handled in the callback.
	- The @ref m2m_wifi_handle_events MUST be called to receive the responses in the callback.

@see
	tpfAppWifiCb
	tenuM2mSecType
	m2m_wifi_init
	M2M_WIFI_REQ_DHCP_CONF
	tstrM2mWifiStateChanged
	tstrM2MAPConfig

@return
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
\section Example
  The code snippet demonstrates how the AP mode is enabled after the driver is initialized in the application's main function and the handling
  of the event @ref M2M_WIFI_REQ_DHCP_CONF, to indicate successful connection.
@code
	#include "m2m_wifi.h"
	#include "m2m_types.h"
	
	void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)	{
		switch(u8WiFiEvent)
		{
		case M2M_WIFI_REQ_DHCP_CONF:
			{
				uint8	*pu8IPAddress = (uint8*)pvMsg;

				printf("Associated STA has IP Address \"%u.%u.%u.%u\"\n",pu8IPAddress[0],pu8IPAddress[1],pu8IPAddress[2],pu8IPAddress[3]);
			}
			break;
			
		default:
			break;
		}
	}
	
	int main()	{
		tstrWifiInitParam 	param;
		
		param.pfAppWifiCb	= wifi_event_cb;
		if(!m2m_wifi_init(&param))		{
			tstrM2MAPConfig		apConfig;
			
			strcpy(apConfig.au8SSID, "WINC_SSID");
			apConfig.u8ListenChannel 	= 1;
			apConfig.u8SecType			= M2M_WIFI_SEC_OPEN;
			apConfig.u8SsidHide			= 0;
			
			// IP Address
			apConfig.au8DHCPServerIP[0]	= 192;
			apConfig.au8DHCPServerIP[1]	= 168;
			apConfig.au8DHCPServerIP[2]	= 1;
			apConfig.au8DHCPServerIP[0]	= 1;
			
			// Trigger AP
			m2m_wifi_enable_ap(&apConfig);
			
			while(1)			{
				m2m_wifi_handle_events(NULL);
			}
		}
	}

@endcode

*/
NMI_API sint8 m2m_wifi_enable_ap(CONST tstrM2MAPConfig* pstrM2MAPConfig);
 /**@}*/
/** @defgroup WifiDisableApFn m2m_wifi_disable_ap
 *   @ingroup WLANAPI
 *    Synchronous wi-fi hot-spot disabling function. Must be called only when the AP is enabled through the @ref m2m_wifi_enable_ap
 *   function. Otherwise the call to this function will not be useful.
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_disable_ap(void);
@see
         m2m_wifi_enable_ap
@return
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_disable_ap(void);
 /**@}*/
/** @defgroup SetStaticIPFn m2m_wifi_set_static_ip
 *   @ingroup WLANAPI
 *   Synchronous static IP Address configuration function. 
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_set_static_ip(tstrM2MIPConfig * pstrStaticIPConf);

@param [in]	pstrStaticIPConf
				Pointer to a structure holding the static IP Configurations (IP,
				Gateway, subnet mask and DNS address).

@warning
	This function should not be used. DHCP configuration is requested automatically after successful Wi-Fi connection is established.
	
@see
	tstrM2MIPConfig
	
@return
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_set_static_ip(tstrM2MIPConfig * pstrStaticIPConf);
 /**@}*/
/** @defgroup RequestDHCPClientFn m2m_wifi_request_dhcp_client
 *   @ingroup WLANAPI
 * 	Starts the DHCP client operation(DHCP requested by the firmware automatically in STA/AP/P2P mode).
 *    
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_request_dhcp_client(void);
	
@warning
	This function should not be used. DHCP configuration is requested automatically after successful Wi-Fi connection is established.

@return
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_request_dhcp_client(void);
 /**@}*/
/** @defgroup RequestDHCPServerFn m2m_wifi_request_dhcp_server
 *   @ingroup WLANAPI
 *   Dhcp requested by the firmware automatically in STA/AP/P2P mode).
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_request_dhcp_server(uint8* addr);

@warning
	This function is not used in the current releases. DHCP server is started automatically when enabling the AP mode.


@return
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_request_dhcp_server(uint8* addr);
 /**@}*/
/** @defgroup WifiSetScanOptionFn m2m_wifi_set_scan_options
 *   @ingroup WLANAPI
 *   Synchronous wi-fi scan settings function. This function sets the time configuration parameters for the scan operation.
 */
 /**@{*/
/*!
@fn	\
	NMI_API  sint8 m2m_wifi_enable_dhcp(uint8  u8DhcpEn );
	
@brief
	Enable/Disable the DHCP client after connection.

@param [in]	 u8DhcpEn 
				Possible values:
				1: Enable DHCP client after connection.
				0: Disable DHCP client after connection.
@warnings
	- DHCP client is enabled by default
	-This Function should be called before using m2m_wifi_set_static_ip()

	
@sa
	m2m_wifi_set_static_ip()
	
@return
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_enable_dhcp(uint8  u8DhcpEn );


/*!
@fn	\
	sint8 m2m_wifi_set_scan_options(tstrM2MScanOption* ptstrM2MScanOption)

@param [in]	ptstrM2MScanOption;
	Pointer to the structure holding the Scan Parameters.

@see
	tenuM2mScanCh
	m2m_wifi_request_scan
	tstrM2MScanOption
	
@return
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_set_scan_options(tstrM2MScanOption* ptstrM2MScanOption);
 /**@}*/
/** @defgroup WifiSetScanRegionFn m2m_wifi_set_scan_region
 *   @ingroup WLANAPI
 *  Synchronous wi-fi scan region setting function.
 *   This function sets the scan region, which will affect the range of possible scan channels. 
 *   For 2.5GHz supported in the current release, the requested scan region can't exceed the maximum number of channels (14).
 *@{*/
/*!
@fn	\
	sint8 m2m_wifi_set_scan_region(uint16 ScanRegion)

@param [in]	ScanRegion;
		ASIA
		NORTH_AMERICA
@see
	tenuM2mScanCh
	m2m_wifi_request_scan
	
@return
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.

*/
NMI_API sint8 m2m_wifi_set_scan_region(uint16  ScanRegion);
 /**@}*/
/** @defgroup WifiRequestScanFn m2m_wifi_request_scan
*   @ingroup WLANAPI
*    Asynchronous wi-fi scan request on the given channel. The scan status is delivered in the wi-fi event callback and then the application
*    is to read the scan results sequentially. 
*    The number of  APs found (N) is returned in event @ref M2M_WIFI_RESP_SCAN_DONE with the number of found
*     APs.
*	The application could read the list of APs by calling the function @ref m2m_wifi_req_scan_result N times.
* 
*@{*/
/*!
@fn	\
	NMI_API sint8 m2m_wifi_request_scan(uint8 ch);

@param [in]	ch
		      RF Channel ID for SCAN operation. It should be set according to tenuM2mScanCh. 
		      With a value of M2M_WIFI_CH_ALL(255)), means to scan all channels.

@warning
	This function is not allowed in P2P or AP modes. It works only for STA mode (connected or disconnected).
				
@pre
	- A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered at initialization. Registering the callback
	  is done through passing it to the @ref m2m_wifi_init.
	- The events @ref M2M_WIFI_RESP_SCAN_DONE and @ref M2M_WIFI_RESP_SCAN_RESULT.
	  must be handled in the callback.
	- The @ref m2m_wifi_handle_events function MUST be called to receive the responses in the callback.

@see
	M2M_WIFI_RESP_SCAN_DONE
	M2M_WIFI_RESP_SCAN_RESULT
	tpfAppWifiCb
	tstrM2mWifiscanResult
	tenuM2mScanCh
	m2m_wifi_init
	m2m_wifi_handle_events
	m2m_wifi_req_scan_result

@return
	The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
\section Example
  The code snippet demonstrates an example of how the scan request is called from the application's main function and the handling of
  the events received in response.
@code

	#include "m2m_wifi.h"
	#include "m2m_types.h"
	
	void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)
	{
		static uint8	u8ScanResultIdx = 0;
		
		switch(u8WiFiEvent)
		{
		case M2M_WIFI_RESP_SCAN_DONE:
			{
				tstrM2mScanDone	*pstrInfo = (tstrM2mScanDone*)pvMsg;
				
				printf("Num of AP found %d\n",pstrInfo->u8NumofCh);
				if(pstrInfo->s8ScanState == M2M_SUCCESS)				{
					u8ScanResultIdx = 0;
					if(pstrInfo->u8NumofCh >= 1)					{
						m2m_wifi_req_scan_result(u8ScanResultIdx);
						u8ScanResultIdx ++;
					}
					else					{
						printf("No AP Found Rescan\n");
						m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
					}
				}
				else				{
					printf("(ERR) Scan fail with error <%d>\n",pstrInfo->s8ScanState);
				}
			}
			break;
		
		case M2M_WIFI_RESP_SCAN_RESULT:
			{
				tstrM2mWifiscanResult		*pstrScanResult =(tstrM2mWifiscanResult*)pvMsg;
				uint8						u8NumFoundAPs = m2m_wifi_get_num_ap_found();
				
				printf(">>%02d RI %d SEC %s CH %02d BSSID %02X:%02X:%02X:%02X:%02X:%02X SSID %s\n",
					pstrScanResult->u8index,pstrScanResult->s8rssi,
					pstrScanResult->u8AuthType,
					pstrScanResult->u8ch,
					pstrScanResult->au8BSSID[0], pstrScanResult->au8BSSID[1], pstrScanResult->au8BSSID[2],
					pstrScanResult->au8BSSID[3], pstrScanResult->au8BSSID[4], pstrScanResult->au8BSSID[5],
					pstrScanResult->au8SSID);
				
				if(u8ScanResultIdx < u8NumFoundAPs)				{
					// Read the next scan result
					m2m_wifi_req_scan_result(index);
					u8ScanResultIdx ++;
				}
			}
			break;
		default:
			break;
		}
	}
	
	int main()	{
		tstrWifiInitParam 	param;
		
		param.pfAppWifiCb	= wifi_event_cb;
		if(!m2m_wifi_init(&param))		{
			// Scan all channels
			m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
			
			while(1)			{
				m2m_wifi_handle_events(NULL);
			}
		}
	}
	
@endcode
*/
NMI_API sint8 m2m_wifi_request_scan(uint8 ch);
/**@}*/
/** @defgroup WifiGetNumAPFoundFn m2m_wifi_get_num_ap_found
 *   @ingroup WLANAPI
*  Synchronous function to retrieve the number of AP's found in the last scan request, The function read the number of AP's from global variable which updated in the Wi-Fi callback function through the M2M_WIFI_RESP_SCAN_DONE event.
*  Function used only in STA mode only. 
 */
 /**@{*/
/*!
@fn        NMI_API uint8 m2m_wifi_get_num_ap_found(void);

@see       m2m_wifi_request_scan 
		   M2M_WIFI_RESP_SCAN_DONE
		   M2M_WIFI_RESP_SCAN_RESULT         
@pre         m2m_wifi_request_scan need to be called first	
		- A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered at initialization. Registering the callback
		   is done through passing it to the @ref m2m_wifi_init.
		- The event @ref M2M_WIFI_RESP_SCAN_DONE must be handled in the callback to receive the requested connection information. 
@warning   This function must be called only in the wi-fi callback function when the events @ref M2M_WIFI_RESP_SCAN_DONE or @ref M2M_WIFI_RESP_SCAN_RESULT
		   are received.
		   Calling this function in any other place will result in undefined/outdated numbers.
@return    Return the number of AP's found in the last Scan Request.
		  
\section Example
  The code snippet demonstrates an example of how the scan request is called from the application's main function and the handling of
  the events received in response.
@code

	#include "m2m_wifi.h"
	#include "m2m_types.h"
	
	void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)	{
		static uint8	u8ScanResultIdx = 0;
		
		switch(u8WiFiEvent)		{
		case M2M_WIFI_RESP_SCAN_DONE:
			{
				tstrM2mScanDone	*pstrInfo = (tstrM2mScanDone*)pvMsg;
				
				printf("Num of AP found %d\n",pstrInfo->u8NumofCh);
				if(pstrInfo->s8ScanState == M2M_SUCCESS)				{
					u8ScanResultIdx = 0;
					if(pstrInfo->u8NumofCh >= 1)					{
						m2m_wifi_req_scan_result(u8ScanResultIdx);
						u8ScanResultIdx ++;
					}
					else					{
						printf("No AP Found Rescan\n");
						m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
					}
				}
				else				{
					printf("(ERR) Scan fail with error <%d>\n",pstrInfo->s8ScanState);
				}
			}
			break;
		
		case M2M_WIFI_RESP_SCAN_RESULT:
			{
				tstrM2mWifiscanResult		*pstrScanResult =(tstrM2mWifiscanResult*)pvMsg;
				uint8						u8NumFoundAPs = m2m_wifi_get_num_ap_found();
				
				printf(">>%02d RI %d SEC %s CH %02d BSSID %02X:%02X:%02X:%02X:%02X:%02X SSID %s\n",
					pstrScanResult->u8index,pstrScanResult->s8rssi,
					pstrScanResult->u8AuthType,
					pstrScanResult->u8ch,
					pstrScanResult->au8BSSID[0], pstrScanResult->au8BSSID[1], pstrScanResult->au8BSSID[2],
					pstrScanResult->au8BSSID[3], pstrScanResult->au8BSSID[4], pstrScanResult->au8BSSID[5],
					pstrScanResult->au8SSID);
				
				if(u8ScanResultIdx < u8NumFoundAPs)				{
					// Read the next scan result
					m2m_wifi_req_scan_result(index);
					u8ScanResultIdx ++;
				}
			}
			break;
		default:
			break;
		}
	}
	
	int main()	{
		tstrWifiInitParam 	param;
		
		param.pfAppWifiCb	= wifi_event_cb;
		if(!m2m_wifi_init(&param))		{
			// Scan all channels
			m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
			
			while(1)			{
				m2m_wifi_handle_events(NULL);
			}
		}
	}
	
@endcode			 
*/
NMI_API uint8 m2m_wifi_get_num_ap_found(void);
/**@}*/
/** @defgroup WifiReqScanResult m2m_wifi_req_scan_result
*   @ingroup WLANAPI
*   Synchronous call to read the AP information from the SCAN Result list with the given index.
*   This function is expected to be called when the response events M2M_WIFI_RESP_SCAN_RESULT or
*   M2M_WIFI_RESP_SCAN_DONE are received in the wi-fi callback function.
*   The response information received can be obtained through the casting to the @ref tstrM2mWifiscanResult structure 	
 */
 /**@{*/
/*!
@fn          NMI_API sint8 m2m_wifi_req_scan_result(uint8 index);
@param [in]  index 
		      Index for the requested result, the index range start from 0 till number of AP's found

@see          tstrM2mWifiscanResult
		   m2m_wifi_get_num_ap_found
		   m2m_wifi_request_scan             
		   
@pre         @ref m2m_wifi_request_scan needs to be called first, then m2m_wifi_get_num_ap_found 
		   to get the number of AP's found	
			- A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered at startup. Registering the callback
			is done through passing it to the @ref m2m_wifi_init function.
			- The event @ref M2M_WIFI_RESP_SCAN_RESULT must be handled in the callback to receive the requested connection information.
@warning     Function used  in STA mode only. the scan results are updated only if the scan request is called.
		     Calling this function only without a scan request will lead to firmware errors. 
		     Refrain from introducing a large delay  between the scan request and the scan result request, to prevent
		     an errors occurring.
			 
@return      The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
\section Example
  The code snippet demonstrates an example of how the scan request is called from the application's main function and the handling of
  the events received in response.
@code
	#include "m2m_wifi.h"
	#include "m2m_types.h"
	
	void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)	{
		static uint8	u8ScanResultIdx = 0;
		
		switch(u8WiFiEvent)
		{
		case M2M_WIFI_RESP_SCAN_DONE:
			{
				tstrM2mScanDone	*pstrInfo = (tstrM2mScanDone*)pvMsg;
				
				printf("Num of AP found %d\n",pstrInfo->u8NumofCh);
				if(pstrInfo->s8ScanState == M2M_SUCCESS)				{
					u8ScanResultIdx = 0;
					if(pstrInfo->u8NumofCh >= 1)					{
						m2m_wifi_req_scan_result(u8ScanResultIdx);
						u8ScanResultIdx ++;
					}
					else					{
						printf("No AP Found Rescan\n");
						m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
					}
				}
				else				{
					printf("(ERR) Scan fail with error <%d>\n",pstrInfo->s8ScanState);
				}
			}
			break;
		
		case M2M_WIFI_RESP_SCAN_RESULT:
			{
				tstrM2mWifiscanResult		*pstrScanResult =(tstrM2mWifiscanResult*)pvMsg;
				uint8						u8NumFoundAPs = m2m_wifi_get_num_ap_found();
				
				printf(">>%02d RI %d SEC %s CH %02d BSSID %02X:%02X:%02X:%02X:%02X:%02X SSID %s\n",
					pstrScanResult->u8index,pstrScanResult->s8rssi,
					pstrScanResult->u8AuthType,
					pstrScanResult->u8ch,
					pstrScanResult->au8BSSID[0], pstrScanResult->au8BSSID[1], pstrScanResult->au8BSSID[2],
					pstrScanResult->au8BSSID[3], pstrScanResult->au8BSSID[4], pstrScanResult->au8BSSID[5],
					pstrScanResult->au8SSID);
				
				if(u8ScanResultIdx < u8NumFoundAPs)				{
					// Read the next scan result
					m2m_wifi_req_scan_result(index);
					u8ScanResultIdx ++;
				}
			}
			break;
		default:
			break;
		}
	}
	
	int main()	{
		tstrWifiInitParam 	param;
		
		param.pfAppWifiCb	= wifi_event_cb;
		if(!m2m_wifi_init(&param))		{
			// Scan all channels
			m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
			
			while(1)			{
				m2m_wifi_handle_events(NULL);
			}
		}
	}
	
@endcode 
*/
NMI_API sint8 m2m_wifi_req_scan_result(uint8 index);
/**@}*/
/** @defgroup WifiReqCurrentRssiFn m2m_wifi_req_curr_rssi
 *   @ingroup WLANAPI
 *   Asynchronous request for the current RSSI of the connected AP.
 *   The response received in through the @ref M2M_WIFI_RESP_CURRENT_RSSI event.
 */
 /**@{*/
/*!
@fn          NMI_API sint8 m2m_wifi_req_curr_rssi(void);
@pre	   - A Wi-Fi notification callback of type @ref tpfAppWifiCb MUST be implemented and registered before initialization. Registering the callback
			is done through passing it to the [m2m_wifi_init](@ref m2m_wifi_init) through the @ref tstrWifiInitParam initialization structure.
		   - The event @ref M2M_WIFI_RESP_CURRENT_RSSI must be handled in the callback to receive the requested connection information.       
@return      The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.	
\section Example
  The code snippet demonstrates how the RSSI request is called in the application's main function and the handling of event received in the callback. 
@code

	#include "m2m_wifi.h"
	#include "m2m_types.h"
	
	void wifi_event_cb(uint8 u8WiFiEvent, void * pvMsg)	{
		static uint8	u8ScanResultIdx = 0;
		
		switch(u8WiFiEvent)		{
		case M2M_WIFI_RESP_CURRENT_RSSI:
			{
				sint8	*rssi = (sint8*)pvMsg;
				M2M_INFO("ch rssi %d\n",*rssi);
			}
			break;
		default:
			break;
		}
	}
	
	int main()	{
		tstrWifiInitParam 	param;
		
		param.pfAppWifiCb	= wifi_event_cb;
		if(!m2m_wifi_init(&param))		{
			// Scan all channels
			m2m_wifi_req_curr_rssi();
			
			while(1)			{
				m2m_wifi_handle_events(NULL);
			}
		}
	}

@endcode	

*/
NMI_API sint8 m2m_wifi_req_curr_rssi(void);
/**@}*/
/** @defgroup WifiGetOtpMacAddFn m2m_wifi_get_otp_mac_address
*   @ingroup WLANAPI
*   Request the MAC address stored on the OTP (one time programmable) memory of the device.
*   The function is blocking until the response is received.
*/
 /**@{*/
/*!
@fn          NMI_API sint8 m2m_wifi_get_otp_mac_address(uint8 *pu8MacAddr, uint8 * pu8IsValid);

@param [out] pu8MacAddr
			 Output MAC address buffer of 6 bytes size. Valid only if *pu8Valid=1.
@param [out] pu8IsValid
		     A output boolean value to indicate the validity of pu8MacAddr in OTP. 
		     Output zero if the OTP memory is not programmed, non-zero otherwise.
@pre         m2m_wifi_init required to call any WIFI/socket function
@see         m2m_wifi_get_mac_address             

@return      The function returns @ref M2M_SUCCESS for success and a negative value otherwise.

*/
NMI_API sint8 m2m_wifi_get_otp_mac_address(uint8 *pu8MacAddr, uint8 * pu8IsValid);
/**@}*/
/** @defgroup WifiGetMacAddFn m2m_wifi_get_mac_address
*   @ingroup WLANAPI
*   Function to retrieve the current MAC address. The function is blocking until the response is received.
*/
/**@{*/
/*!
@fn          NMI_API sint8 m2m_wifi_get_mac_address(uint8 *pu8MacAddr)	
@param [out] pu8MacAddr
			 Output MAC address buffer of 6 bytes size.	
@pre         m2m_wifi_init required to call any WIFI/socket function
@see         m2m_wifi_get_otp_mac_address             
@return      The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.

*/
NMI_API sint8 m2m_wifi_get_mac_address(uint8 *pu8MacAddr);
/**@}*/
/** @defgroup SetSleepModeFn m2m_wifi_set_sleep_mode
 *   @ingroup WLANAPI
 *  Synchronous power-save mode setting function for the NMC1500. 
 */
 /**@{*/
/*!
@fn			NMI_API sint8 m2m_wifi_set_sleep_mode(uint8 PsTyp, uint8 BcastEn);
@param [in]	PsTyp
			Desired power saving mode. Supported types are defined in @ref tenuPowerSaveModes.
@param [in]	BcastEn
			Broadcast reception enable flag. 
			If it is 1, the WINC1500 must be awake each DTIM beacon for receiving broadcast traffic.
			If it is 0, the WINC1500 will not wakeup at the DTIM beacon, but its wakeup depends only 
			on the the configured Listen Interval. 

@warning    The function called once after initialization.

@see	   tenuPowerSaveModes
		   m2m_wifi_get_sleep_mode

@return    The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.

*/
NMI_API sint8 m2m_wifi_set_sleep_mode(uint8 PsTyp, uint8 BcastEn);
/**@}*/
/** @defgroup WifiRequestSleepFn m2m_wifi_request_sleep
 *   @ingroup WLANAPI
 *  Synchronous power save request function, which requests from the NMC1500 device to sleep in the mode previously set 
 *   for a specific time.
 *   This function should be used in the M2M_PS_MANUAL Power save mode (only).
 */
 /**@{*/
/*!
@fn	        NMI_API sint8 m2m_wifi_request_sleep(uint32 u32SlpReqTime);
@param [in]	u32SlpReqTime
			Request Sleep in ms 
@warning 	The function should be called in M2M_PS_MANUAL power save only.
@see         tenuPowerSaveModes 
		  m2m_wifi_set_sleep_mode
@return    The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_request_sleep(uint32 u32SlpReqTime);
/**@}*/
/** @defgroup GetSleepModeFn m2m_wifi_get_sleep_mode
 *   @ingroup WLANAPI
 *  Synchronous power save mode retrieval function.
 */
 /**@{*/
/*!
@fn		    NMI_API uint8 m2m_wifi_get_sleep_mode(void);
@see	    tenuPowerSaveModes 
		    m2m_wifi_set_sleep_mode
@return	    The current operating power saving mode.

*/
NMI_API uint8 m2m_wifi_get_sleep_mode(void);
/**@}*/
/** @defgroup WifiReqClientCtrlFn m2m_wifi_req_client_ctrl
 *   @ingroup WLANAPI
 *  Asynchronous command sending function to the PS Client (An NMC1500 board running the ps_firmware)
*   if the PS client send any commands it will be received through the @ref M2M_WIFI_RESP_CLIENT_INFO event
 */
 /**@{*/
/*!
@fn			NMI_API sint8 m2m_wifi_req_client_ctrl(uint8 cmd);
@brief		
@param [in]	cmd
			Control command sent from PS Server to PS Client (command values defined by the application)
@pre		m2m_wifi_req_server_init should be called first
@warning	       This mode is not supported in the current release.
@see		m2m_wifi_req_server_init
			M2M_WIFI_RESP_CLIENT_INFO
@return		The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_req_client_ctrl(uint8 cmd);
/**@}*/
/** @defgroup WifiReqServerInit m2m_wifi_req_server_init
 *   @ingroup WLANAPI
 *  Synchronous function to initialize the PS Server.
 *  The WINC1500 supports non secure communication with another WINC1500, 
*   (SERVER/CLIENT) through one byte command (probe request and probe response) without any connection setup.
*   The server mode can't be used with any other modes (STA/P2P/AP)
*/
 /**@{*/
/*!
@fn			NMI_API sint8 m2m_wifi_req_server_init(uint8 ch);
@param [in]	ch
			Server listening channel
@see		m2m_wifi_req_client_ctrl
@warning	       This mode is not supported in the current release.
@return		The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_req_server_init(uint8 ch);
/**@}*/
/** @defgroup WifiSetDeviceNameFn m2m_wifi_set_device_name
 *   @ingroup WLANAPI
 *  Set the WINC1500 device name which is to be used as a P2P device name.
 */
 /**@{*/
/*!
@fn			NMI_API sint8 m2m_wifi_set_device_name(uint8 *pu8DeviceName, uint8 u8DeviceNameLength);		
@param [in]	pu8DeviceName
			Buffer holding the device name.
@param [in]	u8DeviceNameLength
			Length of the device name. Should not exceed the maximum device name's length M2M_DEVICE_NAME_MAX.
@warning		The function called once after initialization. 
@return		The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_set_device_name(uint8 *pu8DeviceName, uint8 u8DeviceNameLength);
/**@}*/
/** @defgroup WifiSetLsnIntFn m2m_wifi_set_lsn_int
 *   @ingroup WLANAPI
*	Synchronous function for setting the wi-fi listen interval for power save operation. It is represented in units
*	of AP Beacon periods.  
 */
 /**@{*/
/*!
@fn			NMI_API sint8 m2m_wifi_set_lsn_int(tstrM2mLsnInt * pstrM2mLsnInt);

@param [in]	pstrM2mLsnInt
			Structure holding the listen interval configurations.
@pre		Function m2m_wifi_set_sleep_mode shall be called first
@warning     	The function should be called once after initialization. 
@see		tstrM2mLsnInt
                     m2m_wifi_set_sleep_mode
@return		The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.

*/
NMI_API sint8 m2m_wifi_set_lsn_int(tstrM2mLsnInt *pstrM2mLsnInt);
/**@}*/
/** @defgroup WifiEnableMonitorModeFn m2m_wifi_enable_monitoring_mode
 *   @ingroup WLANAPI
 *     Asynchronous  wi-fi monitoring enable mode (Promiscuous mode) function. This function enables the monitoring mode, which starts transmission
 *    of the packets based on the filter information passed in as a parameter. All packets that meet the filtering criteria are passed to the application layer, to be handled by the assigned monitoring callback function.
 *    The monitoring callback function must be implemented before starting the monitoring mode, in-order to handle the packets received.
 *    Registering of the implemented callback function is through the callback pointer @ref tpfAppMonCb in the @ref tstrWifiInitParam structure.
 *    passed to @ref m2m_wifi_init function at initialization.
 *    
 */
 /**@{*/
/*!
 * @fn             NMI_API sint8 m2m_wifi_enable_monitoring_mode(tstrM2MWifiMonitorModeCtrl *, uint8 *, uint16 , uint16);
 * @param [in]     pstrMtrCtrl
 *                 		Pointer to @ref tstrM2MWifiMonitorModeCtrl structure holding the Filtering parameters.
 * @param [in]     pu8PayloadBuffer
 * 				   Pointer to a Buffer allocated by the application. The buffer SHALL hold the Data field of 
 *				   the WIFI RX Packet (Or a part from it). If it is set to NULL, the WIFI data payload will 
 *				   be discarded by the monitoring driver.
 * @param [in]     u16BufferSize
 *				   The total size of the pu8PayloadBuffer in bytes.
 * @param [in]     u16DataOffset
 *				   Starting offset in the DATA FIELD of the received WIFI packet. The application may be interested
 *				   in reading specific information from the received packet. It must assign the offset to the starting
 *				   position of it relative to the DATA payload start.\n
 *				   \e Example, \e if \e the \e SSID \e is \e needed \e to \e be \e read \e from \e a \e PROBE \e REQ \e packet, \e the \e u16Offset \e MUST \e be \e set \e to \e 0.
 * @warning        This mode available as sniffer ONLY, you can not be connected in any modes (Station, Access Point, or P2P).\n 
 * @see             tstrM2MWifiMonitorModeCtrl
 			   tstrM2MWifiRxPacketInfo
 			   tstrWifiInitParam
 			   tenuM2mScanCh
 			   m2m_wifi_disable_monitoring_mode               
 * @return       The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.

*\section Example
*  The example demonstrates the main function where-by the monitoring enable function is called after the initialization of the driver and the packets are
*   handled in the callback function.
* @code
			
			#include "m2m_wifi.h"
			#include "m2m_types.h"

			//Declare receive buffer 
			uint8 gmgmt[1600];
	
			//Callback functions
			void wifi_cb(uint8 u8WiFiEvent, void * pvMsg)			{
				; 
			}
			void wifi_monitoring_cb(tstrM2MWifiRxPacketInfo *pstrWifiRxPacket, uint8 *pu8Payload, uint16 u16PayloadSize)			{
				if((NULL != pstrWifiRxPacket) && (0 != u16PayloadSize)) {
					if(MANAGEMENT == pstrWifiRxPacket->u8FrameType) {
						M2M_INFO("***# MGMT PACKET #***\n");
					} else if(DATA_BASICTYPE == pstrWifiRxPacket->u8FrameType) {
						M2M_INFO("***# DATA PACKET #***\n");
					} else if(CONTROL == pstrWifiRxPacket->u8FrameType) {
						M2M_INFO("***# CONTROL PACKET #***\n");
					}
				}
			}
			
			int main()			{
				//Register wifi_monitoring_cb 
				tstrWifiInitParam param;
				param.pfAppWifiCb = wifi_cb;
				param.pfAppMonCb  = wifi_monitoring_cb;
				
				nm_bsp_init();
				
				if(!m2m_wifi_init(&param)) {
					//Enable Monitor Mode with filter to receive all data frames on channel 1
					tstrM2MWifiMonitorModeCtrl	strMonitorCtrl = {0};
					strMonitorCtrl.u8ChannelID		= M2M_WIFI_CH_1;
					strMonitorCtrl.u8FrameType		= DATA_BASICTYPE;
					strMonitorCtrl.u8FrameSubtype	= M2M_WIFI_FRAME_SUB_TYPE_ANY; //Receive any subtype of data frame
					m2m_wifi_enable_monitoring_mode(&strMonitorCtrl, gmgmt, sizeof(gmgmt), 0);
					
					while(1) {
						m2m_wifi_handle_events(NULL);
					}
				}
				return 0;
			}
 * @endcode
 */
NMI_API sint8 m2m_wifi_enable_monitoring_mode(tstrM2MWifiMonitorModeCtrl *pstrMtrCtrl, uint8 *pu8PayloadBuffer, 
										   uint16 u16BufferSize, uint16 u16DataOffset);
/**@}*/
/** @defgroup WifiDisableMonitorModeFn m2m_wifi_disable_monitoring_mode
 *   @ingroup WLANAPI
 *   Synchronous function to disable Wi-Fi monitoring mode (Promiscuous mode). Expected to be called, if the enable monitoring mode is set, but if it was called without enabling
 *   no negative impact will reside.
 */
 /**@{*/
/*!
 * @fn             NMI_API sint8 m2m_wifi_disable_monitoring_mode(void);
 * @see           m2m_wifi_enable_monitoring_mode               
 * @return      The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
NMI_API sint8 m2m_wifi_disable_monitoring_mode(void);
 /**@}*/
 /** @defgroup SendWlanPktFn m2m_wifi_send_wlan_pkt
 *   @ingroup WLANAPI
 *   Synchronous function to transmit a WIFI RAW packet while the implementation of this packet is left to the application developer.
 */
 /**@{*/
/*!
 * @fn             NMI_API sint8 m2m_wifi_send_wlan_pkt(uint8 *, uint16, uint16);
 
 * @param [in]     pu8WlanPacket
 *                 	     Pointer to a buffer holding the whole WIFI frame.
 * @param [in]     u16WlanHeaderLength
 * 			      The size of the WIFI packet header ONLY.
 * @param [in]     u16WlanPktSize
 *			     The size of the whole bytes in packet. 
 * @see             m2m_wifi_enable_monitoring_mode
 			   m2m_wifi_disable_monitoring_mode
 * @pre              Enable Monitoring mode first using @ref m2m_wifi_enable_monitoring_mode
 * @warning        This function available in monitoring mode ONLY.\n  
 * @note             Packets are user's responsibility.
 * @return     	    The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
NMI_API sint8 m2m_wifi_send_wlan_pkt(uint8 *pu8WlanPacket, uint16 u16WlanHeaderLength, uint16 u16WlanPktSize);
/**@}*/
/** @defgroup WifiSendEthernetPktFn m2m_wifi_send_ethernet_pkt
 *   @ingroup WLANAPI
 *   Synchronous function to transmit an Ethernet packet. Transmit a packet directly in bypass mode where the TCP/IP stack is disabled and the implementation of this packet is left to the application developer. 
 *   The Ethernet packet composition is left to the application developer. 
 */
 /**@{*/
/*!
 * @fn           NMI_API sint8 m2m_wifi_send_ethernet_pkt(uint8* pu8Packet,uint16 u16PacketSize)
 * @param [in]     pu8Packet
 *                        Pointer to a buffer holding the whole Ethernet frame.
 * @param [in]     u16PacketSize
 * 		            The size of the whole bytes in packet.    
  * @attention     This function available in Bypass mode ONLY. Make sure that firmware version built with macro \ref ETH_MODE.\n  
 * @note             Packets are the user's responsibility.
 * @return         The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.

 */
NMI_API sint8 m2m_wifi_send_ethernet_pkt(uint8* pu8Packet,uint16 u16PacketSize);
/**@}*/
/** @defgroup WifiEnableSntpFn m2m_wifi_enable_sntp
 *   @ingroup WLANAPI
 *  	Synchronous function to Enable/Disable the native SNTP client in the m2m firmware. The SNTP is enabled by default at start-up.
 *  	The SNTP client at firmware is used to sync the system clock to the UTC time from well known time
 *  	servers (e.g. "time-c.nist.gov"). The SNTP client uses a default update cycle of 1 day.
 *  	The UTC is important for checking the expiration date of X509 certificates used while establishing
 *  	TLS (Transport Layer Security) connections.
 *  	It is highly recommended to use it if there is no other means to get the UTC time. If there is a RTC
 *  	on the host MCU, the SNTP could be disabled and the host should set the system time to the firmware 
 *  	using the @ref m2m_wifi_set_system_time function.
 */
 /**@{*/
/*!
 * @fn             NMI_API sint8 m2m_wifi_enable_sntp(uint8);
 * @param [in]     bEnable
*				Enabling/Disabling flag
 *                        '0' :disable SNTP
 *                        '1' :enable SNTP  
 * @see             m2m_wifi_set_sytem_time       
 * @return        The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
NMI_API sint8 m2m_wifi_enable_sntp(uint8 bEnable);
/**@}*/
/** @defgroup WifiSetSystemTime m2m_wifi_set_sytem_time
 *   @ingroup WLANAPI
 *    Synchronous function for setting the system time in time/date format (@ref uint32).\n
 *    The @ref tstrSystemTime structure can be used as a reference to the time values that should be set and pass its value as @ref uint32
 */
 /**@{*/
/*!
 * @fn             NMI_API sint8 m2m_wifi_set_sytem_time(uint32);   
 * @param [in]     u32UTCSeconds
 *                    Seconds elapsed since January 1, 1900 (NTP Timestamp).  
 * @see            m2m_wifi_enable_sntp
 			  tstrSystemTime   
  * @note         If there is an RTC on the host MCU, the SNTP could be disabled and the host should set the system time to the firmware 
 *		         using the API \ref m2m_wifi_set_sytem_time.
 * @return        The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
NMI_API sint8 m2m_wifi_set_sytem_time(uint32 u32UTCSeconds);
/*!
 * @fn             NMI_API sint8 m2m_wifi_get_sytem_time(void);   
 * @see            m2m_wifi_enable_sntp
 			  		tstrSystemTime   
 * @note         get the system time from the sntp client
 *		         using the API \ref m2m_wifi_get_sytem_time.
 * @return        The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
NMI_API sint8 m2m_wifi_get_sytem_time(void);
/**@}*/
/** @defgroup WifiSetCustInfoElementFn m2m_wifi_set_cust_InfoElement
 *   @ingroup WLANAPI
 *   Synchronous function to Add/Remove user-defined Information Element to the WIFIBeacon and Probe Response frames while chip mode is Access Point Mode.\n
 *   According to the information element layout shown bellow, if it is required to set new data for the information elements, pass in the buffer with the
 *   information according to the sizes and ordering defined bellow. However, if it's required to delete these IEs, fill the buffer with zeros.
 */
 /**@{*/
/*!
 * @fn             NMI_API sint8 m2m_wifi_set_cust_InfoElement(uint8*);
 * @param [in]     pau8M2mCustInfoElement
 *                        Pointer to Buffer containing the IE to be sent. It is the application developer's responsibility to ensure on the correctness  of the information element's ordering passed in. 
 * @warning	       - Size of All elements combined must not exceed 255 byte.\n
 *			       - Used in Access Point Mode \n
 * @note              IEs Format will be follow the following layout:\n
 * @verbatim 
               --------------- ---------- ---------- ------------------- -------- -------- ----------- ----------------------  
              | Byte[0]       | Byte[1]  | Byte[2]  | Byte[3:length1+2] | ..... | Byte[n] | Byte[n+1] | Byte[n+2:lengthx+2]  | 
              |---------------|----------|----------|-------------------|-------- --------|-----------|------------------| 
              | #of all Bytes | IE1 ID   | Length1  | Data1(Hex Coded)  | ..... | IEx ID  | Lengthx   | Datax(Hex Coded)     | 
               --------------- ---------- ---------- ------------------- -------- -------- ----------- ---------------------- 
 * @endverbatim
 * @see             m2m_wifi_enable_sntp
 *                      tstrSystemTime               
 * @return        The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 \section Example
   The example demonstrates how the information elements are set using this function.
 *@code
 *
            char elementData[21];
            static char state = 0; // To Add, Append, and Delete
            if(0 == state) {	//Add 3 IEs
                state = 1;
                //Total Number of Bytes
                elementData[0]=12;
                //First IE
                elementData[1]=200; elementData[2]=1; elementData[3]='A';
                //Second IE
                elementData[4]=201; elementData[5]=2; elementData[6]='B'; elementData[7]='C';
                //Third IE
                elementData[8]=202; elementData[9]=3; elementData[10]='D'; elementData[11]=0; elementData[12]='F';
            } else if(1 == state) {	
                //Append 2 IEs to others, Notice that we keep old data in array starting with\n
                //element 13 and total number of bytes increased to 20
                state = 2; 
                //Total Number of Bytes
                elementData[0]=20;
                //Fourth IE
                elementData[13]=203; elementData[14]=1; elementData[15]='G';
                //Fifth IE
                elementData[16]=204; elementData[17]=3; elementData[18]='X'; elementData[19]=5; elementData[20]='Z';
            } else if(2 == state) {	//Delete All IEs
                state = 0; 
                //Total Number of Bytes
                elementData[0]=0;
            }
            m2m_wifi_set_cust_InfoElement(elementData);	
 * @endcode
 */
NMI_API sint8 m2m_wifi_set_cust_InfoElement(uint8* pau8M2mCustInfoElement);

/*!
@fn			NMI_API sint8 m2m_wifi_set_power_profile(uint8 u8PwrMode);
@brief		Change the power profile mode 
@param [in]	u8PwrMode
			Change the WINC power profile to different mode 
			PWR_LOW1/PWR_LOW2/PWR_HIGH/PWR_AUTO (tenuM2mPwrMode)
@return		The function SHALL return M2M_SUCCESE for success and a negative value otherwise.
@sa			tenuM2mPwrMode
@pre		m2m_wifi_init
@warning	must be called after the initializations and before any connection request and can't be changed in run time, 
*/
sint8 m2m_wifi_set_power_profile(uint8 u8PwrMode);
/*!
@fn			NMI_API sint8 m2m_wifi_set_tx_power(uint8 u8TxPwrLevel);
@brief		set the TX power tenuM2mTxPwrLevel
@param [in]	u8TxPwrLevel
			change the TX power tenuM2mTxPwrLevel
@return		The function SHALL return M2M_SUCCESE for success and a negative value otherwise.
@sa			tenuM2mTxPwrLevel
@pre		m2m_wifi_init
@warning	
*/
sint8 m2m_wifi_set_tx_power(uint8 u8TxPwrLevel);

/*!
@fn			NMI_API sint8 m2m_wifi_enable_firmware_logs(uint8 u8Enable);
@brief		Enable or Disable logs in run time (Disable Firmware logs will 
			enhance the firmware start-up time and performance)
@param [in]	u8Enable
			Set 1 to enable the logs 0 for disable
@return		The function SHALL return M2M_SUCCESE for success and a negative value otherwise.
@sa			__DISABLE_FIRMWARE_LOGS__ (build option to disable logs from initializations)
@pre		m2m_wifi_init
@warning	
*/
sint8 m2m_wifi_enable_firmware_logs(uint8 u8Enable);
/*!
@fn			NMI_API sint8 m2m_wifi_set_battery_voltage(uint8 u8BattVolt)
@brief		Set the battery voltage to update the firmware calculations
@param [in]	dbBattVolt
			Battery Volt in double
@return		The function SHALL return M2M_SUCCESE for success and a negative value otherwise.
@sa			
@pre		m2m_wifi_init
@warning	
*/
sint8 m2m_wifi_set_battery_voltage(uint16 u16BattVoltx100);
/**
*	@fn		m2m_wifi_get_firmware_version(tstrM2mRev* pstrRev)
*	@brief	Get Firmware version info
*	@param [out]	M2mRev
*			    pointer holds address of structure "tstrM2mRev" that contains the firmware version parameters
*	@version	1.0
*/
sint8 m2m_wifi_get_firmware_version(tstrM2mRev *pstrRev);
/**@}*/
#ifdef ETH_MODE
/** @defgroup WifiEnableMacMcastFn m2m_wifi_enable_mac_mcast
 *   @ingroup WLANAPI
 *   Synchronous function to Add/Remove MAC addresses in the multicast filter to receive multicast packets in bypass mode.
 */
 /**@{*/
/*!
 * @fn             NMI_API sint8 m2m_wifi_enable_mac_mcast(uint8 *, uint8);
 * @brief        
 * @param [in]     pu8MulticastMacAddress
 *                        Pointer to MAC address
 * @param [in]     u8AddRemove
 *                        A flag to add or remove the MAC ADDRESS, based on the following values:
 *                        -  0 : remove MAC address
 *                        -  1 : add MAC address    
 * @attention    This function is available in bypass mode ONLY. Make sure that firmware version built with the macro @ref ETH_MODE.\n  
 * @note         Maximum number of MAC addresses that could be added is 8.
 * @return       The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
NMI_API sint8 m2m_wifi_enable_mac_mcast(uint8* pu8MulticastMacAddress, uint8 u8AddRemove);
/**@}*/
/** @defgroup SetReceiveBufferFn m2m_wifi_set_receive_buffer
 *   @ingroup WLANAPI
 *    Synchronous function for  setting or changing the receiver buffer's length. 
 *    Changes are made according to the developer option in bypass mode and this function should be called in the receive callback handling.
 *@{*/
/*!
 * @fn             NMI_API sint8 m2m_wifi_set_receive_buffer(void *, uint16);
     
 * @param [in]     pvBuffer
 *                 Pointer to Buffer to receive data.
 *		     NULL pointer causes a negative error @ref M2M_ERR_FAIL.
 *
 * @param [in]     u16BufferLen
 *                 Length of data to be received.  Maximum length of data should not exceed the size defined by TCP/IP
 *      	     defined as @ref SOCKET_BUFFER_MAX_LENGTH
 *		     
 * @warning      This function is available in the bypass mode ONLY. Make sure that firmware version is built with macro @ref ETH_MODE.\n  
 * @return       The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
NMI_API sint8 m2m_wifi_set_receive_buffer(void* pvBuffer,uint16 u16BufferLen);
/**@}*/
#endif /* ETH_MODE */
/*!
 * @fn                  sint8 m2m_wifi_prng_get_random_bytes(uint8 * pu8PRNGBuff,uint16 u16PRNGSize)
 * @param [in]      pu8PrngBuff
 *                 		Pointer to Buffer to receive data.
 *		    		Size greater than the maximum specified (@ref M2M_BUFFER_MAX_SIZE - sizeof(tstrPrng))
 *				causes a negative error @ref M2M_ERR_FAIL.
 * @param [in]      u16PrngSize
 					request size in bytes  
 * @return       The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
sint8 m2m_wifi_prng_get_random_bytes(uint8 * pu8PrngBuff,uint16 u16PrngSize);
#endif /* __M2M_WIFI_H__ */


// mtm_socket_host_if.h *************************************************************************************************

 #ifndef __M2M_SOCKET_HOST_IF_H__
#define __M2M_SOCKET_HOST_IF_H__


#ifdef  __cplusplus
extern "C" {
#endif

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#ifndef	_BOOT_
#ifndef _FIRMWARE_
//#include "socket/include/socket.h"
#else
#include "m2m_types.h"
#endif
#endif

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

//#ifdef _FIRMWARE_
#define HOSTNAME_MAX_SIZE					(64)
//#endif

#define SSL_MAX_OPT_LEN						HOSTNAME_MAX_SIZE


#define SOCKET_CMD_INVALID					0x00
/*!< 
	Invlaid Socket command value.
*/

#define SOCKET_CMD_BIND						0x41
/*!< 
	Socket Binding command value.
*/

#define SOCKET_CMD_LISTEN					0x42
/*!< 
	Socket Listening command value.
*/

#define SOCKET_CMD_ACCEPT					0x43
/*!< 
	Socket Accepting command value.
*/

#define SOCKET_CMD_CONNECT					0x44
/*!< 
	Socket Connecting command value.
*/

#define SOCKET_CMD_SEND						0x45
/*!< 
	Socket send command value.
*/

#define SOCKET_CMD_RECV						0x46
/*!< 
	Socket Receive command value.
*/

#define SOCKET_CMD_SENDTO					0x47
/*!< 
	Socket sendTo command value.
*/

#define SOCKET_CMD_RECVFROM					0x48
/*!< 
	Socket ReceiveFrom command value.
*/

#define SOCKET_CMD_CLOSE					0x49
/*!< 
	Socket Close command value.
*/

#define SOCKET_CMD_DNS_RESOLVE				0x4A
/*!< 
	Socket DNS Resolve command value.
*/

#define SOCKET_CMD_SSL_CONNECT				0x4B
/*!< 
	SSL-Socket Connect command value.
*/

#define SOCKET_CMD_SSL_SEND					0x4C	
/*!< 
	SSL-Socket Send command value.
*/	

#define SOCKET_CMD_SSL_RECV					0x4D
/*!< 
	SSL-Socket Receive command value.
*/

#define SOCKET_CMD_SSL_CLOSE				0x4E
/*!< 
	SSL-Socket Close command value.
*/

#define SOCKET_CMD_SET_SOCKET_OPTION		0x4F
/*!< 
	Set Socket Option command value.
*/

#define SOCKET_CMD_SSL_CREATE				0x50
/*!<
*/

#define SOCKET_CMD_SSL_SET_SOCK_OPT			0x51

#define SOCKET_CMD_PING						0x52

#define SOCKET_CMD_SSL_SET_CS_LIST			0x53


#define PING_ERR_SUCCESS					0
#define PING_ERR_DEST_UNREACH				1
#define PING_ERR_TIMEOUT					2

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
DATA TYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/


/*!
*  @brief	
*/
typedef struct {	
	uint16		u16Family;
	uint16		u16Port;
	uint32		u32IPAddr;
    } tstrSockAddr;


typedef sint8			SOCKET;
typedef tstrSockAddr	tstrUIPSockAddr;


/*!
@brief
*/
typedef struct {
	tstrSockAddr	strAddr;
	SOCKET		sock;
	uint8		u8Void;
	uint16		u16SessionID;
    } tstrBindCmd;


/*!
@brief
*/
typedef struct {
	SOCKET		sock;
	sint8		s8Status;
	uint16		u16SessionID;
    } tstrBindReply;


/*!
*  @brief
*/
typedef struct {
	SOCKET	sock;
	uint8	u8BackLog;
	uint16	u16SessionID;
    } tstrListenCmd;


/*!
@struct	\
	tstrSocketRecvMsg
	
@brief	Socket recv status. 

	It is passed to the APPSocketEventHandler with SOCKET_MSG_RECV or SOCKET_MSG_RECVFROM message type 
	in a response to a user call to the recv or recvfrom.
	If the received data from the remote peer is larger than the USER Buffer size (given at recv call), the data is 
	delivered to the user in a number of consecutive chunks according to the USER Buffer size.
*/
typedef struct {
	SOCKET		sock;
	sint8			s8Status;
	uint16		u16SessionID;
    } tstrListenReply;


/*!
*  @brief
*/
typedef struct {
	tstrSockAddr	strAddr;
	SOCKET			sListenSock;
	SOCKET			sConnectedSock;
	uint16			u16Void;
    } tstrAcceptReply;


/*!
*  @brief
*/
typedef struct {
	tstrSockAddr	strAddr;
	SOCKET			sock;
	uint8			u8SslFlags;
	uint16			u16SessionID;
    } tstrConnectCmd;


/*!
@struct	\
	tstrConnectReply
	
@brief
	Connect Reply, contains sock number and error value
*/
typedef struct {
	SOCKET		sock;
	sint8		s8Error;
	uint16		u16AppDataOffset;
	/*!<
		In further packet send requests the host interface should put the user application
		data at this offset in the allocated shared data packet.
	*/
    } tstrConnectReply;


/*!
@brief
*/
typedef struct {
	SOCKET			sock;
	uint8			u8Void;
	uint16			u16DataSize;
	tstrSockAddr	strAddr;
	uint16			u16SessionID;
	uint16			u16Void;
    } tstrSendCmd;


/*!
@struct	\
	tstrSendReply
	
@brief
	Send Reply, contains socket number and number of sent bytes.
*/
typedef struct {
	SOCKET		sock;
	uint8		u8Void;
	sint16		s16SentBytes;
	uint16		u16SessionID;
	uint16		u16Void;
    } tstrSendReply;


/*!
*  @brief
*/
typedef struct {
	uint32		u32Timeoutmsec;
	SOCKET		sock;
	uint8		u8Void;
	uint16		u16SessionID;
    } tstrRecvCmd;


/*!
@struct
@brief
*/
typedef struct {
	tstrSockAddr		strRemoteAddr;
	sint16			s16RecvStatus;
	uint16			u16DataOffset;
	SOCKET			sock;
	uint8			u8Void;
	uint16			u16SessionID;
    } tstrRecvReply;


/*!
*  @brief
*/
typedef struct {
	uint32		u32OptionValue;
	SOCKET		sock;
	uint8 		u8Option;
	uint16		u16SessionID;
    } tstrSetSocketOptCmd;


typedef struct {
	SOCKET		sslSock;
	uint8		__PAD24__[3];
    } tstrSSLSocketCreateCmd;


/*!
*  @brief
*/
typedef struct {
	SOCKET		sock;
	uint8 		u8Option;
	uint16		u16SessionID;
	uint32		u32OptLen;
	uint8		au8OptVal[SSL_MAX_OPT_LEN];
    } tstrSSLSetSockOptCmd;


/*!
*/
typedef struct {
	uint32	u32DestIPAddr;
	uint32	u32CmdPrivate;
	uint16	u16PingCount;
	uint8	u8TTL;
	uint8	__PAD8__;
    } tstrPingCmd;


typedef struct {
	uint32	u32IPAddr;
	uint32	u32CmdPrivate;
	uint32	u32RTT;
	uint16	u16Success;
	uint16	u16Fail;
	uint8	u8ErrorCode;
	uint8	__PAD24__[3];
    } tstrPingReply;


typedef struct {
	uint32	u32CsBMP;
    } tstrSslSetActiveCsList;

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* __M2M_SOCKET_HOST_IF_H__ */


// socket.h *************************************************************************************************

#ifndef __SOCKET_H__
#define __SOCKET_H__

#ifdef  __cplusplus
extern "C" {
#endif

/** \defgroup SocketHeader Socket
 *          BSD alike socket interface beftween the host layer and the network 
 *          protocol stacks in the firmware.
 *          These functions are used by the host application to send or receive
 *          packets and to do other socket operations.    
 */
 
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

//#include "common/include/nm_common.h"

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/**
 * @defgroup  SocketDefines Defines
 * @ingroup SocketHeader
 */

/** @defgroup  IPDefines TCP/IP Defines
 * @ingroup SocketDefines
 * The following list of macros are used to define constants used throughout the socket layer.
 * @{
 */
//#define HOSTNAME_MAX_SIZE									64
/*!< 
	Maximum allowed size for a host domain name passed to the function gethostbyname @ref gethostbyname. 
	command value. Used with the setsockopt function. 

*/
	
#define SOCKET_BUFFER_MAX_LENGTH							1400
/*!< 
	Maximum allowed size for a socket data buffer. Used with @ref send socket 
	function to ensure that the buffer sent is within the allowed range. 
*/

#define  AF_INET											2
/*!< 
	The AF_INET is the address family used for IPv4. An IPv4 transport address is specified with the @ref sockaddr_in structure.
	(It is the only supported type for the current implementation.) 
*/


#define  SOCK_STREAM										1
/*!< 
	 One of the IPv4 supported socket types for reliable connection-oriented stream connection.
	 Passed to the @ref socket function for the socket creation operation. 
*/

#define  SOCK_DGRAM											2
/*!<
	 One of the IPv4 supported socket types for unreliable connectionless datagram connection.
	 Passed to the @ref socket function for the socket creation operation.
*/


#define SOCKET_FLAGS_SSL									0x01
/*!< 
	This flag shall be passed to the socket API for SSL session. 
*/

#define TCP_SOCK_MAX										(7)
/*!<
	Maximum number of simultaneous TCP sockets.
*/

#define UDP_SOCK_MAX										4
/*!<
	Maximum number of simultaneous UDP sockets.
*/

#define MAX_SOCKET											(TCP_SOCK_MAX + UDP_SOCK_MAX)
/*!<
	Maximum number of Sockets.
*/

#define SOL_SOCKET											1
/*!< 
	Socket option.
	Used with the @ref setsockopt function
*/

#define SOL_SSL_SOCKET										2
/*!< 
	SSL Socket option level.
	Used with the @ref setsockopt function
*/

#define	SO_SET_UDP_SEND_CALLBACK							0x00
/*!<
	Socket option used by the application to enable/disable
	the use of UDP send callbacks.
	Used with the @ref setsockopt function.
*/

#define IP_ADD_MEMBERSHIP									0x01
/*!<
	Set Socket Option Add Membership command value.
	Used with the @ref setsockopt function.
*/


#define IP_DROP_MEMBERSHIP									0x02
/*!<
	Set Socket Option Drop Membership command value.
	Used with the @ref setsockopt function.
*/
 //@}


    
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
PRIVATE DATA TYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/*!
*  @brief
*/
typedef struct{
	SOCKET		sock;
	uint8		u8Dummy;
	uint16		u16SessionID;
  } tstrCloseCmd;


/*!
*  @brief
*/
typedef struct{
	uint8			*pu8UserBuffer;
	uint16			u16UserBufferSize;
	uint16			u16SessionID;
	uint16			u16DataOffset;
	uint8			bIsUsed;
	uint8			u8SSLFlags;
	uint8			bIsRecvPending;
	enum SOCKET_EVENT statusEvents;        // GD, per fare un polled mode al posto delle callback
  } tstrSocket;


/**
 * @defgroup  TLSDefines TLS Defines
 * @ingroup SocketDefines
 */



/** @defgroup  SSLSocketOptions TLS Socket Options
 * @ingroup TLSDefines
 * The following list of macros are used to define SSL Socket options.
 * @{
 * @sa setsockopt
 */

#define SO_SSL_BYPASS_X509_VERIF							0x01
/*!<
	Allow an opened SSL socket to bypass the X509 certificate 
	verification process.
	It is highly required NOT to use this socket option in production
	software applications. It is supported for debugging and testing 
	purposes.
	The option value should be casted to int type and it is handled
	as a boolean flag.
*/


#define SO_SSL_SNI											0x02
/*!<
	Set the Server Name Indicator (SNI) for an SSL socket. The
	SNI is a NULL terminated string containing the server name
	assocated with the connection. It must not exceed the size
	of HOSTNAME_MAX_SIZE.
*/


#define SO_SSL_ENABLE_SESSION_CACHING						0x03
/*!<
	This option allow the TLS to cache the session information for fast
	TLS session establishment in future connections using the
	TLS Protocol session resume features.
*/

//@}



/** @defgroup  SSLCipherSuiteConfiguration TLS Cipher Suite Configurations
 * @ingroup TLSDefines
 * The following list of macros are used to define SSL Ciphersuite Configuration.
 * @sa sslSetActiveCipherSuites
 * @{
 */

#define SSL_ENABLE_ALL_SUITES                               0xfffffffful
/*!<
	Enable all possible supported cipher suites.
*/

#define SSL_ENABLE_RSA_SHA_SUITES							0x01
/*!<
	Enable RSA Hmac_SHA based Ciphersuites. For example,
		TLS_RSA_WITH_AES_128_CBC_SHA
*/

#define SSL_ENABLE_RSA_SHA256_SUITES						0x02
/*!<
	Enable RSA Hmac_SHA256 based Ciphersuites. For example,
		TLS_RSA_WITH_AES_128_CBC_SHA256
*/

#define SSL_ENABLE_DHE_SHA_SUITES							0x04
/*!<
	Enable DHE Hmac_SHA based Ciphersuites. For example,
		TLS_DHE_RSA_WITH_AES_128_CBC_SHA
*/

#define SSL_ENABLE_DHE_SHA256_SUITES						0x08
/*!<
	Enable DHE Hmac_SHA256 based Ciphersuites. For example,
		TLS_DHE_RSA_WITH_AES_128_CBC_SHA256
*/

#define SSL_ENABLE_RSA_GCM_SUITES							0x10
/*!<
	Enable RSA AEAD based Ciphersuites. For example,
		TLS_RSA_WITH_AES_128_GCM_SHA256
*/

#define SSL_ENABLE_DHE_GCM_SUITES							0x20
/*!<
	Enable DHE AEAD based Ciphersuites. For example,
		TLS_DHE_RSA_WITH_AES_128_GCM_SHA256
*/

 //@}




/**************
Socket Errors
**************/
/**@defgroup  SocketErrorCode Error Codes
 * @ingroup SocketHeader
 * The following list of macros are used to define the possible error codes returned as a result of a call to a socket function.
 * Errors are listed in numerical order with the error macro name.
 * @{
 */
#define SOCK_ERR_NO_ERROR									0
/*!<
	Successfull socket operation
*/

#define SOCK_ERR_INVALID_ADDRESS							-1
/*!<
	Socket address is invalid. The socket operation cannot be completed successfully without specifying a specific address 
	For example: bind is called without specifying a port number
*/

#define SOCK_ERR_ADDR_ALREADY_IN_USE						-2
/*!<
	Socket operation cannot bind on the given address. With socket operations, only one IP address per socket is permitted. 
	Any attempt for a new socket to bind with an IP address already bound to another open socket, 
	will return the following error code. States that bind operation failed. 
*/

#define SOCK_ERR_MAX_TCP_SOCK								-3
/*!<
	Exceeded the maximum number of TCP sockets. A maximum number of TCP sockets opened simultaneously is defined through TCP_SOCK_MAX. 
	It is not permitted to exceed that number at socket creation. Identifies that @ref socket operation failed. 
*/

#define SOCK_ERR_MAX_UDP_SOCK								-4
/*!<
	Exceeded the maximum number of UDP sockets. A maximum number of UDP sockets opened simultaneously is defined through UDP_SOCK_MAX. 
	It is not permitted to exceed that number at socket creation. Identifies that @ref socket operation failed
*/

#define SOCK_ERR_INVALID_ARG								-6
/*!<
	An invalid arguement is passed to a function.
*/

#define SOCK_ERR_MAX_LISTEN_SOCK							-7
/*!<
	Exceeded the maximum number of TCP passive listening sockets.
	Identifies Identifies that @ref listen operation failed. 
*/

#define SOCK_ERR_INVALID									-9
/*!<
	The requested socket operation is not valid in the
	current socket state. 
	For example: @ref accept is called on a TCP socket before @ref bind or @ref listen.
*/

#define SOCK_ERR_ADDR_IS_REQUIRED							-11
/*!<
	Destination address is required. Failure to provide the socket address required for the socket operation to be completed.
	It is generated as an error to the @ref sendto function when the address required to send the data to is not known. 
*/

#define SOCK_ERR_CONN_ABORTED								-12
/*!<
	The socket is closed by the peer. The local socket is
	closed also.
*/

#define SOCK_ERR_TIMEOUT									-13
/*!<
	The socket pending operation has  timedout. 
*/

#define SOCK_ERR_BUFFER_FULL								-14
/*!<
	No buffer space available to be used for the requested socket operation.
*/

#ifdef _NM_BSP_BIG_END

#define _htonl(m)				(m)
#define _htons(A)				(A)

#else

#define _htonl(m)		\
	(uint32)(((uint32)(m << 24)) | ((uint32)((m & 0x0000FF00) << 8)) | ((uint32)((m & 0x00FF0000) >> 8)) | ((uint32)(m >> 24)))
/*!<
	Convert a 4-byte integer from the host representation to the Network byte order representation.
*/


#define _htons(A)   	(uint16)((((uint16) (A)) << 8) | (((uint16) (A)) >> 8))
/*!<
	Convert a 2-byte integer (short) from the host representation to the Network byte order representation.
*/


#endif


#define _ntohl      		_htonl
/*!<
	Convert a 4-byte integer from the Network byte order representation to the host representation .
*/


#define _ntohs      		_htons
/*!<
	Convert a 2-byte integer from the Network byte order representation to the host representation .
*/
 //@}

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
DATA TYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/** @defgroup  SocketEnums Enumeration-Typedef
 * @ingroup SocketHeader
 * Specific Enumeration-typedefs used for socket operations
 * @{ */

/*!
@typedef	\
	SOCKET

@brief
	Definition for socket handler data type.
	Socket ID,used with all socket operations to uniquely identify the socket handler.
	Such an ID is uniquely assigned at socket creation when calling @ref socket operation.
*/
typedef sint8  SOCKET;



/*!
@struct	\
	in_addr

@brief
	IPv4 address representation.

	This structure is used as a placeholder for IPV4 address in other structures.
@see
	sockaddr_in
*/
typedef struct {
	uint32		s_addr;
	/*!<
		Network Byte Order representation of the IPv4 address. For example,
		the address "192.168.0.10" is represented as 0x0A00A8C0.
	*/
    } in_addr;


/*!
@struct	\
	sockaddr

@brief
	Generic socket address structure.

@see \
      sockaddr_in
*/
struct sockaddr {
    uint16		sa_family;
	/*!< 
Socket address family.
	*/
    uint8		sa_data[14];
	/*!< 
		    Maximum size of all the different socket address structures.
	*/
    };


/*!
@struct	\
	sockaddr_in

@brief
	Socket address structure for IPV4 addresses. Used to specify socket address information to which to connect to.
	Can be cast to @ref sockaddr structure.
*/
struct sockaddr_in {
	uint16			sin_family;
	/*!<
		Specifies the address family(AF).
		Members of AF_INET address family are IPv4 addresses.
		Hence,the only supported value for this is AF_INET.
	*/
	uint16   		sin_port;
	/*!<
		Port number of the socket. 
		Network sockets are identified by a pair of IP addresses and port number.
		It must be set in the Network Byte Order format , @ref _htons (e.g. _htons(80)).
		Can NOT have zero value.
	*/
	in_addr			sin_addr;
	/*!<
		IP Address of the socket.
		The IP address is of type @ref in_addr structure. 
		Can be set to "0" to accept any IP address for server operation. non zero otherwise.
	*/
	uint8			sin_zero[8];
	/*!<
		Padding to make structure the same size as @ref sockaddr.
	*/
};
 //@}
/**@defgroup  AsyncCalback Asynchronous Events
 * @ingroup SocketEnums
 * Specific Enumuration used for asynchronous operations
 * @{ */
/*!
@enum	\
	tenuSocketCallbackMsgType

@brief
	Asynchronous APIs, make use of callback functions, in-order to return back the results once the corresponding socket operation is completed.
	Hence resuming the normal execution of the application code while the socket operation returns the results. 
	Callback functions expect event messages to be passed in, in-order to identify the operation they're returning the results for.
	The following enum identifes the type of events that are received in the callback function.
	
	Application Use:
	In order for application developers to handle the pending events from the network controller through the callback functions.
	A function call must be made to the function @ref m2m_wifi_handle_events at least once for each socket operation.

@see
     bind
     listen
     accept
     connect
     send
     recv
     
*/
typedef enum{
	SOCKET_MSG_BIND	= 1,
	/*!<
		Bind socket event.
	*/
	SOCKET_MSG_LISTEN,
	/*!<
		Listen socket event.
	*/
	SOCKET_MSG_DNS_RESOLVE,
	/*!<
		DNS Resolution event.
	*/
	SOCKET_MSG_ACCEPT,
	/*!<
		Accept socket event.
	*/
	SOCKET_MSG_CONNECT,     //5
	/*!<
		Connect socket event.
	*/
	SOCKET_MSG_RECV,
	/*!<
		Receive socket event.
	*/
	SOCKET_MSG_SEND,        //7
	/*!<
		Send socket event.
	*/
	SOCKET_MSG_SENDTO,
	/*!<
		sendto socket event.
	*/
	SOCKET_MSG_RECVFROM,     //9
	/*!<
		Recvfrom socket event.
	*/
	SOCKET_MSG_CLOSE           // GD/C extension 5/2021 froci!!
	/*!<
		Clsoe socket event.
	*/
    } tenuSocketCallbackMsgType;


/*!
@struct	\
	tstrSocketBindMsg

@brief	Socket bind status.

	An asynchronous call to the @ref bind socket operation, returns information through this structure in response.
	This structure together with the event @ref SOCKET_MSG_BIND are passed in paramters to the callback function.
@see
     bind
	
*/
typedef struct {
	sint8		status;
	/*!<
		The result of the bind operation. 
		Holding a value of ZERO for a successful bind or otherwise a negative 
		error code corresponding to the type of error.
	*/
    } tstrSocketBindMsg;


/*!
@struct	\
	tstrSocketListenMsg

@brief	Socket listen status.

	Socket listen information is returned through this structure in response to the asynchronous call to the @ref listen function.
	This structure together with the event @ref SOCKET_MSG_LISTEN are passed-in paramters to the callback function.
@see
      listen
*/
typedef struct {
	sint8		status;
	/*!<
		Holding a value of ZERO for a successful listen or otherwise a negative 
		error code corresponding to the type of error.
	*/
    } tstrSocketListenMsg;



/*!
@struct	\
	tstrSocketAcceptMsg

@brief	Socket accept status.

	Socket accept information is returned through this structure in response to the asynchronous call to the @ref accept function.
	This structure together with the event @ref SOCKET_MSG_ACCEPT are passed-in parameters to the callback function. 
*/
typedef struct {
	SOCKET		sock;
	/*!<
		On a successful @ref accept operation, the return information is the socket ID for the accepted connection with the remote peer. 
		Otherwise a negative error code is returned to indicate failure of the accept operation.
	*/
	struct		sockaddr_in	strAddr;
	/*!<
		Socket address structure for the remote peer.
	*/
    } tstrSocketAcceptMsg;


/*!
@struct	\
	tstrSocketConnectMsg

@brief	Socket connect status.

	Socket connect information is returned through this structure in response to the asynchronous call to the @ref connect socket function.
	This structure together with the event @ref SOCKET_MSG_CONNECT are passed-in paramters to the callback function.
*/
typedef struct {
	SOCKET	sock;
	/*!<
		Socket ID referring to the socket passed to the connect function call.
	*/
	sint8		s8Error;
	/*!<
		Connect error code. 
		Holding a value of ZERO for a successful connect or otherwise a negative 
		error code corresponding to the type of error.
	*/
    } tstrSocketConnectMsg;


/*!
@struct	\
	tstrSocketRecvMsg

@brief	Socket recv status.

	Socket receive information is returned through this structure in response to the asynchronous call to the recv or recvfrom socket functions.
	This structure together with the events @ref SOCKET_MSG_RECV or @ref SOCKET_MSG_RECVFROM are passed-in parameters to the callback function.
@remark 
	In case the received data from the remote peer is larger than the USER buffer size defined during the asynchronous call to the @ref recv function, the data is 
	delivered to the user in a number of consecutive chunks according to the USER Buffer size.
	a negative or zero buffer size indicates an error with the following code:
	@ref SOCK_ERR_NO_ERROR     		 : Socket connection  closed
	@ref SOCK_ERR_CONN_ABORTED 	 	 : Socket connection aborted
	@SOCK_ERR_TIMEOUT	 			 : Socket receive timed out
*/
typedef struct {
	uint8					*pu8Buffer;
	/*!<
		Pointer to the USER buffer (passed to @ref recv and @ref recvfrom function) containing the received data chunk.
	*/
	sint16					s16BufferSize;
	/*!<
		The recevied data chunk size.
		Holds a negative value if there is a receive error or ZERO on success upon reception of close socket message.
	*/
	uint16					u16RemainingSize;
	/*!<
		The number of bytes remaining in the current @ref  recv operation.
	*/
	struct sockaddr_in		strRemoteAddr;
	/*!<
		Socket address structure for the remote peer. It is valid for @ref SOCKET_MSG_RECVFROM event.
	*/
    } tstrSocketRecvMsg;

/*!
@struct	\
	tstrDnsReply
	
@brief
	DNS Reply, contains hostName and HostIP.
*/
typedef struct {
	char		acHostName[HOSTNAME_MAX_SIZE];
	uint32		u32HostIP;
    } tstrDnsReply;


/*!
@typedef \
	tpfAppSocketCb

@brief
				The main socket application callback function. Applications register their main socket application callback through this function by calling  @ref registerSocketCallback.
				In response to events received, the following callback function is called to handle the corresponding asynchronous function called. Example: @ref bind, @ref connect,...etc. 

@param [in] sock
				Socket ID for the callback.

				The socket callback function is called whenever a new event is recived in response 
				to socket operations.
				
@param [in] u8Msg
				 Socket event type. Possible values are:
				  - @ref SOCKET_MSG_BIND
				  - @ref SOCKET_MSG_LISTEN
				  - @ref SOCKET_MSG_ACCEPT
				  - @ref SOCKET_MSG_CONNECT
				  - @ref SOCKET_MSG_RECV
				  - @ref SOCKET_MSG_SEND
				  - @ref SOCKET_MSG_SENDTO
				  - @ref SOCKET_MSG_RECVFROM
				
@param [in] pvMsg
				Pointer to message structure. Existing types are:
				  - tstrSocketBindMsg
				  - tstrSocketListenMsg
				  - tstrSocketAcceptMsg
				  - tstrSocketConnectMsg
				  - tstrSocketRecvMsg

@see
	tenuSocketCallbackMsgType 
	tstrSocketRecvMsg
	tstrSocketConnectMsg 
	tstrSocketAcceptMsg
	tstrSocketListenMsg
	tstrSocketBindMsg 
*/
typedef void (*tpfAppSocketCb) (SOCKET sock, uint8 u8Msg, void * pvMsg);


/*!
@typedef	\
	tpfAppResolveCb

@brief
        DNS resolution callback function. 
	Applications requiring DNS resolution should register their callback through this function by calling @ref registerSocketCallback.
	The following callback is triggered in response to asynchronous call to the @ref gethostbyname function (DNS Resolution callback).

@param [in] pu8DomainName
				Domain name of the host.

@param [in]	u32ServerIP
				Server IPv4 address encoded in NW byte order format. If it is Zero, then the DNS resolution failed.
*/
typedef void (*tpfAppResolveCb) (uint8* pu8DomainName, uint32 u32ServerIP);

/*!
@typedef \
	tpfPingCb

@brief	PING Callback

	The function delivers the ping statistics for the sent ping triggered by calling 
	m2m_ping_req.

@param [in]	u32IPAddr
				Destination IP.

@param [in]	u32RTT
				Round Trip Time.

@param [in]	u8ErrorCode
				Ping error code. It may be one of:
				- PING_ERR_SUCCESS
				- PING_ERR_DEST_UNREACH
				- PING_ERR_TIMEOUT
*/
typedef void (*tpfPingCb)(uint32 u32IPAddr, uint32 u32RTT, uint8 u8ErrorCode);
 
 /**@}*/ 
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION PROTOTYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/** \defgroup SocketAPI Function
 *   @ingroup SocketHeader
 */

/** @defgroup SocketInitalizationFn socketInit
 *  @ingroup SocketAPI
 *     The function performs the necessary initializations for the socket library through the following steps:
	- A check made by the global variable gbSocketInit, ensuring that initialization for sockets is performed only once, 
	 in-order to prevent reseting the socket instances already created in the global socket array (gastrSockets).
	- Zero initializations to the global socket array (gastrSockets), which holds the list of TCP sockets.
	- Registers the socket (Host Interface)hif callback function through the call to the hif_register_cb function.
	   This facilitates handling  all of the socket related functions received through interrupts from the firmware.
	   
 */
 /**@{*/ 
/*!
@fn	\
	NMI_API void socketInit(void);

@param [in]	void

@return          void

@remarks 
	This initialization function must be invoked before any socket operation is performed.
	No error codes from this initialization function since the socket array is statically allocated based in the maximum number of 
	sockets @ref MAX_SOCKET based on the systems capibility.
\section Example
This example demonstrates the use of the socketinit for socket initialization for an mqtt chat application.
 \code
	tstrWifiInitParam param;
	int8_t ret;
	char topic[strlen(MAIN_CHAT_TOPIC) + MAIN_CHAT_USER_NAME_SIZE + 1];

	//Initialize the board.
	system_init();

	//Initialize the UART console. 
	configure_console();

	// Initialize the BSP.
	nm_bsp_init();
	
	 ----------
	 
	// Initialize socket interface.
	socketInit();
	registerSocketCallback(socket_event_handler, socket_resolve_handler);

	// Connect to router. 
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
			MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

\endcode
*/
NMI_API void socketInit(void);

/*!
@fn	\			
	NMI_API void socketDeinit(void);

@brief	Socket Layer De-initialization

	The function performs the necessary cleanup for the socket library static data
	It must be invoked as the last any socket operation is performed on any active sockets.
*/
NMI_API void socketDeinit(void);
/** @} */
/** @defgroup SocketCallbackFn registerSocketCallback
 *    @ingroup SocketAPI
	  Register two callback functions one for asynchronous socket events and the other one for DNS callback registering function. 
	  The registered callback functions are used to retrieve information in response to the asynchronous socket functions called.
 */
 /**@{*/ 


/*!
@fn	\
	NMI_API void registerSocketCallback(tpfAppSocketCb socket_cb, tpfAppResolveCb resolve_cb);
             
@param [in]   tpfAppSocketCb 
                                 Assignment of callback function to the global callback @ref tpfAppSocketCb gpfAppSocketCb. Delivers
                                 socket messages to the host application. In response to the asynchronous function calls, such as @ref bind
                                 @ref listen @ref accept @ref connect
                        
@param [in] 	tpfAppResolveCb 
                                 Assignment of callback function to the global callback @ref tpfAppResolveCb gpfAppResolveCb. 
                                 Used for DNS resolving functionalites. The DNS resolving technique is determined by the application 
                                 registering the callback.
                                 NULL is assigned when, DNS resolution is not required.
				  
@return          void
@remarks 
		If any of the socket functionaities is not to be used, NULL is passed in as a parameter.
      	It must be invoked after socketinit and before other socket layer operations.
		
\section Example
	This example demonstrates the use of the registerSocketCallback to register a socket callback function with DNS resolution CB set to null
	for a simple UDP server example.
 \code
      	tstrWifiInitParam param;
	int8_t ret;
	struct sockaddr_in addr;

	// Initialize the board
	system_init();

	//Initialize the UART console. 
	configure_console();
	
	// Initialize the BSP.
	nm_bsp_init();

	// Initialize socket address structure.
	addr.sin_family = AF_INET;
	addr.sin_port = _htons(MAIN_WIFI_M2M_SERVER_PORT);
	addr.sin_addr.s_addr = _htonl(MAIN_WIFI_M2M_SERVER_IP);

	// Initialize Wi-Fi parameters structure. 
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	// Initialize Wi-Fi driver with data and status callbacks.
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) {
		}
	}

	// Initialize socket module
	socketInit();
	registerSocketCallback(socket_cb, NULL);

	// Connect to router.
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
	\endcode
*/
NMI_API void registerSocketCallback(tpfAppSocketCb socket_cb, tpfAppResolveCb resolve_cb);
/** @} */

/** @defgroup SocketFn socket
 *    @ingroup SocketAPI
 * 	Synchronous socket allocation function based on the specified socket type. Created sockets are non-blocking and their possible types are either TCP or a UDP sockets. 
 *  The maximum allowed number of TCP sockets is @ref TCP_SOCK_MAX sockets while the maximum number of UDP sockets that can be created simultaneously is @ref UDP_SOCK_MAX sockets. 
 *    
*/
 /**@{*/
/*!
@fn	\
	NMI_API SOCKET socket(uint16 u16Domain, uint8 u8Type, uint8 u8Flags);

	
@param [in]	u16Domain
				Socket family. The only allowed value is AF_INET (IPv4.0) for TCP/UDP sockets.

@param [in] u8Type
				Socket type. Allowed values are:
				- [SOCK_STREAM](@ref SOCK_STREAM)
				- [SOCK_DGRAM](@ref SOCK_DGRAM)

@param [in] u8Flags
				Used to specify the socket creation flags. It shall be set to zero for normal TCP/UDP sockets.
				If could be SOCKET_FLAGS_SSL if the socket is used for SSL session. The use of the flag
				[SOCKET_FLAGS_SSL](@ref SOCKET_FLAGS_SSL) has no meaning in case of UDP sockets.

@pre
	The @ref socketInit function must be called once at the beginning of the application to initialize the socket handler.
	before any call to the socket function can be made.

@see
	connect
	bind
	listen
	accept
	recv
	recvfrom
	send
	sendto
	close
	setsockopt
	getsockopt
	
@return	
	On successful socket creation, a non-blocking socket type is created and a socket ID is returned
	In case of failure the function returns a negative value, identifying one of the socket error codes defined.
	For example: @ref SOCK_ERR_INVALID for invalid argument or 
	                    @ref SOCK_ERR_MAX_TCP_SOCK	if the number of TCP allocated sockets exceeds the number of available sockets. 

@remarks
 	       The socket function must be called apriori any other related socket functions "e.g. send, recv, close ..etc"
\section Example
	This example demonstrates the use of the socket function to allocate the socket, returning the socket handler to be used for other
socket operations. Socket creation is dependent on the socket type.
\subsection sub1 UDP example
@code
	SOCKET UdpServerSocket = -1;
	
	UdpServerSocket = socket(AF_INET, SOCK_DGRAM, 0);
	
@endcode
\subsection sub2 TCP example
@code
	static SOCKET tcp_client_socket = -1;

	tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0));
@endcode
*/
NMI_API SOCKET socket(uint16 u16Domain, uint8 u8Type, uint8 u8Flags);
/** @} */
/** @defgroup BindFn bind
 *  @ingroup SocketAPI
*	Asynchronous bind function associates the provided address and local port to the socket. 
*   The function can be used with both TCP and UDP sockets it's mandatory to call the @ref bind function before starting any UDP or TCP server operation. 
*   Upon socket bind completion, the application will receive a @ref SOCKET_MSG_BIND message in the socket callback. 
*/
 /**@{*/
/*!
\fn	\
	NMI_API sint8 bind(SOCKET sock, struct sockaddr *pstrAddr, uint8 u8AddrLen);
 

@param [in]	sock
				Socket ID, must hold a non negative value.
				A negative value will return a socket error @ref SOCK_ERR_INVALID_ARG. Indicating that an invalid argument is passed in.

@param [in] pstrAddr
				Pointer to socket address structure "sockaddr_in" 
				[sockaddr_in](@ref sockaddr_in)
								    

@param [in] u8AddrLen
				Size of the given socket address structure in bytes. 

@pre
	The socket function must be called to allocate a socket before passing the socket ID to the bind function.

@see
	socket
	connect
	listen
	accept
	recv
	recvfrom
	send
	sendto

@return		
	The function returns ZERO for successful operations and a negative value otherwise. 
	The possible error values are:
	- [SOCK_ERR_NO_ERROR](@ref SOCK_ERR_NO_ERROR)	
		Indicating that the operation was successful.
		
	- [SOCK_ERR_INVALID_ARG](@ref SOCK_ERR_INVALID_ARG)
		Indicating passing invalid arguments such as negative socket ID or NULL socket address structure.

	- [SOCK_ERR_INVALID](@ref SOCK_ERR_INVALID)
		Indicate socket bind failure.
\section Example
	This example demonstrates the call of the bind socket operation after a successful socket operation.
@code
	struct sockaddr_in	addr;
	SOCKET udpServerSocket =-1;
	int ret = -1;
	
	if(udpServerSocket == -1)	{
		udpServerSocket = socket(AF_INET,SOCK_DGRAM,0);
		if(udpServerSocket >= 0)		{
			addr.sin_family		= AF_INET;
			addr.sin_port			= _htons(UDP_SERVER_PORT);
			addr.sin_addr.s_addr	= 0;
			ret = bind(udpServerSocket,(struct sockaddr*)&addr,sizeof(addr));

			if(ret == 0)
				printf("Bind success!\n");
			else {
				printf("Bind Failed. Error code = %d\n",ret);
				close(udpServerSocket);
		}
		else		{
			printf("UDP Server Socket Creation Failed\n");
			return;
		}
	}
@endcode	
*/
NMI_API sint8 bind(SOCKET sock, struct sockaddr *pstrAddr, uint8 u8AddrLen);
/** @} */

/** @defgroup ListenFn listen
 *   @ingroup SocketAPI
 * 	After successful socket binding to an IP address and port on the system, start listening on a passive socket for incoming connections. 
       The socket must be bound on a local port or the listen operationfails. 
       Upon the call to the asynchronous listen function, response is received through the event [SOCKET_MSG_BIND](@ref SOCKET_MSG_BIND)
	in the socket callback.
	A successful listen means the TCP server operation is active. If a connection is accepted, then the application socket callback function is 
	notified with the new connected socket through the event @ref SOCKET_MSG_ACCEPT. Hence there is no need to call the @ref accept function
	after calling @ref listen.
	
	After a connection is accepted, the user is then required to call the @ref recv to receive any packets transmitted by the remote host or to receive notification of socket connection
	termination.
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 listen(SOCKET sock, uint8 backlog);

@param [in]	sock
				Socket ID, must hold a non negative value.
				A negative value will return a socket error @ref SOCK_ERR_INVALID_ARG. Indicating that an invalid argument is passed in.

@param [in] backlog
				Not used by the current implementation.
				
@pre
	The bind function must be called to assign the port number and IP address to the socket before the listen operation.

@see
	bind
	accept
	recv
	recvfrom
	send
	sendto

@return		
	The function returns ZERO for successful operations and a negative value otherwise. 
	The possible error values are:
	- [SOCK_ERR_NO_ERROR](@ref SOCK_ERR_NO_ERROR)	
		Indicating that the operation was successful.
		
	- [SOCK_ERR_INVALID_ARG](@ref SOCK_ERR_INVALID_ARG)
		Indicating passing invalid arguments such as negative socket ID.

	- [SOCK_ERR_INVALID](@ref SOCK_ERR_INVALID)
		Indicate socket listen failure.
\section Example
This example demonstrates the call of the listen socket operation after a successful socket operation.
@code
	static void TCP_Socketcallback(SOCKET sock, uint8 u8Msg, void * pvMsg)	{	
		int ret =-1;
		
		switch(u8Msg)		{	
		case SOCKET_MSG_BIND:
			{
				tstrSocketBindMsg	*pstrBind = (tstrSocketBindMsg*)pvMsg;
				if(pstrBind != NULL)				{
					if(pstrBind->status == 0)					{
						ret = listen(sock, 0);
						
						if(ret <0)
							printf("Listen failure! Error = %d\n",ret);
					}
					else					{
						M2M_ERR("bind Failure!\n");
						close(sock);
					}
				}
			}
			break;

		case SOCKET_MSG_LISTEN:
			{

				tstrSocketListenMsg	*pstrListen = (tstrSocketListenMsg*)pvMsg;
				if(pstrListen != NULL)				{
					if(pstrListen->status == 0)					{
						ret = accept(sock,NULL,0);
					}
					else					{
						M2M_ERR("listen Failure!\n");
						close(sock);
					}
				}
			}
			break;

		case SOCKET_MSG_ACCEPT:
			{
				tstrSocketAcceptMsg	*pstrAccept = (tstrSocketAcceptMsg*)pvMsg;
				
				if(pstrAccept->sock >= 0)				{
					TcpNotificationSocket = pstrAccept->sock;
					recv(pstrAccept->sock,gau8RxBuffer,sizeof(gau8RxBuffer),TEST_RECV_TIMEOUT);
				}
				else				{
					M2M_ERR("accept failure\n");
				}
			}
			break;
		
		default:
			break;
		}
	}

@endcode
*/
NMI_API sint8 listen(SOCKET sock, uint8 backlog);
/** @} */
/** @defgroup AcceptFn accept
 *    @ingroup SocketAPI
 *	The function has no current implementation. An empty deceleration is used to prevent errors when legacy application code is used. 
 *     For recent application use, the accept function can be saferIt has no effect and could be safely removed from any application using it.
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 accept(SOCKET sock, struct sockaddr *addr, uint8 *addrlen);

@param [in]	sock
				Socket ID, must hold a non negative value.
				A negative value will return a socket error @ref SOCK_ERR_INVALID_ARG. Indicating that an invalid argument is passed in.
@param [in] addr
				Not used in the current implementation.

@param [in] addrlen
				Not used in the current implementation.

@return		
	The function returns ZERO for successful operations and a negative value otherwise. 
	The possible error values are:
	- [SOCK_ERR_NO_ERROR](@ref SOCK_ERR_NO_ERROR)	
		Indicating that the operation was successful.
		
	- [SOCK_ERR_INVALID_ARG](@ref SOCK_ERR_INVALID_ARG)
		Indicating passing invalid arguments such as negative socket ID.
*/
NMI_API sint8 accept(SOCKET sock, struct sockaddr *addr, uint8 *addrlen);
/** @} */
/** @defgroup ConnectFn connect
 *    @ingroup SocketAPI
 *  	Establishes a TCP connection with a remote server.
	The asynchronous connect function must be called after receiving a valid socket ID from the @ref socket function.
	The application socket callback function is notified of a successful new  socket connection through the event @ref SOCKET_MSG_CONNECT. 
	A successful connect means the TCP session is active. The application is then required to make a call to the @ref recv
	to receive any packets transmitted by the remote server, unless the application is interrupted by a notification of socket connection
	termination.
 */
 /**@{*/
/*!
@fn	\	
	NMI_API sint8 connect(SOCKET sock, struct sockaddr *pstrAddr, uint8 u8AddrLen);

@param [in]	sock
				Socket ID, must hold a non negative value.
				A negative value will return a socket error @ref SOCK_ERR_INVALID_ARG. Indicating that an invalid argument is passed in.

@param [in]	pstrAddr
				Address of the remote server.
@param [in] 	pstrAddr
				Pointer to socket address structure "sockaddr_in" 
				[sockaddr_in](@ref sockaddr_in)

@param [in]	u8AddrLen
				 Size of the given socket address structure in bytes. 
				 Not currently used, implemented for BSD compatibility only.
@pre
	The socket function must be called to allocate a TCP socket before passing the socket ID to the bind function.
	If the socket is not bound, you do NOT have to call bind before the "connect" function.

@see
	socket
	recv
	send
	close
	
@return		
	The function returns ZERO for successful operations and a negative value otherwise. 
	The possible error values are:
	- [SOCK_ERR_NO_ERROR](@ref SOCK_ERR_NO_ERROR)	
		Indicating that the operation was successful.
		
	- [SOCK_ERR_INVALID_ARG](@ref SOCK_ERR_INVALID_ARG)
		Indicating passing invalid arguments such as negative socket ID or NULL socket address structure.

	- [SOCK_ERR_INVALID](@ref SOCK_ERR_INVALID)
		Indicate socket connect failure.
\section Example
   The example demonstrates a TCP application, showing how the asynchronous call to the connect function is made through the main function and how the 
   callback function handles the @ref SOCKET_MSG_CONNECT event.
\subsection sub1 Main Function
@code
	struct sockaddr_in	Serv_Addr;
	SOCKET TcpClientSocket =-1;
	int ret = -1
	
	TcpClientSocket = socket(AF_INET,SOCK_STREAM,0);
	Serv_Addr.sin_family = AF_INET;
	Serv_Addr.sin_port = _htons(1234);
	Serv_Addr.sin_addr.s_addr = inet_addr(SERVER);
	printf("Connected to server via socket %u\n",TcpClientSocket);

	do	{
		ret = connect(TcpClientSocket,(sockaddr_in*)&Serv_Addr,sizeof(Serv_Addr));
		if(ret != 0)		{
			printf("Connection Error\n");
		}
		else		{
			printf("Connection successful.\n");
			break;
		}
	} while(1)	
@endcode
\subsection sub2 Socket Callback
@code
	if(u8Msg == SOCKET_MSG_CONNECT)	{
		tstrSocketConnectMsg	*pstrConnect = (tstrSocketConnectMsg*)pvMsg;
		if(pstrConnect->s8Error == 0)		{
			uint8	acBuffer[GROWL_MSG_SIZE];
			uint16	u16MsgSize;

			printf("Connect success!\n");
			
			u16MsgSize = FormatMsg(u8ClientID, acBuffer);
			send(sock, acBuffer, u16MsgSize, 0);
			recv(pstrNotification->Socket, (void*)au8Msg,GROWL_DESCRIPTION_MAX_LENGTH, GROWL_RX_TIMEOUT);
			u8Retry = GROWL_CONNECT_RETRY;
		}
		else		{
			M2M_DBG("Connection Failed, Error: %d\n",pstrConnect->s8Error");
			close(pstrNotification->Socket);
		}
	}
@endcode
*/
NMI_API sint8 connect(SOCKET sock, struct sockaddr *pstrAddr, uint8 u8AddrLen);
/** @} */
/** @defgroup ReceiveFn recv
 *    @ingroup SocketAPI
 * 	An asynchrnonous receive function, used to retrieve data from a TCP stream. 
 	Before calling the recv function, a successful socket connection status must have been received through any of the two socket events 
 	[SOCKET_MSG_CONNECT] or [SOCKET_MSG_ACCEPT], from  the socket callback. Hence, indicating that the socket is already connected to a remote
	host. 
	The application receives the required data in response to this asynchronous call through the reception of the event @ref SOCKET_MSG_RECV in the 
	socket callback.

	Recieving the SOCKET_MSG_RECV message in the callback with zero or negative buffer length indicates the following:
	- SOCK_ERR_NO_ERROR     		 : Socket connection  closed
	- SOCK_ERR_CONN_ABORTED 	 : Socket connection aborted
	- SOCK_ERR_TIMEOUT	 		 : Socket receive timed out
	The application code is expected to close the socket through the call to the @ref close function upon the appearance of the above mentioned  errors.
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint16 recv(SOCKET sock, void *pvRecvBuf, uint16 u16BufLen, uint32 u32Timeoutmsec);
	
@param [in]	sock
				Socket ID, must hold a non negative value.
				A negative value will return a socket error @ref SOCK_ERR_INVALID_ARG. Indicating that an invalid argument is passed in.


@param [in]	pvRecvBuf
				Pointer to a buffer that will hold the received data. The buffer is used 
				in the recv callback to deliver the received data to the caller. The buffer must
				be resident in memory (heap or global buffer).

@param [in]	u16BufLen
				The buffer size in bytes.

@param [in]	u32Timeoutmsec
				Timeout for the recv function in milli-seconds. If the value is set to ZERO, the timeout
				will be set to infinite (the recv function waits forever). If the timeout period is
				elapsed with no data received, the socket will get a timeout error.
@pre
	- The socket function must be called to allocate a TCP socket before passing the socket ID to the recv function.
	- The socket in a connected state is expected to receive data through the socket interface.
	
@see
	socket
	connect
	bind
	listen
	recvfrom
	close
	

@return		
	The function returns ZERO for successful operations and a negative value otherwise.
	The possible error values are:
	- [SOCK_ERR_NO_ERROR](@ref SOCK_ERR_NO_ERROR)	
		Indicating that the operation was successful.
		
	- [SOCK_ERR_INVALID_ARG](@ref SOCK_ERR_INVALID_ARG)
		Indicating passing invalid arguments such as negative socket ID or NULL Receive buffer.

	- [SOCK_ERR_BUFFER_FULL](@ref SOCK_ERR_BUFFER_FULL)
		Indicate socket receive failure.
\section Example
   The example demonstrates a code snippet for the calling of the recv function in the socket callback upon notification of the accept or connect events, and the parsing of the 
   received data when the  SOCKET_MSG_RECV event is received.
@code
	
	switch(u8Msg)	{	

	case SOCKET_MSG_ACCEPT:
		{
			tstrSocketAcceptMsg	*pstrAccept = (tstrSocketAcceptMsg*)pvMsg;

			if(pstrAccept->sock >= 0)			{
				recv(pstrAccept->sock,gau8RxBuffer,sizeof(gau8RxBuffer),TEST_RECV_TIMEOUT);
			}
			else			{
				M2M_ERR("accept\n");
			}
		}
		break;


	case SOCKET_MSG_RECV:
		{
			tstrSocketRecvMsg	*pstrRx = (tstrSocketRecvMsg*)pvMsg;
			
			if(pstrRx->s16BufferSize > 0)			{
				recv(sock,gau8RxBuffer,sizeof(gau8RxBuffer),TEST_RECV_TIMEOUT);			
			}
			else			{
				printf("Socket recv Error: %d\n",pstrRx->s16BufferSize);
				close(sock);
			}
		}
		break;
	
	default:
		break;
	}
}
@endcode
*/
NMI_API sint16 recv(SOCKET sock, void *pvRecvBuf, uint16 u16BufLen, uint32 u32Timeoutmsec);
/** @} */
/** @defgroup ReceiveFromSocketFn recvfrom
 *   @ingroup SocketAPI
 * 	Receives data from a UDP Scoket.
*
*	The asynchronous recvfrom function is used to retrieve data from a UDP socket. The socket must already be bound to
*	a local port before a call to the recvfrom function is made (i.e message @ref SOCKET_MSG_BIND is received
*	with successful status in the socket callback).
*
*	Upon calling the recvfrom function with a successful return code, the application is expected to receive a notification
*	in the socket callback whenever a message is received through the @ref SOCKET_MSG_RECVFROM event. 
*
*	Receiving the SOCKET_MSG_RECVFROM message in the callback with zero, indicates that the socket is closed. 
*	Whereby a negative buffer length indicates one of the socket error codes such as socket timeout error @SOCK_ERR_TIMEOUT:
*
*	The recvfrom callback can also be used to show the IP address of the remote host that sent the frame by
*	using the "strRemoteAddr" element in the @ref tstrSocketRecvMsg structure. (refer to the code example)
 */
 /**@{*/
/*!
@fn	\	
	NMI_API sint16 recvfrom(SOCKET sock, void *pvRecvBuf, uint16 u16BufLen, uint32 u32TimeoutSeconds);

@param [in]	sock
				Socket ID, must hold a non negative value.
				A negative value will return a socket error @ref SOCK_ERR_INVALID_ARG. Indicating that an invalid argument is passed in.

@param [in]	pvRecvBuf
				Pointer to a buffer that will hold the received data. The buffer shall be used
				in the recv callback to deliver the received data to the caller. The buffer must
				be resident in memory (heap or global buffer).

@param [in]	u16BufLen
				The buffer size in bytes.

@param [in]	u32TimeoutSeconds
				Timeout for the recv function in milli-seconds. If the value is set to ZERO, the timeout
				will be set to infinite (the recv function waits forever).

@pre
	- The socket function must be called to allocate a TCP socket before passing the socket ID to the recv function.
 	- The socket corresponding to the socket ID must be successfully bound to a local port through the call to a @ref bind function.
	
@see
	socket
	bind
	close

@return		
	The function returns ZERO for successful operations and a negative value otherwise.
	The possible error values are:
	- [SOCK_ERR_NO_ERROR](@ref SOCK_ERR_NO_ERROR)	
		Indicating that the operation was successful.
		
	- [SOCK_ERR_INVALID_ARG](@ref SOCK_ERR_INVALID_ARG)
		Indicating passing invalid arguments such as negative socket ID or NULL Receive buffer.

	- [SOCK_ERR_BUFFER_FULL](@ref SOCK_ERR_BUFFER_FULL)
		Indicate socket receive failure.
\section Example
   The example demonstrates a code snippet for the calling of the recvfrom function in the socket callback upon notification of a successful bind event, and the parsing of the 
   received data when the  SOCKET_MSG_RECVFROM event is received.
@code
	switch(u8Msg)	{	

	case SOCKET_MSG_BIND:
		{
			tstrSocketBindMsg	*pstrBind = (tstrSocketBindMsg*)pvMsg;

			if(pstrBind != NULL)			{
				if(pstrBind->status == 0)				{
					recvfrom(sock, gau8SocketTestBuffer, TEST_BUFFER_SIZE, 0);
				}
				else				{
					M2M_ERR("bind\n");
				}
			}
		}
		break;


	case SOCKET_MSG_RECVFROM:
		{
			tstrSocketRecvMsg	*pstrRx = (tstrSocketRecvMsg*)pvMsg;
			
			if(pstrRx->s16BufferSize > 0)			{
				//get the remote host address and port number
				uint16 u16port = pstrRx->strRemoteAddr.sin_port;
				uint32 strRemoteHostAddr = pstrRx->strRemoteAddr.sin_addr.s_addr;

				printf("Received frame with size = %d.\tHost address=%x, Port number = %d\n\n",pstrRx->s16BufferSize,strRemoteHostAddr, u16port);
				
				ret = recvfrom(sock,gau8SocketTestBuffer,sizeof(gau8SocketTestBuffer),TEST_RECV_TIMEOUT);
			}
			else			{
				printf("Socket recv Error: %d\n",pstrRx->s16BufferSize);
				ret = close(sock);
			}
		}
		break;
	
	default:
		break;
	}
}
@endcode
*/
NMI_API sint16 recvfrom(SOCKET sock, void *pvRecvBuf, uint16 u16BufLen, uint32 u32Timeoutmsec, struct sockaddr *pstrDestAddr, uint8 u8AddrLen);
/** @} */
/** @defgroup SendFn send
 *   @ingroup SocketAPI
*  Asynchronous sending function, used to send data on a TCP/UDP socket.

*  Called by the application code when there is outgoing data available required to be sent on a specific socket handler.
*  The only difference between this function and the similar @ref sendto function, is the type of socket the data is sent on and the parameters passed in.
*  @ref send function is most commonly called for sockets in a connected state.
*  After the data is sent, the socket callback function registered using registerSocketCallback(), is expected to receive an event of type 
*  @ref SOCKET_MSG_SEND holding information containing the number of data bytes sent.
 */
 /**@{*/
/*!
@fn	\		
	NMI_API sint16 send(SOCKET sock, void *pvSendBuffer, uint16 u16SendLength, uint16 u16Flags);

@param [in]	sock
			Socket ID, must hold a non negative value.
			A negative value will return a socket error @ref SOCK_ERR_INVALID_ARG. Indicating that an invalid argument is passed in.
	
@param [in]	pvSendBuffer
	Pointer to a buffer  holding data to be transmitted. 

@param [in]	u16SendLength
	The buffer size in bytes.

@param [in]	u16Flags
	Not used in the current implementation.
	
@pre 
	Sockets must be initialized using socketInit. \n 
	
	For TCP Socket:\n
		Must use a successfully connected Socket (so that the intended recipient address is known ahead of sending the data). 
		Hence this function is expected to be called after a successful socket connect operation(in client case or accept in the
		the server case).\n
		
	For UDP Socket:\n
		UDP sockets most commonly use @ref sendto function, where the destination address is defined. However, in-order to send outgoing data
		using the @ref send function, atleast one successful call must be made to the @ref sendto function apriori the consecutive calls to the @ref send function, 
		to ensure that the destination address is saved in the firmware.
	
@see
	socketInit
	recv
	sendto
	socket
	connect
	accept 
	sendto  
	
@warning
	u16SendLength must not exceed @ref SOCKET_BUFFER_MAX_LENGTH. \n
	Use a valid socket identifier through the aprior call to the @ref socket function.
	Must use a valid buffer pointer.
	Successful completion of a call to send() does not guarantee delivery of the message,
	A negative return value indicates only locally-detected errors
	
	
@return		
	The function shall return @ref SOCK_ERR_NO_ERROR for successful operation and a negative value (indicating the error) otherwise. 
*/
NMI_API sint16 send(SOCKET sock, void *pvSendBuffer, uint16 u16SendLength, uint16 u16Flags);

/** @} */
/** @defgroup SendToSocketFn sendto
 *  @ingroup SocketAPI
*    Asynchronous sending function, used to send data on a UDP socket.
*    Called by the application code when there is data required to be sent on a UDP socket handler.
*    The application code is expected to receive data from a successful bounded socket node. 
*    The only difference between this function and the similar @ref send function, is the type of socket the data is received on. This function works
*    only with UDP sockets.
*    After the data is sent, the socket callback function registered using registerSocketCallback(), is expected to receive an event of type 
*    @ref SOCKET_MSG_SENDTO.
*/
 /**@{*/
/*!
@fn	\
	NMI_API sint16 sendto(SOCKET sock, void *pvSendBuffer, uint16 u16SendLength, uint16 flags, struct sockaddr *pstrDestAddr, uint8 u8AddrLen);

@param [in]	sock
				Socket ID, must hold a non negative value.
				A negative value will return a socket error @ref SOCK_ERR_INVALID_ARG. Indicating that an invalid argument is passed in.

@param [in]	pvSendBuffer
				Pointer to a buffer holding data to be transmitted. 
				A NULL value will return a socket error @ref SOCK_ERR_INVALID_ARG. Indicating that an invalid argument is passed in.

@param [in]	u16SendLength
				The buffer size in bytes. It must not exceed @ref SOCKET_BUFFER_MAX_LENGTH.

@param [in]	flags
				Not used in the current implementation

@param [in]	pstrDestAddr
				The destination address.

@param [in]	u8AddrLen
				Destination address length in bytes. 
				Not used in the current implementation, only included for BSD compatibility.
@pre 
		Sockets must be initialized using socketInit.
				
@see
	socketInit 
	recvfrom
	sendto
	socket
	connect
	accept 
	send  
	
@warning
	u16SendLength must not exceed @ref SOCKET_BUFFER_MAX_LENGTH. \n
	Use a valid socket (returned from socket ).
	A valid buffer pointer must be used (not NULL). \n 
	Successful completion of a call to sendto() does not guarantee delivery of the message,
	A negative return value indicates only locally-detected errors

@return
	The function  returns @ref SOCK_ERR_NO_ERROR for successful operation and a negative value (indicating the error) otherwise. 
*/
NMI_API sint16 sendto(SOCKET sock, void *pvSendBuffer, uint16 u16SendLength, uint16 flags, struct sockaddr *pstrDestAddr, uint8 u8AddrLen);
/** @} */
/** @defgroup CloseSocketFn close
 *  @ingroup SocketAPI
 *  Synchronous close function, releases all the socket assigned resources.
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 close(SOCKET sock);

@param [in]	sock
				Socket ID, must hold a non negative value.
				A negative value will return a socket error @ref SOCK_ERR_INVALID_ARG. Indicating that an invalid argument is passed in.
	
@pre 
		Sockets must be initialized through the call of the socketInit function.
		@ref close is called only for valid socket identifers created through the @ref socket function.
		
@warning
	If @ref close is called while there are still pending messages (sent or received ) they will be discarded.

@see
	socketInit
	socket

@return		
	The function returned @ref SOCK_ERR_NO_ERROR for successful operation and a negative value (indicating the error) otherwise. 
*/
NMI_API sint8 close(SOCKET sock);
NMI_API sint8 shutdown(SOCKET,unsigned char);

/** @} */
/** @defgroup InetAddressFn nmi_inet_addr
*  @ingroup SocketAPI
*  Synchronous  function which returns a BSD socket compliant Internet Protocol (IPv4) socket address.
*  This IPv4 address in the input string parameter could either be specified as a hostname, or as a numeric string representation like n.n.n.n known as the IPv4 dotted-decimal format
*   (i.e. "192.168.10.1").
*  This function is used whenever an ip address needs to be set in the proper format 
*  (i.e. for the @ref tstrM2MIPConfig structure). 
*/
 /**@{*/
/*!
@fn	\
	NMI_API uint32 nmi_inet_addr(char *pcIpAddr);

@param [in]	pcIpAddr
			A null terminated string containing the IP address in IPv4 dotted-decimal address.

@return
	Unsigned 32-bit integer representing the IP address in Network byte order
	(eg. "192.168.10.1" will be expressed as 0x010AA8C0).
		
*/
NMI_API uint32 nmi_inet_addr(const char *pcIpAddr);
/** @} */
/** @defgroup gethostbynameFn gethostbyname
 *  @ingroup SocketAPI
*   Asynchronous DNS resolving function. This function use DNS to resolve a domain name into the corresponding IP address. 
*   A call to this function will cause a DNS request to be sent and the response will be delivered to the DNS callback function registered using registerSocketCallback() 
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 gethostbyname(uint8 * pcHostName);

@param [in]	pcHostName
				NULL terminated string containing the domain name for the remote host.
				Its size must not exceed [HOSTNAME_MAX_SIZE](@ref HOSTNAME_MAX_SIZE).
				
@see
	registerSocketCallback
	
@warning
	Successful completion of a call to gethostbyname() does not guarantee success of the DNS request,
	a negative return value indicates only locally-detected errors
	
@return		
	- [SOCK_ERR_NO_ERROR](@ref SOCK_ERR_NO_ERROR)
	- [SOCK_ERR_INVALID_ARG](@ref SOCK_ERR_INVALID_ARG)
*/
NMI_API sint8 gethostbyname(uint8 * pcHostName);



/** @} */
/** @defgroup sslSetActiveCipherSuitesFn sslSetActiveCipherSuites
 *  @ingroup SocketAPI
 *   Overrides the default active SSL ciphers in the SSL module with a certain combination of ciphers selected by the caller using
 *   a bitmap containing the required ciphers list.
 *   There API is required only if the will not change the default ciphersuites, otherwise, it is not recommended to use.
 */
 /**@{*/
/*!
@fn	\
	NMI_API sint8 sslSetActiveCipherSuites(uint32 u32SslCsBMP);

@param [in]	u32SslCsBMP
<p>A non-zero 32-bit integer bitmap containing the bitwise OR of the desired ciphers to be enabled 
for the SSL module. The ciphersuites are defined in groups as follows:</p>
<ul>
	<li>@ref SSL_ENABLE_ALL_SUITES</li>
	<li>@ref SSL_ENABLE_RSA_SHA_SUITES</li>
	<li>@ref SSL_ENABLE_RSA_SHA256_SUITES</li>
	<li>@ref SSL_ENABLE_DHE_SHA_SUITES</li>
	<li>@ref SSL_ENABLE_DHE_SHA256_SUITES</li>
	<li>@ref SSL_ENABLE_RSA_GCM_SUITES</li>
	<li>@ref SSL_ENABLE_DHE_GCM_SUITES</li>
</ul>
@return		
	Possible return values are [SOCK_ERR_NO_ERROR](@ref SOCK_ERR_NO_ERROR) if case of success 
	or [SOCK_ERR_INVALID_ARG](@ref SOCK_ERR_INVALID_ARG) if the map is zero.
@remarks
The default supported ciphersuites are the combination of all the above groups. The caller can override the default with any desired combination. 
For example, to disable SHA based ciphers the function should be called with this syntax:
\code
	sslSetActiveCipherSuites(SSL_ENABLE_ALL_SUITES & ~(SSL_ENABLE_RSA_SHA_SUITES|SSL_ENABLE_DHE_SHA_SUITES));
\endcode
@note Passing the u32SslCsBMP as zero <strong>will not</strong> change the current active list.
*/
NMI_API sint8 sslSetActiveCipherSuites(uint32 u32SslCsBMP);


/** @} */

/** @defgroup SetSocketOptionFn setsockopt
 *  @ingroup SocketAPI
*The setsockopt() function shall set the option specified by the option_name
*	argument, at the protocol level specified by the level argument, to the value
*	pointed to by the option_value argument for the socke specified by the socket argument.
*
* <p>Possible protcol level values supported are @ref SOL_SOCKET and @ref SOL_SSL_SOCKET. 
* Possible options when the protocol level is @ref SOL_SOCKET :</p>
* <table style="width: 100%">
* 	<tr>
* 		<td style="height: 22px"><strong>@ref SO_SET_UDP_SEND_CALLBACK</strong></td>
* 		<td style="height: 22px">Enable/Disable callback messages for sendto(). 
* 		Since UDP is unreliable by default the user maybe interested (or not) in 
* 		receiving a message of @ref SOCKET_MSG_SENDTO for each call of sendto(). 
* 		Enabled if option value equals @ref TRUE, disabled otherwise.</td>
* 	</tr>
* 	<tr>
* 		<td><strong>@ref IP_ADD_MEMBERSHIP</strong></td>
* 		<td>Valid for UDP sockets. This option is used to receive frames sent to 
* 		a multicast group. option_value shall be a pointer to Unsigned 32-bit 
* 		integer containing the multicast IPv4 address. </td>
* 	</tr>
* 	<tr>
* 		<td><strong>@ref IP_DROP_MEMBERSHIP</strong></td>
* 		<td>Valid for UDP sockets. This option is used to stop receiving frames 
* 		sent to a multicast group. option_value shall be a pointer to Unsigned 
* 		32-bit integer containing the multicast IPv4 address.</td>
* 	</tr>
* </table>
* <p>Possible options when the protcol leve&nbsp; is @ref SOL_SSL_SOCKET</p>
* <table style="width: 100%">
* 	<tr>
* 		<td style="height: 22px"><strong>
* 		@ref SO_SSL_BYPASS_X509_VERIF</strong></td>
* 		<td style="height: 22px">Allow an opened SSL socket to bypass the X509 
* 		certificate verification process. It is highly recommended <strong>NOT</strong> to use 
* 		this socket option in production software applications. The option is 
* 		supported for debugging and testing purposes. The option value should be 
* 		casted to int type and it is handled as a boolean flag.</td>
* 	</tr>
* 	<tr>
* 		<td><strong>@ref SO_SSL_SNI</strong></td>
* 		<td>Set the Server Name Indicator (SNI) for an SSL socket. The SNI is a 
* 		null terminated string containing the server name assocated with the 
* 		connection. It must not exceed the size of @ref HOSTNAME_MAX_SIZE.</td>
* 	</tr>
* 	<tr>
* 		<td><strong>@ref SO_SSL_ENABLE_SESSION_CACHING</strong></td>
* 		<td>This option allow the TLS to cache the session information for fast 
* 		TLS session establishment in future connections using the TLS Protocol 
* 		session resume features.</td>
* 	</tr>
* </table>
 */
 /**@{*/
/*!
@fn	\		
	NMI_API sint8 setsockopt(SOCKET socket, uint8 u8Level, uint8 option_name,
       const void *option_value, uint16 u16OptionLen);

@param [in]	sock
				Socket handler.

@param [in]	level
				protocol level. See description above.

@param [in]	option_name
				option to be set. See description above.

@param [in]	option_value
				pointer to user provided value.

@param [in]	option_len
				 length of the option value in bytes.
@return		
	The function shall return \ref SOCK_ERR_NO_ERROR for successful operation 
	and a negative value (indicating the error) otherwise. 
@sa SOL_SOCKET, SOL_SSL_SOCKET, 
*/
NMI_API sint8 setsockopt(SOCKET socket, uint8 u8Level, uint8 option_name,
       const void *option_value, uint16 u16OptionLen);


/** @} */
/** @defgroup GetSocketOptionsFn getsockopt
 *  @ingroup SocketAPI
 *   Get socket options retrieves
*    This Function isn't implemented yet but this is the form that will be released later.
 */
 /**@{*/
/*!
@fn	\
	sint8 getsockopt(SOCKET sock, uint8 u8Level, uint8 u8OptName, const void *pvOptValue, uint8 * pu8OptLen);

@brief
	

@param [in]	sock
				Socket Identifie.
@param [in] u8Level
				The protocol level of the option.
@param [in] u8OptName
				The u8OptName argument specifies a single option to get.
@param [out] pvOptValue
				The pvOptValue argument contains pointer to a buffer containing the option value.
@param [out] pu8OptLen
				Option value buffer length.
@return
	The function shall return ZERO for successful operation and a negative value otherwise.
*/
NMI_API sint8 getsockopt(SOCKET sock, uint8 u8Level, uint8 u8OptName, const void *pvOptValue, uint8* pu8OptLen);
/** @} */

/**@}*/
/** @defgroup PingFn m2m_ping_req
 *   @ingroup SocketAPI
 *  	The function request to send ping request to the given IP Address.
 */
 /**@{*/
/*!
 * @fn             NMI_API sint8 m2m_ping_req(uint32 u32DstIP, uint8 u8TTL);
 * @param [in]  u32DstIP
 *					Target Destination IP Address for the ping request. It must be represented in Network
 *					byte order.
 *					The function nmi_inet_addr could be used to translate the dotted decimal notation IP
 *					to its Network bytes order integer represntative.
 * 
 * @param [in]	u8TTL
 *					IP TTL value for the ping request. If set to ZERO, the dfault value SHALL be used.
 *
 * @param [in]	fpPingCb
 *					Callback will be called to deliver the ping statistics.
 *
 * @see           nmi_inet_addr       
 * @return        The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
NMI_API sint8 m2m_ping_req(uint32 u32DstIP, uint8 u8TTL, tpfPingCb fpPingCb);
/**@}*/


#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* __SOCKET_H__ */


// spi_flash.h *************************************************************************************************

#ifndef __SPI_FLASH_H__
#define __SPI_FLASH_H__
//#include "common/include/nm_common.h"
//#include "bus_wrapper/include/nm_bus_wrapper.h"
//#include "driver/source/nmbus.h"
//#include "driver/source/nmasic.h"
//#include "spi_flash/include/spi_flash_map.h"

/**
 *	@fn		spi_flash_enable
 *	@brief	Enable spi flash operations
 *	@version	1.0
 */
sint8 spi_flash_enable(uint8 enable);
/** \defgroup SPIFLASHAPI Function
 *   @ingroup SPIFLASH
 */

 /** @defgroup SPiFlashGetFn spi_flash_get_size
 *  @ingroup SPIFLASHAPI
 */
  /**@{*/
/*!
 * @fn             uint32 spi_flash_get_size(void);
 * @brief         Returns with \ref uint32 value which is total flash size\n
 * @note         Returned value in Mb (Mega Bit).
 * @return      SPI flash size in case of success and a ZERO value in case of failure.
 */
uint32 spi_flash_get_size(void);
 /**@}*/

  /** @defgroup SPiFlashRead spi_flash_read
 *  @ingroup SPIFLASHAPI
 */
  /**@{*/
/*!
 * @fn             sint8 spi_flash_read(uint8 *, uint32, uint32);
 * @brief          Read a specified portion of data from SPI Flash.\n
 * @param [out]    pu8Buf
 *                 Pointer to data buffer which will fill in with data in case of successful operation.
 * @param [in]     u32Addr
 *                 Address (Offset) to read from at the SPI flash.
 * @param [in]     u32Sz
 *                 Total size of data to be read in bytes
 * @warning	       
 *                 - Address (offset) plus size of data must not exceed flash size.\n
 *                 - No firmware is required for reading from SPI flash.\n
 *                 - In case of there is a running firmware, it is required to pause your firmware first 
 *                   before any trial to access SPI flash to avoid any racing between host and running firmware on bus using 
 *                   @ref m2m_wifi_download_mode
 * @note           
 *                 - It is blocking function\n
 * @sa             m2m_wifi_download_mode, spi_flash_get_size
 * @return        The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
 */
sint8 spi_flash_read(uint8 *pu8Buf, uint32 u32Addr, uint32 u32Sz);
 /**@}*/

  /** @defgroup SPiFlashWrite spi_flash_write
 *  @ingroup SPIFLASHAPI
 */
  /**@{*/
/*!
 * @fn             sint8 spi_flash_write(uint8 *, uint32, uint32);
 * @brief          Write a specified portion of data to SPI Flash.\n
 * @param [in]     pu8Buf
 *                 Pointer to data buffer which contains the required to be written.
 * @param [in]     u32Offset
 *                 Address (Offset) to write at the SPI flash.
 * @param [in]     u32Sz
 *                 Total number of size of data bytes
 * @note           
 *                 - It is blocking function\n
 *                 - It is user's responsibility to verify that data has been written successfully 
 *                   by reading data again and compare it with the original.
 * @warning	       
 *                 - Address (offset) plus size of data must not exceed flash size.\n
 *                 - No firmware is required for writing to SPI flash.\n
 *                 - In case of there is a running firmware, it is required to pause your firmware first 
 *                   before any trial to access SPI flash to avoid any racing between host and running firmware on bus using 
 *                   @ref m2m_wifi_download_mode.
 *                 - Before writing to any section, it is required to erase it first.
 * @sa             m2m_wifi_download_mode, spi_flash_get_size, spi_flash_erase
 * @return       The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.
 
 */
sint8 spi_flash_write(uint8* pu8Buf, uint32 u32Offset, uint32 u32Sz);
 /**@}*/

  /** @defgroup SPiFlashErase spi_flash_erase
 *  @ingroup SPIFLASHAPI
 */
  /**@{*/
/*!
 * @fn             sint8 spi_flash_erase(uint32, uint32);
 * @brief          Erase a specified portion of SPI Flash.\n
 * @param [in]     u32Offset
 *                 Address (Offset) to erase from the SPI flash.
 * @param [in]     u32Sz
 *                 Size of SPI flash required to be erased.
 * @note         It is blocking function \n  
* @warning	       
*                 - Address (offset) plus size of data must not exceed flash size.\n
*                 - No firmware is required for writing to SPI flash.\n
 *                 - In case of there is a running firmware, it is required to pause your firmware first 
 *                   before any trial to access SPI flash to avoid any racing between host and running firmware on bus using 
 *                   @ref m2m_wifi_download_mode
 *                 - It is blocking function\n
 * @sa             m2m_wifi_download_mode, spi_flash_get_size
 * @return       The function returns @ref M2M_SUCCESS for successful operations  and a negative value otherwise.

 */
sint8 spi_flash_erase(uint32 u32Offset, uint32 u32Sz);
 /**@}*/
#endif	//__SPI_FLASH_H__


// spi_flash_map.h *************************************************************************************************

/**
*  @file		spi_flash_map.h
*  @brief		This module contains spi flash CONTENT
*/
#ifndef __SPI_FLASH_MAP_H__
#define __SPI_FLASH_MAP_H__

#define FLASH_MAP_VER_0		(0)
#define FLAAH_MAP_VER_1		(1)

#define FLASH_MAP_VERSION	FLAAH_MAP_VER_1	

//#define DOWNLOAD_ROLLBACK
//#define OTA_GEN
#define _PROGRAM_POWER_SAVE_

/* =======*=======*=======*=======*=======
 * 	  General Sizes for Flash Memory
 * =======*=======*=======*=======*=======
 */

#define FLASH_START_ADDR					(0UL)
/*!<Starting Address of Flash Memory
 *
 */
#define FLASH_BLOCK_SIZE					(32 * 1024UL)
/*!<Block Size in Flash Memory
 */
#define FLASH_SECTOR_SZ						(4 * 1024UL)
/*!<Sector Size in Flash Memory
 */
#define FLASH_PAGE_SZ						(256)
/*!<Page Size in Flash Memory
 */
#define FLASH_2M_TOTAL_SZ					(256 * 1024UL)
/*!<Total Size of 2M Flash Memory
 */
#define FLASH_4M_TOTAL_SZ					(512 * 1024UL)
/*!<Total Size of 4M Flash Memory
 */
#define FLASH_8M_TOTAL_SZ					(1024 * 1024UL)
/*!<Total Size of 8M Flash Memory
 */

/*
 * Detailed Sizes and locations for Flash Memory:
 *  ____________________ ___________ ___________________________ _______________________________________________
 * | Starting Address	|	Size	|	Location's Name			|	Description						   			|
 * |____________________|___________|___________________________|_______________________________________________|
 * |	  0 K  			|	  4	K	| 	Boot Firmware			|	Firmware to select which version to run		|
 * |	  4	K 			|	  8 K	|	Control Section			|	Structured data used by Boot firmware		|
 * |	 12 K			|     4	K	|	PLL+GAIN :				|	LookUp Table for PLL and Gain calculations	|
 * |	  				|     		|	PLL  Size = 1K			|		PLL				 						|
 * |	  				|     		|	GAIN Size = 3K			|		Gain configuration				 		|
 * |	 16	K			|	  4	K	|	CERTIFICATE				|	X.509 Certificate storage					|
 * |	 20 K			|	  4	K	|	Scratch Section			|	Empty Section								|
 * |	 24	K			|	  4	K	|	Reserved TLS Server		|	Reserved									|
 * |	 28	K			|	  8	K	|	HTTP Files				|	Files used with Provisioning Mode			|
 * |	 36	K			|	  4	K	|	Connection Parameters	|	Parameters for success connection to AP		|
 * |	 40	K			|	236 K 	|	Main Firmware/program	|	Main Firmware to run WiFi Chip				|
 * |	276	K			|	236 K	|	OTA Firmware		    |	OTA firmware								|
 * |    512 K                                                       Total flash size							|
 * |____________________|___________|___________________________|_______________________________________________|
 *
 *
 * *Keys for Comments with each MACRO:
 * 		"L:xxxK" -means-> location 	:xxxK
 *		"S:xxxK" -means-> Size is 	:xxxK
 */

/*
 * Boot Firmware: which used to select which firmware to run
 *
 */
#define M2M_BOOT_FIRMWARE_STARTING_ADDR		(FLASH_START_ADDR)
#define M2M_BOOT_FIRMWARE_FLASH_SZ			(FLASH_SECTOR_SZ)

/*
 * Control Section: which used by Boot firmware
 *
 */
#define M2M_CONTROL_FLASH_OFFSET			(M2M_BOOT_FIRMWARE_STARTING_ADDR + M2M_BOOT_FIRMWARE_FLASH_SZ)
#define M2M_CONTROL_FLASH_BKP_OFFSET		(M2M_CONTROL_FLASH_OFFSET + FLASH_SECTOR_SZ)
#define M2M_CONTROL_FLASH_SEC_SZ			(FLASH_SECTOR_SZ)
#define M2M_CONTROL_FLASH_TOTAL_SZ			(FLASH_SECTOR_SZ * 2)

/*
 * LUT for PLL and TX Gain settings:
 *
 */
#define M2M_PLL_FLASH_OFFSET				(M2M_CONTROL_FLASH_OFFSET + M2M_CONTROL_FLASH_TOTAL_SZ)
#define M2M_PLL_FLASH_SZ					(1024 * 1)
#define M2M_GAIN_FLASH_OFFSET				(M2M_PLL_FLASH_OFFSET + M2M_PLL_FLASH_SZ)
#define M2M_GAIN_FLASH_SZ					(M2M_CONFIG_SECT_TOTAL_SZ - M2M_PLL_FLASH_SZ)
#define M2M_CONFIG_SECT_TOTAL_SZ			(FLASH_SECTOR_SZ)

/*
 * Certificate:
 *
 */
#define M2M_TLS_FLASH_ROOTCERT_CACHE_OFFSET			(M2M_PLL_FLASH_OFFSET + M2M_CONFIG_SECT_TOTAL_SZ)
#define M2M_TLS_FLASH_ROOTCERT_CACHE_SIZE			(FLASH_SECTOR_SZ * 1)

/*
 * Scratch:
 *
 */
#define M2M_TLS_FLASH_SESSION_CACHE_OFFSET		(M2M_TLS_FLASH_ROOTCERT_CACHE_OFFSET + M2M_TLS_FLASH_ROOTCERT_CACHE_SIZE)
#define M2M_TLS_FLASH_SESSION_CACHE_SIZE		(FLASH_SECTOR_SZ * 1)

/*
 * reserved section
 *
 */
#define M2M_RESERVED_FLASH_OFFSET				(M2M_TLS_FLASH_SESSION_CACHE_OFFSET + M2M_TLS_FLASH_SESSION_CACHE_SIZE)
#define M2M_RESERVED_FLASH_SZ					(FLASH_SECTOR_SZ * 1)
/*
 * HTTP Files
 *
 */
#define M2M_HTTP_MEM_FLASH_OFFSET				(M2M_RESERVED_FLASH_OFFSET + M2M_RESERVED_FLASH_SZ)
#define M2M_HTTP_MEM_FLASH_SZ					(FLASH_SECTOR_SZ * 2)
/*
 * Saved Connection Parameters:
 *
 */
#define M2M_CACHED_CONNS_FLASH_OFFSET			(M2M_HTTP_MEM_FLASH_OFFSET + M2M_HTTP_MEM_FLASH_SZ)
#define M2M_CACHED_CONNS_FLASH_SZ				(FLASH_SECTOR_SZ * 1)

/*
 *
 * Common section size
 */

#define M2M_COMMON_DATA_SEC						(M2M_BOOT_FIRMWARE_FLASH_SZ + M2M_CONTROL_FLASH_TOTAL_SZ + M2M_CONFIG_SECT_TOTAL_SZ + \
												M2M_TLS_FLASH_ROOTCERT_CACHE_SIZE + M2M_TLS_FLASH_SESSION_CACHE_SIZE + \
												M2M_HTTP_MEM_FLASH_SZ  + M2M_CACHED_CONNS_FLASH_SZ + M2M_RESERVED_FLASH_SZ)
/*
 *
 * OTA image1 Offset
 */								

#define M2M_OTA_IMAGE1_OFFSET					(M2M_CACHED_CONNS_FLASH_OFFSET + M2M_CACHED_CONNS_FLASH_SZ)
/*
 * Firmware Offset
 *
 */
#if (defined _FIRMWARE_)||(defined OTA_GEN)
#define M2M_FIRMWARE_FLASH_OFFSET			(0UL)
#else
#if (defined DOWNLOAD_ROLLBACK)
#define M2M_FIRMWARE_FLASH_OFFSET			(M2M_OTA_IMAGE2_OFFSET)
#else
#define M2M_FIRMWARE_FLASH_OFFSET			(M2M_OTA_IMAGE1_OFFSET)
#endif
#endif
/*
 *
 * Firmware
 */
#define M2M_FIRMWARE_FLASH_SZ				(236*1024UL)
/**
 *
 * OTA image Size
 */
#define OTA_IMAGE_SIZE						(M2M_FIRMWARE_FLASH_SZ)
/**
 *
 * Flash Total size
 */
#define FLASH_IMAGE1_CONTENT_SZ 			(M2M_COMMON_DATA_SEC  +  OTA_IMAGE_SIZE)
									
/**
 *
 * OTA image 2 offset
 */
#define M2M_OTA_IMAGE2_OFFSET				(FLASH_IMAGE1_CONTENT_SZ)

/*
 * App(Cortus App 4M): App. which runs over firmware
 *
 */
#define M2M_APP_4M_MEM_FLASH_SZ				(FLASH_SECTOR_SZ * 10)
#define M2M_APP_4M_MEM_FLASH_OFFSET			(FLASH_4M_TOTAL_SZ - M2M_APP_4M_MEM_FLASH_SZ)

/* Check if total size of content
 *  don't exceed total size of memory allowed
 **/
#if (M2M_COMMON_DATA_SEC  +  (OTA_IMAGE_SIZE *2)> FLASH_4M_TOTAL_SZ)
#error "Excced 4M Flash Size"
#endif /* (FLASH_CONTENT_SZ > FLASH_TOTAL_SZ) */


#endif /* __SPI_FLASH_MAP_H__ */


// nmbus.h *************************************************************************************************

#ifndef _NMBUS_H_
#define _NMBUS_H_

//#include "common/include/nm_common.h"
//#include "bus_wrapper/include/nm_bus_wrapper.h"


/**
*	@fn		nm_bus_iface_init
*	@brief	Initialize bus interface
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_bus_iface_init(void *);

/**
*	@fn		nm_bus_iface_deinit
*	@brief	Deinitialize bus interface
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_bus_iface_deinit(void);

/**
*	@fn		nm_bus_iface_reconfigure
*	@brief	reconfigure bus interface
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_bus_iface_reconfigure(void *ptr);

/**
*	@fn		nm_read_reg
*	@brief	Read register
*	@param [in]	u32Addr
*				Register address
*	@return	Register value
*/
uint32 nm_read_reg(uint32 u32Addr);

/**
*	@fn		nm_read_reg_with_ret
*	@brief	Read register with error code return
*	@param [in]	u32Addr
*				Register address
*	@param [out]	pu32RetVal
*				Pointer to u32 variable used to return the read value
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_read_reg_with_ret(uint32 u32Addr, uint32* pu32RetVal);

/**
*	@fn		nm_write_reg
*	@brief	write register
*	@param [in]	u32Addr
*				Register address
*	@param [in]	u32Val
*				Value to be written to the register
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_write_reg(uint32 u32Addr, uint32 u32Val);

/**
*	@fn		nm_read_block
*	@brief	Read block of data
*	@param [in]	u32Addr
*				Start address
*	@param [out]	puBuf
*				Pointer to a buffer used to return the read data
*	@param [in]	u32Sz
*				Number of bytes to read. The buffer size must be >= u32Sz
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_read_block(uint32 u32Addr, uint8 *puBuf, uint32 u32Sz);

/**
*	@fn		nm_write_block
*	@brief	Write block of data
*	@param [in]	u32Addr
*				Start address
*	@param [in]	puBuf
*				Pointer to the buffer holding the data to be written
*	@param [in]	u32Sz
*				Number of bytes to write. The buffer size must be >= u32Sz
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_write_block(uint32 u32Addr, uint8 *puBuf, uint32 u32Sz);



#endif /* _NMBUS_H_ */


// nmi2c.h *************************************************************************************************
#ifndef _NMI2C_H_
#define _NMI2C_H_

//#include "common/include/nm_common.h"

/**
*	@fn		nm_i2c_read_reg
*	@brief	Read register
*	@param [in]	u32Addr
*				Register address
*	@return	Register value
*/
uint32 nm_i2c_read_reg(uint32 u32Addr);

/**
*	@fn		nm_i2c_read_reg_with_ret
*	@brief	Read register with error code return
*	@param [in]	u32Addr
*				Register address
*	@param [out]	pu32RetVal
*				Pointer to u32 variable used to return the read value
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_i2c_read_reg_with_ret(uint32 u32Addr, uint32* pu32RetVal);

/**
*	@fn		nm_i2c_write_reg
*	@brief	write register
*	@param [in]	u32Addr
*				Register address
*	@param [in]	u32Val
*				Value to be written to the register
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_i2c_write_reg(uint32 u32Addr, uint32 u32Val);

/**
*	@fn		nm_i2c_read_block
*	@brief	Read block of data
*	@param [in]	u32Addr
*				Start address
*	@param [out]	puBuf
*				Pointer to a buffer used to return the read data
*	@param [in]	u16Sz
*				Number of bytes to read. The buffer size must be >= u16Sz
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_i2c_read_block(uint32 u32Addr, uint8 *puBuf, uint16 u16Sz);

/**
*	@fn		nm_i2c_write_block
*	@brief	Write block of data
*	@param [in]	u32Addr
*				Start address
*	@param [in]	puBuf
*				Pointer to the buffer holding the data to be written
*	@param [in]	u16Sz
*				Number of bytes to write. The buffer size must be >= u16Sz
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_i2c_write_block(uint32 u32Addr, uint8 *puBuf, uint16 u16Sz);

#endif /* _NMI2C_H_ */


// nmuart.h *************************************************************************************************
#ifndef _NMUART_H_
#define _NMUART_H_

//#include "common/include/nm_common.h"

/*
*	@fn			nm_uart_sync_cmd
*	@brief		Check COM Port
*	@return		ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_uart_sync_cmd(void);
/**
*	@fn			nm_uart_read_reg
*	@brief		Read register
*	@param [in]	u32Addr
*				Register address
*	@return		Register value
*/
uint32 nm_uart_read_reg(uint32 u32Addr);

/**
*	@fn			nm_uart_read_reg_with_ret
*	@brief		Read register with error code return
*	@param [in]	u32Addr
*				Register address
*	@param [out]	pu32RetVal
*				Pointer to u32 variable used to return the read value
*	@return		ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_uart_read_reg_with_ret(uint32 u32Addr, uint32* pu32RetVal);

/**
*	@fn			nm_uart_write_reg
*	@brief		write register
*	@param [in]	u32Addr
*				Register address
*	@param [in]	u32Val
*				Value to be written to the register
*	@return		ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_uart_write_reg(uint32 u32Addr, uint32 u32Val);

/**
*	@fn			nm_uart_read_block
*	@brief		Read block of data
*	@param [in]	u32Addr
*				Start address
*	@param [out]	puBuf
*				Pointer to a buffer used to return the read data
*	@param [in]	u16Sz
*				Number of bytes to read. The buffer size must be >= u16Sz
*	@return		ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_uart_read_block(uint32 u32Addr, uint8 *puBuf, uint16 u16Sz);

/**
*	@fn			nm_uart_write_block
*	@brief		Write block of data
*	@param [in]	u32Addr
*				Start address
*	@param [in]	puBuf
*				Pointer to the buffer holding the data to be written
*	@param [in]	u16Sz
*				Number of bytes to write. The buffer size must be >= u16Sz
*	@return		ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_uart_write_block(uint32 u32Addr, uint8 *puBuf, uint16 u16Sz);

/**
*	@fn			nm_uart_reconfigure
*	@brief		Reconfigures the UART interface
*	@param [in]	ptr
*				Pointer to a DWORD containing baudrate at this moment.
*	@return		ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_uart_reconfigure(void *ptr);
#endif /* _NMI2C_H_ */


// nmspi.h *************************************************************************************************

#ifndef _NMSPI_H_
#define _NMSPI_H_

//#include "common/include/nm_common.h"


/**
*	@fn		nm_spi_init
*	@brief	Initialize the SPI
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_spi_init(void);

/**
*	@fn		nm_spi_deinit
*	@brief	DeInitialize the SPI
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_spi_deinit(void);

/**
*	@fn		nm_spi_read_reg
*	@brief	Read register
*	@param [in]	u32Addr
*				Register address
*	@return	Register value
*/
uint32 nm_spi_read_reg(uint32 u32Addr);

/**
*	@fn		nm_spi_read_reg_with_ret
*	@brief	Read register with error code return
*	@param [in]	u32Addr
*				Register address
*	@param [out]	pu32RetVal
*				Pointer to u32 variable used to return the read value
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_spi_read_reg_with_ret(uint32 u32Addr, uint32* pu32RetVal);

/**
*	@fn		nm_spi_write_reg
*	@brief	write register
*	@param [in]	u32Addr
*				Register address
*	@param [in]	u32Val
*				Value to be written to the register
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_spi_write_reg(uint32 u32Addr, uint32 u32Val);

/**
*	@fn		nm_spi_read_block
*	@brief	Read block of data
*	@param [in]	u32Addr
*				Start address
*	@param [out]	puBuf
*				Pointer to a buffer used to return the read data
*	@param [in]	u16Sz
*				Number of bytes to read. The buffer size must be >= u16Sz
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_spi_read_block(uint32 u32Addr, uint8 *puBuf, uint16 u16Sz);

/**
*	@fn		nm_spi_write_block
*	@brief	Write block of data
*	@param [in]	u32Addr
*				Start address
*	@param [in]	puBuf
*				Pointer to the buffer holding the data to be written
*	@param [in]	u16Sz
*				Number of bytes to write. The buffer size must be >= u16Sz
*	@return	ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_spi_write_block(uint32 u32Addr, uint8 *puBuf, uint16 u16Sz);

#ifdef __cplusplus
	 }
#endif

#endif /* _NMSPI_H_ */

