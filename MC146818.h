/* 
 * File:   MC146818.h
 * Author: dario
 *
 * Created on 16 marzo 2022, 13.57
 */

#ifndef MC146818_H
#define	MC146818_H

#ifdef	__cplusplus
extern "C" {
#endif

    
#define _XTAL_FREQ 32000000UL
#define VERNUMH 0
#define VERNUML 1

typedef unsigned char BYTE;
typedef unsigned int WORD;
typedef unsigned long DWORD;
    
typedef struct {
    unsigned char   weekday;    // 
    unsigned char   mday;       // 
    unsigned char   mon;        // 
    unsigned short  year;       // 
	} PIC16_DATE;

typedef struct {
    unsigned char   sec;        // BCD codification for seconds, 00-59
    unsigned char   min;        // BCD codification for minutes, 00-59
    unsigned char   hour;       // BCD codification for hours, 00-24
	} PIC16_TIME;

void UserInit(void);
void Timer_Init(void);
void PWM_Init(void);
void RTCC_Init(void);
void SetTimeFromNow(DWORD,PIC16_DATE *,PIC16_TIME *);
DWORD PIC16RTCCGetDate(void);
DWORD PIC16RTCCGetTime(void);
void PIC16RTCCSetDate(WORD xx_year, WORD month_day);
void PIC16RTCCSetTime(WORD weekDay_hours, WORD minutes_seconds);
void PIC16RTCCSetAlarm(BYTE month, BYTE day, BYTE hours, BYTE minutes, BYTE seconds);

    
/** LED ************************************************************/
#define mInitAllLEDs()      LATA &= 0xFFF7; TRISA &= 0xFFF7; 

#define mLED_1              LATAbits.LATA3     // pin 20
#define mGetLED_1()         mLED_1
#define mLED_1_On()         mLED_1 = 1;
#define mLED_1_Off()        mLED_1 = 0;
#define mLED_1_Toggle()     mLED_1 = !mLED_1;

    
/** SWITCH *********************************************************/
#define mInitSwitch1()      TRISAbits.TRISA10=1;

#define mInitAllSwitches()  mInitSwitch1(); WPUAbits.WPUA2=1;
#define sw1                 PORTAbits.RA2

    
// PORT B=bus dati/indirizzo
#define	SPISCKTris TRISCbits.TRISC3			// SCK
#define	SPISDITris TRISCbits.TRISC4         // SDI
#define	SPISDOTris TRISCbits.TRISC6			// SDO
#define	SPICSTris  TRISDbits.TRISD3         // CS WiFi
#define	WINCResetTris  TRISDbits.TRISD5     // 
#define	WINCWakeTris  TRISDbits.TRISD4     // 
#define	WINCIRQTris  TRISCbits.TRISC7     // pin 1
	
#define	m_SPISCKBit LATCbits.LATC3		// pin 33
#define	m_SPISDIBit PORTCbits.RC4		// pin 38
#define	m_SPISDOBit LATCbits.LATC6		// pin 40
#define	m_SPICSBit  LATDbits.LATD3
#define	m_WINCResetBit  LATDbits.LATD5
#define	m_WINCWakeBit  LATDbits.LATD4
#define SPIOUT      m_SPISDOBit 
#define SPIIN       m_SPISDIBit
#define SPICLOCK    SPISCKTris
#define SPIENABLE   m_SPIWCSBit

#define m_CE PORTEbits.RE0		// pin 32
#define m_AS PORTEbits.RE2   		// pin 25
#define m_DS PORTDbits.RD1  		// pin 35
#define m_RW PORTEbits.RE1         // pin 24
#define m_STBY PORTAbits.RA1         // pin 18
#define m_IRQ PORTDbits.LATD0         // pin 34

#define m_CKFS PORTDbits.RD2         // pin 36
#define m_CKOUT LATAbits.LATA0         // pin 17

#define m_MOT PORTDbits.RD7         // pin 5
#define m_PS  PORTAbits.RA4         // pin 21
#define m_SQW LATCbits.LATC2         // pin 32 PWM

#ifdef	__cplusplus
}
#endif

#endif	/* MC146818_H */

