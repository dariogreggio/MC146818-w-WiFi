/* MC146818 RTC chip emulator
 * featuring optional WiFi interface for NTP
 * 16/3/2022 (die humans die, #gowargo)
 * */


// PIC16F19176 Configuration Bit Settings

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINTPLL// Power-up default value for COSC bits (HFINTOSC with 2x PLL, with OSCFRQ = 16 MHz and CDIV = 1:1 (FOSC = 32 MHz))
#pragma config CLKOUTEN = ON    // Clock Out Enable bit (CLKOUT function is enabled; FOSC/4 clock appears at OSC2)
#pragma config VBATEN = ON      // VBAT Pin Enable bit (VBAT functionality is enabled)
#pragma config LCDPEN = OFF     // LCD Charge Pump Mode bit (LCD Charge Pump is disabled.)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = PWRT_16  // Power-up Timer selection bits (PWRT set at 16 ms)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = ON        // WDT operating mode (WDT enabled regardless of sleep; SWDTEN ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config BBSIZE = 512     // Boot Block Size Selection bits (Boot Block Size (Words) 512)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block NOT write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block NOT write-protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration Words NOT write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM NOT write-protected)
#pragma config WRTSAF = OFF     // Storage Area Flash Write Protection bit (SAF NOT write-protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (UserNVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#include "MC146818.h"
#include "at_winc1500.h"


#define DEFAULT_YEARS               0x0021
#define DEFAULT_MONTH_DAY           0x0830
#define DEFAULT_WEEKDAY_HOURS       0x0121
#define DEFAULT_MINUTES_SECONDS     0x3900

volatile unsigned long now=315532800;   // 1/1/1980 per MSDOS :)
volatile unsigned char second_10;
signed char timeZone;
// Days on each month for regular and leap years
const unsigned char days_month[2][12] = {
  { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
  { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
	};
const char wdays[7][4] = {
  "Mon","Tue","Wed","Thu","Fri","Sat","Sun"
  };
static const char months[13][4] = {
  "   ", "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
  };
int isleap(int year);


int main() {
  
  CLRWDT();

  CFGCONbits.IOLOCK = 0;      // PPS Unlock
  RPF2Rbits.RPF2R = 12;        // Assign RPF2 as OC1, pin 57

  RPD3Rbits.RPD3R = 5;        // Assign RPD3 as SDO1, pin 78
  SDI1Rbits.SDI1R = 0;        // Assign RPD2 as SDI1, pin 77

  RPD11Rbits.RPD11R = 8;        // Assign RPD11 as SDO4, pin 70
  SDI4Rbits.SDI4R = 11;      // Assign RPD15 as SDI4, pin 48

#if 0
  RPB0Rbits.RPB0R = 15;        // RefClk3 su pin 57 (RF2)
	REFO3CONbits.RSLP=1;
	REFO3CONbits.ROSEL=1;
	REFO3CONbits.RODIV=1;        // ok 50MHz 27/7/20
	REFO3CONbits.OE=1;
	REFO3CONbits.ON=1;
	TRISFbits.TRISF2=1;
#endif
  
  SYSKEY = 0x00000000;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
  CFGCONbits.OCACLK=1;      // sceglie timer per PWM [serve SYSLOCK cmq)
  SYSKEY = 0x00000000;
  
  CFGCONbits.IOLOCK = 1;      // PPS Lock

  
  ANSELA=0;
  ANSELB=0;
  ANSELC=0;
  ANSELD=0;
  ANSELE=0;

  LATA=0b01000000;		// 
  LATB=0b00000000;
  LATC=0b00011000;		// 
  LATD=0b00110000;		// 
  LATE=0b00000000;
  
  TRISA=0b10001000;       // 
  TRISB=0b00000000;       // 
  TRISC=0b00000000;
  TRISD=0b00000100;       // 
  TRISE=0b00000000;       // 

	mInitAllLEDs();
  mInitAllSwitches();

//  CNPUDbits.CNPUD9=CNPUDbits.CNPUD10=1;   // [I2C tanto per] ev. gli IRQ?
  WPUA=0b10001000;   // 
  WPUB=0b11111111;   // boh
  WPUC=0b00000000;   // 
  WPUD=0b00000100;   // 
  WPUE=0b11111111;   // altrimenti la PMP fluttua..; SDCD

  
  UserInit();
    
  Timer_Init();
  PWM_Init();

  
  #ifdef USA_WIFI
  //https://microchipsupport.force.com/s/article/How-to-retrieve-the-Connection-Parameters-after-WINC1500-is-connected-to-an-Access-Point-AP

	tstrWifiInitParam param;
	nm_bsp_init();
   
  
/*  nm_bsp_reset();   // per warm reset... o si blocca, tipo https://community.atmel.com/comment/3132631
  __delay_ms(200);//non va cmq 22/12/21
	nm_bus_iface_init(NULL); //idem...
 * provo dunque con cmd reset a audio_card...
  nm_warm_reset(); */
            

//      m_Led0Bit = 1;
	memset((BYTE*)&param, 0, sizeof(param));
	param.pfAppWifiCb = wifi_cb;
	/*initialize the WINC Driver*/
		M2M_INFO("Init WiFi...\n");
	int ret = m2m_wifi_init(&param);
	if(M2M_SUCCESS != ret){
		M2M_ERR("Driver Init Failed <%d>\n",ret);
		while(1) {
//      m_Led0Bit^=1;
      __delay_ms(100);
      }
		}
  m2m_wifi_connect("wlan_greggio",12,M2M_WIFI_SEC_WPA_PSK,"dariog20",M2M_WIFI_CH_ALL /*7*/);
	// Handle the app state machine plus the WINC event handler 
	while(m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
//      m_Led0Bit^=1;
    CLRWDT();
		}
 

//    m2m_wifi_set_sleep_mode(M2M_NO_PS,1);
     // SERVE per broadcast??

  // Initialize the socket layer.
  socketInit();
  

#endif

  while(1) {
    
#ifdef USA_WIFI
  	m2m_wifi_handle_events(NULL);
#endif
    
    CLRWDT();
    }

  }

void UserInit(void) {
  
	RTCC_Init();
  
	PIC16RTCCSetDate( DEFAULT_YEARS, DEFAULT_MONTH_DAY );
	PIC16RTCCSetTime( DEFAULT_WEEKDAY_HOURS, DEFAULT_MINUTES_SECONDS );
  }

void Timer_Init(void) {
  
  T1CON=0;
  T1CONbits.TCS = 0;            // clock from peripheral clock
  T1CONbits.TCKPS = 0b11;      // 1:256 prescaler 
  PR1 = 39062;                 // 10Hz x clock
  
  IPC1bits.T1IP=2;            // set IPL 2, sub-priority 2??
  IPC1bits.T1IS=0;
  IEC0bits.T1IE=1;             // enable Timer 3 interrupt se si vuole
  
  T1CONbits.TON = 1;    // start timer (for clock)
  
  T4CON=0;
  T4CONbits.TCS = 0;            // clock from peripheral clock
  T4CONbits.TCKPS = 0b010;      // 1:4 prescaler 
  T4CONbits.T32 = 0;            // 16bit
  PR4 = 6250;                   // 4KHz x buzzer
  
  T4CONbits.TON = 1;    // start timer (for pwm)
  }
  
void PWM_Init(void) {

// v. init sopra  CFGCONbits.OCACLK=1;      // sceglie timer per PWM ossia Timer6 ;) [serve SYSLOCK cmq)
  
  OC1CON = 0x0006;      // TimerX ossia Timer6; PWM mode no fault; Timer 16bit, TimerX
  OC1R    = 6250/2;		 // su PIC32 è read-only!
  OC1RS   = 6250/2;      // 50%, relativo a PR2 del Timer6
  OC1CONbits.ON=0;    // on

  }

void RTCC_Init(void) {
  
  RTCCON=0;
  
	UnlockRTCC();
  
  RTCCONbits.CAL=0;
  RTCCONbits.RTCCLKSEL=0b01;      // ext xtal
  RTCCONbits.RTCOUTSEL=0b01;      // seconds clock su RTCC pin out
  RTCCONbits.RTCOE=1;      // idem

  RTCALRM=0;
  
  RTCCONbits.ON=1;
  
	LockRTCC();
  }
  

/****************************************************************************
  Function:
    DWORD   PIC16RTCCGetDate( void )

  Description:
    This routine reads the date from the RTCC module.

  Precondition:
    The RTCC module has been initialized.

  Parameters:
    None

  Returns:
    DWORD in the format <xx><YY><MM><DD>

  Remarks:
    To catch roll-over, we do two reads.  If the readings match, we return
    that value.  If the two do not match, we read again until we get two
    matching values.

  ***************************************************************************/
DWORD PIC16RTCCGetDate(void) {
  unsigned int timeCopy1, timeCopy2;
  
  if(RTCCONbits.RTCSYNC == 0)  {
    return RTCDATE; // return time
    }
  else  {
    // read time twice and compare result, retry until a match occurs
    while( (timeCopy1 = RTCDATE) != (timeCopy2 = RTCDATE) )
      CLRWDT();
    return timeCopy1; // return time when both reads matched
    }
  }


/****************************************************************************
  Function:
    DWORD   PIC16RTCCGetTime( void )

  Description:
    This routine reads the time from the RTCC module.

  Precondition:
    The RTCC module has been initialized.

  Parameters:
    None

  Returns:
    DWORD in the format <xx><HH><MM><SS>

  Remarks:
    To catch roll-over, we do two reads.  If the readings match, we return
    that value.  If the two do not match, we read again until we get two
    matching values.

  ***************************************************************************/

DWORD PIC16RTCCGetTime(void) {
  unsigned int timeCopy1, timeCopy2;
  
  if(RTCCONbits.RTCSYNC == 0) {
    return RTCTIME; // return time
    }
  else {
    // read time twice and compare result, retry until a match occurs
    while( (timeCopy1 = RTCTIME) != (timeCopy2 = RTCTIME) )
      CLRWDT();
    return timeCopy1; // return time when both reads matched
    }
	}


/****************************************************************************
  Function:
    void PIC16RTCCSetDate( WORD xx_year, WORD month_day )

  Description:
    This routine sets the RTCC date to the specified value.


  Precondition:
    The RTCC module has been initialized.

  Parameters:
    WORD xx_year    - BCD year in the lower byte
    WORD month_day  - BCD month in the upper byte, BCD day in the lower byte

  Returns:
    None

  Remarks:

  ***************************************************************************/
void PIC16RTCCSetDate(WORD xx_year, WORD month_day) {

	UnlockRTCC();
  RTCDATEbits.YEAR10 = LOBYTE(xx_year) / 10;
  RTCDATEbits.YEAR01 = LOBYTE(xx_year) % 10;
  RTCDATEbits.MONTH10=from_bcd(HIBYTE(month_day)) / 10;
  RTCDATEbits.MONTH01=from_bcd(HIBYTE(month_day)) % 10;
  RTCDATEbits.DAY10=from_bcd(LOBYTE(month_day)) / 10;
  RTCDATEbits.DAY01=from_bcd(LOBYTE(month_day)) % 10;
	LockRTCC();
  
	}


/****************************************************************************
  Function:
    void PIC16RTCCSetTime( WORD weekDay_hours, WORD minutes_seconds )

  Description:
    This routine sets the RTCC time to the specified value.

  Precondition:
    The RTCC module has been initialized.

  Parameters:
    WORD weekDay_hours      - BCD weekday in the upper byte, BCD hours in the
                                lower byte
    WORD minutes_seconds    - BCD minutes in the upper byte, BCD seconds in
                                the lower byte

  Returns:
    None

  Remarks:

  ***************************************************************************/
void PIC16RTCCSetTime(WORD weekDay_hours, WORD minutes_seconds) {

  UnlockRTCC();
  RTCDATEbits.WDAY01=HIBYTE(weekDay_hours);
  RTCTIMEbits.HR10 = from_bcd(LOBYTE(weekDay_hours)) / 10;
  RTCTIMEbits.HR01 = from_bcd(LOBYTE(weekDay_hours)) % 10;
  RTCTIMEbits.MIN10 = from_bcd(HIBYTE(minutes_seconds)) / 10;
  RTCTIMEbits.MIN01 = from_bcd(HIBYTE(minutes_seconds)) % 10;
  RTCTIMEbits.SEC10 = from_bcd(LOBYTE(minutes_seconds)) / 10;
  RTCTIMEbits.SEC01 = from_bcd(LOBYTE(minutes_seconds)) % 10;
	LockRTCC();

	}

void PIC16RTCCSetAlarm(BYTE month, BYTE day, BYTE hours, BYTE minutes, BYTE seconds) {

  UnlockRTCC();
  ALRMDATEbits.MONTH10=from_bcd(month) / 10;
  ALRMDATEbits.MONTH01=from_bcd(month) % 10;
  ALRMDATEbits.DAY10=from_bcd(day) / 10;
  ALRMDATEbits.DAY01=from_bcd(day) % 10;
  ALRMTIMEbits.HR10 = from_bcd(hours) / 10;
  ALRMTIMEbits.HR01 = from_bcd(hours) % 10;
  ALRMTIMEbits.MIN10 = from_bcd(minutes) / 10;
  ALRMTIMEbits.MIN01 = from_bcd(minutes) % 10;
  ALRMTIMEbits.SEC10 = from_bcd(seconds) / 10;
  ALRMTIMEbits.SEC01 = from_bcd(seconds) % 10;
	LockRTCC();

	}

/****************************************************************************
  Function:
    void UnlockRTCC( void )

  Description:
    This function unlocks the RTCC so we can write a value to it.

  Precondition:
    None

  Parameters:
    None

  Return Values:
    None

  Remarks:
    
  ***************************************************************************/

#define RTCC_INTERRUPT_REGISTER IEC5
#define RTCC_INTERRUPT_VALUE    0x0040

void UnlockRTCC(void) {
  BOOL interruptsWereOn;

  interruptsWereOn = FALSE;
  // sta roba arriva da PIC24, non so serve ancora qua...
  if((RTCC_INTERRUPT_REGISTER & RTCC_INTERRUPT_VALUE) == RTCC_INTERRUPT_VALUE) {
    interruptsWereOn = TRUE;
    RTCC_INTERRUPT_REGISTER &= ~RTCC_INTERRUPT_VALUE;
    }
  
  SYSKEY = 0xaa996655; // write first unlock key to SYSKEY
  SYSKEY = 0x556699aa;
  RTCCONbits.RTCWREN=1;      // 
  SYSKEY = 0x00000000;
  
  if(interruptsWereOn) {
    RTCC_INTERRUPT_REGISTER |= RTCC_INTERRUPT_VALUE;
    }
  
	}
void LockRTCC(void) {

  SYSKEY = 0xaa996655; // write first unlock key to SYSKEY
  SYSKEY = 0x556699aa;
  RTCCONbits.RTCWREN=0;      // 
  SYSKEY = 0x00000000;
	}



// -----------------------------------------------------------------------------
#ifdef USA_WIFI
void wifi_cb(BYTE u8MsgType, void *pvMsg) {
  static /*DAVVERO?? ***** */uint8 u8ScanResultIdx=0;
  
  switch(u8MsgType) {
      case M2M_WIFI_REQ_DHCP_CONF:
      {
        m2m_wifi_get_connection_info();
        BYTE *pu8IPAddress = (BYTE *)pvMsg;
//        wifi_connected = 1;
        /* Turn LED0 on to declare that IP address received. */
//        m2m_periph_gpio_set_dir(4, 1);    // get_gpio_idx(
//        m2m_periph_gpio_set_val(4, 0);
//        port_pin_set_output_level(LED_0_PIN, false);
//        printf("m2m_wifi_state: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\n",
//          pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
        /*TODO: add socket initialization here. */
        registerSocketCallback(clientSocketEventHandler, dnsResolveCallback);
        tcpStartServer(80);

				m2m_wifi_get_sytem_time();   // va direttamente in now!

//        setStatusLed(LED_NORMALE_CONNESSO_WIFI);
//        advertise(UDPclientSocket);      // 

      }
        break;
      case M2M_WIFI_RESP_CON_STATE_CHANGED:   // https://www.avrfreaks.net/forum/winc1500-connecting-wifi-disconnects-immediately-error-code-1
      {
        tstrM2mWifiStateChanged *strState=(tstrM2mWifiStateChanged *)pvMsg;
        switch(strState->u8CurrState) {
          case M2M_WIFI_DISCONNECTED:
            m2m_periph_gpio_set_val(M2M_PERIPH_GPIO18,1);
            close(UDPclientSocket);
            UDPclientSocket=INVALID_SOCKET;
            close(TCPlistenSocket);   // in effetti, chissà se serve...
            close(TCPacceptedSocket);
            TCPacceptedSocket=TCPlistenSocket=INVALID_SOCKET;
            bTCPIsfinished = 0;
            break;
          case M2M_WIFI_CONNECTED:
            // messo sopra...
            m2m_periph_gpio_set_val(M2M_PERIPH_GPIO18,0);
            m2m_wifi_req_curr_rssi();
            break;
          case M2M_WIFI_ROAMED:
            break;
          case M2M_WIFI_UNDEF:
            break;
          }

      }
        break;
      case M2M_WIFI_RESP_CONN_INFO:
      {
        tstrM2MConnInfo *pstrConnInfo = (tstrM2MConnInfo *)pvMsg;

        m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO15,1);    // GP15 su pdf (v. cmq setpin ecc)
        m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO16,1);    // GP16 su pdf 
        m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO18,1);    // GP18 su pdf schema FROCI! (opp. 6 merdeee get_gpio_idx) RIFATTO IO
        m2m_periph_gpio_set_val(M2M_PERIPH_GPIO15,1);
        m2m_periph_gpio_set_val(M2M_PERIPH_GPIO16,1);
        m2m_periph_gpio_set_val(M2M_PERIPH_GPIO18,0);

        M2M_INFO("AP Connection Information\r\n*********************************\r\n"); 
        myIp.ip=MAKELONG(MAKEWORD(pstrConnInfo->au8IPAddr[0], pstrConnInfo->au8IPAddr[1]), MAKEWORD(pstrConnInfo->au8IPAddr[2], pstrConnInfo->au8IPAddr[3]));
        M2M_INFO("Local IP Address    : %d.%d.%d.%d\r\n",
          pstrConnInfo->au8IPAddr[0] , pstrConnInfo->au8IPAddr[1], pstrConnInfo->au8IPAddr[2], pstrConnInfo->au8IPAddr[3]);
        M2M_INFO("SSID             : %s\r\n",pstrConnInfo->acSSID);
        M2M_INFO("SEC TYPE         : %d\r\n",pstrConnInfo->u8SecType);
        M2M_INFO("AP MAC Address        : %02x:%02x:%02x:%02x:%02x:%02x\r\n",
          pstrConnInfo->au8MACAddress[0], pstrConnInfo->au8MACAddress[1],pstrConnInfo->au8MACAddress[2],pstrConnInfo->au8MACAddress[3],
          pstrConnInfo->au8MACAddress[4],pstrConnInfo->au8MACAddress[5]);
        M2M_INFO("Signal Strength        : %d\r\n", pstrConnInfo->s8RSSI);
        M2M_INFO("Current Channel        : %d\r\n", pstrConnInfo->u8CurrChannel);   
      }
        break;
    case M2M_WIFI_RESP_GET_SYS_TIME: 
      { 
      tstrSystemTime *mytime = (tstrSystemTime *)pvMsg; 
//      M2M_INFO("wifi_cb:M2M_WIFI_RESP_GET_SYS_TIME \r\n"); 
//      M2M_INFO("My hour %d \r\n",mytime->u8Hour); 
//      M2M_INFO("My min %d \r\n",mytime->u8Minute); 
//      M2M_INFO("My sec %d \r\n",mytime->u8Second); 
      uint16_t y;
      uint16_t m;
      uint16_t d;
      DWORD t;
//https://www.oryx-embedded.com/doc/date__time_8c_source.html
// v. anche DWORD SetNowFromTime(PIC16_DATE date,PIC16_TIME time) {
      
      //Year
      y = mytime->u16Year;
      //Month of year
      m = mytime->u8Month;
      //Day of month
      d = mytime->u8Day;

      //January and February are counted as months 13 and 14 of the previous year
      if(m <= 2) {
        m += 12;
        y -= 1;
        }

      //Convert years to days
      t = (365 * y) + (y / 4) - (y / 100) + (y / 400);
      //Convert months to days
      t += (30 * m) + (3 * (m + 1) / 5) + d;
      //Unix time starts on January 1st, 1970
      t -= 719561;
      //Convert days to seconds
      t *= 86400;
      //Add hours, minutes and seconds
      t += (3600 * mytime->u8Hour) + (60 * mytime->u8Minute) + mytime->u8Second;
      now=t;

      SetRTCC(mytime->u8Day,mytime->u8Month,mytime->u16Year,mytime->u8Hour,mytime->u8Minute,mytime->u8Second);

      } 
      break; 
    case M2M_WIFI_RESP_CURRENT_RSSI:
      myRSSI=*(BYTE *)pvMsg;
      break; 
	  case M2M_WIFI_RESP_SCAN_DONE:
    {
    tstrM2mScanDone *pstrInfo = (tstrM2mScanDone*)pvMsg;
//    printf("Num of AP found %d\n",pstrInfo->u8NumofCh);
    if(pstrInfo->s8ScanState == M2M_SUCCESS) {
      u8ScanResultIdx=0;
      if(pstrInfo->u8NumofCh >= 1) {
        *internetBuffer=0;
        m2m_wifi_req_scan_result(u8ScanResultIdx);
        u8ScanResultIdx++;
        }
      else {
//        printf("No AP Found Rescan\n");
        m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
        }
      }
    else {
//      printf("(ERR) Scan fail with error <%d>\n",pstrInfo->s8ScanState);
      }
      }
    break;
  case M2M_WIFI_RESP_SCAN_RESULT:
    {
    tstrM2mWifiscanResult *pstrScanResult =(tstrM2mWifiscanResult*)pvMsg;
    uint8 u8NumFoundAPs = m2m_wifi_get_num_ap_found();
/*    printf(">>%02d RI %d SEC %s CH %02d BSSID %02X:%02X:%02X:%02X:%02X:%02X SSID %s\n",
    pstrScanResult->u8index,pstrScanResult->s8rssi,
    pstrScanResult->u8AuthType,
    pstrScanResult->u8ch,
    pstrScanResult->au8BSSID[0], pstrScanResult->au8BSSID[1], pstrScanResult->au8BSSID[2],
    pstrScanResult->au8BSSID[3], pstrScanResult->au8BSSID[4], pstrScanResult->au8BSSID[5],
    pstrScanResult->au8SSID);*/
    strcat(internetBuffer,pstrScanResult->au8SSID); strcat(internetBuffer,";");
    if(u8ScanResultIdx < u8NumFoundAPs) {
      // Read the next scan result
      m2m_wifi_req_scan_result(u8ScanResultIdx);
      u8ScanResultIdx++;
      }
    }
      break;
    default:
      {
      }
      break;
    }
  }

/* Socket event handler.
*/

// This is the DNS callback. The response of gethostbyname is here.
// SE SERVE TCP CONNECT, AMPLIARE!
void dnsResolveCallback(BYTE* pu8HostName, DWORD u32ServerIP) {
  struct sockaddr_in strAddr;
  BYTE u8Flags=0;   // boh??
  
  if(u32ServerIP != 0) {
/*    DNSclientSocket = socket(AF_INET,SOCK_STREAM,u8Flags);
    if(DNSclientSocket >= 0) {
      strAddr.sin_family = AF_INET;
      strAddr.sin_port = _htons(80);
      strAddr.sin_addr.s_addr = u32ServerIP;
      connect(DNSclientSocket, (struct sockaddr*)&strAddr, sizeof(struct sockaddr_in));
      M2M_INFO("DNS fatto\r\n");
      }*/
    *(unsigned long *)internetBuffer=u32ServerIP;
    }
  else {
    M2M_INFO("DNS Resolution Failed\n");
    }
  }

 
/* This function needs to be called from main function. For the callbacks to be invoked correctly, the API
  m2m_wifi_handle_events should be called continuously from main. */
void tcpStartServer(uint16 u16ServerPort) {
  struct sockaddr_in strAddr;
  
  // Register socket application callbacks.
//  registerSocketCallback(tcpServerSocketEventHandler, NULL);
  // Create the server listen socket.
  TCPlistenSocket = socket(AF_INET, SOCK_STREAM, 0);
  if(TCPlistenSocket >= 0) {
    strAddr.sin_family = AF_INET;
    strAddr.sin_port = _htons(u16ServerPort);
    strAddr.sin_addr.s_addr = 0; //INADDR_ANY
    bind(TCPlistenSocket, (struct sockaddr*)&strAddr, sizeof(struct sockaddr_in));
    }
  }

const char *HTTP_SERVER_OK="HTTP/1.1 200 OK\r\n";
const char *HTTP_SERVER_ERR="HTTP/1.1 404 Not found\r\n";
const char *HTTP_SERVER_HEADER="Server: MC146818_WiFi/1.0\r\nDate: 20/12/2021 12:00:00\r\nContent-type: %s\r\nContent-length: %u\r\n\r\n";
const char *HTTP_SERVER_HEADER2="Server: MC146818_WiFi/1.0\r\nDate: 20/12/2021 12:00:00\r\nContent-type: %s\r\nTransfer-Encoding: chunked\r\n\r\n";
const char *HTTP_SERVER_HEAD2="<html><head><title>MC146818_WiFi WebServer</title><meta http-equiv='refresh' content=60></head>\r\n";
const char *HTTP_SERVER_HEAD="<html><head><title>MC146818_WiFi WebServer</title></head>\r\n";
// mettere <meta http-equiv="refresh" content="30">
const char *HTTP_SERVER_PAGE="<body bgcolor='#00c0d8'><center><h2><img src='logo.gif'> MC146818_WiFi</h2></center><br><br>\r"
  "<i><a href='/copyright.html'>Versione</a>: %u.%02u; Chip: %08X, RFrev: %x; CPU: PIC16 16KB ROM 2KB RAM</i><br>\n"
  "<a href='/stato.html'>Stato</a><br>\n"
  "<a href='/config.html'>Configurazione</a><br>\n"
  "<a href='/help.html'>Guida</a><br>\n"
  "<br><a href='http://cyberdyne.biz.ly'>Home page Dario's Automation</a><br>\n"
  "</body></html>\r\n";
const char *HTTP_STATO_PAGE="<body bgcolor='#00d0b0'><center><b>Stato di PC_PIC</b></center><br><br>\n"
  "RSSI: %u<br>\n"
  "UnixTime: %u<br>\n"
  "<br><a href='/'>Torna indietro</a><br>\n"
  "</body></html>\r\n";
const char *HTTP_CONFIG_PAGE="<body bgcolor='#a05040' textcolor='#fff0f0'><center><b>Configurazione PC_PIC</b></center><br><br>\n"
	"<form method=post action='config.cgi'\n>"
	"Access point: <input type='text' name='ap' value='%s'><br>\n"
	"Indirizzo IP: <input type='text' name='ip' value='%u.%u.%u.%u'><br>\n"
	"Porta: <input type='text' name='port' value='%u'><br>\n"
	"DHCP: <input type='checkbox' checked name='dhcp' value='%u'><br>\n"      // sistemare... checked
	"<input type=submit value=\"Imposta\"><br>\n"
	"</form>\n"
  "<br><a href='/'>Torna indietro</a><br>\n"
  "</body></html>\r\n";

const unsigned char HTTP_SITEICON[]={
  0x47, 0x49, 0x46, 0x38, 0x39, 0x61, 0x30, 0x00, 0x30, 0x00, 0xF7, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0xFF, 0x00, 0x08, 0x31, 0x00, 0x18, 0x63, 0x00, 0x18, 0xE7, 0x00, 0x21, 0x5A, 0x00, 
  0x21, 0x9C, 0x00, 0x31, 0x4A, 0x00, 0x31, 0xCE, 0x00, 0x4A, 0xB5, 0x00, 0x4A, 0xCE, 0x00, 0x4A, 
  0xDE, 0x00, 0x52, 0xCE, 0x00, 0x5A, 0x8C, 0x00, 0x63, 0x9C, 0x00, 0x63, 0xB5, 0x00, 0x63, 0xCE, 
  0x00, 0x63, 0xE7, 0x00, 0x63, 0xFF, 0x00, 0x73, 0xC6, 0x00, 0x7B, 0xCE, 0x00, 0x7B, 0xE7, 0x00, 
  0x9C, 0xE7, 0x00, 0x9C, 0xFF, 0x00, 0xA5, 0xF7, 0x00, 0xB5, 0xFF, 0x00, 0xBD, 0xF7, 0x00, 0xCE, 
  0xFF, 0x00, 0xE7, 0xFF, 0x00, 0xFF, 0xFF, 0x18, 0xAD, 0xFF, 0x18, 0xE7, 0xE7, 0x21, 0x84, 0xC6, 
  0x21, 0x9C, 0xCE, 0x21, 0xA5, 0xEF, 0x21, 0xAD, 0xFF, 0x21, 0xE7, 0xE7, 0x29, 0x9C, 0xDE, 0x29, 
  0xAD, 0xFF, 0x29, 0xB5, 0xFF, 0x31, 0x63, 0x7B, 0x31, 0x63, 0x84, 0x31, 0x6B, 0x7B, 0x31, 0x6B, 
  0x84, 0x31, 0x84, 0xBD, 0x31, 0x8C, 0xBD, 0x31, 0x9C, 0xDE, 0x31, 0xAD, 0xEF, 0x31, 0xAD, 0xFF, 
  0x31, 0xB5, 0xFF, 0x31, 0xCE, 0xCE, 0x39, 0x63, 0x73, 0x39, 0x63, 0x7B, 0x39, 0x6B, 0x84, 0x39, 
  0x6B, 0x8C, 0x39, 0x73, 0x8C, 0x39, 0x8C, 0xBD, 0x39, 0x8C, 0xC6, 0x39, 0x9C, 0xDE, 0x39, 0xA5, 
  0xDE, 0x39, 0xAD, 0xEF, 0x39, 0xB5, 0xFF, 0x39, 0xBD, 0xFF, 0x42, 0x6B, 0x7B, 0x42, 0x6B, 0x94, 
  0x42, 0x73, 0x94, 0x42, 0x84, 0xA5, 0x42, 0x8C, 0xB5, 0x42, 0x8C, 0xBD, 0x42, 0x94, 0xBD, 0x42, 
  0x94, 0xC6, 0x42, 0xA5, 0xDE, 0x42, 0xAD, 0xEF, 0x42, 0xB5, 0xF7, 0x42, 0xB5, 0xFF, 0x42, 0xBD, 
  0xFF, 0x42, 0xC6, 0xC6, 0x4A, 0x63, 0x6B, 0x4A, 0x9C, 0xC6, 0x4A, 0xA5, 0xDE, 0x4A, 0xAD, 0xE7, 
  0x4A, 0xB5, 0xB5, 0x4A, 0xB5, 0xEF, 0x4A, 0xB5, 0xF7, 0x4A, 0xB5, 0xFF, 0x4A, 0xBD, 0xFF, 0x52, 
  0xAD, 0xAD, 0x52, 0xAD, 0xE7, 0x52, 0xB5, 0xEF, 0x52, 0xBD, 0xF7, 0x52, 0xBD, 0xFF, 0x5A, 0x63, 
  0x63, 0x5A, 0x6B, 0x73, 0x5A, 0x6B, 0x7B, 0x5A, 0xAD, 0xAD, 0x5A, 0xBD, 0xFF, 0x5A, 0xC6, 0xFF, 
  0x63, 0x6B, 0x73, 0x63, 0x73, 0x7B, 0x63, 0x73, 0x84, 0x63, 0x73, 0x8C, 0x63, 0x7B, 0x84, 0x63, 
  0x9C, 0x9C, 0x6B, 0x73, 0x7B, 0x6B, 0x73, 0x84, 0x73, 0x73, 0x7B, 0x73, 0x7B, 0x7B, 0x73, 0x7B, 
  0x84, 0x7B, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x8C, 0x8C, 0x73, 0x73, 0x8C, 0x94, 0x94, 
  0x8C, 0x94, 0x9C, 0x94, 0x94, 0x9C, 0x94, 0x9C, 0x9C, 0x94, 0x9C, 0xA5, 0x94, 0xA5, 0xA5, 0x9C, 
  0x63, 0x63, 0x9C, 0x9C, 0x9C, 0x9C, 0x9C, 0xA5, 0x9C, 0xA5, 0xA5, 0x9C, 0xA5, 0xAD, 0xA5, 0xA5, 
  0xA5, 0xA5, 0xA5, 0xAD, 0xA5, 0xAD, 0xAD, 0xA5, 0xAD, 0xB5, 0xAD, 0xAD, 0xAD, 0xAD, 0xAD, 0xB5, 
  0xAD, 0xB5, 0xB5, 0xAD, 0xBD, 0xBD, 0xB5, 0xB5, 0xB5, 0xB5, 0xB5, 0xBD, 0xB5, 0xBD, 0xBD, 0xB5, 
  0xBD, 0xC6, 0xBD, 0xBD, 0xBD, 0xBD, 0xBD, 0xC6, 0xBD, 0xC6, 0xC6, 0xBD, 0xC6, 0xCE, 0xC6, 0xC6, 
  0xC6, 0xC6, 0xC6, 0xCE, 0xC6, 0xCE, 0xCE, 0xC6, 0xCE, 0xD6, 0xC6, 0xCE, 0xDE, 0xC6, 0xD6, 0xD6, 
  0xC6, 0xD6, 0xDE, 0xCE, 0xCE, 0xCE, 0xCE, 0xCE, 0xD6, 0xCE, 0xD6, 0xD6, 0xCE, 0xD6, 0xDE, 0xCE, 
  0xDE, 0xDE, 0xD6, 0xD6, 0xD6, 0xD6, 0xD6, 0xDE, 0xD6, 0xDE, 0xDE, 0xD6, 0xDE, 0xE7, 0xDE, 0xDE, 
  0xDE, 0xDE, 0xDE, 0xE7, 0xDE, 0xE7, 0xE7, 0xDE, 0xE7, 0xEF, 0xE7, 0xE7, 0xE7, 0xE7, 0xE7, 0xEF, 
  0xE7, 0xEF, 0xEF, 0xEF, 0xEF, 0xEF, 0xEF, 0xEF, 0xF7, 0xEF, 0xF7, 0xF7, 0xF7, 0xF7, 0xF7, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x21, 0xF9, 0x04, 
  0x01, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x30, 0x00, 0x00, 0x08, 
  0xFE, 0x00, 0x01, 0x08, 0x1C, 0x48, 0xB0, 0xA0, 0xC1, 0x83, 0x08, 0x13, 0x2A, 0x5C, 0x38, 0x30, 
  0xD0, 0x1C, 0x47, 0x9F, 0x18, 0x4A, 0x9C, 0x58, 0x10, 0xCE, 0x90, 0x20, 0x65, 0xE8, 0x48, 0x8A, 
  0x48, 0xB1, 0x63, 0x42, 0x38, 0x50, 0xA6, 0x60, 0x71, 0x12, 0x64, 0xCC, 0x1C, 0x49, 0xA0, 0x3C, 
  0xAA, 0x1C, 0x08, 0xE7, 0x4A, 0x96, 0x2C, 0x5A, 0xB2, 0x4C, 0x71, 0x72, 0x23, 0xE3, 0xC6, 0x95, 
  0x1D, 0x5B, 0x6A, 0xF9, 0xA2, 0xA5, 0x67, 0x16, 0x2A, 0x52, 0x8C, 0x94, 0xD4, 0xC8, 0x11, 0xA7, 
  0xC2, 0x96, 0x58, 0xC0, 0xF4, 0x8C, 0x49, 0xA5, 0x29, 0x15, 0x24, 0x46, 0x6A, 0x9E, 0x4C, 0x69, 
  0xF4, 0x60, 0xCB, 0x2C, 0x3C, 0x7D, 0xFE, 0xA4, 0xE2, 0x83, 0x4A, 0x92, 0x24, 0x3C, 0x72, 0xD4, 
  0x24, 0x5A, 0x95, 0xA0, 0xCE, 0x9D, 0x3E, 0x9D, 0x72, 0xFD, 0xDA, 0xA3, 0xED, 0x8B, 0x16, 0x37, 
  0xC8, 0xE4, 0xA1, 0x54, 0x14, 0xE7, 0xD9, 0xA5, 0x2F, 0x9D, 0x2A, 0xF9, 0x9A, 0xA4, 0x07, 0x8C, 
  0x1E, 0x26, 0x60, 0x88, 0x00, 0x51, 0x03, 0x8D, 0x1F, 0x4D, 0x46, 0xEF, 0xC6, 0xFC, 0x59, 0x65, 
  0xAB, 0x8F, 0xBD, 0x6D, 0x4D, 0x98, 0x10, 0x31, 0xD9, 0x44, 0x88, 0x34, 0x94, 0x12, 0x5F, 0x59, 
  0xBA, 0x78, 0x6B, 0x53, 0xBE, 0x80, 0x25, 0x8F, 0x18, 0x21, 0xF9, 0x72, 0x66, 0xBB, 0x50, 0xB0, 
  0x68, 0xCD, 0xEB, 0xD4, 0x47, 0x12, 0xC9, 0xB0, 0x63, 0x97, 0x38, 0x73, 0x7A, 0x25, 0xC8, 0xCE, 
  0x79, 0x1D, 0x87, 0x8E, 0x0D, 0x7B, 0xC4, 0xEC, 0xDA, 0x2A, 0x41, 0xBE, 0x64, 0xFD, 0x73, 0x89, 
  0x0F, 0x18, 0xBC, 0x63, 0x8F, 0xE8, 0xF1, 0x5B, 0xF3, 0xF0, 0xE7, 0xAF, 0x93, 0xF3, 0x06, 0xAC, 
  0x83, 0x76, 0xE2, 0x90, 0x2F, 0x95, 0x64, 0xE9, 0xCB, 0x1B, 0x06, 0x8C, 0x17, 0x80, 0xC3, 0xFE, 
  0x9B, 0x78, 0xE1, 0xC2, 0xBA, 0x5D, 0x21, 0x58, 0xB6, 0xEE, 0x16, 0xCF, 0xDE, 0x84, 0xDF, 0xB6, 
  0xD5, 0x81, 0x7B, 0x04, 0x35, 0x48, 0xCC, 0x0F, 0x1C, 0x2F, 0xBC, 0xBB, 0xDF, 0x1F, 0xFE, 0x6F, 
  0x5B, 0x18, 0x7D, 0xF5, 0x10, 0x5F, 0x59, 0x00, 0x80, 0xF2, 0x87, 0x18, 0x28, 0xB0, 0xC0, 0x03, 
  0x78, 0x7F, 0x01, 0x08, 0x60, 0x5B, 0x10, 0x02, 0xB8, 0x83, 0x79, 0x04, 0x16, 0x58, 0x5F, 0x0D, 
  0x0A, 0xFE, 0x17, 0x60, 0x5B, 0xA0, 0x4D, 0x28, 0x5F, 0x85, 0x06, 0x8A, 0x41, 0x03, 0x0E, 0x3C, 
  0xB0, 0xD5, 0xD7, 0x89, 0x49, 0xE8, 0x80, 0xC6, 0x87, 0x15, 0x0A, 0x64, 0x20, 0x17, 0x23, 0x22, 
  0xB1, 0x17, 0x5F, 0x3E, 0x1C, 0xB1, 0x62, 0x8B, 0x04, 0x7D, 0xA2, 0x89, 0x24, 0x91, 0x44, 0x22, 
  0xC9, 0x21, 0x6E, 0x34, 0xA1, 0x02, 0x11, 0x49, 0x28, 0xD1, 0xD4, 0x11, 0x14, 0xAA, 0x44, 0x4A, 
  0x41, 0xA4, 0x50, 0x22, 0x09, 0x24, 0x8E, 0x44, 0xE9, 0x08, 0x25, 0x94, 0x14, 0x22, 0x08, 0x26, 
  0x74, 0x84, 0x51, 0xC3, 0x10, 0x52, 0x3C, 0x91, 0xE4, 0x44, 0x9F, 0x80, 0x42, 0x49, 0x22, 0x9C, 
  0x70, 0xC2, 0xD1, 0x1E, 0x6C, 0xF8, 0x21, 0x65, 0x94, 0x91, 0x50, 0xE2, 0x48, 0x21, 0x50, 0x56, 
  0x22, 0x49, 0x1F, 0x5D, 0xCC, 0x80, 0x19, 0x45, 0xA0, 0x68, 0xD2, 0x09, 0x23, 0x89, 0x20, 0x72, 
  0x48, 0x22, 0x92, 0x90, 0x69, 0x26, 0x27, 0x7D, 0x9C, 0x91, 0x66, 0x94, 0x8C, 0x44, 0xB2, 0xA6, 
  0x22, 0x89, 0x20, 0xD6, 0x89, 0x24, 0x12, 0x3D, 0x9A, 0x09, 0x25, 0x88, 0x48, 0xC2, 0x08, 0x22, 
  0x98, 0x26, 0xD2, 0x27, 0x22, 0x8C, 0x30, 0xC2, 0x49, 0x26, 0x9F, 0x70, 0x92, 0x87, 0x1A, 0x87, 
  0x4A, 0xA9, 0xC7, 0x19, 0x5B, 0x74, 0xE2, 0x11, 0x22, 0x7A, 0xE4, 0xC1, 0xA7, 0x24, 0xFE, 0x95, 
  0x46, 0x82, 0x29, 0xA6, 0x97, 0x16, 0x22, 0x09, 0x95, 0x83, 0xE6, 0xB1, 0x86, 0x1A, 0x61, 0x6C, 
  0x21, 0x07, 0x8B, 0x13, 0x91, 0x42, 0xC8, 0x1E, 0x7A, 0x38, 0xC2, 0x23, 0x25, 0x9C, 0x70, 0xEA, 
  0xE7, 0xAC, 0x85, 0x24, 0x82, 0x09, 0x25, 0x9A, 0xE8, 0x48, 0x55, 0x85, 0xA2, 0x14, 0x42, 0x6C, 
  0xA2, 0x54, 0x82, 0x82, 0x48, 0x21, 0x97, 0x0E, 0xA2, 0x29, 0x21, 0x88, 0xEC, 0x48, 0xC9, 0xB4, 
  0x2D, 0x56, 0x4B, 0xC7, 0x1F, 0x8C, 0x48, 0xD2, 0xEC, 0xA6, 0xB0, 0x52, 0x72, 0x69, 0x9F, 0x89, 
  0xE8, 0x81, 0x23, 0x41, 0xA0, 0x14, 0x42, 0x87, 0xAB, 0x55, 0x26, 0xC2, 0x88, 0x26, 0x87, 0x6C, 
  0x3B, 0x07, 0x1C, 0x80, 0xD4, 0x35, 0xAF, 0x8B, 0x81, 0xC8, 0xB1, 0x47, 0xBA, 0x84, 0xE4, 0x31, 
  0x07, 0x20, 0xE4, 0x1E, 0x24, 0xC0, 0x00, 0x00, 0x14, 0x30, 0xC0, 0xC4, 0x14, 0x43, 0xEC, 0xD1, 
  0x27, 0x05, 0xFF, 0x21, 0xB0, 0x42, 0x01, 0x00, 0x40, 0x40, 0x00, 0x20, 0x87, 0x1C, 0xF2, 0xC0, 
  0x07, 0x11, 0xE0, 0x71, 0x07, 0x28, 0xA7, 0x9C, 0xF2, 0xC7, 0x26, 0x93, 0x3C, 0x90, 0xC9, 0x04, 
  0xA8, 0x2C, 0x33, 0xCA, 0x04, 0xB4, 0x8C, 0x63, 0xCD, 0x1F, 0x9F, 0xDC, 0x81, 0x1D, 0x32, 0xF0, 
  0x3C, 0x73, 0x07, 0x36, 0x0F, 0x0C, 0xF3, 0x07, 0x1F, 0xD8, 0x11, 0x05, 0xCA, 0x3C, 0xFB, 0x4C, 
  0xB3, 0xCB, 0x26, 0x07, 0xD0, 0x41, 0xD1, 0x32, 0x3C, 0x5D, 0xF4, 0xD1, 0x2A, 0x07, 0xDD, 0x22, 
  0xCC, 0x4F, 0x2B, 0x0D, 0xB5, 0xCC, 0x01, 0xE0, 0x6C, 0x31, 0x81, 0x0B, 0x9C, 0xBC, 0x75, 0xD6, 
  0x51, 0xAB, 0xDC, 0xB1, 0xD0, 0x62, 0x6B, 0xAD, 0x34, 0xCA, 0x67, 0x57, 0x88, 0x33, 0xD6, 0x64, 
  0xA3, 0x4C, 0xB4, 0xD9, 0x4C, 0xEB, 0x3C, 0xF7, 0xCC, 0x20, 0xBB, 0x1C, 0x76, 0x10, 0xCC, 0x3F, 
  0x77, 0x30, 0xB2, 0xCB, 0x1E, 0x7B, 0x2C, 0xF2, 0xE0, 0x80, 0x0B, 0x14, 0x10, 0x00, 0x00, 0x3B
  };

int sendChunk(SOCKET s,const char *str) {
  int i;
  char buf[16];
  WORD tOut;
  
  i= str ? strlen(str) : 0;
  sprintf(buf,"%x\r\n",i);
  sendEx(s, buf, strlen(buf));

  if(str) {  
    sendEx(s, (char *)str, strlen(str));
    }

  strcpy(buf,"\r\n");
  sendEx(s, buf, 2);

// https://www.avrfreaks.net/forum/winc1500-send-more-1-mtu-1400-bytes
//	PROVARE ASPETTARE internetBufferLen...

  return i;
  }

sint16 sendEx(SOCKET sock, void *pvSendBuffer, uint16 u16SendLength) {
  uint16 n,n2;
//  WORD tOut;
  int i;
//  DWORD u32EnableCallbacks=1;

// 	setsockopt(sock, SOL_SOCKET, SO_SET_UDP_SEND_CALLBACK, &u32EnableCallbacks, 4);
#if 0     // non va.. è molto lento (le callback arrivano male, pare un bug) e spesso con -1... 
  do {
    n=min(u16SendLength,SOCKET_BUFFER_MAX_LENGTH);
    if(send(sock, pvSendBuffer, n, 0) == SOCK_ERR_NO_ERROR) {
      tOut=0;
      internetBufferLen=0;
      while(!internetBufferLen && tOut<1000) {
        while(m2m_wifi_handle_events(NULL) != M2M_SUCCESS)
          CLRWDT();
        tOut++;
        __delay_ms(1);
        }
      n=internetBufferLen;
      }
    else
      break;
    pvSendBuffer+=n;
    u16SendLength-=n;
    } while(u16SendLength);
#endif

  n2=0;
  do {
    n=min(u16SendLength,SOCKET_BUFFER_MAX_LENGTH);
rifo:
    if((i=send(sock, pvSendBuffer, n, 0)) == SOCK_ERR_NO_ERROR) {
      // alle volte arriva SOCK_ERR_INVALID -9 in callback... checcazz'è?? parrebbe invalid ovvero magari socket chiuso... ma cmq non uso + la callback
      while(m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
        CLRWDT();
        }
      }
    else {
//      M2M_INFO("sendex: %d\r\n",i);
      while(m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
        CLRWDT();
        }
      switch(i) {
        case SOCK_ERR_BUFFER_FULL:
          goto rifo;
          break;
//        case -3:
        // arriva -3=SOCK_ERR_MAX_TCP_SOCK che non ha senso... di tanto, v. di là
//          goto rifo;
//          break;
        case SOCK_ERR_INVALID_ARG:    // per socket chiuso... ma il close non arriva cmq se interrompo un download lungo (wireshark dice di sì)
          break;
        default:
          goto fine;
          break;
        }
      }
    pvSendBuffer+=n;
    n2+=n;
    u16SendLength-=n;
    } while(u16SendLength);
    
fine:

  return n2;
  }

/* Socket event handler */
void clientSocketEventHandler(SOCKET sock, BYTE u8Msg, void *pvMsg) {
  int ret;
  char *p;
//  BYTE buff[2048];
  
  if(sock==UDPclientSocket)
  switch(u8Msg) {
    case SOCKET_MSG_BIND:
		{
			tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg *)pvMsg;
			if(pstrBind && pstrBind->status == 0) {
				// Prepare next buffer reception. 
//				recv(sock, tempBuff, 1536, 0);
        } 
			else {
//				M2M_INFO("socket_cb: bind error!\r\n");
        }
		} 
    case SOCKET_MSG_RECV:
    case SOCKET_MSG_RECVFROM:
    {
      tstrSocketRecvMsg *pstrRecvMsg = (tstrSocketRecvMsg*)pvMsg;

      WORD nRead;

//      m_Led1Bit ^= 1;		// CHECK 
      // qua arrivano circa ogni 10mS... su wireshark/PC/wired dice 500uS...
      

 			if(pstrRecvMsg->s16BufferSize > 0) {
				//get the remote host address and port number

          
//				ret = recvfrom(sock,tempBuff,1024+VBAN_HEADER_SIZE /*1536*/,0 /*TEST_RECV_TIMEOUT*/, NULL, 0);
        // con 1536 il datagram + corto, l'ultimo del pacchetto, arriva in ritardo... pare andare in timeout: bisognerebbe sapere quanto manca...
        // con 1024+header sembra meglio
    		}
      else {
        /* Prepare next buffer reception. */
//        recvfrom(sock, tempBuff, 1024 /*1536*/, 0, NULL, 0);
        }
/*			else			{
//				M2M_INFO("Socket recv Error: %d\n",pstrRx->s16BufferSize);
				ret = close(sock);
      	}*/
invalid_packet:
	;


    }
      break;
    case SOCKET_MSG_SEND:
    case SOCKET_MSG_SENDTO:
      break;
    case SOCKET_MSG_CLOSE: 
      ret = close(sock);
      UDPclientSocket=INVALID_SOCKET;
      break;
    default:
      break;
    }
  else if(sock==TCPlistenSocket)
  switch(u8Msg) {
    case SOCKET_MSG_BIND:
      {
      tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg*)pvMsg;
      if(pstrBind->status == 0) {
        listen(TCPlistenSocket, 0);
        }
      else {
        M2M_INFO("Bind Failed\n");
        }
      }
      break;
    case SOCKET_MSG_LISTEN:
      {
      tstrSocketListenMsg *pstrListen = (tstrSocketListenMsg*)pvMsg;
      if(pstrListen->status != 0) {
        M2M_INFO("listen Failed\n");
        }
      }
      break;
    case SOCKET_MSG_ACCEPT:
      {
      // New Socket is accepted.
      tstrSocketAcceptMsg *pstrAccept = (tstrSocketAcceptMsg *)pvMsg;
      if(pstrAccept->sock >= 0) {
        // Get the accepted socket.
        if(TCPacceptedSocket==INVALID_SOCKET)
          ;

        TCPacceptedSocket = pstrAccept->sock;
        recv(TCPacceptedSocket, rxBuffer, sizeof(rxBuffer), 0);
        }
      else {
        M2M_INFO("Accept Failed\n");
        }
      }
      break;
    case SOCKET_MSG_RECV:

      break;
    case SOCKET_MSG_SEND:
    {
      int sentBytes = *((sint16*)pvMsg);
//      internetBufferLen=sentBytes;
    }
    case SOCKET_MSG_CLOSE:
      bTCPIsfinished=TRUE;
      close(TCPlistenSocket);   // in effetti, chissà se serve...
      close(TCPacceptedSocket);
      TCPacceptedSocket=TCPlistenSocket=INVALID_SOCKET;
      break;
    default:
      break;
    }
  else if(sock==TCPacceptedSocket)
  switch(u8Msg) {
    case SOCKET_MSG_RECV:
      {
      tstrSocketRecvMsg *pstrRecvMsg = (tstrSocketRecvMsg*)pvMsg;
      if((pstrRecvMsg->pu8Buffer) /*&& (pstrRecvMsg->s16BufferSize > 0)*/) {
        // Process the received message
        // Perform data exchange
        BYTE acHeadBuffer[256],acSendBuffer[768];
        // Fill in the acSendBuffer with some data here
        // Send some data.
        
        if(!strncmp(pstrRecvMsg->pu8Buffer,"GET",3)) {    // stricmp non c'è... diocheffroci 
          //m2m_stricmp
          if(!strncmp(pstrRecvMsg->pu8Buffer+4,"/stato.html",11)) {
            sendEx(TCPacceptedSocket, (char *)HTTP_SERVER_OK, strlen(HTTP_SERVER_OK));
            sprintf(acSendBuffer,HTTP_STATO_PAGE,
                    myRSSI,
										now, (((unsigned int)&ret) - ((unsigned int)&_splim)) / 1024, extRAMtot/1024 );
  //          printf("Chip ID : \r\t\t\t%x\r\n", (unsigned int)nmi_get_chipid());
  //	printf("RF Revision ID : \r\t\t\t%x\r\n", (unsigned int)nmi_get_rfrevid());
            goto send_refreshed_page;
            }
          else if(!strncmp(pstrRecvMsg->pu8Buffer+4,"/config.html",12)) {
            sendEx(sock, (char *)HTTP_SERVER_OK, strlen(HTTP_SERVER_OK));
            sprintf(acSendBuffer,HTTP_CONFIG_PAGE,"wlan_greggio",   // finire pstrConnInfo->acSSID
                    myIp.addr[0],myIp.addr[1],myIp.addr[2],myIp.addr[3],
                    80,
                    1 /*dhcp*/);
send_standard_page:
            sprintf(acHeadBuffer,HTTP_SERVER_HEADER,"text/html",strlen(acSendBuffer)+strlen(HTTP_SERVER_HEAD));
            sendEx(TCPacceptedSocket, acHeadBuffer, strlen(acHeadBuffer));
            sendEx(TCPacceptedSocket, (char *)HTTP_SERVER_HEAD, strlen(HTTP_SERVER_HEAD));
            goto send_standard_body;
            /*	tstrM2MIPConfig conf;

    if (!_init) {
      init();
    }

    conf.u32DNS = (DWORD)dns_server;
    conf.u32Gateway = (DWORD)gateway;
    conf.u32StaticIP = (DWORD)local_ip;
    conf.u32SubnetMask = (DWORD)subnet;
    _dhcp = 0;
    m2m_wifi_enable_dhcp(0); // disable DHCP
    m2m_wifi_set_static_ip(&conf);
    _localip = conf.u32StaticIP;
    _submask = conf.u32SubnetMask;
    _gateway = conf.u32Gateway;*/
            }
          else if(!strncmp(pstrRecvMsg->pu8Buffer+4,"/command.html",13)) {
            sendEx(sock, (char *)HTTP_SERVER_OK, strlen(HTTP_SERVER_OK));
            sprintf(acSendBuffer,HTTP_CMD_PAGE);
            goto send_standard_page;
            }
          else if(!strncmp(pstrRecvMsg->pu8Buffer+4,"/logo.gif",9)) {
            p=(char *)HTTP_SITEICON; ret=sizeof(HTTP_SITEICON);
send_gif:
            sendEx(sock, (char *)HTTP_SERVER_OK, strlen(HTTP_SERVER_OK));
            sprintf(acHeadBuffer,HTTP_SERVER_HEADER,"image/gif",ret);
            sendEx(sock, acHeadBuffer, strlen(acHeadBuffer));
            sendEx(sock, p, ret);
            }
          else if(!strncmp(pstrRecvMsg->pu8Buffer+4,"/index.html",11)) {
            goto index_page;
            }
          else if(pstrRecvMsg->pu8Buffer[4]=='/' && pstrRecvMsg->pu8Buffer[5]==' ') {
index_page:
            sendEx(TCPacceptedSocket, (char *)HTTP_SERVER_OK, strlen(HTTP_SERVER_OK));
            sprintf(acSendBuffer,HTTP_SERVER_PAGE,
                    VERNUMH,VERNUML,
                    nmi_get_chipid(),nmi_get_rfrevid());
send_refreshed_page:
            sprintf(acHeadBuffer,HTTP_SERVER_HEADER,"text/html",strlen(acSendBuffer)+strlen(HTTP_SERVER_HEAD2));
            sendEx(TCPacceptedSocket, acHeadBuffer, strlen(acHeadBuffer));
            sendEx(TCPacceptedSocket, (char *)HTTP_SERVER_HEAD2, strlen(HTTP_SERVER_HEAD2));
send_standard_body:
            sendEx(TCPacceptedSocket, acSendBuffer, strlen(acSendBuffer));
            }
          else if(!strncmp(pstrRecvMsg->pu8Buffer+4,"/copyright.html",15)) {
            sendEx(TCPacceptedSocket, (char *)HTTP_SERVER_OK, strlen(HTTP_SERVER_OK));
            strcpy(acSendBuffer,_PC_PIC_CPU_C);
            goto send_standard_page;
            }
          else {
send_non_trovato:
            sendEx(TCPacceptedSocket, (char *)HTTP_SERVER_ERR, strlen(HTTP_SERVER_ERR));
            strcpy(acSendBuffer,"Non trovato.\r\n");
            goto send_standard_page;
            }
          }
        else if(!strncmp(pstrRecvMsg->pu8Buffer,"POST",4)) {    // stricmp non c'è... diocheffroci
          if(!strncmp(pstrRecvMsg->pu8Buffer+5,"/config.cgi",11)) {
            
          //          Reset();    // aggiungere bottone a parte??

goto send_non_trovato; //FARE!
            }
          }
        else
          goto invalid_request;   // 
        
        // Recv response from client. bah serve davvero??
        recv(TCPacceptedSocket, rxBuffer, sizeof(rxBuffer), 0);
        
invalid_request:        
        bTCPIsfinished=1;    // non è il massimo...
         
        // Close the socket when finished.
        if(bTCPIsfinished) {
          close(TCPacceptedSocket);
          TCPacceptedSocket=INVALID_SOCKET;
//          close(TCPlistenSocket);
          }
        }
      }
      break;
    case SOCKET_MSG_SEND:
    {
      int sentBytes = *((sint16*)pvMsg);
      internetBufferLen=sentBytes;
    }
      break;
    case SOCKET_MSG_CLOSE:    // diofa non c'era!
      bTCPIsfinished=TRUE;
      close(TCPacceptedSocket);
      TCPacceptedSocket=INVALID_SOCKET;
      break;
    default:
      break;
    }
      
  
  }


#endif


void SetTimeFromNow(DWORD now,PIC16_DATE *date,PIC16_TIME *time) {
  unsigned int i,y;
  unsigned long d,j;
  
  d= now + ((signed long)timeZone)*3600;         // Correct for TZ/DST offset
  d= d / 86400L;                          // Integer number of days
  // Count the number of days per year taking in account leap years
  // to determine the year number and remaining days
  y=1970;   // unix
  while(d >= (j = isleap(y) ? 366 : 365) ) {
    y++;
    d -= j;
    }
  while(d < 0) {
    y--;
    d += isleap(y) ? 366 : 365;
    }
  i = isleap(y);
  // Count the days for each month in this year to determine the month
  for(date->mon=0; d >= days_month[i][date->mon]; ++date->mon) {
    d -= days_month[i][date->mon];
    }
  
  j= (now + ((signed long)timeZone)*3600) % 86400;     // Fraction of a day
	time->hour=j/3600;
  j %= 3600;                                 // Fraction of hour
  time->min=j/60;
  time->sec=j % 60;

  date->year=y;
  date->mon++;
  date->mday=d+1;
  }



  
void __ISR ( _CHANGE_NOTICE_F_VECTOR, IPL3SRS ) CNFInt(void) {
  
#ifdef USA_WIFI
  if(m_WIRQ_CN) {
    extern tpfNmBspIsr gpfIsr;
    if(gpfIsr)
      gpfIsr();
    m_WIRQ_CN=0;
    }
#endif

//  PORTF;
//  CNFF=0;
  IFS3bits.CNFIF=0;
  }


