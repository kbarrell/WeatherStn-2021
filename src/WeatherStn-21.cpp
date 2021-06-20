/*******************************************************************************
 *
 *  File:          WeatherStn-21.cpp
 * 
 *  Function:      WeatherStn-21 main application file.
 * 
 *  Copyright:     Copyright (c) 2021 Kevin Barrell
 *                 Copyright (c) 2021 Leonel Lopes Parente
 *                 Copyright (c) 2018 Terry Moore, MCCI
 *                 Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *                 
 *                 Permission is hereby granted, free of charge, to anyone 
 *                 obtaining a copy of this document and accompanying files to do, 
 *                 whatever they want with them without any restriction, including,
 *                 but not limited to, copying, modification and redistribution.
 *                 The above copyright notice and this permission notice shall be 
 *                 included in all copies or substantial portions of the Software.
 * 
 *                 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 * 
 *  License:       MIT License. See accompanying LICENSE file.
 * 
 *  Author:        Kevin Barrell,  Leonel Lopes Parente
 * 
 *  Description:   Migration of initial 2020 version of bespoke arduino Weather Station
 *                 code  https://github.com/kbarrell/Arduino-WeatherStation to adopt:
 * 
 *                    ::  Latest MCCI arduino-lmic library
 *                    ::  LMIC-node orchestration of lmic processing 
 *                    ::  OTA Activation of LoraWAN TTN v3
 *                    ::  Introduction of replacement humidity sensor SHT-31D
 * 
 *                  Multiple functions from LMIC-node have been left in place for future
 *                  flexibility, even if no longer invoked in this version of WeatherStn
 * 
 * 
 *  LMIC-node Dependencies:  External libraries:
 *                 MCCI LoRaWAN LMIC library  https://github.com/mcci-catena/arduino-lmic
 *                 IBM LMIC framework         https://github.com/matthijskooijman/arduino-lmic  
 *                 U8g2                       https://github.com/olikraus/u8g2
 *                 EasyLed                    https://github.com/lnlp/EasyLed
 *
 * 
 ******************************************************************************/

#include "LMIC-node.h"        //Handling of mulit-board generalised LMIC integration

#include <SPI.h>
#include <cactus_io_BME280_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_SHT31.h>

#include "TimerOne.h"     // Timer Interrupt set to 2.5 sec for read sensors
#include <math.h>
#include <Wire.h>         // For accessing RTC
#include <SD2405RTC.h>    // For Gravity RTC breakout board.   Set RTC to UTC time
#include <TimeLib.h>      // For epoch time en/decode
#include <Timezone.h>	  // For AU Eastern STD/DST so that daily readings are 24hr to 9am (local)

// Sensor-related definitions
// Set hardware pin assignments & pre-set constants
#define TX_Pin 4 				   // used to indicate lora link tx via external LED
#define ONE_WIRE_BUS_PIN 29 	  //Data bus pin for DS18B20's

#define WindSensor_Pin (18)       //The pin location of the anemometer sensor
#define WindVane_Pin  (A13)       // The pin connecting to the wind vane sensor
#define VaneOffset  0		   // The anemometer offset from magnetic north
#define Bucket_Size  0.2 	   // mm bucket capacity to trigger tip count
#define RG11_Pin  19        		 // Interrupt pin for rain sensor
#define BounceInterval  15		// Number of ms to allow for debouncing
#define SampleInt_Pin   3		// Interrupt pin for RTC-generated sampling clock (when used)
#define SHT31_Addr 0x44         //  Second humidity sensor primary I2C address
#define HighHumidityLevel 975   //  97.5% Reltive humidity - above which, sensor heater is applied

// Set timer related settings for sensor sampling & calculation
#define Timing_Clock  500000    //  0.5sec in millis
#define Sample_Interval   5		//  = number of Timing_Clock cycles  i.e. 2.5sec interval
#define Report_Interval   120    //  = number of sample intervals contributing to each upload report (each 5 min)
#define Speed_Conversion  1.4481   // convert rotations to km/h.  = 2.25/(Sample_Interval x Timing_Clock)* 1.609 
									// refer Davis anemometer technical spec
//#define TIMER_FROM_RTC 1		// Uncomment this line if timing clock for sampler drawn from RTC frequency interrupt
									
volatile bool isSampleRequired;    		// set true every Sample_Interval.   Get wind speed
volatile unsigned int timerCount;  		// used to determine when Sample_Interval is reached
volatile unsigned int sampleCount;		// used to determin when Report_Interval is reached
volatile unsigned long rotations;  		// cup rotation counter for wind speed calcs
volatile unsigned long contactBounceTime;  // Timer to avoid contact bounce in wind speed sensor
volatile float windSpeed, windGust;        // speed in km per hour

volatile unsigned long tipCount;  	 	// rain bucket tipcounter used in interrupt routine
volatile unsigned long contactTime; 	// timer to manage contact bounce in interrupt routine
volatile unsigned long obsRainfallCount; // total count of rainfall tips recorded in observatoin period (5min)
volatile float obsReportRainfallRate;    	// total amount of rainfall in the reporting period  (5 min)
volatile unsigned long dailyRainfallCount;	//  total count of rainfall tips in 24 hrs to 9am (local time)
const float reportIntervalSec = Report_Interval * Sample_Interval * float(Timing_Clock) / 1000000;

// Define structures for handling reporting via TTN
typedef struct obsSet {
	uint16_t 	windGustX10; // observed windgust speed (km/h) X10  ~range 0 -> 1200
	uint16_t	windGustDir; // observed wind direction of Gust (compass degrees)  0 -> 359
	uint16_t	tempX10;	// observed temp (°C) +100 x 10   ~range -200->600
	uint16_t	humidX10;	// observed relative humidty (%) x 10  from BME280 range 0->1000
	uint16_t 	pressX10;	// observed barometric pressure at station level (hPa)  x 10  ~range 8700 -> 11000 
	uint16_t	rainflX10;	// observed accumulated rainfall (mm) x10   ~range 0->1200
	uint16_t	windspX10;	// observed windspeed (km/h) x10 ~range 0->1200
	uint16_t	windDir;	// observed wind direction (compass degrees)  range 0->359
	uint16_t	dailyRainX10; //  accumulated rainfall (mm) X10 for period to 9am daily
	uint16_t	casetempX10;	// station case temperature (for alarming)
    uint16_t    humid2X10;  // observed relative humidity (%) x10 from additional sensor
 } obsSet;
		
union obsPayload
{
	obsSet	obsReport;
	uint8_t	readAccess[sizeof(obsSet)];
}sensorObs[2];

// AU Eastern Time Zone (Sydney, Melbourne)   Use next 3 lines for one time setup to be written to EEPROM
//TimeChangeRule auEDST = {"AEDT", First, Sun, Oct, 2, 660};    //Daylight time = UTC + 11:00 hours
//TimeChangeRule auESTD = {"AEST", First, Sun, Apr, 2, 600};     //Standard time = UTC + 10:00 hours
//Timezone auEastern(auEDST, auESTD);

// If TimeChangeRules are already stored in EEPROM, comment out the three
// lines above and uncomment the line below.
Timezone auEastern(100);       // assumes rules stored at EEPROM address 100 & that RTC set to UTC
TimeChangeRule *tcr;		// pointer to the timechange rule
time_t utc, localTime;
boolean	dailyTotalsDue;		// flags that totals for 24hr to 9am local are to be reported & reset

int  currentObs, reportObs;   //References which obsPayload [0 or 1]is being filled vs. reported 
int vaneValue;         	 	//  raw analog value from wind vane
int vaneDirection;          //  translated 0-360 direction
int calDirection, calGustDirn;     	//  converted value with offset applied
const int count = 4;				// average the last 4 wind directions
int boxcar[count];					// stack of wind direction values for averaging calculation
const bool BaseRange = true;
const bool ExtdRange = false;
bool enableHeater = false;        // SHT31 sensor heater
bool highHumidity = false;        // flags a high humidity (> HighHumidityLevel e.g.97%) condition

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 300 ;		// 5 min reporting cycle
const int EOD_HOUR = 9;			// Daily totals are reset at 9am (local);

//  Create temp & humidity sensor objects
BME280_I2C bme;     // I2C using address 0x77  Now used for pressure (not humidity nor temp)
Adafruit_SHT31 sht31 = Adafruit_SHT31();  // Second humidity sensor.  Added following condensation 
                                          // issues with BME280 (which is retained as pressure sensor)

// Setup a oneWire instance to communicate with OneWire devices
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature DSsensors(&oneWire);    // Pass the OneWire reference to Dallas Temperature lib

// Assign the addresses of the DS18B20 sensors (determined by reading them previously)
DeviceAddress airTempAddr = { DS18_AIR_ADDR };
DeviceAddress caseTempAddr = { DS18_CASE_ADDR };



static osjob_t sendjob;    

// Note: LoRa module pin mappings are defined in the Board Support Files.

// Set LoRaWAN keys defined in lorawan-keys.h.
#ifdef OTAA_ACTIVATION
    static const u1_t PROGMEM DEVEUI[8]  = { OTAA_DEVEUI } ;
    static const u1_t PROGMEM APPEUI[8]  = { OTAA_APPEUI };
    static const u1_t PROGMEM APPKEY[16] = { OTAA_APPKEY };
    // Below callbacks are used by LMIC for reading above values.
    void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
    void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
    void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }    
#else
    // ABP activation
    static const u4_t DEVADDR = ABP_DEVADDR ;
    static const PROGMEM u1_t NWKSKEY[16] = { ABP_NWKSKEY };
    static const u1_t PROGMEM APPSKEY[16] = { ABP_APPSKEY };
    // Below callbacks are not used be they must be defined.
    void os_getDevEui (u1_t* buf) { }
    void os_getArtEui (u1_t* buf) { }
    void os_getDevKey (u1_t* buf) { }
#endif


int16_t getSnrTenfold()
{
    // Returns ten times the SNR (dB) value of the last received packet.
    // Ten times to prevent the use of float but keep 1 decimal digit accuracy.
    // Calculation per SX1276 datasheet rev.7 §6.4, SX1276 datasheet rev.4 §6.4.
    // LMIC.snr contains value of PacketSnr, which is 4 times the actual SNR value.
    return (LMIC.snr * 10) / 4;
}


int16_t getRssi(int8_t snr)
{
    // Returns correct RSSI (dBm) value of the last received packet.
    // Calculation per SX1276 datasheet rev.7 §5.5.5, SX1272 datasheet rev.4 §5.5.5.

    #define RSSI_OFFSET            64
    #define SX1276_FREQ_LF_MAX     525000000     // per datasheet 6.3
    #define SX1272_RSSI_ADJUST     -139
    #define SX1276_RSSI_ADJUST_LF  -164
    #define SX1276_RSSI_ADJUST_HF  -157

    int16_t rssi;

    #ifdef MCCI_LMIC

        rssi = LMIC.rssi - RSSI_OFFSET;

    #else
        int16_t rssiAdjust;
        #ifdef CFG_sx1276_radio
            if (LMIC.freq > SX1276_FREQ_LF_MAX)
            {
                rssiAdjust = SX1276_RSSI_ADJUST_HF;
            }
            else
            {
                rssiAdjust = SX1276_RSSI_ADJUST_LF;   
            }
        #else
            // CFG_sx1272_radio    
            rssiAdjust = SX1272_RSSI_ADJUST;
        #endif    
        
        // Revert modification (applied in lmic/radio.c) to get PacketRssi.
        int16_t packetRssi = LMIC.rssi + 125 - RSSI_OFFSET;
        if (snr < 0)
        {
            rssi = rssiAdjust + packetRssi + snr;
        }
        else
        {
            rssi = rssiAdjust + (16 * packetRssi) / 15;
        }
    #endif

    return rssi;
}


void printEvent(ostime_t timestamp, 
                const char * const message, 
                PrintTarget target = PrintTarget::All,
                bool clearDisplayStatusRow = true,
                bool eventLabel = false)
{
    int hour_now = hour();
    int minute_now = minute();
    int second_now = second();

    #ifdef USE_DISPLAY 
        if (target == PrintTarget::All || target == PrintTarget::Display)
        {
            display.clearLine(TIME_ROW);
            display.setCursor(COL_0, TIME_ROW);
            display.print(F("Time:"));                 
            display.print(timestamp); 
            display.clearLine(EVENT_ROW);
            if (clearDisplayStatusRow)
            {
                display.clearLine(STATUS_ROW);    
            }
            display.setCursor(COL_0, EVENT_ROW);               
            display.print(message);
        }
    #endif  
    
    #ifdef USE_SERIAL
        // Create padded/indented output without using printf().
        // printf() is not default supported/enabled in each Arduino core. 
        // Not using printf() will save memory for memory constrainted devices.
        String timeString(timestamp);
        uint8_t len = timeString.length();
        uint8_t zerosCount = TIMESTAMP_WIDTH > len ? TIMESTAMP_WIDTH - len : 0;

        if (target == PrintTarget::All || target == PrintTarget::Serial)
        {
            serial.print(hour_now); serial.print(":"); serial.print(minute_now); serial.print(":");
            serial.print(second_now); serial.print("::\t");
            printChars(serial, '0', zerosCount);
            serial.print(timeString);
            serial.print(":  ");
            if (eventLabel)
            {
                serial.print(F("Event: "));
            }
            serial.println(message);
        }
    #endif   
}           

void printEvent(ostime_t timestamp, 
                ev_t ev, 
                PrintTarget target = PrintTarget::All, 
                bool clearDisplayStatusRow = true)
{
    #if defined(USE_DISPLAY) || defined(USE_SERIAL)
        printEvent(timestamp, lmicEventNames[ev], target, clearDisplayStatusRow, true);
    #endif
}


void printFrameCounters(PrintTarget target = PrintTarget::All)
{
    #ifdef USE_DISPLAY
        if (target == PrintTarget::Display || target == PrintTarget::All)
        {
            display.clearLine(FRMCNTRS_ROW);
            display.setCursor(COL_0, FRMCNTRS_ROW);
            display.print(F("Up:"));
            display.print(LMIC.seqnoUp);
            display.print(F(" Dn:"));
            display.print(LMIC.seqnoDn);        
        }
    #endif

    #ifdef USE_SERIAL
        if (target == PrintTarget::Serial || target == PrintTarget::All)
        {
            printSpaces(serial, MESSAGE_INDENT);
            serial.print(F("Up: "));
            serial.print(LMIC.seqnoUp);
            serial.print(F(",  Down: "));
            serial.println(LMIC.seqnoDn);        
        }
    #endif        
}      


void printSessionKeys()
{    
    #if defined(USE_SERIAL) && defined(MCCI_LMIC)
        u4_t networkId = 0;
        devaddr_t deviceAddress = 0;
        u1_t networkSessionKey[16];
        u1_t applicationSessionKey[16];
        LMIC_getSessionKeys(&networkId, &deviceAddress, 
                            networkSessionKey, applicationSessionKey);

        printSpaces(serial, MESSAGE_INDENT);    
        serial.print(F("Network Id: "));
        serial.println(networkId, DEC);

        printSpaces(serial, MESSAGE_INDENT);    
        serial.print(F("Device Address: "));
        serial.println(deviceAddress, HEX);

        printSpaces(serial, MESSAGE_INDENT);    
        serial.print(F("Application Session Key: "));
        printHex(serial, applicationSessionKey, 16, true, '-');

        printSpaces(serial, MESSAGE_INDENT);    
        serial.print(F("Network Session Key:     "));
        printHex(serial, networkSessionKey, 16, true, '-');
    #endif
}


void printDownlinkInfo(void)
{
    #if defined(USE_SERIAL) || defined(USE_DISPLAY)

        uint8_t dataLength = LMIC.dataLen;
        // bool ackReceived = LMIC.txrxFlags & TXRX_ACK;

        int16_t snrTenfold = getSnrTenfold();
        int8_t snr = snrTenfold / 10;
        int8_t snrDecimalFraction = snrTenfold % 10;
        int16_t rssi = getRssi(snr);

        uint8_t fPort = 0;        
        if (LMIC.txrxFlags & TXRX_PORT)
        {
            fPort = LMIC.frame[LMIC.dataBeg -1];
        }        

        #ifdef USE_DISPLAY
            display.clearLine(EVENT_ROW);        
            display.setCursor(COL_0, EVENT_ROW);
            display.print(F("RX P:"));
            display.print(fPort);
            if (dataLength != 0)
            {
                display.print(" Len:");
                display.print(LMIC.dataLen);                       
            }
            display.clearLine(STATUS_ROW);        
            display.setCursor(COL_0, STATUS_ROW);
            display.print(F("RSSI"));
            display.print(rssi);
            display.print(F(" SNR"));
            display.print(snr);                
            display.print(".");                
            display.print(snrDecimalFraction);                      
        #endif

        #ifdef USE_SERIAL
            printSpaces(serial, MESSAGE_INDENT);    
            serial.println(F("Downlink received"));

            printSpaces(serial, MESSAGE_INDENT);
            serial.print(F("RSSI: "));
            serial.print(rssi);
            serial.print(F(" dBm,  SNR: "));
            serial.print(snr);                        
            serial.print(".");                        
            serial.print(snrDecimalFraction);                        
            serial.println(F(" dB"));

            printSpaces(serial, MESSAGE_INDENT);    
            serial.print(F("Port: "));
            serial.println(fPort);
   
            if (dataLength != 0)
            {
                printSpaces(serial, MESSAGE_INDENT);
                serial.print(F("Length: "));
                serial.println(LMIC.dataLen);                   
                printSpaces(serial, MESSAGE_INDENT);    
                serial.print(F("Data: "));
                printHex(serial, LMIC.frame+LMIC.dataBeg, LMIC.dataLen, true, ' ');
            }
        #endif
    #endif
} 


void printHeader(void)
{
    #ifdef USE_DISPLAY
        display.clear();
        display.setCursor(COL_0, HEADER_ROW);
        display.print(F("LMIC-node"));
        #ifdef ABP_ACTIVATION
            display.drawString(ABPMODE_COL, HEADER_ROW, "ABP");
        #endif
        #ifdef CLASSIC_LMIC
            display.drawString(CLMICSYMBOL_COL, HEADER_ROW, "*");
        #endif
        display.drawString(COL_0, DEVICEID_ROW, deviceId);
        display.setCursor(COL_0, INTERVAL_ROW);
        display.print(F("Interval:"));
        display.print(doWorkIntervalSeconds);
        display.print("s");
    #endif

    #ifdef USE_SERIAL
        serial.println(F("\n\nLMIC-node\n"));
        serial.print(F("Device-id:     "));
        serial.println(deviceId);            
        serial.print(F("LMIC library:  "));
        #ifdef MCCI_LMIC  
            serial.println(F("MCCI"));
        #else
            serial.println(F("Classic [Deprecated]")); 
        #endif
        serial.print(F("Activation:    "));
        #ifdef OTAA_ACTIVATION  
            serial.println(F("OTAA"));
        #else
            serial.println(F("ABP")); 
        #endif
        #if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
            serial.print(F("LMIC debug:    "));  
            serial.println(LMIC_DEBUG_LEVEL);
        #endif

        if (activationMode == ActivationMode::OTAA)
        {
            serial.println();
        }
    #endif
}     


#ifdef ABP_ACTIVATION
    void setAbpParameters(dr_t dataRate = DR_SF7, s1_t txPower = 14) 
    {
        // Set static session parameters. Instead of dynamically establishing a session
        // by joining the network, precomputed session parameters are be provided.
        #ifdef PROGMEM
            // On AVR, these values are stored in flash and only copied to RAM
            // once. Copy them to a temporary buffer here, LMIC_setSession will
            // copy them into a buffer of its own again.
            uint8_t appskey[sizeof(APPSKEY)];
            uint8_t nwkskey[sizeof(NWKSKEY)];
            memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
            memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
            LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
        #else
            // If not running an AVR with PROGMEM, just use the arrays directly
            LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
        #endif

        #if defined(CFG_eu868)
            // Set up the channels used by the Things Network, which corresponds
            // to the defaults of most gateways. Without this, only three base
            // channels from the LoRaWAN specification are used, which certainly
            // works, so it is good for debugging, but can overload those
            // frequencies, so be sure to configure the full frequency range of
            // your network here (unless your network autoconfigures them).
            // Setting up channels should happen after LMIC_setSession, as that
            // configures the minimal channel set. The LMIC doesn't let you change
            // the three basic settings, but we show them here.
            LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
            LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
            LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
            LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
            LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
            LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
            LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
            LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
            LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
            // TTN defines an additional channel at 869.525Mhz using SF9 for class B
            // devices' ping slots. LMIC does not have an easy way to define set this
            // frequency and support for class B is spotty and untested, so this
            // frequency is not configured here.
        #elif defined(CFG_us915) || defined(CFG_au915)
            // NA-US and AU channels 0-71 are configured automatically
            // but only one group of 8 should (a subband) should be active
            // TTN recommends the second sub band, 1 in a zero based count.
            // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
            LMIC_selectSubBand(1);
        #elif defined(CFG_as923)
            // Set up the channels used in your country. Only two are defined by default,
            // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
            // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
            // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

            // ... extra definitions for channels 2..n here
        #elif defined(CFG_kr920)
            // Set up the channels used in your country. Three are defined by default,
            // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
            // BAND_MILLI.
            // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
            // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
            // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

            // ... extra definitions for channels 3..n here.
        #elif defined(CFG_in866)
            // Set up the channels used in your country. Three are defined by default,
            // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
            // BAND_MILLI.
            // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
            // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
            // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

            // ... extra definitions for channels 3..n here.
        #endif

        // Disable link check validation
        LMIC_setLinkCheckMode(0);

        // TTN uses SF9 for its RX2 window.
        LMIC.dn2Dr = DR_SF9;

        // Set data rate and transmit power (note: txpow seems to be ignored by the library)
        LMIC_setDrTxpow(dataRate, txPower);    
    }
#endif //ABP_ACTIVATION


void initLmic(bit_t adrEnabled = 1, 
              dr_t dataRate = DR_SF7, 
              s1_t txPower = 14, 
              bool setDrTxPowForOtaaExplicit = false) 
{
    // Initialize LMIC runtime environment
    os_init();
    // Reset MAC state
    LMIC_reset();

    #ifdef ABP_ACTIVATION
        setAbpParameters(dataRate, txPower);
    #endif

    // Enable or disable ADR (data rate adaptation). 
    // Should be turned off if the device is not stationary (mobile).
    // 1 is on, 0 is off.
    LMIC_setAdrMode(adrEnabled);

    if (activationMode == ActivationMode::OTAA)
    {
        #if defined(CFG_us915) || defined(CFG_au915)
            // NA-US and AU channels 0-71 are configured automatically
            // but only one group of 8 should (a subband) should be active
            // TTN recommends the second sub band, 1 in a zero based count.
            // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
            LMIC_selectSubBand(1); 
        #endif

        // Optional: set/override data rate and transmit power for OTAA (only use if ADR is disabled).
        if (setDrTxPowForOtaaExplicit && !adrEnabled)
        {
            // Below dataRate will be overridden again when joined.
            LMIC_setDrTxpow(dataRate, txPower);
        }
    }

    // Relax LMIC timing if defined
    #if defined(LMIC_CLOCK_ERROR_PPM) && LMIC_CLOCK_ERROR_PPM > 0
        #if defined(MCCI_LMIC) && LMIC_CLOCK_ERROR_PPM > 4000
            // Allow clock error percentage to be > 0.4%
            #define LMIC_ENABLE_arbitrary_clock_error 1
        #endif    
        uint32_t clockError = (LMIC_CLOCK_ERROR_PPM / 100) * (MAX_CLOCK_ERROR / 100) / 100;
        LMIC_setClockError(clockError);

        #ifdef USE_SERIAL
            serial.print(F("Clock Error:   "));
            serial.print(LMIC_CLOCK_ERROR_PPM);
            serial.print(" ppm (");
            serial.print(clockError);
            serial.println(")");            
        #endif
    #endif

    #ifdef MCCI_LMIC
        // Register a custom eventhandler and don't use default onEvent() to enable
        // additional features (e.g. make EV_RXSTART available). User data pointer is omitted.
        LMIC_registerEventCb(&onLmicEvent, nullptr);
    #endif
}


#ifdef MCCI_LMIC 
void onLmicEvent(void *pUserData, ev_t ev)
#else
void onEvent(ev_t ev) 
#endif
{
    // LMIC event handler
    ostime_t timestamp = os_getTime(); 

    switch (ev) 
    {
#ifdef MCCI_LMIC
        // Only supported in MCCI LMIC library:
        case EV_RXSTART:
            // Do not print anything for this event or it will mess up timing.
            break;

        case EV_TXSTART:
            setTxIndicatorsOn();
            digitalWrite(TX_Pin, HIGH);		//  Tx/Rx LED ON for external visual
            printEvent(timestamp, ev);            
            break;               

        case EV_JOIN_TXCOMPLETE:
        case EV_TXCANCELED:
            setTxIndicatorsOn(false);
            printEvent(timestamp, ev);
            break;               
#endif
        case EV_JOINED:
            setTxIndicatorsOn(false);
            printEvent(timestamp, ev);
            printSessionKeys();

            // Disable link check validation.
            // Link check validation is automatically enabled
            // during join, but because slow data rates change
            // max TX size, it is not used in this example.                    
            LMIC_setLinkCheckMode(0);

            // The doWork job has probably run already (while
            // the node was still joining) and have rescheduled itself.
            // Cancel the next scheduled doWork job and re-schedule
            // for immediate execution to prevent that any uplink will
            // have to wait until the current doWork interval ends.
            os_clearCallback(&sendjob);
            os_setCallback(&sendjob, do_send);
            break;

        case EV_TXCOMPLETE:
            // Transmit completed, includes waiting for RX windows.
            setTxIndicatorsOn(false);   
            digitalWrite(TX_Pin, LOW);		// Tx/Rx LED off
            printEvent(timestamp, ev);
            printFrameCounters();

            // Check if downlink was received
            if (LMIC.dataLen != 0 || LMIC.dataBeg != 0)
            {
                uint8_t fPort = 0;
                if (LMIC.txrxFlags & TXRX_PORT)
                {
                    fPort = LMIC.frame[LMIC.dataBeg -1];
                }
                printDownlinkInfo();
                processDownlink(timestamp, fPort, LMIC.frame + LMIC.dataBeg, LMIC.dataLen);                
            }
            break;     
          
        // Below events are printed only.
        case EV_SCAN_TIMEOUT:
        case EV_BEACON_FOUND:
        case EV_BEACON_MISSED:
        case EV_BEACON_TRACKED:
        case EV_RFU1:                    // This event is defined but not used in code
        case EV_JOINING:        
        case EV_JOIN_FAILED:           
        case EV_REJOIN_FAILED:
        case EV_LOST_TSYNC:
        case EV_RESET:
        case EV_RXCOMPLETE:
        case EV_LINK_DEAD:
        case EV_LINK_ALIVE:
#ifdef MCCI_LMIC
        // Only supported in MCCI LMIC library:
        case EV_SCAN_FOUND:              // This event is defined but not used in code 
#endif
            printEvent(timestamp, ev);    
            break;

        default: 
            printEvent(timestamp, "Unknown Event");    
            break;
    }
}

lmic_tx_error_t scheduleUplink(uint8_t fPort, uint8_t* data, uint8_t dataLength, bool confirmed = false)
{
    // This function is called from the processWork() function to schedule
    // transmission of an uplink message that was prepared by processWork().
    // Transmission will be performed at the next possible time

    ostime_t timestamp = os_getTime();
    printEvent(timestamp, "Packet queued");

    lmic_tx_error_t retval = LMIC_setTxData2(fPort, data, dataLength, confirmed ? 1 : 0);
    timestamp = os_getTime();

    if (retval == LMIC_ERROR_SUCCESS)
    {
        #ifdef CLASSIC_LMIC
            // For MCCI_LMIC this will be handled in EV_TXSTART        
            setTxIndicatorsOn();  
        #endif        
    }
    else
    {
        String errmsg; 
        #ifdef USE_SERIAL
            errmsg = "LMIC Error: ";
            #ifdef MCCI_LMIC
                errmsg.concat(lmicErrorNames[abs(retval)]);
            #else
                errmsg.concat(retval);
            #endif
            printEvent(timestamp, errmsg.c_str(), PrintTarget::Serial);
        #endif
        #ifdef USE_DISPLAY
            errmsg = "LMIC Err: ";
            errmsg.concat(retval);
            printEvent(timestamp, errmsg.c_str(), PrintTarget::Display);
        #endif         
    }
    return retval;    
}

static void do_send(osjob_t* j){

    ostime_t timestamp = os_getTime();
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        // TxRx is currently pending, do not send.
        #ifdef USE_SERIAL
            printEvent(timestamp, "Uplink not scheduled because TxRx pending", PrintTarget::Serial);
        #endif    
        #ifdef USE_DISPLAY
            printEvent(timestamp, "UL not scheduled", PrintTarget::Display);
        #endif
    }
     else 
    {
        // Prepare upstream data transmission at the next possible time.
        scheduleUplink(1, sensorObs[reportObs].readAccess, sizeof(obsSet), 0);     // Use the last completed set of obs
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

//  Left-over from LMIC-node where "counter" used as proxy sensor data
static volatile uint16_t counter_ = 0;

uint16_t getCounterValue()
{
    // Increments counter and returns the new value.
    //delay(50);         // Fake this takes some time
    return ++counter_;
}

void resetCounter()
{
    // Reset counter to 0
    counter_ = 0;
}

// Interrupt handler routine for timer interrupt
void isr_timer() {
	
	timerCount++;

	if(timerCount == Sample_Interval) {
		// convert to km/h using the formula V=P(2.25/T)*1.609 where T = sample interval
		// i.e. V = P(2.25/2.5)*1.609 = P * Speed_Conversion factor  (=1.4481  for 2.5s interval)
		windSpeed = rotations * Speed_Conversion; 
		rotations = 0;   
		isSampleRequired = true;
		timerCount = 0;						// Restart the interval count
	}
}

// Interrupt handler routine to increment the rotation count for wind speed
void isr_rotation ()   {

    if ((millis() - contactBounceTime) > BounceInterval ) {  // debounce the switch contact.
        rotations++;
        contactBounceTime = millis();
    }
}

// Interrrupt handler routine that is triggered when the rg-11 detects rain   
void isr_rg ()   { 

   if ((millis() - contactTime) > BounceInterval ) {  // debounce of sensor signal
      tipCount++;
      contactTime = millis();
   } 
} 

//  Calculate average of 'count' readings using boxcar method
int average(int value)
{
  static int i;
  static long sum=0;

  sum -= boxcar[i];  //remove oldest value from sum
  boxcar[i] = value;  // add new value to array
  sum += boxcar[i]; // add new value to sum

  i++;
  if (i == count) i=0;
  return sum/count;
}

// Get Wind Direction
void getWindDirection(bool baseRange) {
	static int recentAvgDirn = 0;		// average of last 3 adjusted measurements
	int altReading, deltaAsRead, deltaExtd;		// candidate alternative to raw direction measurement
	
	if (baseRange) {		// take a reading in standard 0-360 deg. range
		vaneValue = analogRead(WindVane_Pin);
		vaneDirection = map(vaneValue, 0, 1023, 0, 359);
		calDirection = vaneDirection + VaneOffset;
		if(calDirection > 360)
			calDirection = calDirection - 360;
		return;			// returns value via calDirection
	}   
	
	// Here (baseRange is FALSE) we find if +/- 360 gives a reading closer to the most recent wind direction
	// Does not take a new direction reading - uses the last one 
	// This averaging is only invoked for directions included in the 5 min reports
	if (calDirection > 270) altReading	= calDirection - 360;
	else if (calDirection < 90) altReading = calDirection + 360;
	else altReading = calDirection;
	
	deltaAsRead = abs(calDirection - recentAvgDirn);
	deltaExtd = abs(altReading - recentAvgDirn);
	calDirection = (deltaAsRead < deltaExtd) ? calDirection : altReading;
	
	// Update the average etc
	recentAvgDirn = average(calDirection);
}

// Field format utility for printing
void print2digits(int number)  {
	if (number >= 0 && number <10) {
		Serial.write('0');
	}
	Serial.print(number);
}

// Check if a report is the last of a daily sequence.  Relies on window open +/- 1hr 
//  either side of EOD_HOUR
boolean resetDaily(time_t localTime, int windowOpensHr, int windowClosesHr) {
	int checkHour = hour(localTime);
	if ((checkHour < windowOpensHr) || (checkHour >= windowClosesHr)) {
		dailyTotalsDue = true;
		return false; 		// Outside the reset time window
	}
	if (dailyTotalsDue) {
		if (checkHour == EOD_HOUR) {
			dailyTotalsDue = false;
			return true;
		}
		if ((minute(localTime) + TX_INTERVAL/60) < 60 ) {
			dailyTotalsDue = true;
			return false;    // Next report will still be ahead of EOD_HOUR
		}
		else {
			dailyTotalsDue = false;
			return true;	//  Next report period should start from zero daily totals
		}
	}
	else return false;
}
				

// Print utility for packed structure
void printIt(uint8_t *charArray, int length) {
  int i;
	char charMember;
	Serial.print("buff length:"); Serial.println(length);
	for (i=0; i<length; i++) {
		charMember = charArray[i];
		Serial.println(charMember, BIN);
	}
	Serial.println("===EndOfBuffer========");
}

void processWork(ostime_t doWorkJobTimeStamp)
{
    // This is where the main work is performed like
    // reading sensor and GPS data and schedule uplink
    // messages if anything needs to be transmitted.

    // Skip processWork if using OTAA and still joining.
    if (LMIC.devaddr != 0)
    {
        // Collect input data.  All sensorObs content is WeatherStation related
        // For simplicity LMIC-node used a counter to simulate a sensor. 
        // The counter is increased automatically by getCounterValue()
        // and can be reset with a 'reset counter' command downlink message.
        // This is currently retained for convenience

        uint16_t counterValue = getCounterValue();
        ostime_t timestamp = os_getTime();

        obsRainfallCount = tipCount - dailyRainfallCount;
		dailyRainfallCount = tipCount;
		getWindDirection(ExtdRange);	// Update direction to reflect recent average in {-90 to 450 deg}

		obsReportRainfallRate = obsRainfallCount * Bucket_Size * 3600 / reportIntervalSec;   //  mm/hr
		sensorObs[currentObs].obsReport.windGustX10 = windGust * 10.0;
		sensorObs[currentObs].obsReport.windGustDir = calGustDirn;
		sensorObs[currentObs].obsReport.tempX10 = (DSsensors.getTempC(airTempAddr)+ 100.0)* 10.0;
		sensorObs[currentObs].obsReport.humidX10 = bme.getHumidity()*10.0;
		sensorObs[currentObs].obsReport.pressX10 = bme.getPressure_MB()*10.0;
		sensorObs[currentObs].obsReport.rainflX10 = obsReportRainfallRate * 10.0;
		sensorObs[currentObs].obsReport.windspX10 = windSpeed * 10.0;
		sensorObs[currentObs].obsReport.windDir =  calDirection +90;   // NB: Offset caters for extended range -90 to 450
		sensorObs[currentObs].obsReport.dailyRainX10 = dailyRainfallCount * Bucket_Size * 10.0;
		sensorObs[currentObs].obsReport.casetempX10 = (DSsensors.getTempC(caseTempAddr)+ 100.0) * 10.0;
        sensorObs[currentObs].obsReport.humid2X10 = sht31.readHumidity()*10.0;   // 2nd humidity sensor

        highHumidity = (sensorObs[currentObs].obsReport.humid2X10 > HighHumidityLevel);   // High RH% -> condensation danger

        #ifdef USE_DISPLAY
            // Interval and Counter values are combined on a single row.
            // This allows to keep the 3rd row empty which makes the
            // information better readable on the small display.
            display.clearLine(INTERVAL_ROW);
            display.setCursor(COL_0, INTERVAL_ROW);
            display.print("I:");
            display.print(doWorkIntervalSeconds);
            display.print("s");        
            display.print(" Ctr:");
            display.print(counterValue);
        #endif
        #ifdef USE_SERIAL
            printEvent(timestamp, "Input data collected", PrintTarget::Serial);
            printSpaces(serial, MESSAGE_INDENT);
            serial.print(F("COUNTER value: "));
            serial.println(counterValue);
        #endif    

        // For simplicity LMIC-node will try to send an uplink
        // message every time processWork() is executed.

        // Schedule uplink message if possible
        if (LMIC.opmode & OP_TXRXPEND)
        {
            // TxRx is currently pending, do not send.
            #ifdef USE_SERIAL
                printEvent(timestamp, "Uplink not scheduled because TxRx pending", PrintTarget::Serial);
            #endif    
            #ifdef USE_DISPLAY
                printEvent(timestamp, "UL not scheduled", PrintTarget::Display);
            #endif
        }
        else
        {
            // Prepare uplink payload.
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL/10), do_send);
        }
    }
}    
 

void processDownlink(ostime_t txCompleteTimestamp, uint8_t fPort, uint8_t* data, uint8_t dataLength)
{
    // This function is called from the onEvent() event handler
    // on EV_TXCOMPLETE when a downlink message was received.

    // Implements a 'reset counter' command that can be sent via a downlink message.
    // To send the reset counter command to the node, send a downlink message
    // (e.g. from the TTN Console) with single byte value resetCmd on port cmdPort.

    const uint8_t cmdPort = 100;
    const uint8_t resetCmd= 0xC0;

    if (fPort == cmdPort && dataLength == 1 && data[0] == resetCmd)
    {
        #ifdef USE_SERIAL
            printSpaces(serial, MESSAGE_INDENT);
            serial.println(F("Reset cmd received"));
        #endif
        ostime_t timestamp = os_getTime();
        resetCounter();
        printEvent(timestamp, "Counter reset", PrintTarget::All, false);
    }          
}


void setup() 
{
    // boardInit(InitType::Hardware) must be called at start of setup() before anything else.
    bool hardwareInitSucceeded = boardInit(InitType::Hardware);

    #ifdef USE_DISPLAY 
        initDisplay();
    #endif

    #ifdef USE_SERIAL
        initSerial(MONITOR_SPEED, WAITFOR_SERIAL_S);
    #endif    

    boardInit(InitType::PostInitSerial);

    #if defined(USE_SERIAL) || defined(USE_DISPLAY)
        printHeader();
    #endif

    if (!hardwareInitSucceeded)
    {   
        #ifdef USE_SERIAL
            serial.println(F("Error: hardware init failed."));
            serial.flush();            
        #endif
        #ifdef USE_DISPLAY
            // Following mesage shown only if failure was unrelated to I2C.
            display.setCursor(COL_0, FRMCNTRS_ROW);
            display.print(F("HW init failed"));
        #endif
        abort();
    }

    initLmic();  

    setSyncProvider(RTC.get);
    setSyncInterval(500);     // resync system time to RTC every 500 sec

    // prepare obsPayload selection indices
    currentObs = 0;
	reportObs = 1;
	dailyTotalsDue = true;
  
	// initialise anemometer values
	rotations = 0;
	isSampleRequired = false;
	windGust = 0;
	calGustDirn = 0;
  
	// setup RG11 rain totals & conversion factor
	obsRainfallCount = 0;
	obsReportRainfallRate = 0.0;
	dailyRainfallCount = 0;
  
	// setup timer values
	timerCount = 0;
	sampleCount = 0;
	
  
	// Initialise the Temperature measurement library & set sensor resolution to 12 (10) bits
	DSsensors.setResolution(airTempAddr, 12);
	DSsensors.setResolution(caseTempAddr, 10);
 
	if (!bme.begin())  {    
      Serial.println("Could not find BME280 sensor -  check wiring");
     while (1);
	}

	if (! sht31.begin(SHT31_Addr)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
    }
    if ( enableHeater || sht31.isHeaterEnabled() ) {
        enableHeater = false;
        sht31.heater(enableHeater);     // Setup with heater off initially
    }

    // Setup pins & interrupts	
	pinMode(TX_Pin, OUTPUT);
	pinMode(WindSensor_Pin, INPUT);
	pinMode(RG11_Pin, INPUT);

	attachInterrupt(digitalPinToInterrupt(WindSensor_Pin), isr_rotation, FALLING);
	attachInterrupt(digitalPinToInterrupt(RG11_Pin),isr_rg, FALLING);
	
	//Setup the timer for 0.5s
	#ifdef TIMER_FROM_RTC
		// For RTC-generated clock timer
		pinMode(SampleInt_Pin, INPUT_PULLUP);
		attachInterrupt(digitalPinToInterrupt(SampleInt_Pin), isr_timer, FALLING); 
	#else
		// timer drawn from internal MCU Timer1 interrupt
		Timer1.initialize(Timing_Clock);     
		Timer1.attachInterrupt(isr_timer);
	#endif

    resetCounter();

    if (activationMode == ActivationMode::OTAA)
    {
        LMIC_startJoining();
    }

    // Schedule initial doWork job for immediate execution.
    os_setCallback(&sendjob, do_send);
    	
	sei();   // Enable Interrupts
	
}


void loop()     
{
    ostime_t timestamp;

    if(isSampleRequired) {
		sampleCount++;
		DSsensors.requestTemperatures();    // Read temperatures from all DS18B20 devices
		bme.readSensor();					// Read humidity & barometric pressure

        if (enableHeater && (sampleCount > Report_Interval/4) ) {    
            enableHeater = false;           //Turn off heater after a quarter of interval to
            sht31.heater(enableHeater);     //    avoid measurement bias
        }

	
		getWindDirection(BaseRange);			//  Read dirn in range 0 - 360 deg.
		
		if (windSpeed > windGust) {      // Check last sample of windspeed for new Gust record
			windGust = windSpeed;
			calGustDirn = calDirection;
		}
			

	//  Does this sample complete a reporting cycle?   If so, prepare payload.
		if (sampleCount == Report_Interval) {
            timestamp = os_getTime();

			processWork(timestamp);     // called directly from loop() rather than LMIC-node 
                                        //  approach of separate doWorkJob()

            if (highHumidity) {             //  Turn on heater for start of new report interval
                enableHeater = true;
                sht31.heater(enableHeater);
            }

			sampleCount = 0;
			currentObs = 1- currentObs;		//
			reportObs = 1 - currentObs;   	// switch reporting to last collected observation
			windGust = 0;					// Gust reading is reset for every reporting period
		
			
		// Check if this report completes a daily cycle
			utc = now();
			localTime = auEastern.toLocal(utc, &tcr);
			if (resetDaily(localTime, EOD_HOUR - 1, EOD_HOUR + 1) ){
				tipCount = 0;
				dailyRainfallCount = 0;     // Next report cycle starts daily total from 0mm
				obsRainfallCount = 0;
			}
		}
			
		isSampleRequired = false;
	}
	
    os_runloop_once();
}
