/*******************************************************************************
 * 
 *  File:         bsf_arduino_mega_lora_shld.h
 * 
 *  Function:     Board Support File for Arduino Mega 2560 +  RFMx LoRa.
 * 
 *  Copyright:    Copyright (c) 2021 Leonel Lopes Parente
 * 
 *  License:      MIT License. See accompanying LICENSE file.
 * 
 *  Author:       Leonel Lopes Parente
 *                Kevin Barrell
 * 
 *  Description:  This board has onboard USB (provided by the MCU).
 *                It supports automatic firmware upload and serial over USB.
 *                No onboard display. Optionally an external display can be connected.
 * 
 * 
 *                Connect DIO1 and optional display
 *                according to below connection details.
 * 
 *                CONNECTIONS AND PIN DEFINITIONS:
 * 
 *                Indentifiers between parentheses are defined in the board's 
 *                Board Support Package (BSP) which is part of the Arduino core. 
 * 
 *                Leds                GPIO 
 *                ----                ----        
 *                LED   <――――――――――>  13  (LED_BUILTIN, PIN_LED, PIN_LED_13) active-high
 *                LED2  <――――――――――>  25  (RX, PIN_LED2, PIN_LED_RXL)
 *                LED3  <――――――――――>  26  (TX, PIN_LED3, PIN_LED_TXL)
 * 
 *                I2C [display]       GPIO
 *                ---                 ----
 *                SDA   <――――――――――>  20  (SDA)
 *                SCL   <――――――――――>  21  (SCL)
 *
 *                SPI/LoRa            GPIO
 *                ---                 ----
 *                MOSI  <――――――――――>  51  (MOSI)
 *                MISO  <――――――――――>  50  (MISO)
 *                SCK   <――――――――――>  52  (SCK)
 *                NSS   <――――――――――>   10
 *                RST   <――――――――――>   9
 *                DIO0  <――――――――――>   2
 *                DIO1  <――――――――――>   6  
 *                DIO2  <――――――――――>   7  
 *
 *  Docs:         https://docs.platformio.org/en/latest/boards/atmelavr/megaatmega2560.html
 *                 
 *
 *  Identifiers:  LMIC-node
 *                    board-id:      megaatmmega2560
 *                PlatformIO
 *                    board:         megaatmmega2560
 *                    platform:      atmelavr
 *                Arduino
 *                    board:         ARDUINO_SAMD_FEATHER_M0
 *                    architecture:  ARDUINO_ARCH_SAMD
 * 
 ******************************************************************************/

#pragma once

#ifndef BSF_ARDUINO_MEGA_LORA_SHLD_H_
#define BSF_ARDUINO_MEGA_LORA_SHLD_H_

#include "LMIC-node.h"

#define DEVICEID_DEFAULT "megaatmega2560"  // Default deviceid value

// Wait for Serial
// Can be useful for boards with MCU with integrated USB support.
//#define WAITFOR_SERIAL_SECONDS_DEFAULT 10   // -1 waits indefinitely  

// LMIC Clock Error
// This is only needed for slower 8-bit MCUs (e.g. 8MHz ATmega328 and ATmega32u4).
// Value is defined in parts per million (of MAX_CLOCK_ERROR).
// #define LMIC_CLOCK_ERROR_PPM 0

// Pin mappings for LoRa tranceiver
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = { /*dio0*/ 2, /*dio1*/ 6, /*dio2*/ 7 }
#ifdef MCCI_LMIC
    ,
    .rxtx_rx_active = 0,
    .rssi_cal = 8,
    .spi_freq = 8000000     /* 8 MHz */
#endif    
};

#ifdef USE_SERIAL
    HardwareSerial& serial = Serial;
#endif    

#ifdef USE_LED
    EasyLed led(LED_BUILTIN, EasyLed::ActiveLevel::High);
#endif

#ifdef USE_DISPLAY
    // Create U8x8 instance for SSD1306 OLED display (no reset) using hardware I2C.
    U8X8_SSD1306_128X64_NONAME_HW_I2C display(/*rst*/ U8X8_PIN_NONE, /*scl*/ SCL, /*sda*/ SDA);
#endif


bool boardInit(InitType initType)
{
    // This function is used to perform board specific initializations.
    // Required as part of standard template.

    // InitType::Hardware        Must be called at start of setup() before anything else.
    // InitType::PostInitSerial  Must be called after initSerial() before other initializations.    

    bool success = true;
    switch (initType)
    {
        case InitType::Hardware:
            // Note: Serial port and display are not yet initialized and cannot be used use here.
            // No actions required for this board.
            break;

        case InitType::PostInitSerial:
            // Note: If enabled Serial port and display are already initialized here.
            // No actions required for this board.
            break;           
    }
    return success;
}


#endif  // BSF_ARDUINO_MEGA_LORA_SHLD_H_