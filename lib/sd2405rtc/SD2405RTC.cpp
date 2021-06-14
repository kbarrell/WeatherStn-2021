/*
 * SD2405RTC.h - library for i2c SD2405 Real Time Clock
 *
 * Copyright (c) Netgrowing 2015
 * Written by Julien Gautier for Netgrowing Electronics.
 * 
 * This library is intended to be uses with Arduino Time.h library functions
 * 
 * The library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 * 
 * 2015/01/30 - Initial release: Real-Time Clock Functions
 * 2015/02/03 - Alarm Interrupt Functions
 */

#include "Arduino.h"
#include <Wire.h>
#include "SD2405RTC.h"

#define SD2405_ADDR 0x32

SD2405RTC::SD2405RTC()
{
  Wire.begin();
}
  
// PUBLIC FUNCTIONS

time_t SD2405RTC::get()   // Aquire data from buffer and convert to time_t
{
  tmElements_t tm;
  read(tm);
  return(makeTime(tm));
}

void  SD2405RTC::set(time_t t)
{
  tmElements_t tm;
  breakTime(t, tm);
  write(tm); 
}

// Aquire datetime data from the RTC chip in BCD format
void SD2405RTC::read( tmElements_t &tm)
{
  unsigned char date[7];
  unsigned char n=0;
  unsigned char i;
 
  Wire.requestFrom(SD2405_ADDR,tmNbrFields);
  while(Wire.available())
  {  
    date[n++]=Wire.read();
  }
  delayMicroseconds(1);
  Wire.endTransmission();
  for(i=0;i<7;i++)
  {
    if(i==2) {
      date[i]=(date[i]&0x7f); //clear the hour's highest bit 12_/24 ; 0x7F is bcd 0111-1111
    }
    //date[i]=(((date[i]&0xf0)>>4)*10)+(date[i]&0x0f); // ; 0xF0 is bcd 1111-0000 ; 0x0f is bcd 0000-1111
    date[i]=bcd2dec(date[i]);
  }
  tm.Second = date[0];
  tm.Minute = date[1];
  tm.Hour = date[2];
  tm.Wday = date[3];
  tm.Day = date[4];
  tm.Month = date[5];
  //tm.Year = date[6];            // Number of years since 2K
  tm.Year = y2kYearToTm(date[6]); // We add 30 years to get time from 1970
}

// Write datetime data to the RTC chip in BCD format
void SD2405RTC::write(tmElements_t &tm)
{
  enableWrite();
  Wire.beginTransmission(SD2405_ADDR);
  Wire.write(byte(0));                  // reset register pointer  
  Wire.write(dec2bcd(tm.Second)) ;   
  Wire.write(dec2bcd(tm.Minute));
  Wire.write(dec2bcd(tm.Hour+80));      // +80: sets 24 hours format
  Wire.write(dec2bcd(tm.Wday-1));       // days values come from 0 to 6: Sunday, Monday, Tuesday, Wednesday, Thursday, Friday, Saturday
  Wire.write(dec2bcd(tm.Day));
  Wire.write(dec2bcd(tm.Month));
  Wire.write(dec2bcd(tm.Year));
  Wire.endTransmission();
  Wire.beginTransmission(SD2405_ADDR);
  Wire.write(0x12);                     // set the address for writing as 12H : Time Trimming Register
  Wire.write(byte(0));                  // Counts will not change when (F6, F5, F4, F3, F2, F1, F0) are set to (*, 0, 0, 0, 0, 0, *).
  Wire.endTransmission();
  disableWrite(false);
}

// Aquire alarm data from the RTC chip in BCD format
void SD2405RTC::readAlarm( tmElements_t &al)
{
  
  unsigned char alarm[20];
  unsigned char n=0;
  unsigned char i;
 
  Wire.requestFrom(SD2405_ADDR,20);
  while(Wire.available())
  {
    alarm[n++]=Wire.read();
  }
  delayMicroseconds(1);
  Wire.endTransmission();
  
  for(i=0;i<20;i++)
  {
    alarm[i]=bcd2dec(alarm[i]);
  }
  
  al.Second = alarm[7];
  al.Minute = alarm[8];
  al.Hour = alarm[9];
  al.Wday = alarm[10];
  al.Day = alarm[11];
  al.Month = alarm[12];
  al.Year = y2kYearToTm(alarm[13]);
}

// Write alarm data to the RTC chip in BCD format
void SD2405RTC::writeAlarm(tmElements_t &al, boolean periodic, boolean dateAlarm)
{
  enableWrite();
  Wire.beginTransmission(SD2405_ADDR);
  Wire.write(0x07);                // Set the address for writing as 07H
  Wire.write(dec2bcd(al.Second));  // 07H Alarm Second
  Wire.write(dec2bcd(al.Minute));  // 08H Alarm Minute
  Wire.write(dec2bcd(al.Hour));    // 09H Alarm Hour
  if (dateAlarm) {
    Wire.write(0b00000000);        // 0AH Alarm Week : This is not the name of the day, but the days the alarm will be enabled ;
    Wire.write(dec2bcd(al.Day));   // 0BH Alarm Day
    Wire.write(dec2bcd(al.Month)); // 0CH Alarm Month
    Wire.write(dec2bcd(al.Year));  // 0DH Alarm Year
    Wire.write(0b01110111);        // 0EH Disable Week Alarm ; Alarm will be on when reaching the defined date.
  } else {
    Wire.write(al.Wday);           // 0AH Alarm Week : This is not the name of the day, but the days the alarm will be enabled ; 0b01111111 : Each days of the week
    Wire.write(0b00000000);        // When the week alarm and the date alarm are
    Wire.write(0b00000000);        // both enable at the same time, only the date
    Wire.write(0b00000000);        // alarm is valid and the week alarm is invalid.
    Wire.write(0b00001111);        // 0EH Enable Alarm: Week / Hour / Minute / Second ; This is the week alarm: it will be enable on some days at a defined time ; 0EH=0b00001111
  }
  Wire.write(0b10000100);          // 0FH WRTC3=1 0 INTAF=0 INTDF=0 0 WRTC2=1 0 RTCF=0
  if (periodic) {                  // If IM=0 this is a single event
    Wire.write(0b11010010);        // 10H WRTC1=1 IM=1 INTS1=0 INTS0=1 FOBAT=0 INTDE=0 INTAE=1 INTFE=0
  } else {                         // If IM=1 this is a periodic event
    Wire.write(0b10010010);        // 10H WRTC1=1 IM=0 INTS1=0 INTS0=1 FOBAT=0 INTDE=0 INTAE=1 INTFE=0
  }
  Wire.endTransmission();
  
  disableWrite(true);
  al.Year = y2kYearToTm(al.Year);
}

// Write interrupt control register for Frequency Interrupt
void SD2405RTC::writeFreqInt(byte frequency)
{
	enableWrite();
	Wire.beginTransmission(SD2405_ADDR);
	Wire.write(0x10);				// Set the address for writing as 10H
	Wire.write(0b10101001);			// 10H WRTC1=1 IM=0 INTS1=1 INTS0=0 FOBAT=1 INTDE=0 INTAE=0 INTFE=1
	Wire.write(0x0F & frequency); 	// 11H Set low-order bits FS3,FS2,FS1,FS0 to frequency selection
	Wire.endTransmission();
	disableWrite(true);
}
	

// Print the RTC registers values in differents formats (for debugging purpose).
void SD2405RTC::readRegisters(int nb)
{
  unsigned char data[nb];
  unsigned char n=0;
  unsigned char i;
 
  Wire.requestFrom(SD2405_ADDR,nb);
  while(Wire.available())
  {
    data[n++]=Wire.read();
  }
  delayMicroseconds(1);
  Wire.endTransmission();
  Serial.println("RTC Registers Values: ");
  Serial.println("-------------------------");
  Serial.println("Add.\tB2D \tDEC \tBIN");
  Serial.println("-------------------------");
  for(i=0;i<nb;i++)
  {
    if ((i==7)|(i==15)|(i==20)) {
      Serial.println("-------------------------");
    }
    //Serial.print("Add. ");
    if (i<16) {
      Serial.print("0");
    }
    Serial.print(i, HEX);
    Serial.print("H \t");
    //Serial.print(data[i]);
    if (bcd2dec(data[i])<10) {
      Serial.print("  ");
    } else if (bcd2dec(data[i])<100) {
      Serial.print(" ");
    }
    Serial.print(bcd2dec(data[i]));
    Serial.print(" \t");
    Serial.print(data[i]);
    Serial.print(" \t");
    for (unsigned int mask = 0x80; mask; mask >>= 1) {
      Serial.print(mask&data[i]?'1':'0');
    }
    Serial.println("B");
  }
  Serial.println("-------------------------");
}

// PRIVATE FUNCTIONS

// Convert Decimal to Binary Coded Decimal (BCD)
uint8_t SD2405RTC::dec2bcd(uint8_t num)
{
  return ((num/10 * 16) + (num % 10));
}

// Convert Binary Coded Decimal (BCD) to Decimal
uint8_t SD2405RTC::bcd2dec(uint8_t num)
{
  return ((num/16 * 10) + (num % 16));
}

//Enable writing to SD2405
void SD2405RTC::enableWrite(void)
{
  Wire.beginTransmission(SD2405_ADDR);
  Wire.write(0x10);       // Set the address for writing as 10H
  Wire.write(0x80);       // Set WRTC1=1
  Wire.endTransmission();
  
  Wire.beginTransmission(SD2405_ADDR);    
  Wire.write(0x0F);       // Set the address for writing as OFH
  Wire.write(0x84);       // Set WRTC2=1,WRTC3=1      
  Wire.endTransmission();
}

//Disable writing to SD2405
void SD2405RTC::disableWrite(boolean alarm)
{
  Wire.beginTransmission(SD2405_ADDR);   
  Wire.write(0x0F);       // Set the address for writing as OFH          
  Wire.write(byte(0));    // Set WRTC2=0,WRTC3=0
  if (alarm) {
    Wire.endTransmission();
    Wire.beginTransmission(SD2405_ADDR);
    Wire.write(0x10);     // Set the address for writing as 10H
  }
  Wire.write(byte(0));    // Set WRTC1=0  
  Wire.endTransmission();
}

//Enable Alarm Interrupt
void SD2405RTC::enableAlarm(void)
{
  
}

//Disable Alarm Interrupt
void SD2405RTC::disableAlarm(void)
{
  
}

// create an instance for the user
SD2405RTC RTC = SD2405RTC();

