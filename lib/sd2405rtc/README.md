SD2405 Real-Time Clock Library for Arduino
=======================================================

This library is similar to popular DS1307RTC library and based on DFRobot Wiki example :
http://www.dfrobot.com/wiki/index.php/SD2405_Real_time_clock_Module_(SKU:TOY0021)
The SD2405RTC library is provided to demonstrate the Arduino Time library.
It provides a simple interface to the SD2405AL and facilitate use of the DFRobot SD2405 RTC Module:
DFRobots SD2405 RTC Module supplies the time data via I2C interface and the time format is configurable to be switched between 12 hours and 24 hours. However, another good feature is the time alarm function via interrupt event output.
http://www.dfrobot.com/index.php?route=product/product&product_id=842


The SD2405AL is an extremely accurate I2C real-time clock (RTC) with crystal
compensation, inner chargeable battery. The SD2405AL is available in industrial
temperature ranges.
The SD2405AL is dual power supply system. When the primary power supply
goes down to an assigned value or resumes from low power, the system can switch
between the primary power supply and battery automatically.
The SD2405AL can generates various periodic interrupt clock pulses lasting for
long period (one year), and three alarm interrupts can be made by year,month,date,days
of the week, hours, and minutes, seconds. It also provides a selectable 32.768KHz~1Hz
clock output for an external MCU. The product incorporates a time trimming circuit that
adjusts the clock with higher precision by adjusting any errors in crystal oscillator
frequencies based on signals from the CPU. A 12-bytes general SRAM is implemented in
the SD2405AL.

Installation
------------
Download the directory "SD2405RTC » and move it into the "libraries"
directory inside your sketchbook directory, then restart the Arduino
IDE. You will then see it listed under File->Examples->SD2405RTC.

Usage
-----
The library is instantiated as an object with methods provided to read
and write datetime to RTC chip SD2405.
It also provides the basic alarm interrupt functions.

    #include <Wire.h>
    #include <SD2405RTC.h>
    
    #define RTC_ADDR 0x32

    tmElements_t tm;
    RTC.read(tm);

A complete example program is included with the library and can be accessed
from the File->Examples->SD2405RTC menu.

### get() ###

Aquire data from buffer and convert to time_t

Example:

    setSyncProvider(RTC.get);

### set(time_t t) ###

Convert a time_t element to tmElements_t and write it to the RTC

Example:

    RTC.set(t);

### read(tmElements_t &tm) ###

Aquire datetime data from the RTC chip in BCD format

Example:

    RTC.read(tm);

### write(tmElements_t &tm) ###

Write datetime data to the RTC chip in BCD format

Example:

    RTC.write(tm);

### readAlarm(tmElements_t &al) ###

Aquire alarm data from the RTC chip in BCD format

Example:

    RTC.readAlarm(al);

### writeAlarm(tmElements_t &al, boolean periodic, boolean dateAlarm) ###

Write alarm data to the RTC chip in BCD format

Example:

    RTC.writeAlarm(al, false, true); // Date Alarm in single event mode

### readRegisters(int nb) ###

Print the RTC registers values in different formats (for debugging purpose)

Example:

    RTC.readRegisters(20); // RTC registers, Time Alarm registers and Control registers


Copyright (c) 2015 Julien Gautier electronics@netgrowing.com / http://www.netgrowing.com
