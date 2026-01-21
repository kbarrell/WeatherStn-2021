/*
 * TimeRTC.ino
 * example code illustrating Time library with Real Time Clock SD2405.
 * 
 */

#include <Wire.h>  
#include <Time.h>
#include <SD2405RTC.h>

#define RTC_ADDR     0x32

tmElements_t tm;
const char* days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const char* months[] = {"January", "February", "March", "April", "May", "June", "July", "August","September", "October", "November", "December"};

void setup()  {
  Serial.begin(9600);
  while (!Serial) delay(10); // wait until Arduino Serial Monitor opens
  i2CReadDate();
  i2CEditTime();
}

void loop()
{
  i2CReadDate();
  dateTimeDisplay();
  delay(1000);
}

//Read the Real-time data from DS1307
void i2CReadDate(void)
{
  RTC.read(tm);
}

void i2CEditTime(void)
{
  
  Serial.print("The current date and time is: ");
  dateTimeDisplay();
  Serial.println("Please change to newline ending the settings on the lower right of the Serial Monitor");
  Serial.println("Would you like to set the date and time now? Y/N");
 
  while (!Serial.available()) delay(10);
  if (Serial.read() == 'y' || Serial.read() == 'Y') {
    Serial.read();
    i2CSetTime();
    i2CReadDate();
    Serial.print("The current date and time is now: ");
    dateTimeDisplay();
  }
}

// This set of codes is allows input of data
void i2CSetTime() {
  Serial.print("Please enter the current year, 00-99. - ");
  tm.Year = readByte();
  Serial.println(tm.Year);
  Serial.print("Please enter the current month, 1-12. - ");
  tm.Month = readByte();
  Serial.println(months[tm.Month-1]);
  Serial.print("Please enter the current day of the month, 1-31. - ");
  tm.Day = readByte();
  Serial.println(tm.Day);
  Serial.println("Please enter the current day of the week, 1-7.");
  Serial.print("1 Sun | 2 Mon | 3 Tues | 4 Weds | 5 Thu | 6 Fri | 7 Sat - ");
  tm.Wday = readByte();
  Serial.println(days[tm.Wday-1]);
  Serial.print("Please enter the current hour in 24hr format, 0-23. - ");
  tm.Hour = readByte();
  Serial.println(tm.Hour);
  Serial.print("Please enter the current minute, 0-59. - ");
  tm.Minute = readByte();
  Serial.println(tm.Minute);
  Serial.print("Please enter the current second, 0-59. - ");
  tm.Second = readByte();
  Serial.println(tm.Second);
  Serial.println("The data has been entered.");
  
  RTC.write(tm);
}

void dateTimeDisplay(void){
  Serial.print(days[tm.Wday-1]); //weekday
  Serial.print(" ");
  Serial.print(months[tm.Month-1]); //month name
  Serial.print(" ");
  Serial.print(tm.Day); //day
  Serial.print(", ");
  Serial.print(tmYearToCalendar(tm.Year));
  Serial.print(" - ");
  printTimePrefix();
  Serial.println(); 
}

void printTimePrefix(void){
  printDigits(tm.Day);
  Serial.print("/");
  printDigits(tm.Month);
  Serial.print("/");
  Serial.print(tmYearToCalendar(tm.Year));
  Serial.print(" ");
  printDigits(tm.Hour);
  Serial.print(":");
  printDigits(tm.Minute);
  Serial.print(":");
  printDigits(tm.Second);
}

void printDigits(int digits){
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

byte readByte() {
  while (!Serial.available()) delay(10);
  byte reading = 0;
  byte incomingByte = Serial.read();
  while (incomingByte != '\n') {
    if (incomingByte >= '0' && incomingByte <= '9')
      reading = reading * 10 + (incomingByte - '0');
    else;
    incomingByte = Serial.read();
  }
  Serial.flush();
  return reading;
}
