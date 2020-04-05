/* 
 *  Portable EC and pH meter by 
 *  Luca Tringali - Gruppo Speleologico Talpe del Carso
 *  
 *  Based on this documentation:
 *  
 *  EC Module code:
 * https://wiki.dfrobot.com/Gravity__Analog_Electrical_Conductivity_Sensor___Meter_V2__K=1__SKU_DFR0300
 * Copyright   [DFRobot](http://www.dfrobot.com), 2018
 * Version 1: https://www.dfrobot.com/wiki/index.php/Analog_EC_Meter_SKU:DFR0300
 * 
 * LCD panel and buttons:
 * https://wiki.dfrobot.com/Arduino_LCD_KeyPad_Shield__SKU__DFR0009_
 *Mark Bramwell, July 2010

 * SD card logging by Tom Igoe
 * 
 ** SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4
 *  
 *  
 * Requirements:
 * EEPROM by Arduino
 * Wire by Arduino
 * SD by Arduino
 * SPI by Arduino
 * DFRobot_EC-master on https://github.com/DFRobot/DFRobot_EC
 * DallasTemperature by Miles Burton et al
 * OneWire by Jim Studt et al
 * LiquidCrystal by Arduino, Adafruit
 * RTClib by Adafruit
 * 
 */

#include "DFRobot_EC.h"
#include <EEPROM.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"

#define buttonsPin A0
#define EC_PIN A1
#define ONE_WIRE_BUS 2 
#define chipSelect 10

bool datalog = false;
bool menu1 = false;

int lcd_key     = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

#define KVALUEADDR 0x0A

OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

float voltage,ecValue,temperature = 25;
const float defaultTemp = 20.0;

unsigned long logcount = 0;
int ecOldLen = 10;
float* ecOld;
int cycleiter = 0;

DFRobot_EC ec;

RTC_DS1307 rtc;


void setup()
{
  Serial.begin(9600);
  lcd.begin(16, 2);              // start the library
  lcd.setCursor(0,0);
  lcd.print("Warm up..."); // print a simple message
  ec.begin();
  sensors.begin();
  //pinMode(10, OUTPUT); 
  if (!SD.begin(chipSelect)) {
    lcd.setCursor(0,0);
    lcd.setCursor(12,0);
    lcd.print("noSD");
    delay(2000);
  }
  resizeEcOld();

  rtc.begin();

  //Set time at compilation time, if not already set
  DateTime comp = DateTime(F(__DATE__), F(__TIME__));
  DateTime now = rtc.now();
  if (comp.unixtime()> now.unixtime()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void loop()
{
    static unsigned long timepoint = millis();
    if (menu1) {
      menu();
    } else if (millis()-timepoint>1000U)  //time interval: 1s
    {
      timepoint = millis();
      voltage = analogRead(EC_PIN)/1024.0*5000;   // read the voltage
      temperature = readTemperature();          // read your temperature sensor to execute temperature compensation
      if (temperature == -127) {
        temperature = defaultTemp;
      }
      ecValue =  ec.readEC(voltage,temperature);  // convert voltage to EC with temperature compensation
      ecValue = ecValue*1000; //looking for microsiemens

      //get a mean value
      for (int i = 0; i < ecOldLen; i++) {
        if (i==cycleiter || ecOld[i]<10) ecOld[i] = ecValue;
      }
      cycleiter++;
      if (cycleiter>=ecOldLen) cycleiter = 0;
      float ecT = 0;
      for (int i = 0; i < ecOldLen; i++) {
        ecT = ecT + ecOld[i];
      }
      ecT = ecT/ecOldLen;
      lcd.setCursor(0,0);
      lcd.print("T = ");
      lcd.setCursor(4,0);
      lcd.print(temperature,2);
      lcd.setCursor(9,0);
      lcd.print("^C");
      lcd.setCursor(0,1);
      lcd.print("EC = ");
      lcd.setCursor(5,1);
      lcd.print(ecT,0);
      //lcd.print(ecValue,0);
      lcd.setCursor(10,1);
      lcd.print("uS/cm");
      if (datalog) {
        File dataFile = SD.open("datalog.txt", FILE_WRITE);
        if (dataFile) {
          dataFile.print(String(logcount));
          dataFile.print(",");
          dataFile.print(String(rtc.now().timestamp(DateTime::TIMESTAMP_FULL)));
          dataFile.print(",");
          dataFile.print(String(temperature, DEC));
          dataFile.print(",");
          dataFile.println(String(ecT, DEC));
          dataFile.close();
          lcd.setCursor(12,0);
          lcd.print(" LOG");
        } else {
          lcd.setCursor(0,0);
          lcd.print("Error on SD      ");
        }
        logcount++;
      } else {
        lcd.setCursor(11,0);
        lcd.print("noLOG");
      }
      lcd_key = read_LCD_buttons();  // read the buttons
      if (lcd_key==btnSELECT) menu1=true;
    }
}

float readTemperature()
{
  //add your code here to get the temperature from your temperature sensor
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}


// read the buttons
int read_LCD_buttons()
{
 //int adc_key_in  = 0;
 int adc_key_in = analogRead(buttonsPin);      // read the value from the sensor
 // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
 // we add approx 50 to those values and check to see if we are close
 if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
 // For V1.1 use this threshold
 if (adc_key_in < 50)   return btnRIGHT;
 if (adc_key_in < 250)  return btnUP;
 if (adc_key_in < 450)  return btnDOWN;
 if (adc_key_in < 650)  return btnLEFT;
 if (adc_key_in < 850)  return btnSELECT;

 // For V1.0 comment the other threshold and use the one below:
/*
 if (adc_key_in < 50)   return btnRIGHT;
 if (adc_key_in < 195)  return btnUP;
 if (adc_key_in < 380)  return btnDOWN;
 if (adc_key_in < 555)  return btnLEFT;
 if (adc_key_in < 790)  return btnSELECT;
*/


 return btnNONE;  // when all others fail, return this...
}

void menu()
{
  lcd.setCursor(0,0);
  lcd.print("Su e giu, select");
  lcd.setCursor(0,1);
  lcd.print("                    ");
  int pos = 0;
  int maxoptions = 6;
  delay(1000);
  while (menu1) {
    lcd_key = read_LCD_buttons();  // read the buttons
    if (lcd_key==btnUP) pos+=1;
    if (lcd_key==btnDOWN) pos-=1;
    if (abs(pos)%maxoptions == 0) {
      lcd.setCursor(0,1);
      lcd.print("Scrivi su SD      ");
      if (lcd_key==btnSELECT) {
        if (datalog) {
          datalog=false;
        } else if (!datalog) {
          datalog=true;
        }
      }
      if (datalog) {
        lcd.setCursor(14,1);
        lcd.print("*");
      } else {
        lcd.setCursor(13,1);
        lcd.print("  ");
      }
    }
    if (abs(pos)%maxoptions == 1) {
      lcd.setCursor(0,1);
      lcd.print("Media su       s");
      if (lcd_key==btnLEFT) {
        ecOldLen--;
        if (ecOldLen <1) ecOldLen = 1;
        resizeEcOld();
      }
      if (lcd_key==btnRIGHT) {
        ecOldLen++;
        if (ecOldLen >30) ecOldLen = 30;
        resizeEcOld();
      }
      lcd.setCursor(13,1);
      if (ecOldLen<10) lcd.setCursor(14,1);
      lcd.print(ecOldLen);
    }
    if (abs(pos)%maxoptions == 2) {
      DateTime now = rtc.now();
      lcd.setCursor(0,1);
      //lcd.print(now.timestamp(DateTime::TIMESTAMP_FULL));
      lcd.print(now.timestamp(DateTime::TIMESTAMP_TIME));
      //lcd.print(now.timestamp(DateTime::TIMESTAMP_DATE));
      lcd.setCursor(12,1);
      lcd.print(now.year());
      lcd.setCursor(10,1);
      if (now.day()<10) lcd.print("0");
      lcd.print(now.day());
      lcd.setCursor(12,1);
      if (now.month()<10) lcd.print("0");
      lcd.print(now.month());
      if (lcd_key==btnSELECT) setDateTime();
    }
    if (abs(pos)%maxoptions == 3) {
      lcd.setCursor(0,1);
      lcd.print("Calibrazione      ");
      if (lcd_key==btnSELECT) calibrate();
    }
    if (abs(pos)%maxoptions == 4) {
      lcd.setCursor(0,1);
      lcd.print("Reset memoria      ");
      if (lcd_key==btnSELECT) resetMemory();
    }
    if (abs(pos)%maxoptions == 5) {
      lcd.setCursor(0,1);
      lcd.print("Esci               ");
      if (lcd_key==btnSELECT) menu1=false;
    }
    delay(200);
  }
}

void resizeEcOld() 
{
  if (ecOld != 0) {
    delete [] ecOld;
  }
  ecOld = new float [ecOldLen];
  for (int i = 0; i < ecOldLen; i++) {
    ecOld[i] = 0.0;
  }
}

void setDateTime() 
{
  // December 16, 2020 at 10pm you would call:
  rtc.adjust(DateTime(2019, 12, 16, 22, 0, 0));
}

void resetMemory() 
{
  for(byte i = 0;i< 8; i   ){
    //EEPROM.write(KVALUEADDR i, 0xFF);
  }
}

void calibrate() 
{
  bool active = true;
  while (active) {
    static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U)  //time interval: 1s
    {
      timepoint = millis();
      voltage = analogRead(EC_PIN)/1024.0*5000;  // read the voltage
      temperature = readTemperature();          // read your temperature sensor to execute temperature compensation
      if (temperature == -127) {
        temperature = defaultTemp;
      }
      ecValue =  ec.readEC(voltage,temperature);  // convert voltage to EC with temperature compensation
      Serial.print("temperature:");
      Serial.print(temperature,1);
      Serial.print("^C  EC:");
      Serial.print(ecValue,2);
      Serial.println("ms/cm");
    }
    ec.calibration(voltage,temperature);  // calibration process by Serail CMD
  }
}

