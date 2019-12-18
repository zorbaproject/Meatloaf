/* 
 *  Portable EC and pH meter by 
 *  Luca Tringali - Gruppo Speleologico Talpe del Carso
 *  
 *  Based on this documentation:
 *  
 *  EC Module code:
 * https://wiki.dfrobot.com/Gravity__Analog_Electrical_Conductivity_Sensor___Meter_V2__K=1__SKU_DFR0300
 * Copyright   [DFRobot](http://www.dfrobot.com), 2018
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
 */

#include "DFRobot_EC.h"
#include <EEPROM.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <SD.h>

#define EC_PIN A1
#define ONE_WIRE_BUS 2 
const int chipSelect = 4;

bool datalog = false;

int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 3, 5, 6, 7);

OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

unsigned long logcount = 0;
float voltage,ecValue,temperature = 25;
DFRobot_EC ec;

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
    Serial.println("Card not found");
  }
}

void loop()
{
    static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U)  //time interval: 1s
    {
      timepoint = millis();
      voltage = analogRead(EC_PIN)/1024.0*5000*1000;   // read the voltage
      temperature = readTemperature();          // read your temperature sensor to execute temperature compensation
      ecValue =  ec.readEC(voltage,temperature);  // convert voltage to EC with temperature compensation
      /*Serial.print("temperature:");
      Serial.print(temperature,1);
      Serial.print("^C  EC [K=1]:");
      Serial.print(ecValue,2);
      Serial.println("uS/cm");*/
      lcd.setCursor(0,0);
      lcd.print("T = ");
      lcd.setCursor(4,0);
      lcd.print(temperature,2);
      lcd.setCursor(9,0);
      lcd.print("^C");
      lcd.setCursor(0,1);
      lcd.print("EC = ");
      lcd.setCursor(5,1);
      lcd.print(ecValue,0);
      lcd.setCursor(10,1);
      lcd.print("uS/cm");
      if (datalog) {
        File dataFile = SD.open("datalog.txt", FILE_WRITE);
        if (dataFile) {
          String tval = String(temperature, DEC);
          String cval = String(ecValue, DEC);
          dataFile.println(String(logcount)+String(",")+tval+String(",")+cval);
          dataFile.close();
          lcd.setCursor(13,0);
          lcd.print("LOG");
        } else {
          lcd.setCursor(0,0);
          lcd.print("error opening datalog.txt");
        }
        logcount++;
      }
      lcd_key = read_LCD_buttons();  // read the buttons
      if (lcd_key==btnSELECT) datalog=true;
    }
    ec.calibration(voltage,temperature);          // calibration process by Serail CMD
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
 adc_key_in = analogRead(0);      // read the value from the sensor
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
