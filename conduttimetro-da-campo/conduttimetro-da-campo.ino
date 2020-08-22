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
 * pH Module code:
 * https://wiki.dfrobot.com/Gravity__Analog_pH_Sensor_Meter_Kit_V2_SKU_SEN0161-V2
 * Copyright   [DFRobot](http://www.dfrobot.com), 2018
 * Version 1: https://wiki.dfrobot.com/PH_meter_SKU__SEN0161_
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
 ** CS - pin 10
 *  
 *  
 * Requirements:
 * EEPROM by Arduino
 * Wire by Arduino
 * SD_MEGA by Arduino, Adafruit, Tringali
 * SPI by Arduino
 * DFRobot_EC-master on https://github.com/DFRobot/DFRobot_EC
 * DFRobot_PH-master on https://github.com/DFRobot/DFRobot_PH
 * DallasTemperature by Miles Burton et al
 * OneWire by Jim Studt et al
 * LiquidCrystal by Arduino, Adafruit
 * RTClib by Adafruit
 * Keypad library by Mark Stanley and Alexander Brevig
 * 
 */

#include "DFRobot_EC.h"
#include "DFRobot_PH.h"
#include <EEPROM.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
//#include <LiquidCrystal.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <SD_MEGA.h>
#include "RTClib.h"
//#include <RTClib_Tiny.h>
//#include <SoftI2CMaster.h> 

#include <Keypad.h>

#define buttonsPin A0
#define EC_PIN A1
#define PH_PIN A2
#define ONE_WIRE_BUS 2 

#define chipSelect 10
#define MOSIpin 11
#define MISOpin 12
#define CLKpin 13

bool datalog = false;
bool menu1 = false;
int mode = 1;  //mode 1 reads EC, mode 2 reads PH

// select the pins used on the LCD panel
//LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

#define KVALUEADDR 0x0A

OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

float voltageC,voltageP,ecValue,phValue,temperature = 25;
const float defaultTemp = 20.0;

unsigned long logcount = 0;
int allOldLen = 10;
float* ecOld;
float* phOld;
int cycleiter = 0;

DFRobot_EC ec;
DFRobot_PH ph;

//SoftI2CMaster softi2c=SoftI2CMaster( A5, A4 ); //sclPin=5, sdaPin=4
RTC_DS1307 rtc;


// https://cdn.instructables.com/FUL/940T/ITCHJSHC/FUL940TITCHJSHC.LARGE.jpg
const byte ROWS = 5; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
  {'C','P','#','*'},
  {'1','2','3','U'},
  {'4','5','6','D'},
  {'7','8','9','X'},
  {'L','0','R','E'}
};
byte rowPins[ROWS] = {38, 36, 34, 32, 30}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {22, 24, 26, 28}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );


void setup()
{
  Serial.begin(9600);
  //lcd.begin(16, 2);              // start the library
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Warm up..."); // print a simple message
  ec.begin();
  ph.begin();
  sensors.begin();
  
  //pinMode(chipSelect, OUTPUT); 
  if (!SD.begin(chipSelect, MOSIpin, MISOpin, CLKpin)) {
    lcd.setCursor(0,0);
    lcd.setCursor(12,0);
    lcd.print("noSD");
    delay(2000);
  }
  resizeAllOld();

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
      voltageC = analogRead(EC_PIN)/1024.0*5000;   // read the voltage
      voltageP = analogRead(PH_PIN)/1024.0*5000;
      temperature = readTemperature();          // read your temperature sensor to execute temperature compensation
      if (temperature == -127) {
        temperature = defaultTemp;
      }
      ecValue =  ec.readEC(voltageC,temperature);  // convert voltage to EC with temperature compensation
      ecValue = ecValue*1000; //looking for microsiemens
      phValue = ph.readPH(voltageP,temperature);
      
      //get a mean value
      for (int i = 0; i < allOldLen; i++) {
        if (i==cycleiter || ecOld[i]<10) ecOld[i] = ecValue;
      }
      //cycleiter++;
      //if (cycleiter>=allOldLen) cycleiter = 0;
      float ecT = 0;
      for (int i = 0; i < allOldLen; i++) {
        ecT = ecT + ecOld[i];
      }
      ecT = ecT/allOldLen;

      //get a mean value
      for (int i = 0; i < allOldLen; i++) {
        if (i==cycleiter || phOld[i]<10) phOld[i] = phValue;
      }
      float phT = 0;
      for (int i = 0; i < allOldLen; i++) {
        phT = phT + phOld[i];
      }
      phT = phT/allOldLen;
      cycleiter++;
      if (cycleiter>=allOldLen) cycleiter = 0;
      
      lcd.setCursor(0,0);
      lcd.print("T = ");
      lcd.setCursor(4,0);
      lcd.print(temperature,2);
      lcd.setCursor(9,0);
      lcd.print("^C");
      lcd.setCursor(0,1);
      lcd.print("                 ");
      if (mode==1) {
        lcd.setCursor(0,1);
        lcd.print("EC = ");
        lcd.setCursor(5,1);
        lcd.print(ecT,0);
        lcd.setCursor(10,1);
        lcd.print("uS/cm");
      }
      if (mode==2) {
        lcd.setCursor(0,1);
        lcd.print("pH = ");
        lcd.setCursor(5,1);
        lcd.print(phT,2);
      }
      if (datalog) {
        File dataFile = SD.open("datalog.txt", FILE_WRITE);
        if (dataFile) {
          dataFile.print(String(logcount));
          dataFile.print(",");
          //dataFile.print(String(rtc.now().timestamp(DateTime::TIMESTAMP_FULL)));
          dataFile.print(timestamp_full(rtc.now()));
          dataFile.print(",");
          dataFile.print(String(temperature, DEC));
          dataFile.print(",");
          dataFile.print(String(ecT, DEC));
          dataFile.print(",");
          dataFile.println(String(phT, 2));
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
      char key = keypad.getKey();
      if (key=='E') menu1=true;
      if (key=='C') mode=1;
      if (key=='P') mode=2;
      /*if (key){
        Serial.println(key);
      }*/
    }
}

float readTemperature()
{
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

String timestamp_full(DateTime mynow) {
  String mytimestamp = "";
  mytimestamp += String(mynow.hour());
  mytimestamp += String(":");
  mytimestamp += String(mynow.minute());
  mytimestamp += String(":");
  mytimestamp += String(mynow.second());
  mytimestamp += String(" ");
  mytimestamp += String(mynow.day());
  mytimestamp += String("/");
  mytimestamp += String(mynow.month());
  mytimestamp += String("/");
  mytimestamp += String(mynow.year());
  return mytimestamp;
}

String timestamp_time(DateTime mynow) {
  String mytimestamp = "";
  mytimestamp += String(mynow.hour());
  mytimestamp += String(":");
  mytimestamp += String(mynow.minute());
  mytimestamp += String(":");
  mytimestamp += String(mynow.second());
  return mytimestamp;
}

void clearScreen() 
{
  lcd.setCursor(0,0);
  lcd.print("                    ");
  lcd.setCursor(0,1);
  lcd.print("                    ");
  delay(1);
}

void menu()
{
  clearScreen();
  lcd.setCursor(0,0);
  lcd.print("Scorri il menu ");
  lcd.setCursor(0,1);
  lcd.print("premendo su/giu");
  int pos = 0;
  int maxoptions = 6;
  delay(1500);
  while (menu1) {
    char key = keypad.getKey();
    if (key=='U') pos+=1;
    if (key=='D') pos-=1;
    if (key=='X') {
      pos=5;
      menu1=false;
    }
    if (abs(pos)%maxoptions == 0) {
      clearScreen();
      lcd.setCursor(0,0);
      lcd.print("Premi Enter");
      lcd.setCursor(0,1);
      lcd.print("Scrivi su SD      ");
      if (key=='E') {
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
      clearScreen();
      lcd.setCursor(0,0);
      lcd.print("Premi < >");
      lcd.setCursor(0,1);
      lcd.print("Media su       s");
      if (key=='L') {
        allOldLen--;
        if (allOldLen <1) allOldLen = 1;
        resizeAllOld();
      }
      if (key=='R') {
        allOldLen++;
        if (allOldLen >30) allOldLen = 30;
        resizeAllOld();
      }
      lcd.setCursor(13,1);
      if (allOldLen<10) lcd.setCursor(14,1);
      lcd.print(allOldLen);
    }
    if (abs(pos)%maxoptions == 2) {
      clearScreen();
      lcd.setCursor(0,0);
      lcd.print("Premi Enter");
      DateTime now = rtc.now();
      //time
      lcd.setCursor(0,1);
      if (now.hour()<10) lcd.print("0");
      lcd.print(now.hour());
      lcd.print(":");
      lcd.setCursor(3,1);
      if (now.minute()<10) lcd.print("0");
      lcd.print(now.minute());
      lcd.print(":");
      lcd.setCursor(6,1);
      if (now.second()<10) lcd.print("0");
      lcd.print(now.second());
      //date
      lcd.setCursor(12,1);
      lcd.print(now.year());
      lcd.setCursor(10,1);
      if (now.day()<10) lcd.print("0");
      lcd.print(now.day());
      lcd.setCursor(12,1);
      if (now.month()<10) lcd.print("0");
      lcd.print(now.month());
      if (key=='E') setDateTime();
    }
    if (abs(pos)%maxoptions == 3) {
      clearScreen();
      lcd.setCursor(0,0);
      lcd.print("Premi Enter");
      lcd.setCursor(0,1);
      lcd.print("Calibrazione      ");
      if (key=='E') calibrate();
    }
    if (abs(pos)%maxoptions == 4) {
      clearScreen();
      lcd.setCursor(0,0);
      lcd.print("Premi Enter");
      lcd.setCursor(0,1);
      lcd.print("Reset memoria      ");
      if (key=='E') resetMemory();
    }
    if (abs(pos)%maxoptions == 5) {
      clearScreen();
      lcd.setCursor(0,0);
      lcd.print("Premi Enter");
      lcd.setCursor(0,1);
      lcd.print("Esci               ");
      if (key=='E') menu1=false;
    }
    delay(200);
  }
}

void resizeAllOld() 
{
  if (ecOld != 0) {
    delete [] ecOld;
  }
  ecOld = new float [allOldLen];
  for (int i = 0; i < allOldLen; i++) {
    ecOld[i] = 0.0;
  }

  if (phOld != 0) {
    delete [] phOld;
  }
  phOld = new float [allOldLen];
  for (int i = 0; i < allOldLen; i++) {
    phOld[i] = 0.0;
  }
}

void setDateTime() 
{
  clearScreen();
  lcd.setCursor(0,0);
  lcd.print("Digita ora data");
  Serial.println(rtc.now().unixtime());
  // December 16, 2020 at 10pm you would call:
  //rtc.adjust(DateTime(2019, 12, 16, 22, 0, 0));
  int year = 2000;
  int month = 0;
  int day = 0;
  int hour = 0;
  int minute = 0;
  int second = 0;
  int pos = 0;
  while (true) {
    char key = keypad.getKey();
    if (key=='X') return;
    int num = 0;
    if (key=='0') num = 0;
    if (key=='1') num = 1;
    if (key=='2') num = 2;
    if (key=='3') num = 3;
    if (key=='4') num = 4;
    if (key=='5') num = 5;
    if (key=='6') num = 6;
    if (key=='7') num = 7;
    if (key=='8') num = 8;
    if (key=='9') num = 9;
    
    if (pos==0) hour=num*10;
    if (pos==1) hour=hour+num;
    if (pos==2) minute=num*10;
    if (pos==3) minute=minute+num;
    if (pos==4) second=num*10;
    if (pos==5) second=second+num;

    if (pos==6) day=num*10;
    if (pos==7) day=day+num;
    if (pos==8) month=num*10;
    if (pos==9) month=month+num;
    if (pos==10) year=year+(num*10);
    if (pos==11) year=year+num;
    if (key=='0' || key=='1' || key=='2' || key=='3' || key=='4' || key=='5' || key=='6' || key=='7' || key=='8' || key=='9') pos++;
    if (key=='E') {
      rtc.adjust(DateTime(year, month, day, hour, minute, second));
      return;
    }
    //TODO: use left right arrow to move between pos
    
    //time
    lcd.setCursor(0,1);
    if (hour<10) lcd.print("0");
    lcd.print(hour);
    lcd.print(":");
    lcd.setCursor(3,1);
    if (minute<10) lcd.print("0");
    lcd.print(minute);
    lcd.print(":");
    lcd.setCursor(6,1);
    if (second<10) lcd.print("0");
    lcd.print(second);
    //date
    lcd.setCursor(12,1);
    lcd.print(year);
    lcd.setCursor(10,1);
    if (day<10) lcd.print("0");
    lcd.print(day);
    lcd.setCursor(12,1);
    if (month<10) lcd.print("0");
    lcd.print(month);
    
    int curpos = pos;
    if (pos>1) curpos=curpos+1;
    if (pos>3) curpos=curpos+1;
    if (pos>5) curpos=curpos+2;
    lcd.setCursor(curpos,1);
    lcd.print("_");
    delay(200);
  }
}

void resetMemory() 
{
  lcd.setCursor(0,0);
  lcd.print("Reset memoria     ");
  lcd.setCursor(0,1);
  lcd.print("In corso...       ");
  delay(200);
  for(byte i = 0;i< 8; i++){
    EEPROM.write(KVALUEADDR+i, 0xFF);
  }
  lcd.setCursor(0,1);
  lcd.print("Terminato          ");
  delay(200);
}

void calibrate() 
{
  bool active = true;
  while (active) {
    char cmd[10];
    static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U)  //time interval: 1s
    {
      timepoint = millis();
      temperature = readTemperature();          // read your temperature sensor to execute temperature compensation
      if (temperature == -127) {
        temperature = defaultTemp;
      }
      if (mode == 1) {
        voltageC = analogRead(EC_PIN)/1024.0*5000;  // read the voltage
        ecValue =  ec.readEC(voltageC,temperature);  // convert voltage to EC with temperature compensation
        Serial.print("temperature:");
        Serial.print(temperature,1);
        Serial.print("^C  EC:");
        Serial.print(ecValue,2);
        Serial.println("ms/cm");
      }
      if (mode == 2) {
        voltageP = analogRead(PH_PIN)/1024.0*5000;  // read the voltage
        phValue = ph.readPH(voltageP,temperature);  // convert voltage to pH with temperature compensation
        Serial.print("temperature:");
        Serial.print(temperature,1);
        Serial.print("^C  pH:");
        Serial.println(phValue,2);
      }
    }
    int iteration = 0;
    if (mode==1) {
      if (iteration==0) {
        Serial.println("Available commands:");
        Serial.println("enterec");
        Serial.println("calec");
        Serial.println("exitec");
        ec.calibration(voltageC,temperature,"enterec\0");
        lcd.setCursor(0,0);
        lcd.print("Immergi in 1413us/cm");
        lcd.setCursor(0,1);
        lcd.print("Poi premi Enter");
        char key = keypad.getKey();
        while (true) {
          temperature = readTemperature();
          voltageC = analogRead(EC_PIN)/1024.0*5000;
          Serial.println(voltageC);
          key = keypad.getKey();
          if (key=='X') return;
          if (key=='E') {
            iteration++;
            break;
          }
          delay(200);
        }
      }
      if (iteration==1) {
        ec.calibration(voltageC,temperature,"calec\0");
        lcd.setCursor(0,0);
        lcd.print("Immergi in 12.88ms/cm");
        lcd.setCursor(0,1);
        lcd.print("Poi premi Enter");
        char key = keypad.getKey();
        while (true) {
          temperature = readTemperature();
          voltageC = analogRead(EC_PIN)/1024.0*5000;
          Serial.println(voltageC);
          key = keypad.getKey();
          if (key=='X') return;
          if (key=='E') {
            iteration++;
            break;
          }
          delay(200);
        }
      }
      if (iteration==2) {
        ec.calibration(voltageC,temperature,"calec\0");
        lcd.setCursor(0,0);
        lcd.print("Premi Enter     ");
        lcd.setCursor(0,1);
        lcd.print("per salvare     ");
        char key = keypad.getKey();
        while (true) {
          temperature = readTemperature();
          voltageC = analogRead(EC_PIN)/1024.0*5000;
          Serial.println(voltageC);
          key = keypad.getKey();
          if (key=='X') return;
          if (key=='E') {
            iteration++;
            break;
          }
          delay(200);
        }
      }
      if (iteration==3) {
        ec.calibration(voltageC,temperature,"exitec\0");
        lcd.setCursor(0,0);
        lcd.print("Salvato          ");
        lcd.setCursor(0,1);
        lcd.print("Premi Enter      ");
        char key = keypad.getKey();
        while (true) {
          key = keypad.getKey();
          if (key=='X') return;
          if (key=='E') {
            return;
          }
          delay(200);
        }
      }
    }
 
    if (mode==2) {
      if (iteration==0) {
        Serial.println("Available commands:");
        Serial.println("enterph");
        Serial.println("calph");
        Serial.println("exitph");
        ph.calibration(voltageP,temperature,"enterph\0");
        lcd.setCursor(0,0);
        lcd.print("Immergi in pH7.0");
        lcd.setCursor(0,1);
        lcd.print("Poi premi Enter");
        char key = keypad.getKey();
        while (true) {
          temperature = readTemperature();
          voltageP = analogRead(PH_PIN)/1024.0*5000;
          Serial.println(voltageP);
          key = keypad.getKey();
          if (key=='X') return;
          if (key=='E') {
            iteration++;
            break;
          }
          delay(200);
        }
      }
      if (iteration==1) {
        ph.calibration(voltageP,temperature,"calph\0");
        lcd.setCursor(0,0);
        lcd.print("Immergi in ph4.0");
        lcd.setCursor(0,1);
        lcd.print("Poi premi Enter");
        char key = keypad.getKey();
        while (true) {
          temperature = readTemperature();
          voltageP = analogRead(PH_PIN)/1024.0*5000;
          Serial.println(voltageP);
          key = keypad.getKey();
          if (key=='X') return;
          if (key=='E') {
            iteration++;
            break;
          }
          delay(200);
        }
      }
      if (iteration==2) {
        ph.calibration(voltageP,temperature,"calph\0");
        lcd.setCursor(0,0);
        lcd.print("Premi Enter     ");
        lcd.setCursor(0,1);
        lcd.print("per salvare     ");
        char key = keypad.getKey();
        while (true) {
          temperature = readTemperature();
          voltageP = analogRead(PH_PIN)/1024.0*5000;
          Serial.println(voltageP);
          key = keypad.getKey();
          if (key=='X') return;
          if (key=='E') {
            iteration++;
            break;
          }
          delay(200);
        }
      }
      if (iteration==3) {
        ph.calibration(voltageP,temperature,"exitph\0");
        lcd.setCursor(0,0);
        lcd.print("Salvato          ");
        lcd.setCursor(0,1);
        lcd.print("Premi Enter      ");
        char key = keypad.getKey();
        while (true) {
          key = keypad.getKey();
          if (key=='X') return;
          if (key=='E') {
            return;
          }
          delay(200);
        }
      }
    }

    if(readSerial(cmd)){
        strupr(cmd);
        if(strstr(cmd,"PH")){
            ph.calibration(voltageP,temperature,cmd);       //PH calibration process by Serail CMD
        }
        if(strstr(cmd,"EC")){
            ec.calibration(voltageC,temperature,cmd);       //EC calibration process by Serail CMD
        }
    }
  }
}

bool readSerial(char result[]){
  int i = 0;
    while(Serial.available() > 0){
        char inChar = Serial.read();
        if(inChar == '\n'){
             result[i] = '\0';
             Serial.flush();
             i=0;
             return true;
        }
        if(inChar != '\r'){
             result[i] = inChar;
             i++;
        }
        delay(1);
    }
    return false;
}
