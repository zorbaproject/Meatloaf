
#include <Servo.h>
#include <SPI.h>
#include <Ethernet.h>
#include <avr/pgmspace.h>

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 117);
EthernetServer server(80);

Servo myservo;  

const int servoPin = 9;
const int antenna = A4;

const int posOn = 180;
const int posOff = 90;

const unsigned long waitfor = 1*1000UL*60; //minutes

const float soglia = 20; //100.0;  //range: 10-500

unsigned long lastOff = 0;



// Thanks to https://github.com/klauscam/Arduino-Lightning-Detector/blob/master/sketch/LightningDetector/LightningDetector.ino
#define FASTADC 1

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

int data = 512;
int storage[512];

long batchStarted;
long batchEnded;
int reading;
int count;
int maximum;
int minimum;
bool toSend;

const char httpheader[] PROGMEM = {"HTTP/1.1 200 OK\nContent-Type: text/html\nConnection: close\nRefresh: 5\n\n<!DOCTYPE HTML>\n<html>\n"};


void setup() {
  #if FASTADC
  // set prescale to 16
  sbi(ADCSRA,ADPS2) ;
  cbi(ADCSRA,ADPS1) ;
  cbi(ADCSRA,ADPS0) ;
  #endif
  myservo.attach(servoPin);  
  pinMode(antenna, INPUT);
  myservo.write(posOn);
  Serial.begin(9600);
  Ethernet.begin(mac, ip);
  server.begin();
  //Serial.print("server is at ");
  //Serial.println(Ethernet.localIP());

  batchStarted=0;
  batchEnded=0;
  reading=0;
  count=0;
  maximum=0;
  minimum=1023;
  toSend=false;
  //Serial.println(waitfor);
}

void loop() {
  float reading = analogRead(antenna);
  storage[count]=reading;
  //Serial.println(reading);
  if ((!toSend)&&(count!=0)&&((reading>storage[count-1]+soglia)||(reading<storage[count-1]-soglia))){
      toSend=true;
      //Serial.println(reading);
  }

  count=count+1;
  if ((count == 512) && (toSend))
  {
    count=0;
    batchEnded = millis();
    if ((millis()-lastOff)>10000) myservo.write(posOff);
    //myservo.write(posOff);
    //Serial.println("Off");
    Serial.println(reading);
    lastOff = millis();
    batchStarted = millis();
    toSend=false;
  }
  else if (count==512 && (millis()-lastOff)>waitfor){
    count=0;
    batchEnded = millis();
    myservo.write(posOn);
    batchStarted = millis();
  
  }
  weblisten();
}


void weblisten() {
  EthernetClient client = server.available();
  if (client) {
    //Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        //Serial.write(c);
        if (c == '\n' && currentLineIsBlank) {
          //Serial.println((millis()-lastOff));
          client.println(httpheader);
          if (((millis()-lastOff) > waitfor)) {
            client.println("Power On");
          } else {
            client.println("Power Off");
          }
          client.print("Last Off: ");
          client.println((millis()-lastOff)/1000);  //seconds
          //client.println("</html>");
          break;
        }
        if (c == '\n') {
          currentLineIsBlank = true;
        } else if (c != '\r') {
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    client.stop();
    //Serial.println("client disconnected");
  }
}

