#define FASTADC 1

// Thanks to https://github.com/klauscam/Arduino-Lightning-Detector/blob/master/sketch/LightningDetector/LightningDetector.ino

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


#include <Servo.h>
#include <avr/pgmspace.h>


Servo myservo;  

const int servoPin = 9;
const int antenna = A4;

const int posOn = 180;
const int posOff = 90;

const unsigned long waitfor = 1*1000UL*60; //minutes

const float soglia = 20; //100.0;  //range: 10-500

unsigned long lastOff = 0;



int data = 512;
int storage[512];

long batchStarted;
long batchEnded;
int reading;
int count;
int maximum;
int minimum;
bool toSend;

void setup() {
#if FASTADC
 // set prescale to 16
 sbi(ADCSRA,ADPS2) ;
 cbi(ADCSRA,ADPS1) ;
 cbi(ADCSRA,ADPS0) ;
#endif
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(antenna, INPUT);
  Serial.println(micros());
  
  batchStarted=0;
  batchEnded=0;
  reading=0;
  count=0;
  maximum=0;
  minimum=1023;
  toSend=false;
  myservo.attach(servoPin);
  myservo.write(posOn);
  delay(15);
}


void loop() {
  // put your main code here, to run repeatedly:
  reading = analogRead(antenna);//(analogRead(A4));
  storage[count]=reading;
  //if ((!toSend)&&(count!=0)&&((reading>storage[count-1]+10)||(reading<storage[count-1]-10))){
  if ((!toSend)&&(count!=0)&&((reading>storage[count-1]+soglia)||(reading<storage[count-1]-soglia))){
      toSend=true;
  }
  
  count=count+1;
  if ((count == 512) && (toSend))
  {
    count=0;
    batchEnded = millis();
    sendData();
    batchStarted = millis();
    
  }
  else if (count==512){
    count=0;
    batchEnded = millis();
    //sendData();
    batchStarted = millis();
    if ((millis()-lastOff)>waitfor) myservo.write(posOn);
  }

 
}

void sendData()
{
  //Serial.print(">>>");
  //Serial.println(batchStarted);
  myservo.write(posOff);
  lastOff = millis();
  for (int i=0;i<data;i++){
    Serial.println(storage[i]);
  }
  //Serial.print("<<<");
  //Serial.println(batchEnded);
  //Serial.println("END");
  
  toSend=false;
}
