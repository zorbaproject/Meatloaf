#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position

bool continuous = false;

int scan[360];

int timeout = 1000;
const int servopin = 6;

//Arduino Mega has 4 hardware ports, we use Serial0 for USB, Serial1 and Serial2 for rangefinders

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  myservo.attach(servopin);

  Serial.println("LaserOn");

  // set the data rate for rangefinders
  Serial1.begin(19200);
  Serial1.println("O");
  Serial2.begin(19200);
  Serial2.println("O");
  for (int i = 0; i < 360; i++) {
    scan[i] = 0.0;
  }

  //reset servo
  for (pos = 180; pos >= 0; pos -= 1) { 
    myservo.write(pos);              
    delay(15);                       
  }
}

void loop() {

  bool single = false;
  if (!continuous) {
    while (!Serial.available());
    String mycmd = Serial.readString();
    //Serial.println(mycmd);
    if (mycmd.indexOf('S') > -1) single = true;
    if (mycmd.indexOf('C') > -1) continuous = true;
    if (mycmd.indexOf('Q') > -1) {
      single = false;
      continuous = false;
    }
  }
  if (single || continuous) {
  
  //Init array
  for (int i = 0; i < 360; i++) {
    scan[i] = 0.0;
  }
  
  //reset servo
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  
  //start measure
  for (pos = 0; pos < 180; pos += 1) { 
    myservo.write(pos);              
    delay(15);                       
    Serial1.write("F");
    Serial2.write("F");
    char fullnum[7] = "000000";

    if (Serial.available()) {
    String mycmd = Serial.readString();
    if (mycmd.indexOf('S') > -1) single = true;
    if (mycmd.indexOf('C') > -1) continuous = true;
    if (mycmd.indexOf('Q') > -1) {
      Serial.println("Quit");
      single = false;
      continuous = false;
      break;
    }
    }

    char mybuffer[50];
    unsigned long myTime;
    int i = 0;
    int v = 0;
    int m = 0;
    String mystr = "";
    int size = 0;
    bool store = false;
    myTime = millis();
    while (!Serial1.available() && (millis()-myTime) < timeout);
    size = Serial1.readBytesUntil('\n',mybuffer,50);
    mystr = String(mybuffer);
    v = mystr.indexOf(':');
    m = mystr.indexOf('m');
    if ( v > 0 && m > 0) {
      mystr.substring(v+1,m).toCharArray(fullnum,7);
    }
    float tmpnum = atof(fullnum);
    scan[pos] = int(tmpnum*100);

    store = false;
    i = 0;
    strcpy(fullnum,"000000");
    while (!Serial2.available() && (millis()-myTime) < timeout);
    size = Serial2.readBytesUntil('\n',mybuffer,50);
    mystr = String(mybuffer);
    v = mystr.indexOf(':');
    m = mystr.indexOf('m');
    if ( v > 0 && m > 0) {
      mystr.substring(v+1,m).toCharArray(fullnum,7);
    }
    float tmpnum2 = atof(fullnum);
    scan[pos+180] = int(tmpnum2*100);
  }

  for (int i = 0; i < 360; i++) {
    //Serial.print(scan[i], 3);
    Serial.print(scan[i]/100);
    if (i<359) Serial.print(",");
  }
  Serial.println();
  }

}
 
