/*

 SoftwareSerial1:
 * RX is digital pin 10 (connect to TX of rangefinder) yellow
 * TX is digital pin 11 (connect to RX of rangefinder) orange
 SoftwareSerial2:
 * RX is digital pin 12 (connect to TX of rangefinder) blue
 * TX is digital pin 13 (connect to RX of rangefinder) green

 Note:
 Not all digital pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 */
#include <SoftwareSerial.h>

#include <Servo.h>

Servo myservo;

int pos = 0;

int scan[360];

SoftwareSerial mySerial(10, 11); // RX, TX



void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  myservo.attach(9);


  //Serial.println("Open");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(19200);
  mySerial.println("O");
  for (int i = 0; i < 360; i++) {
    scan[i] = 0.0;
  }
}

void loop() { // run over and over
  //reset servo
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the positio
    mySerial.write("F");
    char fullnum[6] = "000000";
    char mybuffer[50];
    int i = 0;
    bool store = false;
    Serial.println("Waiting for serial");
    while (!mySerial.available());
    Serial.println("Reading from serial");
    int size = mySerial.readBytesUntil('\n',mybuffer,50);
    Serial.println("Interpreting message");
    for (int c = 0; c < size; c++) {
      if (mybuffer[c] == 'm') store = false;
      if (store && i < 6) {
        fullnum[i] = mybuffer[c];
        if (mybuffer[c] == ' ') fullnum[i] = '0';
        i++;
      }
      if (mybuffer[c] == ':') store = true;
    }
    float tmpnum = atof(fullnum);
    scan[pos] = int(tmpnum*100);
    Serial.println(pos);
    //Serial.println(tmpnum);
  }

  for (int i = 0; i < 360; i++) {
    //Serial.print(scan[i], 3);
    Serial.print(scan[i]);
    Serial.print(",");
  }
  Serial.println();

}
