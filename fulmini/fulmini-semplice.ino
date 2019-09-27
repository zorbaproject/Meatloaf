
#include <Servo.h>
#include <SPI.h>
#include <Ethernet.h>


Servo myservo;  

const int servoPin = 9;
const int antenna = A4;

const int posOn = 180;
const int posOff = 90;

const int waitfor = 10*60*10; //minutes

float soglia = 700.0;

int lastOff = 0;


byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 117);
EthernetServer server(80);

void setup() {
  myservo.attach(servoPin);  
  pinMode(antenna, INPUT);
  myservo.write(posOn);
  Serial.begin(9600);
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
}

void loop() {
  float reading = analogRead(antenna);
  Serial.println(reading);
  if (reading > soglia && lastOff > 10) {
    myservo.write(posOff);
    lastOff = 0;
  } else if (lastOff > waitfor) {
    myservo.write(posOn);
  }
  delay(100); 
  lastOff = lastOff +1;
  weblisten();
}


void weblisten() {
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        if (c == '\n' && currentLineIsBlank) {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close"); 
          client.println("Refresh: 5"); 
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          if (lastOff > waitfor) {
            client.print("Power On");
          } else {
            client.print("Power Off");
          }
          client.println("<br />");
          client.print("Last Off: ");
          client.print(lastOff/10);  //seconds
          client.println("<br />");
          client.println("</html>");
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
    Serial.println("client disconnected");
  }
}

