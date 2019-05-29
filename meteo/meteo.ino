#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <OneWire.h> 
#include <DallasTemperature.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

IPAddress server(192,168,0,103);  // numeric IP

const char* host = "192.168.0.103";

unsigned long towait = (1000UL*60*10);
int lastdrop = 1000;
int lastarray = 1000;

const float drop = (1.8/(11*5))*10; // (9ml/5click)/(area in cm2)=cm of rain Note: 1ml = 1cm3

bool debug = false;
bool usenetwork = true;

bool firstrun = true;

float rain = 0.0;
float rainV[60];
//unsigned long itert = 0;

IPAddress ip(192, 168, 0, 177);

EthernetClient client;

Adafruit_BMP085 bmp;

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on 'Arduino Uno thats Analog 5
// Connect SDA to i2c data - on 'Arduino Uno thats Analog 4

#define DHTPIN            2         // Pin which is connected to the DHT sensor.
#define DHTTYPE           DHT22     // DHT 22 (AM2302)

#define RAINPIN           8         //Pin connected to the rain gauge

#define ONE_WIRE_BUS 7              //Pin connected to the DS81B20

OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

void(* resetFunc) (void) = 0;

void setup() {
    // Open serial communications and wait for port to open:
    Serial.begin(9600);
    if (usenetwork) {
    if (debug) Serial.println("Looking for network...");
    // start the Ethernet connection:
    if (Ethernet.begin(mac) == 0) {
        Ethernet.begin(mac, ip);
    }
    if (!Ethernet.localIP() || Ethernet.localIP() == "") {
        Serial.println("Could not connect to network, resetting.");
        delay(2000);
        resetFunc();
    }
    Serial.print("Connected to network. IP: ");
    Serial.println(Ethernet.localIP());
    // give the Ethernet shield a second to initialize:
    delay(1000);
    }
    
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor");
        delay(2000);
    }
    pinMode(RAINPIN, INPUT);
    for (int i = 0; i < 60; i++) rainV[i] = 0.0;
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    dht.humidity().getSensor(&sensor);
    delayMS = sensor.min_delay / 1000;
    delay(delayMS);
    sensors.begin(); 
    if (debug) Serial.println("Setup finished");
}

void loop() {

    if ((millis() % 10) == 0) {
        //itert = itert + 10;
        lastdrop = lastdrop + 10;
        if (lastdrop > 20000) lastdrop = 1100;
        lastarray = lastarray + 10;
        if (lastarray > 20000) lastarray = 1100;
    }
    
    if ((millis() % 10) == 0 && lastdrop > 1000) {
        if (digitalRead(RAINPIN) == HIGH) {
          rain = rain + drop;
          lastdrop = 0;
        }
    }
    
    if (millis() % (1000UL*60) == 0 && lastarray > 1000) {
        int i = (millis() / (1000UL*60))-1;
        i = i % 60; //ogni ora si ricomincia
        rainV[i] = rain;
        rain = 0.0;
        lastarray = 0;
    }
    
    if ((millis() % towait) == 0 || firstrun == true) {
        
        float pressure = 0.0;
        pressure = bmp.readPressure()/100;
        if (debug) {
            Serial.println(pressure);
        }

        sensors_event_t event;
        
        float temperature = 0.0;
        /*dht.temperature().getEvent(&event);
        if (!isnan(event.temperature)) {
            temperature = (bmp.readTemperature()+event.temperature)/2;
        }*/
        sensors.requestTemperatures();
        temperature = sensors.getTempCByIndex(0);
        if (debug) {
            Serial.println(temperature);
        }
        
        float humidity = 0.0;
        dht.humidity().getEvent(&event);
        if (!isnan(event.relative_humidity)) {
            humidity = event.relative_humidity;
        }
        if (debug) {
            Serial.println(humidity);
        }

        float tmprain = 0.0;
        for (int i =0 ; i < 60; i++) tmprain = tmprain + rainV[i];
        if (debug) {
            Serial.println(tmprain);
        }
        
        if (!client.connected()) {
            client.stop();
        }
        // Make a HTTP request:
        if (debug) Serial.println("Connecting to server");
        if (usenetwork) {
        if (client.connect(server, 80)) {
            String pval = String(pressure, DEC);
            String tval = String(temperature, DEC);
            String hval = String(humidity, DEC);
            String rval = String(tmprain, DEC);
            String frval = String("&firstrun=0");
            if (firstrun) frval = String("&firstrun=1");
            client.println("GET /meteo/write-values.php?temperature=" + tval + "&pressure=" + pval + "&humidity=" + hval + "&rain=" + rval + frval + " HTTP/1.1");
            client.println("Host: " + String(host));
            //client.println("Connection: keep-open");
            client.println("Connection: close");
            client.println();
            if (debug) Serial.println("Data sent");
            client.stop();
        } else {
            Serial.println("Connection failed");
            client.stop();
            delay(1000);
            resetFunc();
        }
        }
        firstrun = false;
    }
    
}
