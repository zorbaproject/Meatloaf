#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

IPAddress server(192,168,0,103);  // numeric IP
//char server[] = "raspberrypi.local";    // name address

const char* host = "192.168.0.103";

unsigned long towait = 1000UL*60*5;
unsigned long iter = towait - 1000UL*10;

const float drop = 2.5;

bool debug = false;

float rain = 0.0;
float rainV[60];
unsigned long itert = 0;

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

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

void setup() {
    // Open serial communications and wait for port to open:
    Serial.begin(9600);
    // start the Ethernet connection:
    if (Ethernet.begin(mac) == 0) {
        Ethernet.begin(mac, ip);
    }
    // give the Ethernet shield a second to initialize:
    delay(1000);
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1) {}
    }
    pinMode(RAINPIN, INPUT);
    for (int i = 0; i < 60; i++) rainV[i] = 0.0;
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    dht.humidity().getSensor(&sensor);
    delayMS = sensor.min_delay / 1000;
    delay(delayMS);
}

void loop() {
    delay(100);
    iter = iter + 100;
    itert = itert + 100;
    if (digitalRead(RAINPIN) == HIGH) {
        rain = rain + drop;
    }
    if (itert >= (1000UL*60) && itert % (1000UL*60) == 0) {
        int i = (itert / (1000UL*60))-1;
        rainV[i] = rain;
        rain = 0;
    }
    if (itert > 1000UL*60*60) {
        itert = 0;
    }
    
    if (iter > towait) {
        /*Serial.println(iter);
         *        Serial.println(towait);*/
        float pressure = bmp.readPressure()/100;
        if (debug) {
            Serial.print("Pressure = ");
            Serial.print(pressure);
            Serial.println(" hPa");
        }
        
        float temperature = 0.0;
        sensors_event_t event;  
        dht.temperature().getEvent(&event);
        if (!isnan(event.temperature)) {
            temperature = (bmp.readTemperature()+event.temperature)/2;
            if (debug) {
                Serial.print("Temperature: ");
                Serial.print(temperature);
                Serial.println(" °C");
            }
        }
        
        float humidity = 0.0;
        dht.humidity().getEvent(&event);
        if (!isnan(event.relative_humidity)) {
            humidity = event.relative_humidity;
            if (debug) {
                Serial.print("Humidity: ");
                Serial.print(humidity);
                Serial.println("%");
            }
        }
        
        float tmprain = 0.0;
        for (int i =0 ; i < 60; i++) tmprain = tmprain + rainV[i];
        if (debug) {
            Serial.print("Rain mm/h: ");
            Serial.println(tmprain);
        }
        
        if (!client.connected()) {
            //Serial.println();
            //Serial.println("disconnecting.");
            client.stop();
        }
        // Make a HTTP request:
        if (client.connect(server, 80)) {
            String pval = String(pressure, DEC);
            String tval = String(temperature, DEC);
            String hval = String(humidity, DEC);
            String rval = String(tmprain, DEC);
            client.println("GET /meteo/write-values.php?temperature=" + tval + "&pressure=" + pval + "&humidity=" + hval + "&rain=" + rval + " HTTP/1.1");
            client.println("Host: " + String(host));
            client.println("Connection: keep-open");
            client.println();
            //Serial.println("connected");
        } else {
            Serial.println("connection failed");
            client.stop();
        }
        
        /*if (client.available()) {
         *            //char c = client.read();
         *            //Serial.print(c);
         *            delay(10);
    }*/
        /*
         *        // if the server's disconnected, stop the client:
         *        if (!client.connected()) {
         *            //Serial.println();
         *            //Serial.println("disconnecting.");
         *            client.stop();
    }*/
        iter = 0;
    }
    
}
