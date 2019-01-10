#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

IPAddress server(192,168,0,103);  // numeric IP for Google (no DNS)
//char server[] = "raspberrypi.local";    // name address for Google (using DNS)

const char* host = "192.168.0.103";

long towait = 1000*60*10;

IPAddress ip(192, 168, 0, 177);

EthernetClient client;

Adafruit_BMP085 bmp;

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on 'Arduino Uno thats Analog 5
// Connect SDA to i2c data - on 'Arduino Uno thats Analog 4

#define DHTPIN            2         // Pin which is connected to the DHT sensor.
#define DHTTYPE           DHT22     // DHT 22 (AM2302)

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
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    dht.humidity().getSensor(&sensor);
    delayMS = sensor.min_delay / 1000;
    
}

void loop() {
    
    float pressure = bmp.readPressure()/100;
    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" hPa");
    
    delay(delayMS);
    
    float temperature = 0.0;
    sensors_event_t event;  
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        Serial.println("Error reading temperature!");
    }
    else {
        temperature = (bmp.readTemperature()+event.temperature)/2;
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" °C");
    }
    
    float humidity = 0.0;
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        Serial.println("Error reading humidity!");
    }
    else {
        humidity = event.relative_humidity;
        Serial.print("Humidity: ");
        Serial.print(humidity);
        Serial.println("%");
    }
    
    
    // Make a HTTP request:
    if (client.connect(server, 80)) {
        
        String pval = String(pressure, DEC);
        String tval = String(temperature, DEC);
        String hval = String(humidity, DEC);
        client.print("GET /write-values.php?temperature=" + tval + "&pressure=" + pval + "&humidity=" + hval + " HTTP/1.1\r\n" +
        "Host: " + host + "\r\n" +
        "Connection: close\r\n\r\n");
        Serial.println("connected");
        
        
        client.println();
    } else {
        // if you didn't get a connection to the server:
        Serial.println("connection failed");
    }
    // if there are incoming bytes available
    // from the server, read them and print them:
    if (client.available()) {
        char c = client.read();
        Serial.print(c);
    }
    
    // if the server's disconnected, stop the client:
    if (!client.connected()) {
        Serial.println();
        Serial.println("disconnecting.");
        client.stop();
        
        // do nothing forevermore:
        delay(towait);
    }
}
