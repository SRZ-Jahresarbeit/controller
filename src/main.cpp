// MIT License
// Copyright (c) 2021 Marco Grunert

#include <Arduino.h>
#include <SSD1306.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <classWifi.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <ctime>
#include <PubSubClient.h>

#define SensorUUIDTemp      <uuid for Temperatur>
#define SensorUUIDPreassur  <uuid for Preassur>
#define SensorUUIDHumidity  <uuid for Humidity>

#define OLED_I2C_ADDR   0x3C
#define OLED_RESET      16
#define OLED_SDA        4
#define OLED_SCL        15

#define DISPLAY_UPDATE_RATE 15 // seconds

#define SSID        <WLAN SSID>
#define SSID_PASS   <WLAN PASS>

#define MQTT_ADDRESS    <MQTT SERVER>
#define MQTT_PORT       1883

#define SDA 21
#define SCL 13

SSD1306 display(OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

Wifi wifi(SSID, SSID_PASS);
WiFiUDP udp;
NTPClient timeClient(udp, "de.pool.ntp.org");

WiFiClient client;
PubSubClient mqttClient(client);

TwoWire I2Cone = TwoWire(1);
Adafruit_BME280 bme;

float temperature; 
String topicTemp,topicPreassur,topicHumidity;

void setup()
{   
    Serial.begin(115200);
    sleep(5); // wait for serial
    
    #ifdef SensorUUIDTemp 
        topicTemp = "sensor_id/" + String(SensorUUIDTemp); 
    #endif
    #ifdef SensorUUIDPreassur 
        topicPreassur = "sensor_id/" + String(SensorUUIDPreassur);
    #endif
    #ifdef SensorUUIDHumidity
        topicHumidity = "sensor_id/" + String(SensorUUIDHumidity);
    #endif

    Serial.println("started...");
    Serial.print("Setting up display...");
    //Set up and reset the OLED
    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(50);
    digitalWrite(OLED_RESET, HIGH);
    display.init();
    display.setFont(ArialMT_Plain_24);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    Serial.println(" done.");

    display.drawString(5,10,"Setting up ...");
    display.display();

    Serial.print("Setting up bme280...");
    // setup the BME280 sensor
    I2Cone.begin(SDA, SCL, 100000);
    bool status_bme = bme.begin(0x76, &I2Cone);
    
    if (!status_bme)
    {
        Serial.println(" error.");
        Serial.println("Could not find a valid BME280_1 sensor, check wiring!");
        while (1)
            ; // effectively halts the whole system
    }
    Serial.println(" done.");
    Serial.print("Setting up mqtt...");
    mqttClient.setServer(MQTT_ADDRESS, MQTT_PORT);
    mqttClient.setKeepAlive(DISPLAY_UPDATE_RATE+10);
    mqttClient.setSocketTimeout(DISPLAY_UPDATE_RATE+10);
    Serial.println(" done.");
    
    display.drawString(20,35,"done");
    display.display();
}

void loop()
{
    if (!wifi.update())
    {
        return;
    }

    if (!mqttClient.connected())
    {
        Serial.print("Connection to mqtt...");
        if (mqttClient.connect(SensorUUIDTemp))
        {
            Serial.println(" done.");
        }
        else
        {
            Serial.println(" error.");
            sleep(10); // seconds
            return;
        }
    }
    
    temperature = bme.readTemperature();
    // update current time (may don't do this in every tick)
    timeClient.update();

    // publish data
    if(topicTemp!=NULL)     mqttClient.publish(topicTemp.c_str(), String(temperature).c_str());             
    if(topicHumidity!=NULL) mqttClient.publish(topicHumidity.c_str(), String(bme.readHumidity()).c_str());     
    if(topicPreassur!=NULL) mqttClient.publish(topicPreassur.c_str(), String(bme.readPressure()).c_str());

    display.resetDisplay();
    display.drawString(20,5,timeClient.getFormattedTime());
    display.drawString(20,35,String(temperature).c_str());
    display.drawString(85,35,"°C");
    display.display();

    mqttClient.loop();

    sleep(DISPLAY_UPDATE_RATE);
}    
