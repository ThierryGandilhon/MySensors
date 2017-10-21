/***************************************************************************

 ***************************************************************************/
#define MY_DEBUG
#define MY_RADIO_NRF24

// Flash leds on rx/tx/err
#define MY_DEFAULT_ERR_LED_PIN 4  // Error led pin
#define MY_DEFAULT_RX_LED_PIN  6  // Receive led pin
#define MY_DEFAULT_TX_LED_PIN  5  // the PCB, on board LED


#include <MySensors.h>
#define BARO_CHILD 0
#define TEMP_CHILD 1
#define HUMI_CHILD 2

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"


Adafruit_BMP280 bme; // I2C

#define DHTPIN 3     // what digital pin we're connected to
#define DHTTYPE DHT22 
DHT dht(DHTPIN, DHTTYPE);


MyMessage msgTemp(TEMP_CHILD, V_TEMP);
MyMessage msgPressure(BARO_CHILD, V_PRESSURE);
MyMessage msgHum(HUMI_CHILD, V_HUM);


  
void setup() {
  Serial.println(F("BMP280 + DHT22 + NRF24L01 test"));
  
  if (!bme.begin((uint8_t) 0x76)) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  dht.begin();
}

void presentation() {
  sendSketchInfo("Pressure Sensor", "0.1");

  present(BARO_CHILD, S_BARO);
  present(TEMP_CHILD, S_TEMP);
  present(HUMI_CHILD, S_HUM);
}
  
void loop() {
    float tempBMP  = bme.readTemperature();
    float pressBMP = bme.readPressure() / 100 + 14.92; // Correction for 125m elevation
    float hDHT     = dht.readHumidity();
    float tDHT     = dht.readTemperature();

    send(msgTemp.set(tDHT, 1));
    send(msgHum.set(hDHT, 1));
    send(msgPressure.set(pressBMP, 1));
    
#ifdef MY_DEBUG
    Serial.print("Temperature = ");
    Serial.print(tempBMP);
    Serial.print(" *C\t");
    Serial.print("Pressure = ");
    Serial.print( pressBMP);
    Serial.print(" hPa\t");
    Serial.print("Temperature = ");
    Serial.print(tDHT);
    Serial.print(" *C\t");
    Serial.print("Humidity = ");
    Serial.print(hDHT);
    Serial.println(" %");
 #endif
 
  sleep(60000);
}
