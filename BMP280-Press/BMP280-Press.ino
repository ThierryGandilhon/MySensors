/***************************************************************************

 ***************************************************************************/
#define MY_DEBUG
#define MY_RADIO_NRF24


#include <MySensors.h>
#define BARO_CHILD 0
#define TEMP_CHILD 1

#define SLEEP_TIME 60000 // ms

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>


Adafruit_BMP280 bme; // I2C


MyMessage msgTemp(TEMP_CHILD, V_TEMP);
MyMessage msgPressure(BARO_CHILD, V_PRESSURE);


  
void setup() {
#ifdef MY_DEBUG
  Serial.println(F("BMP280 + NRF24L01 sensor -- pression adjusted for Versailles elevation : 125m."));
#endif
  
  if (!bme.begin((uint8_t) 0x76)) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
}

void presentation() {
  sendSketchInfo("Pressure + Temperature Sensor", "1.0");

  present(BARO_CHILD, S_BARO);
  present(TEMP_CHILD, S_TEMP);
}
  
void loop() {
    float tempBMP  = bme.readTemperature();
    float pressBMP = bme.readPressure() / 100 + 14.92; // Correction for 125m elevation

    send(msgTemp.set(tempBMP, 1));
    send(msgPressure.set(pressBMP, 1));
    
#ifdef MY_DEBUG
    Serial.print("Temperature = ");
    Serial.print(tempBMP);
    Serial.print(" *C\t");
    Serial.print("Pressure = ");
    Serial.print( pressBMP);
    Serial.print(" hPa\t");
    Serial.println("");
#endif
 
  sleep(SLEEP_TIME);
}
