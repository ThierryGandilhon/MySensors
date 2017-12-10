/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0: Henrik EKblad
 * Version 1.1 - 2016-07-20: Converted to MySensors v2.0 and added various improvements - Torben Woltjen (mozzbozz)
 * 
 * DESCRIPTION
 * This sketch provides an example of how to implement a humidity/temperature
 * sensor using a DHT11/DHT-22.
 *  
 * For more information, please visit:
 * http://www.mysensors.org/build/humidity
 * 
 */

// Enable debug prints
//#define MY_DEBUG
#undef MY_DEBUG

// Enable and select radio type attached 
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
//#define MY_RS485
 
#include <SPI.h>
#include <MySensors.h>  
#include <DHT.h>
#include <DHT_U.h>


// Set this to the pin you connected the DHT's data pin to
#define DHT_DATA_PIN 3
#define DHT_TYPE     DHT22

// Set this offset if the sensor has a permanent small offset to the real temperatures
#define SENSOR_TEMP_OFFSET 0

// Sleep time between sensor updates (in milliseconds)
// Must be >1000ms for DHT22 and >2000ms for DHT11
static const uint64_t UPDATE_INTERVAL = 60000;


#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1

float   lastTemp;
float   lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
bool    metric = true;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);


DHT_Unified dht(DHT_DATA_PIN, DHT_TYPE);
uint32_t    delayMS;



void presentation()  
{ 
  // Send the sketch version information to the gateway
  sendSketchInfo("DHT22 Temperature+Humidity ", "1.1");
  
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM,  S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  
  metric = getControllerConfig().isMetric;
}


void setup()
{
  sensor_t sensor;
  dht.begin();
  
  dht.temperature().getSensor(&sensor);
#ifdef MY_DEBUG
  Serial.begin(115200); 
  // Initialize device.
  Serial.println("DHTxx Unified Sensor");
  // Print temperature sensor details.
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.println("------------------------------------");
#endif

  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
#ifdef MY_DEBUG
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
  Serial.println("------------------------------------");
#endif
  
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}


void loop()      
{  
  // Get temperature event.
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    send(msgTemp.set(event.temperature, 1));
  }

#ifdef MY_DEBUG
  if (!isnan(event.temperature)) {
    Serial.print("Temperature: ");
    Serial.print(event.temperature);
    Serial.println(" *C");
  }
#endif

  
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    send(msgHum.set(event.relative_humidity, 1));
  }

#ifdef MY_DEBUG
  if ( !isnan(event.relative_humidity) ) {
    Serial.print("Humidity: ");
    Serial.print(event.relative_humidity);
    Serial.println("%");
  }
#endif


  // Sleep for a while to save energy
  sleep(UPDATE_INTERVAL); 
}
