/*                  BME280 I2C for MySensors

// this program is for BMEP280 library :Adafriut BME280 

I use some parts from oryginal MySensors PressureSensor for BMP085 module  

To use with MySensors 2.0.0 dev.branch

* REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * Version 1.2 - Marek Dajnowicz


*/


// Enable debug prints to serial monitor
#define MY_DEBUG
//#define MY_REPEATER_FEATURE

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_NODE_ID 33 
//#define MY_PARENT_NODE_ID

#include <SPI.h>
#include <MySensors.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> 
#include "Wire.h"



Adafruit_BME280 bme; // I2C

#define CHILD_ID_HUM   0
#define CHILD_ID_TEMP  1
#define CHILD_ID_PRESS 2

const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
  STABLE = 0,      // "Stable Weather Pattern"
  SUNNY = 1,      // "Slowly rising Good Weather", "Clear/Sunny "
  CLOUDY = 2,     // "Slowly falling L-Pressure ", "Cloudy/Rain "
  UNSTABLE = 3,   // "Quickly rising H-Press",     "Not Stable"
  THUNDERSTORM = 4, // "Quickly falling L-Press",    "Thunderstorm"
  UNKNOWN = 5     // "Unknown (More Time needed)
};


// for forecast - untouched by me :)

  float lastPressure = -1;
  float lastTemp = -1;
  int lastForecast = -1;
  const int LAST_SAMPLES_COUNT = 5;
  float lastPressureSamples[LAST_SAMPLES_COUNT];
  // this CONVERSION_FACTOR is used to convert from Pa to kPa in forecast algorithm
  // get kPa/h be dividing hPa by 10
  #define CONVERSION_FACTOR (1.0/10.0)
  int minuteCount = 0;
  bool firstRound = true;
  // average value is used in forecast algorithm.
  float pressureAvg;
  // average after 2 hours is used as reference value for the next iteration.
  float pressureAvg2;
  float dP_dt;

// MyMessage to controler
MyMessage msgT1(CHILD_ID_TEMP,  V_TEMP);
MyMessage msgP1(CHILD_ID_PRESS, V_PRESSURE);
MyMessage msgF1(CHILD_ID_PRESS, V_FORECAST);
MyMessage msgH1(CHILD_ID_HUM,   V_HUM);

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("PressionAndTemperatureAndHumidity", "1.2");
  present(CHILD_ID_TEMP,  S_TEMP);
  present(CHILD_ID_PRESS, S_BARO);
  present(CHILD_ID_HUM,   S_HUM);  
}

void setup()
{
  delay(500);// just in case
  if (!bme.begin((uint8_t)(0x76)))
   {
    Serial.println("BME init failed!");
    while (1);
   }
  else Serial.println("BME init success!");
  
  ServerUpdate(); // for first data reading and sending to controler
}

void loop()
{ 
   smartSleep(2000); // adjust sleeping time here 1 minute in this case 
   ServerUpdate();
}

void ServerUpdate() // used to read sensor data and send it to controller
{
  double T, P, H;
  T = bme.readTemperature();
  P = bme.readPressure() / 100.0;
  H = bme.readHumidity();
  
  int forecast = sample(P);
  send(msgT1.set(T, 1));
  send(msgP1.set(P, 1));
  send(msgH1.set(H,1));
  send(msgF1.set(weather[forecast]));
    
  // unmark for debuging
  Serial.print("T = \t"); Serial.print(T, 1); Serial.print(" degC\t");
  Serial.print("P = \t"); Serial.print(P, 1); Serial.print(" mBar\t");
  Serial.print("F = \t"); Serial.print(weather[forecast]); Serial.println(" ?");
}


// Algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure)
{
  // Calculate the average of the last n minutes.
  int index = minuteCount % LAST_SAMPLES_COUNT;
  lastPressureSamples[index] = pressure;

  minuteCount++;
  if (minuteCount > 185)
  {
    minuteCount = 6;
  }

  if (minuteCount == 5)
  {
    pressureAvg = getLastPressureSamplesAverage();
  }
  else if (minuteCount == 35)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change * 2; // note this is for t = 0.5hour
    }
    else
    {
      dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
    }
  }
  else if (minuteCount == 65)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) //first time initial 3 hour
    {
      dP_dt = change; //note this is for t = 1 hour
    }
    else
    {
      dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 95)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 1.5; // note this is for t = 1.5 hour
    }
    else
    {
      dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 125)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    pressureAvg2 = lastPressureAvg; // store for later use.
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2; // note this is for t = 2 hour
    }
    else
    {
      dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 155)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2.5; // note this is for t = 2.5 hour
    }
    else
    {
      dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 185)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 3; // note this is for t = 3 hour
    }
    else
    {
      dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
    }
    pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

  int forecast = UNKNOWN;
  if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
  {
    forecast = UNKNOWN;
  }
  else if (dP_dt < (-0.25))
  {
    forecast = THUNDERSTORM;
  }
  else if (dP_dt > 0.25)
  {
    forecast = UNSTABLE;
  }
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
  {
    forecast = CLOUDY;
  }
  else if ((dP_dt > 0.05) && (dP_dt < 0.25))
  {
    forecast = SUNNY;
  }
  else if ((dP_dt > (-0.05)) && (dP_dt < 0.05))
  {
    forecast = STABLE;
  }
  else
  {
    forecast = UNKNOWN;
  }

  // uncomment when debugging
  //Serial.print(F("Forecast at minute "));
  //Serial.print(minuteCount);
  //Serial.print(F(" dP/dt = "));
  //Serial.print(dP_dt);
  //Serial.print(F("kPa/h --> "));
  //Serial.println(weather[forecast]);

  return forecast;
}

float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
  {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }
  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  return lastPressureSamplesAverage;
}

