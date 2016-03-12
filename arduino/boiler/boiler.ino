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
 * DESCRIPTION
 *
 * Example sketch showing how to send in DS1820B OneWire temperature readings back to the controller
 * http://www.mysensors.org/build/temp
 */


// Enable debug prints to serial monitor
//#define MY_DEBUG 
#define MY_NODE_ID 1
#define DWELL_TIME 300

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

//#define PROBE_DALLAS

#define CHILD_ID_START 10

#define SMOKE
//Smoke
#define MQ_SENSOR_CHILD_ID CHILD_ID_START+3 
#define MQ_SENSOR_ANALOG_PIN 0

#define WEATHER
//Light
#define LIGHT_CHILD_ID CHILD_ID_START+4
#define LIGHT_SENSOR_ANALOG_PIN 6

//Pressure
#define BARO_CHILD_ID CHILD_ID_START+5
#define TEMP_CHILD_ID CHILD_ID_START+6
#define FORECAST_CHILD_ID CHILD_ID_START+7

//Relay
#define RELAY_PUMP 7 // Relay that turns the circulator on/off
#define RELAY_CHILD_ID CHILD_ID_START+8


#define BOILER_STATUS_ID CHILD_ID_START+9
#define REQUIRED_BOILER_TEMP 75
#define MAX_BOILER_TEMP 85
#define REQUIRED_GAS_TEMP 60

#define BOILER_GAS_ID CHILD_ID_START+10
#define BOILER_CIRCU_ID CHILD_ID_START+11

#include <SPI.h>
#include <MySensor.h>  
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "MAX6675.h"
#include <Boiler.h>
#include <Weather.h>

//Temperature
#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No

#define ONE_WIRE_BUS 3 // Pin where dallase sensor is connected 
#define MAX_ATTACHED_DS18B20 3
unsigned long SLEEP_TIME = 10000; // Sleep time between reads (in milliseconds)
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors=0;
                                          
boolean receivedConfig = false;
int ds18b20_index[3]= {2,0,1};

Boiler boiler(&sensors,ds18b20_index,RELAY_PUMP);

#ifdef WEATHER
  Weather Myweather;
#endif


void setup()  
{
  // Startup up the OneWire library
  //sensors.begin();
  // requestTemperatures() will not block current thread
  //sensors.setWaitForConversion(false);
  boiler.begin();
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Boiler&weather Sensor", "1.1");
  // Fetch the number of attached temperature sensors
  sensors.begin();
  sensors.setWaitForConversion(false);
  numSensors = sensors.getDeviceCount();
  
  //Serial.print(numSensors);

  boiler.presentation();
  #ifdef WEATHER
    Myweather.presentation();
  #endif
}

void loop()     
{     
  // Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();
  
  // query conversion time and sleep until conversion completed
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
  // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
  sleep(conversionTime);



  boiler.update();
  #ifdef WEATHER
    Myweather.update();
  #endif
  wait(SLEEP_TIME);
}
void receive(const MyMessage & message)
{
  if ((message.sender == 0) && (message.sensor == BOILER_STATUS_ID)) 
  {
    if (message.getBool() == true) {
      boiler.circulator=1;
      boiler.update();
    } else {
      boiler.circulator=0;
      boiler.update();
    }
  }  
  
}
