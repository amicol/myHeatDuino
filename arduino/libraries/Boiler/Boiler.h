/**
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>

Created by Alexandre Micol <micolalexandre@gmail.com>
Parts of code by https://github.com/mysensors/Arduino

*/

#ifndef Boiler_h
#define Boiler_h

#include "Arduino.h"
#include <DallasTemperature.h>
#include <SPI.h>
#include <MySensor.h>
#include <OneWire.h>
#include "MAX6675.h"

#ifdef SMOKE
	#include "Smoke.h"
#endif

MyMessage msg_temp(0,V_TEMP);
MyMessage msg_boiler(BOILER_STATUS_ID,V_STATUS);
MyMessage msg_gas(BOILER_GAS_ID,V_TEMP);
//MyMessage msg_circulator(BOILER_CIRCU_ID,V_STATUS);

#ifdef PROBE_DALLAS
// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
#endif

int CS_pin = 4;
int SO_pin = 5;
int SCK_pin = 6;
MAX6675 thermocouple(CS_pin, SO_pin, SCK_pin, 1);

class Boiler
{
private:
uint8_t _PumpPin;
DallasTemperature *_sensors;

public:
int lastTemperature[3];
int Temperature_gas;
int circulator;
int *_ds18b20_index;
bool boiler_state;
DeviceAddress tempDeviceAddress;
Boiler(DallasTemperature *sensors, int *ds18b20_index, uint8_t PumpPin);
#ifdef SMOKE
	Smoke smoke;
#endif
void begin(void);
void presentation();
void update();
};

Boiler::Boiler(DallasTemperature *sensors, int *ds18b20_index, uint8_t PumpPin)
{
_sensors = sensors;
_PumpPin = PumpPin;
_ds18b20_index = ds18b20_index;
}

void Boiler::begin(void)
{
pinMode(_PumpPin, OUTPUT);
digitalWrite(_PumpPin, HIGH);
circulator=0;

// Loop through each device, print out address
#ifdef PROBE_DALLAS                                                  
  for(int i=0;i<3; i++)

  {

    // Search the wire for address

    if(_sensors->getAddress(tempDeviceAddress, i))

 {

 Serial.print("Found device ");

 Serial.print(i, DEC);

 Serial.print(" with address: ");

 printAddress(tempDeviceAddress);

 Serial.println();
 }
}
#endif 
#ifdef SMOKE
	smoke.begin();
#endif
}

void Boiler::presentation()
{
for (int i=0; i<3; i++) {   
     present(CHILD_ID_START+i, S_TEMP);
     wait(DWELL_TIME);
}
#ifdef SMOKE
	smoke.presentation();
        wait(DWELL_TIME);
#endif
present(BOILER_STATUS_ID,S_HVAC);
present(BOILER_GAS_ID,S_TEMP);
present(BOILER_CIRCU_ID,S_HVAC);
send(msg_boiler.set(false),true);
//send(msg_circulator.set(false),true);
}

void Boiler::update()
{
  // Read temperatures and send them to controller 
  for (int i=0; i<3; i++) {
    
    // Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((getConfig().isMetric?_sensors->getTempCByIndex(_ds18b20_index[i]):_sensors->getTempFByIndex(_ds18b20_index[i])) * 10.)) / 10.;
    
    // Only send data if temperature has changed and no error
    #if COMPARE_TEMP == 1
    if (lastTemperature[i] != temperature && temperature != -127.00) {
    #else
    if (temperature != -127.00) {
    #endif
    #ifdef PROBE_DALLAS
      Serial.print("\n");
      Serial.print(i);
      Serial.print(":");
      Serial.print(temperature);
    #endif
      // Send in the new temperature
      send(msg_temp.setSensor(CHILD_ID_START+i).set(temperature,1));
      wait(DWELL_TIME);
      // Save new temperatures for next compare
      lastTemperature[i]=temperature;
    }
  }

  Temperature_gas = thermocouple.read_temp();
  if (circulator == 1)
	{
		digitalWrite(_PumpPin, LOW);
                boiler_state = true;
	}
  else if (lastTemperature[0]>MAX_BOILER_TEMP)
	{
		digitalWrite(_PumpPin, LOW);
                boiler_state = true;
	}
  else if ((lastTemperature[0]>REQUIRED_BOILER_TEMP) && (digitalRead(_PumpPin) == HIGH) && Temperature_gas>REQUIRED_GAS_TEMP)
	{
		digitalWrite(_PumpPin, LOW);
                boiler_state = true;
	}
  else if ((Temperature_gas<REQUIRED_GAS_TEMP) && (digitalRead(_PumpPin) == LOW) && lastTemperature[0]<MAX_BOILER_TEMP)
	{
		digitalWrite(_PumpPin, HIGH);
                boiler_state = false;
	}
  send(msg_boiler.set(boiler_state));
  wait(DWELL_TIME);
  #ifdef SMOKE
	smoke.update();
  #endif
  send(msg_gas.set(Temperature_gas,1));
  //send(msg_circulator);
}

#endif
