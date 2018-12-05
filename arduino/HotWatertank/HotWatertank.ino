/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Modified by Alexandre Micol <micolalexandre@gmail.com>
 *
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
 * The ArduinoGateway prints data received from sensors on the serial link.
 * The gateway accepts input on seral which will be sent out on radio network.
 *
 * The GW code is designed for Arduino Nano 328p / 16MHz
 *
 * Wire connections (OPTIONAL):
 * - Inclusion button should be connected between digital pin 3 and GND
 * - RX/TX/ERR leds need to be connected between +5V (anode) and digital pin 6/5/4 with resistor 270-330R in a series
 *
 * LEDs (OPTIONAL):
 * - To use the feature, uncomment MY_LEDS_BLINKING_FEATURE in MyConfig.h
 * - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
 * - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
 * - ERR (red) - fast blink on error during transmission error or recieve crc error
 *
 */

/**
  Heating Zone control.
  This skectch maintain the temperature in an heating zone by powering on/off a water circulator and controlling the temperature emitter with 3 Way Valve (3WV).
  An OLED display/rotary encoder can be added to view temperature and set thermostat.
  Mysensors message permit to set thermostat and view temperatures in controller.
  Energy meter is added with Emonlib library. 3 phases here.
*/


//#define OLED			//define all OLED display and rotary knob feature for a heating zone
//#define LLCD

//#define EEPROM_BACKUP		//thermostat temperature saved in EEPROM.
//#define PRINT_RAM

#define MY_NODE_ID HOTWATER_ID
#define PPID            //PID for temperature and 3-ways valve control#define PPID            //PID for temperature and 3-ways valve control
#define EEPROM_BACKUP		//thermostat temperature saved in EEPROM.#define EEPROM_BACKUP		//thermostat temperature saved in EEPROM.

// Enable debug prints to serial monitor
#define MY_DEBUG
//#define MY_DEBUG_VERBOSE
//#define DEBUG //to big with 32k card TODO : reduce string memory footprint.

// Set LOW transmit power level as default, if you have an amplified NRF-module and
// power your radio separately with a good regulator you can turn up PA level.
//#define MY_RF24_PA_LEVEL RF24_PA_LOW
//#define RF24_PA_LEVEL RF24_PA_MIN
//#define RF24_PA_LEVEL_GW RF24_PA_MIN
// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Enable serial gateway
//#define MY_GATEWAY_SERIAL

// Flash leds on rx/tx/err
#undef MY_LEDS_BLINKING_FEATURE
// Set blinking period
//#define MY_DEFAULT_LED_BLINK_PERIOD 300

//// Enable inclusion mode
//#define MY_INCLUSION_MODE_FEATURE
//// Enable Inclusion mode button on gateway
//#define MY_INCLUSION_BUTTON_FEATURE
//// Set inclusion mode duration (in seconds)
//#define MY_INCLUSION_MODE_DURATION 60
//// Digital pin used for inclusion mode button
//#define MY_INCLUSION_MODE_BUTTON_PIN  2

//#define MY_DEFAULT_ERR_LED_PIN 7  // Error led pin
//#define MY_DEFAULT_RX_LED_PIN  8  // Receive led pin
//#define MY_DEFAULT_TX_LED_PIN  9  // the PCB, on board LED

#define ONE_WIRE_BUS 7		// Pin where dallas sensor is connected
#include "./ids.h"

#define EPSILON_TEMP_TANK 30
#define WAIT_BEFORE_STOP 300000

#define INTERVAL_CHECK 5*3600*1000
#define RELAY_HW_PUMP 2		// Relay that turns the circulator on/off
#define RELAY_ELEC 8    // Relay that turns the elec heating on/off

#define RELAY_PUMP 3		// Relay that turns the circulator on/off
#define SERVO_ON 4		// Relay that powers the 3WV actuator
#define SERVO_DIRECTION 5	// Relay that sets the 3WV actuator direction
#define TIME_KNOB 2000          // time before taking into account that knob has changed
#define STEP_CLOSE 1400ul	// duration of a single step while closing the valve in ms
#define STEP_OPEN 1400ul	// duration of a single step while opening the valve in ms
#define DELAY_PUMP_BUTTON 5000	// How long to delay turning the circulator on?
#define INTERVAL_REG 45000ul	// Interval between 3VW temperature checks
#define INTERVAL_MAIN_REG 900000ul	// Interval between Ambiant temperature checks
#define INTERVAL_TANK_READY 86400000ul //How long water in tank is considered as hot
#define TIME_TO_RESET 140000	// Time to completely close the 3WV if position is unknow
#define MAX_TEMP 800		// Temperature threshold for emitter (40°C for floor heating)
#define EPSILON_TEMP 10		// difference between the two last temperature to do nothing (door or window opened in zone)
#define TIME_HW_ON 9000000
#define MAX_SERVO 100
#define MIN_STEP 2
#define MAX_3WAY 100

#define main_PID_Kp 4  //main PID coeff
#define main_PID_Ki 4
#define main_PID_Kd 0
#define way3_PID_Kp 1.9 //3 way valve PID coeff
#define way3_PID_Ki 1
#define way3_PID_Kd .2

#include <SPI.h>
#include <MySensors.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <HotWatertank.h>
#include <stdlib.h>
#include <HeatingZone.h>

OneWire oneWire(ONE_WIRE_BUS);	// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);	// Pass the oneWire reference to Dallas Temperature.

int devices_hotwater_index[2] = {0,1}; // table for conversion between index of ds18b20 sensors and stored temperature in (int Temperature[3]).
int devices_heatingzone_index[3] = {2,3,4}; // table for conversion between index of ds18b20 sensors and stored temperature in (int Temperature[3]).

HotWatertank hotwater(&sensors,devices_hotwater_index, RELAY_HW_PUMP,RELAY_ELEC);
HeatingZone heatingzone(&sensors,devices_heatingzone_index, RELAY_PUMP, SERVO_ON, SERVO_DIRECTION);

//index for Temperature[3]:
//index 0: ambiant temperature ds18b20
//index 1: input of thermal emitter ds18b20
//index 2: output of thermal emitter ds18b20



void setup()
{
	Serial.begin(115200);	// output
	sensors.begin();
	sensors.setWaitForConversion(true);
  hotwater.begin();
	heatingzone.begin();
  heatingzone.reset();
}

void presentation()
{
	sendSketchInfo("HotWater Sensor", "1.1");
	// Present all sensors to controller
	hotwater.presentation();
	heatingzone.presentation();
}

void loop()
{
	hotwater.update();
	heatingzone.update();
  wait(3000);
}

void receive(const MyMessage & message)
{
	hotwater.receive(message);
	heatingzone.receive(message);
}
