#include <Arduino.h>

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
#define LLCD
#define ENERGY_METER		//define 3 phases electrical comsumption feature
#define PPID            //PID for temperature and 3-ways valve control
#define EEPROM_BACKUP		//thermostat temperature saved in EEPROM.
//#define PRINT_RAM



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
#define MY_GATEWAY_SERIAL
#define MY_NODE_ID 0
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
#include "ids.h"

#define ONE_WIRE_BUS 7		// Pin where dallas sensor is connected
#define KNOB_ENC_PIN_1 2	// Rotary encoder input pin 1
#define KNOB_ENC_PIN_2 3	// Rotary encoder input pin 2
#define ENCODER_SW 4      // Rotary encoder switch pin
#define HW_SWITCH_LED 11    //led turned On hotwatertank
#define HW_SWITCH 18    //switch to turn On hotwatertank
//#define KNOB_BUTTON_PIN 3       // Rotary encoder button pin
#define RELAY_PUMP 6		// Relay that turns the circulator on/off
#define SERVO_ON 5		// Relay that powers the 3WV actuator
#define SERVO_DIRECTION 8	// Relay that sets the 3WV actuator direction
#define TIME_KNOB 2000          // time before taking into account that knob has changed
#define STEP_CLOSE 1400ul	// duration of a single step while closing the valve in ms
#define STEP_OPEN 1400ul	// duration of a single step while opening the valve in ms
#define DELAY_PUMP_BUTTON 5000	// How long to delay turning the circulator on?
#define INTERVAL_REG 45000	// Interval between 3VW temperature checks
#define INTERVAL_MAIN_REG 900000	// Interval between Ambiant temperature checks
#define TIME_TO_RESET 140000	// Time to completely close the 3WV if position is unknow
#define MAX_TEMP 380		// Temperature threshold for emitter (40°C for floor heating)
#define EPSILON_TEMP 10		// difference between the two last temperature to do nothing (door or window opened in zone)
#define MAX_SERVO 100
#define MIN_STEP 2
#define MAX_3WAY 100

#define main_PID_Kp 2  //main PID coeff
#define main_PID_Ki 2
#define main_PID_Kd 0
#define way3_PID_Kp .2 //3 way valve PID coeff
#define way3_PID_Ki 5
#define way3_PID_Kd .1

#include <SPI.h>
#include <MySensors.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <stdlib.h>
#include <HeatingZone.h>

OneWire oneWire(ONE_WIRE_BUS);	// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);	// Pass the oneWire reference to Dallas Temperature.

int devices_heatingzone_index[3] = {0,1,2}; // table for conversion between index of ds18b20 sensors and stored temperature in (int Temperature[3]).
//index for Temperature[3]:
//index 0: ambiant temperature ds18b20
//index 1: input of thermal emitter ds18b20
//index 2: output of thermal emitter ds18b20

#ifdef ENERGY_METER
#include "energy_meter.h"		// Include Emon Library
Energy_meter energy_meter(&lcd);
#endif

#ifdef HW_SWITCH    //switch to turn manually on HotWatertank
#include "HW_switch.h"
HW_Switch hw_switch;
#endif


HeatingZone heatingzone(&sensors,devices_heatingzone_index, RELAY_PUMP, SERVO_ON, SERVO_DIRECTION);

//long debouncing_time = 5; //Debouncing Time in Milliseconds
//volatile unsigned long last_micros;
//MyMessage message_button(CHILD_ID_HW_LEVEL,S_DIMMER);

#ifdef PRINT_RAM
float Ram = 0;
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
#endif

//MyMessage msg_hw_led{CHILD_ID_HW_LED, V_STATUS};	//three temp

// void hotwater_on()
// {
//   uint8_t value;
//   delay(200);
//
//   value = digitalRead(HW_SWITCH);
//   if(value == HIGH) {
//   #ifdef DEBUG
//     Serial.print("\nButton UP");
//   #endif
//   // message_button.set(13);
//   // message_button.setDestination(HOTWATER_ID);
//   // send(message_button);
//   }
// }

void setup()
{
	Serial.begin(115200);	// output
  delay(3000); //delay to wait if a skecth upload is started after reset. Not start relay for 3WV init.
	sensors.begin();
	sensors.setWaitForConversion(true);
	heatingzone.begin();
  #ifdef ENERGY_METER
  energy_meter.begin();
  #endif
  #ifdef HW_SWITCH_LED
    hw_switch.begin();
  #endif
	heatingzone.reset();


}

void presentation()
{
	sendSketchInfo("Gateway&Heating floor Sensor", "1.1");
	// Present all sensors to controller
	heatingzone.presentation();
#ifdef ENERGY_METER
	energy_meter.presentation();
#endif

//present(CHILD_ID_HW_LED, S_DIMMER);
}

void loop()
{
	heatingzone.update();

#ifdef ENERGY_METER
  energy_meter.update();
#endif
#ifdef HW_SWITCH_LED
  hw_switch.update();
#endif
	//delay(1000);
}

void receive(const MyMessage &message)
{
	heatingzone.receive(message);
  #ifdef HW_SWITCH_LED
    hw_switch.receive(message);
  #endif
}
