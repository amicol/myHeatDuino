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
#define PPID
#define EEPROM_BACKUP		//thermostat temperature saved in EEPROM.
//#define PRINT_RAM



// Enable debug prints to serial monitor
//#define MY_DEBUG 
//#define MY_DEBUG_VERBOSE
//#define DEBUG //to big with 32k card TODO : reduce string memory footprint.

// Set LOW transmit power level as default, if you have an amplified NRF-module and
// power your radio separately with a good regulator you can turn up PA level. 
//#define MY_RF24_PA_LEVEL RF24_PA_LOW

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Enable serial gateway
#define MY_GATEWAY_SERIAL

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

#include "ids.h"

#define KNOB_ENC_PIN_1 2	// Rotary encoder input pin 1
#define KNOB_ENC_PIN_2 3	// Rotary encoder input pin 2
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

#ifdef OLED
#define MYDISPLAY
#include <U8glib.h>
#include <Encoder.h>
//Have to choose in library u8g for init...
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST);	// Fast I2C / TWI 
//Init Rotary encoder
Encoder knob(KNOB_ENC_PIN_1, KNOB_ENC_PIN_2);
#endif

#ifdef EEPROM_BACKUP
#include <EEPROM.h>
#define EEPROM_ENC_POS 513
byte encoderPos = EEPROM.read(EEPROM_ENC_POS);	//TODO: replace with mysensors SaveState.
#else
byte encoderPos = 200;		//Target temperature for setting knob position
#endif

#include <SPI.h>
#include <MySensor.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#include <stdlib.h>



#ifdef LLCD
#define MYDISPLAY
#include <Encoder.h>
#include <Wire.h>  // Comes with Arduino IDE
// Get the LCD I2C Library here: 
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
// Move any other LCD libraries to another folder or delete them
// See Library "Docs" folder for possible commands etc.
#include <LiquidCrystal_I2C.h>

/*-----( Declare Constants )-----*/
//none
/*-----( Declare objects )-----*/
// set the LCD address to 0x20 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
Encoder knob(KNOB_ENC_PIN_1, KNOB_ENC_PIN_2);
#endif

#ifndef MYDISPLAY
#undef PRINT_RAM
#endif

//Temperature
//#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No. TODO: Not used
#include <HeatingZone.h>

OneWire oneWire(ONE_WIRE_BUS);	// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);	// Pass the oneWire reference to Dallas Temperature. 

int devices_heatingzone_index[3] = {0,1,2}; // table for conversion between index of ds18b20 sensors and stored temperature in (int Temperature[3]).
//index for Temperature[3]:
//index 0: ambiant temperature ds18b20
//index 1: input of thermal emitter ds18b20
//index 2: output of thermal emitter ds18b20


// servoPosition;
long newPos;
unsigned long knob_changing;

#ifdef ENERGY_METER
#include "EmonLib.h"		// Include Emon Library
EnergyMonitor emon[3];
double Irms[3];			//3 phases
unsigned long start;		//millis()-start makes sure it doesnt get stuck in the loop if there is an error.
unsigned long timeout = 10000;
MyMessage msg_watt(CHILD_ID_ENERGY, V_WATT);
#endif

HeatingZone heatingzone(&sensors,devices_heatingzone_index, RELAY_PUMP, SERVO_ON, SERVO_DIRECTION);
#ifdef PRINT_RAM
float Ram = 0;
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
#endif

#ifdef OLED
char charVal[6];
void init_display(void)
{
}
void draw(void)
{
	// graphic commands to redraw the complete screen should be placed here  
	//u8g.setFont(u8g_font_osb21);
	u8g.firstPage();
	do {
		u8g.setPrintPos(0, 10);
		u8g.print(F("T_des:"));
		dtostrf(encoderPos / 10., 5, 1, charVal);
		u8g.print(charVal);
		u8g.print(char (176));
		u8g.print(F("C"));

		u8g.setPrintPos(0, 20);
		u8g.print(F("position:"));
		dtostrf(heatingzone.servoPosition, 4, 0, charVal);
		u8g.print(charVal);
		u8g.print(F(" %"));

		//u8g.setPrintPos(60, 20);
		//u8g.print(F("/+"));
		//dtostrf(heatingzone.Output, 4, 0, charVal);
		//u8g.print(charVal);

		u8g.setPrintPos(0, 30);
		u8g.print(F("T_int:"));
		dtostrf(heatingzone.Temperature[0] / 10., 5, 1, charVal);
		u8g.print(charVal);
		u8g.print(char (176));
		u8g.print(F("C"));


		u8g.setPrintPos(0, 40);
		u8g.print(F("T_dep :"));
		dtostrf(heatingzone.Temperature[1] / 10., 5, 1, charVal);
		u8g.print(charVal);
		u8g.print(char (176));
		u8g.print(F("C"));

	//	u8g.setPrintPos(60, 40);
	//	u8g.print(F("/"));
	//	dtostrf(heatingzone.target_3WV , 4, 0, charVal);
	//	u8g.print(charVal);
	//	u8g.print(char (176));
	//	u8g.print(F("C"));


		u8g.setPrintPos(0, 50);
		u8g.print(F("T_ret :"));
		dtostrf(heatingzone.Temperature[2] / 10., 5, 1, charVal);
		u8g.print(charVal);
		u8g.print(char (176));
		u8g.print(F("C"));

		//u8g.setPrintPos(60, 50);
		//u8g.print(F("/"));
		//dtostrf(heatingzone.main_Output, 5, 1, charVal);
		//u8g.print(charVal);
		//u8g.print(char (176));
		//u8g.print(F("C"));

#ifdef PRINT_RAM
		u8g.setPrintPos(50, 10);
		u8g.print(F("RAM :"));
		u8g.print(freeRam ());
#endif
		//u8g.drawFrame(100,40,20,30);
		//u8g.drawBox(100,50,20,20);
	} while (u8g.nextPage());
}
#endif

#ifdef LLCD
char charVal[6];
void init_display(void)
{
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(F("T"));
		lcd.print((char)223);
		lcd.print(F(":"));
		
		lcd.setCursor(6, 0);
		lcd.print(F("? /"));
		lcd.setCursor(12, 0);
		lcd.print(F("? /  ?"));

		lcd.setCursor(0, 1);
		lcd.print(F("Pos:"));
		lcd.setCursor(7, 1);
		lcd.print(F("%"));
		lcd.setCursor(9, 1);
		lcd.print(F("P:"));
		lcd.setCursor(16, 1);
		lcd.print(F("W"));

		lcd.setCursor(0, 2);
		lcd.print("PC:");
		lcd.setCursor(6, 2);
		lcd.print(F("? /"));
		lcd.setCursor(12, 2);
		lcd.print(F("? /  ?"));

		lcd.setCursor(0, 3);
		lcd.print(F("Ch:"));
		lcd.setCursor(6, 3);
		lcd.print(F("? /"));
		lcd.setCursor(12, 3);
		lcd.print(F("? /  ?"));
#ifdef PRINT_RAM
		lcd.setCursor(10, 1);
		lcd.print(F("RAM:"));
		lcd.print(freeRam ());
#endif
}
void draw(void)
{

		lcd.setCursor(3, 0);
		dtostrf(encoderPos / 10., 5, 1, charVal);
		lcd.print(charVal);
		lcd.setCursor(9, 0);
		dtostrf(heatingzone.Temperature[0] / 10., 5, 1, charVal);
		lcd.print(charVal);

		lcd.setCursor(3, 1);
		dtostrf(heatingzone.servoPosition, 4, 0, charVal);
		lcd.print(charVal);

		//lcd.setCursor(60, 20);
		//lcd.print(F("/+"));
		//dtostrf(heatingzone.Output, 4, 0, charVal);
		//lcd.print(charVal);


		lcd.setCursor(3, 2);
		dtostrf(heatingzone.target_3WV, 5, 1, charVal);
		lcd.print(charVal);
		lcd.setCursor(9, 2);
		dtostrf(heatingzone.Temperature[1] / 10., 5, 1, charVal);
		lcd.print(charVal);
		lcd.setCursor(15, 2);
		dtostrf(heatingzone.Temperature[2] / 10., 5, 1, charVal);
		lcd.print(charVal);
	//	lcd.setCursor(60, 40);
	//	lcd.print(F("/"));
	//	dtostrf(heatingzone.target_3WV , 4, 0, charVal);
	//	lcd.print(charVal);
	//	lcd.print(char (176));
	//	lcd.print(F("C"));
		
		
#ifdef PRINT_RAM
		lcd.setCursor(15, 1);
		lcd.print(freeRam ());
#endif
}
#endif

void setup()
{
	Serial.begin(115200);	// output
#ifdef MYDISPLAY
	knob.write(encoderPos);
#endif
        delay(3000); //delay to wait if a skecth upload is started after reset. Not start relay for 3WV init.
	sensors.begin();
	lcd.begin(20,4);               // initialize the lcd
	sensors.setWaitForConversion(true);
	heatingzone.begin();
	heatingzone.target = encoderPos;
#ifdef OLED
	// flip screen, if required
	u8g.setRot180();

	// set SPI backup if required
	//u8g.setHardwareBackup(u8g_backup_avr_spi);

	// assign default color value
	u8g.setColorIndex(255);	// white

	//u8g.setFont(u8g_font_unifontr);
	//u8g.setFont(u8g_font_pixelle_micror);
	u8g.setFont(u8g_font_6x13r);
	//u8g.setFont(u8g_font_micro);
	u8g.firstPage();
	do {
		u8g.setPrintPos(0, 30);
		u8g.print(F("Init 3 voies"));
	} while (u8g.nextPage());
#endif

#ifdef LLCD
		lcd.setCursor(2, 1);
		lcd.print(F("Init 3 voies"));
#endif

#ifdef ENERGY_METER
	//emon1.voltage(2, 234.26, 1.7);  // Voltage: input pin, calibration, phase_shift
	for (int i = 0 ; i < 3; i++) {    //voltage on pni A1,A2,A3
		emon[i].current(i, 60.6); //voltage on pni A1,A2,A3
	}
	start = millis();
#endif

	heatingzone.reset();
#ifdef LLCD
	init_display();
#endif
	heatingzone.update_oled = 1;
}

void presentation()
{
	sendSketchInfo("Gateway&Heating floor Sensor", "1.1");
	// Present all sensors to controller
	heatingzone.presentation();
#ifdef ENERGY_METER
	for (int i = CHILD_ID_ENERGY ; i < CHILD_ID_ENERGY+4; i++) {
		present(i, S_POWER);
	}
#endif
}

void loop()
{
	heatingzone.update();
#ifdef MYDISPLAY
	if (heatingzone.update_oled == 1) {
		draw();
		heatingzone.update_oled = 0;
	}
	newPos = knob.read();
	if (newPos != encoderPos) {
		encoderPos = newPos;
		knob_changing = millis();
		draw();

	}
	if (knob_changing != 0 && millis() - TIME_KNOB > knob_changing) {
		encoderPos = newPos;
#ifdef EEPROM_BACKUP
		EEPROM.write(EEPROM_ENC_POS, encoderPos);
#endif
		heatingzone.target = encoderPos;
		send(msg_target.set(encoderPos / 10., 1));
#ifdef DEBUG
		Serial.print(F("\ntarget:"));
		Serial.print(heatingzone.target);
		Serial.print(F("\nposition:"));
		Serial.print(heatingzone.servoPosition);
		Serial.print(F("\n"));
#endif
		heatingzone.regulate_on = true;
		send(msg_S_HEATER_FLOW_STATE.set(1), 1);
   		heatingzone.adjust_PID();
		heatingzone.regulate();
		knob_changing = 0;
	}
#endif
#ifdef ENERGY_METER
	if ((millis() - start) > timeout) {
		for (int i = 0 ; i < 3; i++) {
		Irms[i] = emon[i].calcIrms(1480);	// Calculate Irms only
		send(msg_watt.setSensor(CHILD_ID_ENERGY+i).set(Irms[i] * 230.0,1));
	}
		//Serial.print(Irms*230.0);            // Apparent power
		//Serial.print(" ");
		//Serial.println(Irms);                // Irms
		send(msg_watt.setSensor(CHILD_ID_ENERGY+4).set((Irms[0] + Irms[1] + Irms[2]) * 230.0, 1));
		lcd.setCursor(11, 1);
		dtostrf((Irms[0] + Irms[1] + Irms[2]) * 230.0, 5, 0, charVal);
		lcd.print(charVal);
		
		start = millis();
	}
#endif
#ifdef PRINT_RAM
	  if (Ram != freeRam()) {
	      Ram = freeRam();
	      draw();
	      //send(msgfreeram.set(Ram,1));
	      }
#endif
	//delay(1000);
}

void receive(const MyMessage & message)
{
	if (message.type == V_HVAC_SETPOINT_HEAT) {
#ifdef DEBUG
		Serial.print("\nIncoming change for sensor:");
		Serial.print(message.sensor);
		Serial.print("\n, New status: ");
		Serial.println(message.getFloat());
#endif
		//#endif
		heatingzone.target = int (message.getFloat() * 10);
#ifdef MYDISPLAY
		//encoderPos = heatingzone.target;
		knob.write(heatingzone.target);
		heatingzone.update_oled = 1;
#endif
	} 
	else if ((message.sender == 0) && message.type == V_STATUS) {
#ifdef DEBUG
		Serial.print("\nIncoming change for sensor:");
		Serial.print(message.sensor);
		Serial.print("\n, New status: ");
		Serial.println(message.getBool());
#endif
		if (message.getBool() == true) {
			heatingzone.regulate_on = true;
		} else {
			heatingzone.regulate_on = false;
		}
		heatingzone.adjust_PID();
		heatingzone.regulate();
	} 
	else if ((message.sender == 1) && (message.type == V_STATUS)) {
#ifdef DEBUG
		Serial.print("\nIncoming change for sensor:");
		Serial.print(message.sensor);
		Serial.print("\nIncoming change for type:");
		Serial.print(message.type);
		Serial.print("\nIncoming change for sender:");
		Serial.print(message.sender);
		Serial.print("\n, New status: ");
		Serial.println(message.getBool());
		
#endif
		if ((message.getBool() == true)
		    && (heatingzone.regulate_on == false)) {
			heatingzone.regulate_on = true;
			heatingzone.adjust_PID();
			heatingzone.regulate();
			send(msg_S_HEATER_FLOW_STATE.set(1), 1);
		}
	} 
	else if ((message.sender == 1) && (message.sensor == 10)) {

		lcd.setCursor(3, 3);
		dtostrf(message.getFloat(), 5, 1, charVal);
		lcd.print(charVal);

	}
	else if ((message.sender == 1) && (message.sensor == 11)) {

		lcd.setCursor(9, 3);
		dtostrf(message.getFloat(), 5, 1, charVal);
		lcd.print(charVal);

	}
	else if ((message.sender == 1) && (message.sensor == 12)) {

		lcd.setCursor(15, 3);
		dtostrf(message.getFloat(), 5, 1, charVal);
		lcd.print(charVal);

	}
	else if ((message.sender == 1) && (message.sensor == 16)) {

		lcd.setCursor(15, 0);
		dtostrf(message.getFloat(), 5, 1, charVal);
		lcd.print(charVal);

	}
	//else {
	//	Serial.print("\nIncoming change for sensor:");
	//	Serial.print(message.sensor);
	//	Serial.print("\nIncoming change for type:");
	//	Serial.print(message.type);
	//	Serial.print("\nIncoming change for sender:");
	//	Serial.print(message.sender);
	//}
}
