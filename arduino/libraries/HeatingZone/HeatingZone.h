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
Parts of code by https://github.com/biuklija/Central_Heating_Regulation

*/

#ifndef HeatingZone_h
#define HeatingZone_h

#include "Arduino.h"
#include <DallasTemperature.h>
#include <SPI.h>
#ifdef EEPROM_BACKUP
	#include <EEPROM.h>
#endif

#ifdef PPID
	#include "PID_v1.h"
#endif
#include <MySensors.h>
#include "OneWire.h"

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
// #ifdef MYDISPLAY
// 	#include <ClickEncoder.h>
// 	#include <TimerOne.h>
// #endif

#ifndef MYDISPLAY
	#undef PRINT_RAM
#endif

MyMessage msg_temp(CHILD_ID_TEMP, V_TEMP);	//three temp
MyMessage msg_S_HEATER_FLOW_STATE(CHILD_ID_HEAT_STATE,V_STATUS);
MyMessage msg_target(CHILD_ID_SET_POINT, V_HVAC_SETPOINT_HEAT);
MyMessage msg_target_3WV(CHILD_ID_3WV, V_TEMP);
MyMessage msg_pos_3WV(CHILD_ID_3WV_POS, V_PERCENTAGE);

class HeatingZone
{
  private:
    uint8_t _PumpPin;
    DallasTemperature *_sensors;

    // variables for the 3WV actuator
    unsigned long stopServoAt; // when to switch off the actuator
    uint8_t _ServoOn;
    unsigned long lastRegulation;
    unsigned long last_main_Regulation;
    //unsigned long buttonRegulationStateChange;
    //unsigned long pumpButtonStateChange; // We need to know when the pump button was last pressed, to delay turning it on/off

  public:
    uint8_t _ServoDirection;
    uint8_t servoPosition; // position of the actuator
    int Temperature[3];
    int Temperature_prev[3] = {0,0,0};
    int *_ds18b20_index;
    bool update_oled;
    bool regulate_on;
		bool boiler_on;
    bool start_circulator;


    #ifdef MYDISPLAY
		// servoPosition;
		//ClickEncoder *encoder;
    long newPos;
    unsigned long knob_changing;
    char charVal[6];

    void init_display(void);
    void draw(void);
    #endif

    HeatingZone(DallasTemperature *sensors, int *ds18b20_index, uint8_t PumpPin, uint8_t ServoOn, uint8_t ServoDirection );

    double target;
    #ifdef PPID
      double Output;
      double input;
      double target_3WV;
      PID *way3_PID;
      //    double main_Output;
      double main_input;
      PID *main_PID;
    #endif

    //void timerIsr(void);
    void begin(void);
    void presentation(void);
    void receive(const MyMessage &message);
    void reset(void);
    void reset_PID(void);
    void adjust_PID(void);
    void regulate(void);
    void update(void);
    void getTempsC(void);
    void openValve(int num_step);
    void closeValve(int num_step);
    void stopServo(void);
    void setServoTo(int requestedPosition);
    void circulatorOff(void);
    void circulatorOn(void);


};

HeatingZone::HeatingZone(DallasTemperature *sensors, int *ds18b20_index, uint8_t PumpPin, uint8_t ServoOn, uint8_t ServoDirection)
{
	_sensors = sensors;
	//_addresses = addresses;
	_PumpPin = PumpPin;
	_ServoOn = ServoOn;
	_ServoDirection = ServoDirection;
        _ds18b20_index = ds18b20_index;
	boiler_on = 0;
	start_circulator = false;
	#ifdef EEPROM_BACKUP
		target = EEPROM.read(EEPROM_ENC_POS);
		msg_target.set((float) target/10.,1);
		regulate_on= EEPROM.read(EEPROM_ENC_POS+1);
		//servoPosition = EEPROM.read(0);
	#else
    		target = 200;
		    regulate_on = 0;
	#endif
	#ifdef MYDISPLAY
	//encoder = new ClickEncoder(KNOB_ENC_PIN_1, KNOB_ENC_PIN_2, ENCODER_SW);
	// Timer1.initialize(1000);
  // Timer1.attachInterrupt(timerIsr);
	#endif
  msg_S_HEATER_FLOW_STATE.set(regulate_on);
	lastRegulation = 0;
	last_main_Regulation = 0;
	//reset_PID();

}

void HeatingZone::begin(void)
{
	pinMode(_PumpPin, OUTPUT);
	pinMode(_ServoOn, OUTPUT);
	pinMode(_ServoDirection, OUTPUT);
	//pinMode(ENCODER_SW, INPUT);
	//digitalWrite(ENCODER_SW, HIGH); //turn pullup resistor on
	digitalWrite(_PumpPin, HIGH);
	digitalWrite(_ServoDirection,HIGH);
	digitalWrite(_ServoOn, HIGH);
	getTempsC();

	#ifdef PPID
			//#error error
			//Specify the links and initial tuning parameters
	    target_3WV = target;
			msg_target_3WV.set((float) target_3WV,1);
			main_input = Temperature[0];
			main_PID = new PID (&main_input, &target_3WV, &target,main_PID_Kp, main_PID_Ki, main_PID_Kd, DIRECT);

			Output = 0;
			input = Temperature[1]/10.;
			way3_PID = new PID (&input, &Output, &target_3WV,way3_PID_Kp, way3_PID_Ki, way3_PID_Kd, DIRECT);

			//turn the PID on
	 		way3_PID->SetMode(AUTOMATIC);
			main_PID->SetMode(AUTOMATIC);

			way3_PID->SetOutputLimits(0, MAX_3WAY);
			main_PID->SetOutputLimits(15., MAX_TEMP/10.);

			#ifdef MYDISPLAY
			knob.write(encoderPos);
			#endif

			adjust_PID();


			#ifdef LLCD
			lcd.begin(20,4);
			#endif
			//main_PID->Initialize();
		#endif
	//reset_PID();
	//Serial.print("\ntarget_3WV:");
	//Serial.print(target_3WV);
}
void HeatingZone::reset(void)
{
  #ifdef OLED
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
	//stopServoAt = millis() + TIME_TO_RESET;
	digitalWrite(_ServoDirection,HIGH);
	digitalWrite(_ServoOn, LOW);
	circulatorOn();
	wait(TIME_TO_RESET);
	digitalWrite(_ServoOn,HIGH);
	servoPosition = 0;
	send(msg_pos_3WV.set(servoPosition));

	regulate();

  #ifdef LLCD
	  init_display();
  #endif
	update_oled = 1;
}
// void HeatingZone::timerIsr() {
//   encoder->service();
// }
//}
void HeatingZone::reset_PID(void)
{
		start_circulator = true;
		way3_PID->SetMode(MANUAL);
		Output = 0;
		setServoTo(0);
		way3_PID->SetMode(AUTOMATIC);
}

void HeatingZone::presentation()
{
  for (int i = CHILD_ID_TEMP; i < CHILD_ID_TEMP+3; i++) {
		present(i, S_TEMP);
	}
  present(CHILD_ID_HEAT_STATE, S_HVAC);
  present(CHILD_ID_SET_POINT, S_HVAC);
  present(CHILD_ID_3WV, S_TEMP);
	present(CHILD_ID_3WV_POS, S_DIMMER);
}

void HeatingZone::update(void)
{
		stopServo(); // stop the servo if necessary

    #ifdef MYDISPLAY
	    if (update_oled == 1) {
     		draw();
	    	update_oled = 0;
    	}
			// if(digitalRead(ENCODER_SW)){
    	// 	//button is not being pushed
  		// }else{
			// 	Serial.print("\nENCODER_SW: ");
			// 	if (regulate_on == true) {regulate_on=false;}
			// 	else
			// 	{
			// 	regulate_on = true;
			//   }
			// 	send(msg_S_HEATER_FLOW_STATE.set(regulate_on));
			// 	#ifdef EEPROM_BACKUP
			// 		EEPROM.write(EEPROM_ENC_POS+1, regulate_on);
			//   #endif
    	// 	//button is being pushed
  		// }
	    newPos = knob.read();

			//newPos += encoder->getValue();
			if (newPos != encoderPos) {
				Serial.print("\nnewPos: ");
				Serial.println(newPos);
				encoderPos = newPos ;
				//knob.write(encoderPos);
				knob_changing = millis();
				draw();

			}
			if (knob_changing != 0 && millis() - TIME_KNOB > knob_changing) {
				encoderPos = newPos;
		  #ifdef EEPROM_BACKUP
				EEPROM.write(EEPROM_ENC_POS, encoderPos);
		  #endif
				target = encoderPos;
				send(msg_target.set(encoderPos / 10., 1));
		  #ifdef DEBUG
				Serial.print(F("\ntarget:"));
				Serial.print(target);
				Serial.print(F("\nposition:"));
				Serial.print(servoPosition);
				Serial.print(F("\n"));
		  #endif
     	adjust_PID();
	  	regulate();
	  	knob_changing = 0;
    	}
    #endif
    #ifdef PRINT_RAM
	    if (Ram != freeRam()) {
	      Ram = freeRam();
	      draw();
	      }
    #endif

		if (millis()-INTERVAL_REG >= lastRegulation )// time for regulation if the servo is not running
		{
			regulate();
			send(msg_target);
			send(msg_S_HEATER_FLOW_STATE);
			send(msg_target_3WV);
			//if (Temperature[1]>MAX_TEMP)
			//{
			//	closeValve(5);
			//}
		}
		if ((millis() >= last_main_Regulation +INTERVAL_MAIN_REG) )
		{
			adjust_PID();
		}

}

void HeatingZone::adjust_PID(void){
			main_input = Temperature[0];
			main_PID->Compute();

			//Serial.print("\nt_3WV:");
			//Serial.print(target_3WV);
			//Serial.print(" ");
			//Serial.print(main_Output);
			//Serial.print("\nMOutput :");
			//Serial.print(target_3WV);
        	        //Serial.print("\n");

			send(msg_target_3WV.set((float) target_3WV,1));
			update_oled = 1;
			last_main_Regulation = millis();
}

void HeatingZone::regulate(void){
		getTempsC();
		#ifdef DEBUG
			Serial.print("\nTemp0"+String(Temperature[0]));
			//Serial.print("\nTemp1"+String(Temperature[1]));
			//Serial.print("\n_ServoOn "+String(digitalRead(_ServoOn)));
			//Serial.print("\nservoPosition "+String(servoPosition)+"\n");
		#endif

			if (regulate_on == true)
			{
				if (abs(Temperature[0]-Temperature_prev[0]) < EPSILON_TEMP)
				{
					#ifdef PPID
						input = Temperature[1]/10.;
						//Serial.print("\nint:");
						//Serial.print(input);
						way3_PID->Compute();
						//Serial.print("\nt3WV:");
						//Serial.print(target_3WV);
						//Serial.print("\n");
						//Serial.print("\nOut:");
						//Serial.print(Output);
						//Serial.print("\n");

        		        	        //Serial.print("\n");
						if (abs(servoPosition - Output)>=MIN_STEP)
						{
							setServoTo(int(Output));
						}
					#else

						if (Temperature[1]<MAX_TEMP -50 && (Temperature[1]-Temperature_prev[1]<10))
						{
							openValve(5);
						}
						else if (Temperature[1]>MAX_TEMP)
						{
							closeValve(5);
						}
					#endif
				}
				// if ( (servoPosition > 5) && (servoPosition < 95) )
				if (servoPosition <= MAX_SERVO)
				 {
				 	circulatorOn();
				 }
				//
				else
				// {
				// 	circulatorOff();
				// }
				//if (servoPosition > 95)
				{
					circulatorOff();
				}
			}
			else
				{
				circulatorOff();
				}
			lastRegulation = millis();
}

void  HeatingZone::getTempsC(void)
{
		_sensors->requestTemperatures();
		// query conversion time and sleep until conversion completed
  		//int16_t conversionTime = _sensors->millisToWaitForConversion(_sensors->getResolution());
  		// sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
  		//sleep(conversionTime);

		for( unsigned int i = 0; i < 3; i = i + 1 )
		{
		// Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((getConfig().isMetric?_sensors->getTempCByIndex(_ds18b20_index[i]):_sensors->getTempFByIndex(_ds18b20_index[i])) * 10.));
		Temperature[i] = temperature; // store temperature data into position [i] of the array
		send(msg_temp.setSensor(CHILD_ID_TEMP+i).set(temperature/10.,1));
		#ifdef MYDISPLAY
			if (Temperature_prev[i] != Temperature[i])
			{
				update_oled = 1;
			}
		#endif
		Temperature_prev[i] = Temperature[i];
		#ifdef DEBUG
			Serial.print("\nTemperature "+String(i)+" ");
			Serial.print(Temperature[i]);
		#endif
		}
}

void HeatingZone::openValve(int num_step)
{
	if ((servoPosition+num_step <= 100) && digitalRead(_ServoOn) == HIGH && Temperature[1]<MAX_TEMP)
	{
		#ifdef DEBUG
			Serial.print("\nopen valve");
		#endif
		stopServoAt = millis() + num_step*STEP_OPEN;
		digitalWrite(_ServoDirection,LOW);
		digitalWrite(_ServoOn, LOW);
		servoPosition = servoPosition + num_step;
		send(msg_pos_3WV.set(servoPosition));
		#ifdef MYDISPLAY
			update_oled = 1;
		#endif
                lastRegulation = millis();
	}
}

void HeatingZone::closeValve(int num_step)
{
	if ((servoPosition - num_step >= 0) && digitalRead(_ServoOn) == HIGH)
	{
		#ifdef DEBUG
			Serial.print("\nclose valve");
		#endif
		stopServoAt = millis() + num_step*STEP_CLOSE;
		digitalWrite(_ServoDirection,HIGH);
		digitalWrite(_ServoOn, LOW);
		servoPosition = servoPosition - num_step;
		send(msg_pos_3WV.set(servoPosition));
		#ifdef MYDISPLAY
			update_oled = 1;
		#endif
                lastRegulation = millis();
	}
}

void HeatingZone::stopServo(void)
{	//Serial.print(stopServoAt-millis());
		//#ifdef DEBUG
		//if (stopServoAt > millis())
		//{
		//	Serial.print("\nStop servo at :");
		//	Serial.print(stopServoAt-millis());
		//}
		//#endif

	if (stopServoAt <= millis() && digitalRead(_ServoOn) == LOW)
	{
		#ifdef DEBUG
			Serial.print("\nStop servo");
		#endif
		digitalWrite(_ServoDirection,HIGH);
		digitalWrite(_ServoOn, HIGH);
		if ((start_circulator == true) && (Temperature[0]) < (target  - 5))
		{
			circulatorOn();
		}
                start_circulator = false;
		#ifdef DEBUG
			Serial.print("\n_ServoOn :"+String(digitalRead(_ServoOn)));
			if (digitalRead(_ServoOn) == LOW)
				{
				Serial.print("NOOK");
				}
		#endif
		//#if TO_EEPROM
	//		#error error
	//		byte staro = EEPROM.read(0);
	//		if (staro != servoPosition)
	//		{
	//			EEPROM.write(0,servoPosition);
	//		}
	//	#endif
	}
}

void HeatingZone::setServoTo(int requestedPosition)
{
	if (requestedPosition != servoPosition && digitalRead(_ServoOn) == HIGH )
	{
		if (requestedPosition>95) {requestedPosition=100;}
		if (requestedPosition<5) {requestedPosition=0;}
		int numberOfSteps = requestedPosition - servoPosition;
		if (numberOfSteps > 0) // if the number is positive or higher than current, open the valve and use X steps
		{
			if (requestedPosition == 100) numberOfSteps+=20;
			stopServoAt = millis() + (abs(numberOfSteps) * STEP_OPEN);
			digitalWrite(_ServoDirection, LOW);
			digitalWrite(_ServoOn, LOW);

		}
		else if (numberOfSteps < 0)
		{
			if (requestedPosition == 0) numberOfSteps+=20;
			stopServoAt = millis() + (abs(numberOfSteps)* STEP_CLOSE);
			digitalWrite(_ServoDirection, HIGH);
			digitalWrite(_ServoOn, LOW);

		}
		servoPosition = requestedPosition;
		send(msg_pos_3WV.set(servoPosition));
		#ifdef MYDISPLAY
			update_oled = 1;
		#endif
	}
}
void HeatingZone::circulatorOff(void)
{
	if (digitalRead(_PumpPin) == LOW)
	{
		digitalWrite(_PumpPin, HIGH);
	}
}
void HeatingZone::circulatorOn(void)
{
	if (digitalRead(_PumpPin) == HIGH)
	{
		digitalWrite(_PumpPin, LOW);
	}
}
#ifdef OLED
 void HeatingZone::init_display(void)
 {
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

  }
  void HeatingZone::draw(void)
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
		dtostrf(servoPosition, 4, 0, charVal);
		u8g.print(charVal);
		u8g.print(F(" %"));

		//u8g.setPrintPos(60, 20);
		//u8g.print(F("/+"));
		//dtostrf(Output, 4, 0, charVal);
		//u8g.print(charVal);

		u8g.setPrintPos(0, 30);
		u8g.print(F("T_int:"));
		dtostrf(Temperature[0] / 10., 5, 1, charVal);
		u8g.print(charVal);
		u8g.print(char (176));
		u8g.print(F("C"));


		u8g.setPrintPos(0, 40);
		u8g.print(F("T_dep :"));
		dtostrf(Temperature[1] / 10., 5, 1, charVal);
		u8g.print(charVal);
		u8g.print(char (176));
		u8g.print(F("C"));

	  //	u8g.setPrintPos(60, 40);
  	//	u8g.print(F("/"));
  	//	dtostrf(target_3WV , 4, 0, charVal);
  	//	u8g.print(charVal);
  	//	u8g.print(char (176));
  	//	u8g.print(F("C"));


		u8g.setPrintPos(0, 50);
		u8g.print(F("T_ret :"));
		dtostrf(Temperature[2] / 10., 5, 1, charVal);
		u8g.print(charVal);
		u8g.print(char (176));
		u8g.print(F("C"));

		//u8g.setPrintPos(60, 50);
		//u8g.print(F("/"));
		//dtostrf(main_Output, 5, 1, charVal);
		//u8g.print(charVal);
		//u8g.print(char (176));
		//u8g.print(F("C"));

    #ifdef PRINT_RAM
		 u8g.setPrintPos(80, 10);
		 u8g.print(F("RAM :"));
	 	 u8g.print(freeRam ());
    #endif

		//u8g.drawFrame(100,40,20,30);
		//u8g.drawBox(100,50,20,20);
	} while (u8g.nextPage());
}
#endif

#ifdef LLCD
 void  HeatingZone::init_display(void)
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
 void  HeatingZone::draw(void)
 {

		lcd.setCursor(3, 0);
		dtostrf(encoderPos / 10., 5, 1, charVal);
		lcd.print(charVal);
		lcd.setCursor(9, 0);
		dtostrf(Temperature[0] / 10., 5, 1, charVal);
		lcd.print(charVal);

		lcd.setCursor(3, 1);
		dtostrf(servoPosition, 4, 0, charVal);
		lcd.print(charVal);

		//lcd.setCursor(60, 20);
		//lcd.print(F("/+"));
		//dtostrf(Output, 4, 0, charVal);
		//lcd.print(charVal);


		lcd.setCursor(3, 2);
		dtostrf(target_3WV, 5, 1, charVal);
		lcd.print(charVal);
		lcd.setCursor(9, 2);
		dtostrf(Temperature[1] / 10., 5, 1, charVal);
		lcd.print(charVal);
		lcd.setCursor(15, 2);
		dtostrf(Temperature[2] / 10., 5, 1, charVal);
		lcd.print(charVal);
	//	lcd.setCursor(60, 40);
	//	lcd.print(F("/"));
	//	dtostrf(target_3WV , 4, 0, charVal);
	//	lcd.print(charVal);
	//	lcd.print(char (176));
	//	lcd.print(F("C"));


  #ifdef PRINT_RAM
		lcd.setCursor(15, 1);
		lcd.print(freeRam ());
  #endif
}
#endif

void HeatingZone::receive(const MyMessage &message)
{
#ifdef DEBUG
  		Serial.print("\nIncoming change for sensor:");
  		Serial.print(message.sensor);
			Serial.print("\nfrom sender:");
  		Serial.print(message.sender);
			Serial.print("\nto destination:");
  		Serial.print(message.destination);
  		Serial.print("\n, New status: ");
  		Serial.println(message.getFloat());
#endif
//	if MY_IS_GATEWAY{
		// if ((message.sender == 1) && (message.sensor == BOILER_STATUS_ID)) {
		// 	int i;
		//   MyMessage message2(BOILER_STATUS_ID,V_STATUS);
		//   message2.set(1);
		//   for(i=0; i< HEATINGZONE_count;i++){
		//     #ifdef DEBUG
		//   		Serial.print("\nSend message2 for :");
		//       Serial.println(HEATINGZONE_LIST[i]);
		//       Serial.print("\n");
		//     #endif
		//     message2.setDestination(HEATINGZONE_LIST[i]);
		//     send(message2);
		//   }
	  // }

//  }
	if ((message.type == V_HVAC_SETPOINT_HEAT) && (message.sender == 0)  && (message.destination == MY_NODE_ID)){ // Set temperature from controler
		main_PID->SetMode(MANUAL);
		target = int (message.getFloat() * 10);
		send(msg_target.set(message.getFloat(), 1));
		#ifdef MYDISPLAY
			encoderPos = target;
			knob.write(target);
			update_oled = 1;
		#endif
		main_PID->SetMode(AUTOMATIC);
		adjust_PID();
		regulate();
		#ifdef EEPROM_BACKUP
			EEPROM.write(EEPROM_ENC_POS, target);
		#endif
	}
	else if ((message.sensor == CHILD_ID_HEAT_STATE) && (message.sender == 0)  && (message.destination == MY_NODE_ID)) { //On/Off Heatingzone from controler
		#ifdef DEBUG
			Serial.print("\n, Cas 1\n");
			Serial.print("\n, New status: ");
			Serial.println(message.getBool());
		#endif
    regulate_on = message.getBool();
		msg_S_HEATER_FLOW_STATE.set(regulate_on);
		#ifdef EEPROM_BACKUP
			EEPROM.write(EEPROM_ENC_POS+1, regulate_on);
		#endif
		if (regulate_on == true) {
      reset_PID();
      adjust_PID();
		}
		regulate();
	}
	else if ((message.sensor == BOILER_STATUS_ID) && (message.type == V_STATUS)) { //Boiler status
#ifdef DEBUG
    Serial.print("\n, Cas 2\n");
		Serial.print("\n, New status: ");
		Serial.println(message.getBool());

#endif
		if ((message.getBool() == true)
		    && (boiler_on == false)) {
			boiler_on = true;
			adjust_PID();
      reset_PID();
			regulate();
		}
	}
	#ifdef LLCD
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
	#endif
	//else {
	//	Serial.print("\nIncoming change for sensor:");
	//	Serial.print(message.sensor);
	//	Serial.print("\nIncoming change for type:");
	//	Serial.print(message.type);
	//	Serial.print("\nIncoming change for sender:");
	//	Serial.print(message.sender);
	//}
}

#endif
