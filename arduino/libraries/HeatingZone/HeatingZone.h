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
	#include <PID_v1.h>
#endif
#include <MySensor.h>

#include "OneWire.h"


MyMessage msg_temp(CHILD_ID_TEMP, V_TEMP);	//three temp
MyMessage msg_S_HEATER_FLOW_STATE(CHILD_ID_HEAT_STATE,V_STATUS);
MyMessage msg_target(CHILD_ID_SET_POINT, V_HVAC_SETPOINT_HEAT);
//way3_PID PID (&input, &Output, &target,2, 0, 0, DIRECT);
//PID way3_PID;

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
    bool start_circulator;
    HeatingZone(DallasTemperature *sensors, int *ds18b20_index, uint8_t PumpPin, uint8_t ServoOn, uint8_t ServoDirection );

    double target;
#ifdef PPID
    double Output;
    double input;
    double target_3WV;
    PID *way3_PID;
    double main_Output;
    double main_input;
    PID *main_PID;
#endif 

    void begin(void);
    void presentation(void);
    void reset(void);
    void regulate(void);
    void update(void);
    void getTempsC(void);
    void openValve(int num_step);
    void closeValve(int num_step);
    void stopServo(void);
    void setServoTo(byte requestedPosition);
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
	regulate_on = 1;
	start_circulator = false;
	#ifdef EEPROM_BACKUP
		target = EEPROM.read(EEPROM_ENC_POS);
		//servoPosition = EEPROM.read(0);
	#else
    		target = 200;
		servoPosition = 0;
	#endif
	lastRegulation = 0;
	last_main_Regulation = 0;
	
	#ifdef PPID
		//#error error
		//Specify the links and initial tuning parameters
                target_3WV = 35;
		way3_PID = new PID (&input, &Output, &target_3WV,2, 2, 0, DIRECT);
                main_PID = new PID (&main_input, &main_Output, &target,.2, .2, 0, DIRECT);
		way3_PID->SetOutputLimits(-200, 200);
		main_PID->SetOutputLimits(-50, 50);
  		//turn the PID on
 		way3_PID->SetMode(AUTOMATIC);
		main_PID->SetMode(AUTOMATIC);
	#endif
	
}

void HeatingZone::begin(void)
{
pinMode(_PumpPin, OUTPUT);
pinMode(_ServoOn, OUTPUT);
pinMode(_ServoDirection, OUTPUT);
digitalWrite(_PumpPin, HIGH);
digitalWrite(_ServoDirection,HIGH);
digitalWrite(_ServoOn, HIGH);
}
void HeatingZone::reset(void)
{
	//stopServoAt = millis() + TIME_TO_RESET;
	digitalWrite(_ServoDirection,HIGH);
	digitalWrite(_ServoOn, LOW);
	wait(TIME_TO_RESET);
	digitalWrite(_ServoOn,HIGH);
	servoPosition = 0;
}
void HeatingZone::presentation()
{
for (int i = CHILD_ID_TEMP; i < CHILD_ID_TEMP+3; i++) {
		present(i, S_TEMP);
	}
present(CHILD_ID_HEAT_STATE, S_HVAC);
present(CHILD_ID_SET_POINT, S_HVAC);
send(msg_S_HEATER_FLOW_STATE.set(1));
}
void HeatingZone::update(void)
{				
		stopServo(); // stop the servo if necessary

    		if (millis()-INTERVAL_MAIN_REG >= last_main_Regulation )
		{
			
			main_input = Temperature[0];
			main_PID->Compute();
			if (target_3WV + main_Output > MAX_TEMP/10.)
			{ 
				target_3WV = MAX_TEMP/10.;
			}
			else
			{
				target_3WV += main_Output;
			}
			Serial.print("\nTemp:");
			Serial.print(Temperature[0]);
			Serial.print(" ");
			Serial.print(target);
			Serial.print("\nMain Output :");
			Serial.print(main_Output);
        	        Serial.print("\n");
			last_main_Regulation = millis();
		}
		if (millis()-INTERVAL_REG >= lastRegulation && digitalRead(_ServoOn) == HIGH )// time for regulation if the servo is not running
		{
			regulate();
			//if (Temperature[1]>MAX_TEMP)
			//{
			//	closeValve(5);
			//}
		}
		
}
void HeatingZone::regulate(void){
		getTempsC();
		#ifdef DEBUG
			Serial.print("\nTemp0"+String(Temperature[0]));
			//Serial.print("\nTemp1"+String(Temperature[1]));
			//Serial.print("\n_ServoOn "+String(digitalRead(_ServoOn)));
			//Serial.print("\nservoPosition "+String(servoPosition)+"\n");
		#endif
			
			//if ( abs(Temperature[1]-Temperature_prev[1]) < EPSILON_TEMP)
			//{
			if (regulate_on == true)
			{
		//#ifdef PPID
		//#else
				if ((Temperature[0] ) >= (target)  ) // we're turning temperatures into integers to compare them
				{
					//closeValve(5);
					circulatorOff();
				}
				else //if ((Temperature[0]) < (target  - 5)) // these values are very random and should not be used by anyone
				{
					if (digitalRead(_PumpPin) == HIGH && (Temperature[0]) < (target  - 5))
					{
						//circulatorOn();
						start_circulator = true;
						setServoTo(20);
					}
				}
				if (digitalRead(_PumpPin) == LOW)
				{
				#ifdef PPID
					input = Temperature[1]/10.;
					way3_PID->Compute();
					Serial.print("\nOutput :");
					Serial.print(int(Output));
        		                Serial.print("\n");
					if (abs(Output)>5.)
					{
						setServoTo(servoPosition+int(Output));
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
				if ((servoPosition == 100) && (Temperature[1] < 300))
				{
					circulatorOff();
					regulate_on = false;
			 		send(msg_S_HEATER_FLOW_STATE.set(0),1);
				}
			}
			else
				{
				circulatorOff();
				}
			//}
		//#endif
			lastRegulation = millis();
}

void  HeatingZone::getTempsC(void)
{
		_sensors->requestTemperatures();
		// query conversion time and sleep until conversion completed
  		int16_t conversionTime = _sensors->millisToWaitForConversion(_sensors->getResolution());
  		// sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
  		sleep(conversionTime);
		
		for( unsigned int i = 0; i < 3; i = i + 1 )
		{
		// Fetch and round temperature to one decimal
                float temperature = static_cast<float>(static_cast<int>((getConfig().isMetric?_sensors->getTempCByIndex(_ds18b20_index[i]):_sensors->getTempFByIndex(_ds18b20_index[i])) * 10.));
		Temperature_prev[i] = Temperature[i];
		Temperature[i] = temperature; // store temperature data into position [i] of the array
		send(msg_temp.setSensor(CHILD_ID_TEMP+i).set(temperature/10.,1));
		#ifdef OLED
			if (Temperature_prev[i] != Temperature[i])
			{
				update_oled = 1;
			}
		#endif
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
		update_oled = 1;
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
		update_oled = 1;
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

void HeatingZone::setServoTo(byte requestedPosition)
{
	if (requestedPosition != servoPosition && digitalRead(_ServoOn) == HIGH )
	{
		if (requestedPosition>100) {requestedPosition=100;}
		if (requestedPosition<0) {requestedPosition=0;}
		char numberOfSteps = requestedPosition - servoPosition; 
		if (numberOfSteps > 0) // if the number is positive or higher than current, open the valve and use X steps 
		{
			stopServoAt = millis() + (abs(numberOfSteps) * STEP_OPEN);
			digitalWrite(_ServoDirection, LOW);
			digitalWrite(_ServoOn, LOW);
			servoPosition = servoPosition + numberOfSteps;
		}
		else if (numberOfSteps < 0) 
		{
			stopServoAt = millis() + (abs(numberOfSteps)* STEP_CLOSE);
			digitalWrite(_ServoDirection, HIGH);
			digitalWrite(_ServoOn, LOW);
			servoPosition = servoPosition - abs(numberOfSteps);
		}
		update_oled = 1;
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
//	if (digitalRead(_PumpPin) == HIGH) 
//	{
		digitalWrite(_PumpPin, LOW);
	//}
}




#endif

