
#include <Arduino.h>
#include <EEPROM.h>
#define DEBUG
#define MODEM
#define DHT22

#ifdef MODEM
#include <A6TinyLibSms.h>
#endif



#include <OneWire.h>
#include <DallasTemperature.h>

#define powerPin 10
#define serialRx 7
#define serialTx 8
#define DHT_PIN 4
#define BOILER_PIN 11
#define VMC_PIN 12
#define ONE_WIRE_BUS 2

#define HYSTERESIS 3
#define HYSTERESIS_OUT 2
#define DEBUG
// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)


#include "DHT.h"
DHT dht(DHT_PIN, DHTTYPE);


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

float consigne;
float out_temp;
float in_temp;
float in_humidity;

char catreply[150];
char out_temp_tmp[6];
char in_temp_tmp[6];
char in_humidity_tmp[6];
char catreply2[30];
char consigne_tmp[6];
char temp_req[5]="";
int boiler_state =0;
int vmc_state = 0;

#ifdef MODEM
// using SoftwareSerial:
//A6TinyLibSms A6M(serialRx, serialTx);

// using AltSoftSerial or HardwareSerial:
A6TinyLibSms A6M;
const char* MaincallerId[] = {"+33642182344","+33788842182344"};
#endif


void setup() {
  Serial.begin(9600);
  Serial.print("setup");
  Serial.print("Arret chaudiere");
  Serial.print("Arret VMC");
  consigne = EEPROM.read(1);
  dtostrf(consigne, 5, 1, temp_req);
  #ifdef MODEM

  A6M.init(powerPin);
  delay(10000);
  dtostrf(consigne, 5, 1, consigne_tmp);
  sprintf(catreply2,"Setup! Consigne %s deg",consigne_tmp);
  A6M.sendSMS(MaincallerId[0], catreply2);
  #endif


  //boiler_state = 0;
  //vmc_state = 0;
  pinMode(BOILER_PIN, OUTPUT);
  pinMode(VMC_PIN, OUTPUT);

  digitalWrite(BOILER_PIN, HIGH);
  digitalWrite(VMC_PIN, HIGH);

  sensors.begin();
}

void loop() {
  delay(2000);
  Serial.println("loop");
  #ifdef MODEM
  A6M.checkIncomingSms(&onSmsReceived);
  #endif

  dht.begin();

  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  #ifdef DEBUG
  Serial.print("Requesting temperatures...");
  #endif
  sensors.requestTemperatures(); // Send the command to get temperatures
  out_temp = sensors.getTempCByIndex(0);
  #ifdef DEBUG
  Serial.println("DONE");
  #endif
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  in_temp = dht.readTemperature();
  in_humidity = dht.readHumidity();
  #ifdef DEBUG
  Serial.print("Temperature for the device 1 (index 0) is: ");
  Serial.println(out_temp);
  Serial.println("Humidity (%)\tTemperature (C)");
  Serial.print("\t");
  Serial.print(in_humidity, 1);
  Serial.print("\t\t");
  Serial.print(in_temp, 1);
  Serial.print("\n");
  #endif

  if (in_temp<out_temp && !vmc_state){
    if (boiler_state){
      #ifdef DEBUG
      Serial.print("Arret chaudière");
      #endif
      digitalWrite(BOILER_PIN, HIGH);
      boiler_state = 0;
    }
    #ifdef DEBUG
    Serial.print("Mise en marche VMC");
    #endif
    digitalWrite(VMC_PIN, LOW);
    vmc_state = 1;
    A6M.sendSMS(MaincallerId[0], (char*) "Mise en marche VMC");
  }
  else if (in_temp>out_temp+HYSTERESIS_OUT && vmc_state){
    #ifdef DEBUG
    Serial.print("Arret VMC");
    #endif
    digitalWrite(VMC_PIN, HIGH);
    vmc_state = 0;
    A6M.sendSMS(MaincallerId[0], (char*) "Arret VMC");
  }


  if (consigne>in_temp && !boiler_state && !vmc_state){

  #ifdef DEBUG
  Serial.print("Mise en marche chaudiere");
  #endif
  digitalWrite(BOILER_PIN, LOW);
  boiler_state = 1;
  A6M.sendSMS(MaincallerId[0], (char*) "Mise en marche chaudiere");
  }
  else if (consigne+HYSTERESIS<in_temp && boiler_state){

  #ifdef DEBUG
  Serial.print("Arret chaudiere");
  #endif
  digitalWrite(BOILER_PIN, HIGH);
  boiler_state = 0;
  A6M.sendSMS(MaincallerId[0], (char*) "Arret chaudiere");
  }
}

void onSmsReceived(char *callerId, char *message) {
  logln("Reception");
  logln(message);
  int i;
  for (i=0;i<=sizeof(MaincallerId);i++){
    if (strstr(MaincallerId[i],callerId)){

      int Set_temp = sscanf(message, "Set %s",temp_req);

      if (strcmp(message,"Stat")==0){
        logln("Stat");
        //float temp =0;

        //floatToString(out_temp_tmp,out_temp,0,7,true);
        dtostrf(out_temp, 5, 1, out_temp_tmp);
        dtostrf(in_temp, 5, 1, in_temp_tmp);
        dtostrf(in_humidity, 5, 1, in_humidity_tmp);
        log("Chaudiere :");
        logln(boiler_state);
        sprintf(catreply,"Temperature dehors: %s\n\
        Temperature interieur: %s\n\
        Humidite interieur: %s\n\
        Consigne: %s\n\
        Chaudiere : %d\n\
        VMC : %d",\
        out_temp_tmp,in_temp_tmp,in_humidity_tmp,temp_req,boiler_state,vmc_state);
        A6M.sendSMS(callerId, catreply);
        log("Chaudiere :");
        logln(boiler_state);
      }
      else if (Set_temp){
       logln("Set_temp");
       log("temp : ");
       logln(temp_req);
       logln(atof(temp_req));
       consigne = atof(temp_req);
       EEPROM.write(1,consigne);
       sprintf(catreply,"Bien maitre on regle a %s",temp_req);
       A6M.sendSMS(callerId, catreply);
      }
      else{
        logln("Rien compris");
        A6M.sendSMS(callerId, "Rien compris");
      }
    }
  }
}
