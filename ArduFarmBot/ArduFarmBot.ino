// Include DHT Library
#include <DHT.h>
#include <DHT_U.h>

// Including Libraries for I2C LCD

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Sensor definitions

#define DHTPIN 5 //DHT data pin connected to Arduino pin 5
#define DHTTYPE DHT11 // DHT 11

// Actuators: Buttons and LEDS

#define PUMP_ON 11 //push-button
#define PUMP_PIN 13
#define PUMP 7 //water pump
// Variables to be used by Sensor
int tempDHT; 
int humDHT;

// Variables to be used by Actuators
boolean pumpStatus = 0;
boolean LastStatus = LOW;
boolean InitStatus = LOW;

// Variables to be used with timers
long sampleTimingSeconds = 0; // ==> Define Sample time in seconds to read sensores
long startTiming = 0;
long elapsedTime = 0;
const int timeDebounce = 10;
int cont=0;

// Initialize the DHT sensor

DHT dht(DHTPIN, DHTTYPE);

// Initialize LCD

LiquidCrystal_I2C lcd(0x3F, 20, 4);

int cuenta = 0;
int statusButton;
int statusButtonLast;



void setup()
{
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(PUMP_ON, INPUT_PULLUP); //Button
  pinMode(PUMP, OUTPUT);
  Serial.begin(9600);
  Serial.println("ArduFarmBot Local Station Test");
  dht.begin();
  lcd.begin();
  readSensors(); //innitial reading
  startTiming = millis(); // starting the "program clock"
}

void loop()
{

  //Start timer for measurements
  elapsedTime = millis() - startTiming;

  
  readLocalCmd(); //Read local button status
  showDataLCD();
  if(elapsedTime > (sampleTimingSeconds * 1000))
  {
    readSensors();
    printData();
    startTiming = millis();
  }
}

/***************************************************
* Read data from Sensors
****************************************************/
void readSensors()
{
  tempDHT = dht.readTemperature();   //Read temperature and humidity values from DHT sensor:
  humDHT = dht.readHumidity();
}


 /****************************************************************
* Read local commands (Pump and Lamp buttons are normally "HIGH"):
****************************************************************/

void readLocalCmd()
{
  InitStatus = debounce(LastStatus); //leemos el estado del 
  if(LastStatus == LOW && InitStatus == HIGH)
  {
    pumpStatus = !pumpStatus;
    cont++;
    
  }
  LastStatus = InitStatus;
  showDataLCD();
  aplyCmd();
  
}

/***************************************************
* Receive Commands and act on actuators
****************************************************/

void aplyCmd()
{
  if(pumpStatus == 1)
  {
    digitalWrite(PUMP_PIN, HIGH);
    digitalWrite(PUMP, LOW);
  }
  if(pumpStatus == 0)
  {
    digitalWrite(PUMP_PIN, LOW);
    digitalWrite(PUMP,HIGH);
  }
}

/***************************************************
* Showing capured data at Serial Monitor
****************************************************/
void printData(void)
{
  Serial.print(" Temp DHT ==> ");
  Serial.print(tempDHT);
  Serial.print("oC Hum DHT ==> ");
  Serial.print(humDHT);
}

/***************************************************
* Showing capured data at LCD
****************************************************/
void showDataLCD(void)
{
  lcd.setCursor(0,0);
  lcd.print("ArduFarmBot Ctrl St.");
  lcd.setCursor (0,1);
  lcd.print("Temp: ");
  lcd.print(tempDHT);
  lcd.print("oC  Hum: ");
  lcd.print(humDHT);
  lcd.print("%  ");
  lcd.setCursor (0,2);
  lcd.print("Pump: ");
  lcd.print(pumpStatus);
  lcd.setCursor(0,0);
  
}


/***************************************************
* Debouncing a key
****************************************************/
boolean debounce(boolean eLast)
{
   boolean eInit = digitalRead(PUMP_ON);
  if (eLast != eInit)
  {
    delay(5);
    eInit = digitalRead(PUMP_ON);
  }
  return eInit;
}


