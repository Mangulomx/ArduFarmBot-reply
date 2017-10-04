// Include DHT Library
#include <DHT.h>
#include <DHT_U.h>

// Including Libraries for I2C LCD

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Sensor definitions

#define DHTPIN 5 //DHT data pin connected to Arduino pin 5
#define DHTTYPE DHT11 // DHT 11


// Actuators: Buttons and LEDs
#define PUMP_ON 11  //push-button
#define PUMP_PIN 13
#define LAMP_ON 9  //push-button
#define LAMP_PIN 8

// Variables to be used by Sensor
int tempDHT; 
int humDHT;

// Variables to be used with timers
long sampleTimingSeconds = 300; // ==> Define Sample time in seconds to read sensores
long startTiming = 0;
long elapsedTime = 0;

// Variables to be used by Actuators
boolean pumpStatus = 0;
boolean lampStatus = 0;

// Initialize the DHT sensor

DHT dht(DHTPIN, DHTTYPE);

// Initialize LCD

LiquidCrystal_I2C lcd(0x3F, 20, 4);

void setup() {
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(LAMP_PIN, OUTPUT);
  pinMode(PUMP_ON, INPUT_PULLUP); // Button
  pinMode(LAMP_ON, INPUT_PULLUP); // Button
  Serial.begin(9600); 
  Serial.println("ArduFarmBot Local Station Test");
  dht.begin();
  lcd.begin();
  readSensors(); // innitial reading
  startTiming = millis(); // starting the "program clock"

}

void loop() {
  // Start timer for measurements
  elapsedTime = millis()-startTiming; 

  readLocalCmd(); //Read local button status
  showDataLCD();
  
  if (elapsedTime > (sampleTimingSeconds*1000)) 
  {
    readSensors();
    //printData();
    startTiming = millis();
     
  }

}

/****************************************************************
* Read local commands (Pump and Lamp buttons are normally "HIGH"):
****************************************************************/
void readLocalCmd() 
{  
  int digiValue = debounce(PUMP_ON);
  if (!digiValue) 
  {
    pumpStatus = !pumpStatus;
    showDataLCD();
    aplyCmd();

  }

  digiValue = debounce(LAMP_ON);
  if (!digiValue) 
  {
    lampStatus = !lampStatus;
    showDataLCD();
    aplyCmd();

  }
}


/***************************************************
* Receive Commands and act on actuators
****************************************************/
void aplyCmd()
{
    if (pumpStatus == 1) digitalWrite(PUMP_PIN, HIGH);
    if (pumpStatus == 0) digitalWrite(PUMP_PIN, LOW);
  
    if (lampStatus == 1) digitalWrite(LAMP_PIN, HIGH);
    if (lampStatus == 0) digitalWrite(LAMP_PIN, LOW);
}

/***************************************************
* Read data from Sensors
****************************************************/
void readSensors(void)
{
  tempDHT = dht.readTemperature();   //Read temperature and humidity values from DHT sensor:
  humDHT = dht.readHumidity();
}

/***************************************************
* Showing capured data at Serial Monitor
****************************************************/
void printData(void)
{
  Serial.print("   Temp DHT ==> ");
  Serial.print(tempDHT);
  Serial.print("oC  Hum DHT ==> ");
  Serial.print(humDHT);
}

/***************************************************
* Showing capured data at LCD
****************************************************/
void showDataLCD(void)
{
  lcd.setCursor (0,0);
  lcd.print("ArduFarmBot Ctrl St.");
  lcd.setCursor (0,1);
  lcd.print("Temp: ");
  lcd.print(tempDHT);
  lcd.print("oC  Hum: ");
  lcd.print(humDHT);
  lcd.print("%  ");
  lcd.setCursor(0,0);
}

/***************************************************
* Debouncing a key
****************************************************/
boolean debounce(int pin)
{
  boolean state;
  boolean previousState;
  const int debounceDelay = 60;
  
  previousState = digitalRead(pin);
  for(int counter=0; counter< debounceDelay; counter++)
  {
    delay(1);
    state = digitalRead(pin);
    if(state != previousState)
    {
      counter =0;
      previousState = state;
    } 
  }
  return state;
}
