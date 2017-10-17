// Include DHT Library
#include <DHT.h>
#include <DHT_U.h>

// Including Libraries for I2C LCD

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Sensor definitions

#define DHTPIN 5 //DHT data pin connected to Arduino pin 5
#define DHTTYPE DHT11 // DHT 11
#define LDR_PIN 0 //used for Luminosity (LDR) Sensor Input
#define SOIL_MOIST_PIN 1 //used for Soil Moisture Sensor Input


// Actuators: Buttons and LEDs
#define PUMP_ON 11  //push-button
#define PUMP_PIN 12
#define LAMP_ON 9  //push-button
#define LAMP_PIN 8
#define SENSORS_READ 6  // push-button 

// Alert LED
#define YELLOW_LED 13 // 13

//Actuators

byte actuators[]={7,10};

// Variables to be used by DHT Sensor
int tempDHT; 
int humDHT;
int tempLowAlert = 0;
int HOT_TEMP = 30;
int COLD_TEMP = 12;

// to be used by LDR Sensor

int lumen;
int lumenAlert = 0;
int DARK_LIGHT = 30;

// to be used by SM Sensor
int soilMoist;
int soilMoistAlert = 0;
int DRY_SOIL = 40;
int WET_SOIL = 60;
int numSM = 1; // "numSM" defines number of moisture sensors that are connected
int numSamplesSMS = 1; // "numSamplesSMS" defines number of samples of each reading cycle


// Variables to be used with timers
long sampleTimingSeconds = 30; // ==> Define Sample time in seconds to read sensores
unsigned long startTiming = 0;
unsigned long elapsedTime = 0;

// Variables to be used by Actuators
boolean pumpStatus = 0;
boolean lampStatus = 0;
int timePumpOn = 10; // Turn Pump on in minutes

// Initialize the DHT sensor

DHT dht(DHTPIN, DHTTYPE);

// Initialize LCD

LiquidCrystal_I2C lcd(0x3F, 20, 4);

void setup() {
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(LAMP_PIN, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(actuators[0],OUTPUT);
  pinMode(actuators[1],OUTPUT);
  digitalWrite(actuators[0], LOW);
  pinMode(PUMP_ON, INPUT_PULLUP); // Button
  pinMode(LAMP_ON, INPUT_PULLUP); // Button
  pinMode(SENSORS_READ, INPUT_PULLUP); // Button
  
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

  Serial.println(elapsedTime);
  if(elapsedTime > (sampleTimingSeconds*1000)) 
  {
    readSensors();
    printData();
    autoControlPlantation();
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

  digiValue = debounce(SENSORS_READ);
  if (!digiValue) 
  {
    digitalWrite(YELLOW_LED, HIGH); 
    lcd.setCursor (0,0);
    lcd.print("< Updating Sensors >");
    readSensors();
    digitalWrite(YELLOW_LED, LOW); 
  }
}


/***************************************************
* Receive Commands and act on actuators
****************************************************/
void aplyCmd()
{
    if (pumpStatus == 1)
    {
      digitalWrite(PUMP_PIN, HIGH);
      digitalWrite(actuators[0], LOW);
    }
    if (pumpStatus == 0)
    {
      digitalWrite(PUMP_PIN, LOW);
      digitalWrite(actuators[0], HIGH);
    }
  
    if (lampStatus == 1) 
    {
      digitalWrite(LAMP_PIN, HIGH);
      digitalWrite(actuators[1], LOW);
    }
    if (lampStatus == 0){ 
      digitalWrite(LAMP_PIN, LOW);
      digitalWrite(actuators[1], HIGH);
    }
}

/***************************************************
* Automatically Control the Plantation based on sensors reading
****************************************************/
void autoControlPlantation()
{ 

//--------------------------------- PUMP ------//
  if ((soilMoist < DRY_SOIL) && (lumen > DARK_LIGHT)) 
  {
    Serial.println("dentro");
    if (soilMoistAlert == HIGH)
    {
      Serial.println("dentro1");
      soilMoistAlert = LOW; 
      turnPumpOn();
    }
    else soilMoistAlert = HIGH;
  }
  else soilMoistAlert = LOW;

//--------------------------------- HEAT ------//

int valor = tempDHT < COLD_TEMP;
int valor1 = soilMoist < WET_SOIL;
  if ((tempDHT < COLD_TEMP) && (soilMoist < WET_SOIL)) 
  {
    if (tempLowAlert == HIGH)
    {
      tempLowAlert = LOW; 
      digitalWrite(LAMP_PIN, HIGH);
      digitalWrite(actuators[1], LOW);
      lampStatus = 1;
    }
    else tempLowAlert = HIGH;
  }
  else 
  {
    tempLowAlert = LOW;
    digitalWrite(LAMP_PIN, LOW);
    digitalWrite(actuators[1], HIGH);
    lampStatus = 0; 
  }
}

/***************************************************
* TurnPumOn 
****************************************************/
void turnPumpOn()
{
  Serial.println("Dentro pump");
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(actuators[0], LOW);
  pumpStatus = 1;
  showDataLCD();
  Serial.println("Se ha mostrado datos");
  delay (timePumpOn*1000);
  digitalWrite(PUMP_PIN, HIGH);
  digitalWrite(actuators[0], HIGH);
  pumpStatus = 0;
  showDataLCD();
  Serial.println("He apagado el pump");
}

/***************************************************
* Read data from Sensors
****************************************************/
void readSensors(void)
{
  tempDHT = dht.readTemperature();   //Read temperature and humidity values from DHT sensor:
  humDHT = dht.readHumidity();
  lumen = getLumen(LDR_PIN);
  soilMoist = getSoilMoist();
}

/***************************************************
* Capture soil Moisture data
****************************************************/

int getSoilMoist()
{
  int humidity = 0;
  int sensorValue = analogRead(SOIL_MOIST_PIN);
 
 /* constraint function will limit the values we get so we can work better with map
 * since I only need values in the bottom limit of 300, will make it the outer limit and 1023 the other limit
 */
 sensorValue = constrain (sensorValue, 300,1023);
 // print the values returned by the sensor
 //Serial.println(sensorValue);
 // create a map with the values
 // You can think we have the 100% to 300 and 0 % to 1023 wrong, but no !
 // 300 - or 100 % - is the target value for moisture value in the soil
 // 0% of humidity - or moisture - is when the soil is dry
 humidity = map (sensorValue, 300, 1023, 100, 0);
 return humidity;
 delay(1000); //collecting values between seconds
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
  Serial.print("%  Luminosity ==> ");
  Serial.print(lumen);
  Serial.print("%  Soil Moisture ==> ");
  Serial.print(soilMoist);
  Serial.println("%");
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
  lcd.setCursor(0,2);
  lcd.print("Ligh: ");
  lcd.print(lumen);
  lcd.print("%");
  lcd.print("  Soil: ");
  lcd.print(soilMoist);
  lcd.print("%");
  lcd.setCursor (0,3);
  lcd.print("Pump: ");
  if (soilMoistAlert  == 1) lcd.print ("X");
  else lcd.print(pumpStatus);
  lcd.print("   Lamp: ");
  if (tempLowAlert == 1) lcd.print ("X");
  else lcd.print(lampStatus);
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

/***************************************************
* Capture luminosity data: 0% full dark to 100% full light
****************************************************/

int getLumen(int anaPin)
{
  int anaValue = 0;
  for(int i = 0; i < 10; i++) // read sensor 10X ang get the average
  {
    anaValue += analogRead(anaPin);   
    delay(50);
  }
  
  anaValue = anaValue/10; //Light under 300; Dark over 800
  anaValue = map(anaValue, 1023, 0, 0, 100); //LDRDark:0  ==> light 100%

  return anaValue; 
}

