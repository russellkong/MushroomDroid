#include <Wire.h>
#include "SHTSensor.h"
#include <SPI.h>
#include <RF24.h>
#include <common.h>

#define ROOM_ID 0 //[0:A;1:B]
#define RADIO_ID 1

#define DEBUG
#ifdef MH_LCD
#define I2C_ADDR 0x27 // Define I2C Address for controller
#else
#define I2C_ADDR 0x3F
#endif

#define RF_CE_PIN D8 //RF24 data
#define RF_CS_PIN D9 //RF24 chip select

#define BACKLIGHT 3
#define LCD_SIZE_X 20
#define LCD_SIZE_Y 4

#define SIMPLE_TIME 5000
#define SUBMIT_INTERVAL 5000

//flag for error led
#define OK B00000000
#define E_SENSOR B00000001
#define E_SD B00000010

//struct dataStruct {
//  unsigned long _micros;
//  int id;
//  int type;//1:SHT, 2:CO2
//  float value1;//temp;CO2
//  float value2;//rh;
//} myData;

//Display variables
//LiquidCrystal_I2C lcd(I2C_ADDR, BACKLIGHT, POSITIVE);
SHTSensor sht;//(SHTSensor::SHT3X)
//SHTSensor sht;
RF24 radio(RF_CE_PIN, RF_CS_PIN);

float sensorTemp = 0;
float sensorRh = 0;
unsigned long simpleTime = 0;
unsigned long submitTime = 0;
unsigned long previousMillis = 0;
boolean doSubmit = true;
boolean doSimple = true;

byte lcdCountdown = 0;

byte error = B00000000;
int errorCount = 0;

void(* resetFunc) (void) = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  #ifdef DEBUG
    Serial.print(F("BOOT"));
#endif
  // put your setup code here, to run once:
  //watchdog
//  wdt_enable(WDTO_8S);

  sht.init();
  //sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);
//  lcd.begin (LCD_SIZE_X, LCD_SIZE_Y); // initialize the lcd
//  lcd.backlight();
//  lcd.setCursor(0, 0);
//  lcd.print(F("Sensor"));

//  lcd.setCursor(0, 1);
//  lcd.print("ID: "); lcd.print(RADIO_ID);

  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(RF_CHANNEL);
  //radio.setCRCLength( RF24_CRC_16 ) ;
  radio.setDataRate(RF24_250KBPS);
  if (!radio.isChipConnected()) {
  Serial.println("ERROR: NRF24 Chip not connected");
} 
  //radio.setPayloadSize(sizeof(myData));
  // Open a writing and reading pipe on each radio, with opposite addresses
 // Serial.print("Radio ID: "); Serial.println((const char*)addresses[ROOM_ID][RADIO_ID]);
  radio.openWritingPipe(addresses[ROOM_ID][RADIO_ID]);
  radio.openReadingPipe(1, addresses[ROOM_ID][CONTROLLER]);
  myData.id = RADIO_ID;
  //  int j = 1;
  //  Serial.print("Listen ID: ");
  //  for (int i = 0; i < RADIO_COUNT; i++) {
  //    if (i != RADIO_ID) {
  //      Serial.print((const char*)addresses[ROOM_ID][i]);
  //      Serial.print(";");
  //      radio.openReadingPipe(j, addresses[ROOM_ID][i]);
  //      j++;
  //    }
  //  }
  //  radio.startListening();
  //radio.stopListening();
  simpleTime = millis();
  errorCount = 0;
  #ifdef DEBUG
    Serial.print(F("Done"));
#endif
}

void loop() {
//  wdt_reset();//I am still alive!!

  //millis overflow detection
  unsigned long inTime = millis();
  if (previousMillis > inTime) {
    simpleTime = inTime;
    submitTime = inTime;
  }
  previousMillis = inTime;

  doSimple = ( inTime - simpleTime > SIMPLE_TIME);
  doSubmit = ( inTime - submitTime > SUBMIT_INTERVAL);
  error = 0;
  // put your main code here, to run repeatedly:
  if (doSimple) {
#ifdef DEBUG
    Serial.print(F("Simpling..."));
#endif
//    lcd.setCursor(10, 0);
//    lcd.print("STAT:");
    if (sht.readSample()) {
//      lcd.print("OK ");
      sensorTemp = sht.getTemperature();
      sensorRh = sht.getHumidity();
    } else {
      Serial.println(F("E]SHT"));
//      lcd.print("ERR");
      error = error | E_SENSOR;
    }

#ifdef LCD
//    lcd.setCursor(0, 2);
//    lcd.print("TEMP: "); lcd.print(sensorTemp);
//    lcd.setCursor(0, 3);
//    lcd.print("RH: "); lcd.print(sensorRh);
#endif
  }

  if (doSubmit) {
    //radio.powerUp();
#ifdef DEBUG
    Serial.print(F("Now sending.."));
#endif
    myData.id = RADIO_ID;
    myData._micros = micros();
    myData.type = SENSOR_TYPE_TEMP_RH;
    if (error == 0) {
      myData.value1 = sensorTemp;
      myData.value2 = sensorRh;
    } else {
      myData.value1 =  myData.value2 = 0;
    }
#ifdef DEBUG
    Serial.print(myData._micros);
    Serial.print(F(" : "));
    Serial.print(myData.id); Serial.print(F(" | "));
    Serial.print(myData.type); Serial.print(F(" | "));
    Serial.print(myData.value1);
    Serial.print(F(" | "));
    Serial.print(myData.value2);
#endif
    radio.stopListening();
//    lcd.setCursor(10, 1); lcd.print("CONN:");
    if (radio.write( &myData, sizeof(myData))) {
#ifdef DEBUG
      Serial.println(F("..Done"));
#endif
//      lcd.print("OK  ");
      submitTime = inTime;
    } else {
#ifdef DEBUG
      Serial.println(F("..Fail"));
#endif
//      lcd.print("FAIL");
      error = error | E_SD;
    }
    radio.startListening();
    //radio.powerDown();
  }

  if (error == 0) {
    if (doSimple) simpleTime = inTime;
    errorCount = 0;
  } else if (error != 0) {
#ifdef DEBUG
    Serial.print("Error Count:"); Serial.println(errorCount);
#endif
    errorCount++;
    error = 0;
    delay(2000);
  }
  
  if (errorCount > 30) {
    resetFunc();
  }
}
