//#define HVLCD

#include <LCD.h>
#include <LiquidCrystal_I2C.h> // F Malpartida's NewLiquidCrystal library

#include <MHZ19B_uart.h>
#include <SPI.h>
#include "RF24.h"
#include <avr/wdt.h> //watchdog
#include <common.h>

#define DEBUG
//#define MH_LCD

#define ROOM_ID 0 //[0,1]
#define RADIO_ID 2

//LCD
#ifdef MH_LCD
#define I2C_ADDR 0x27 // Define I2C Address for controller
#else
#define I2C_ADDR 0x3F
#endif

#define CO2_RX_PIN 2  //CO2 Sensor dig rx pin no
#define CO2_TX_PIN 3 //CO2 Sensor dig tx pin no
#define RF_CE_PIN 10 //RF24 data
#define RF_CS_PIN 9 //RF24 chip select

#define BACKLIGHT 3
#define LCD_SIZE_X 16
#define LCD_SIZE_Y 2p

#define SAMPLE_TIME 3000
#define SUBMIT_INTERVAL 5000

//flag for error led
#define OK B00000000
#define E_SENSOR B00000001
#define E_SD B00000010

//variable for display
#ifdef HVLCD
LiquidCrystal_I2C lcd(I2C_ADDR, BACKLIGHT, POSITIVE);
#endif
RF24 radio(10, 9);
MHZ19B_uart mhz19;

int co2Min = 9999;
int co2Max = -1;
int co2Avg = 0;
int measure = 0;
boolean warming = true;
long simpleSum = 0;
int co2ppm = -1;
long bootWait = 0;

unsigned long sampleTime = 0;
unsigned long submitTime = 0;
unsigned long previousMillis = 0;
boolean doSubmit = true;
boolean doSample = true;

byte error = B00000000;
int errorCount = 0;

void(* resetFunc) (void) = 0;

void setup() {
  Serial.begin(9600);
  Serial.println(ROOM_ID);
  //watchdog
  wdt_enable(WDTO_8S);
  // put your setup code here, to run once:
  mhz19.begin(CO2_RX_PIN, CO2_TX_PIN);
  mhz19.setAutoCalibration(true);
  //mhz19.setRange(5000);
  //mhz19.calibrateZero();
  // Serial.println(true);
#ifdef HVLCD
  lcd.begin (LCD_SIZE_X, LCD_SIZE_Y); // initialize the lcd
  lcd.noBacklight();
  lcd.setCursor(0, 0);
  lcd.print("Warm Up..");
#endif
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  //radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(RF_CHANNEL);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  //radio.setCRCLength( RF24_CRC_16 ) ;
  //radio.setPayloadSize(sizeof(myData));
  // Open a writing and reading pipe on each radio, with opposite addresses
  //Serial.print("Radio ID: "); Serial.println((const char*)addresses[ROOM_ID][RADIO_ID]);
  radio.openWritingPipe(addresses[ROOM_ID][RADIO_ID]);
  radio.openReadingPipe(1, addresses[ROOM_ID][CONTROLLER]);
  myData.id = RADIO_ID;
  //int j = 1;
  //Serial.print("Listen ID: ");
  //  for (int i = 0; i < RADIO_COUNT; i++) {
  //    if (i != RADIO_ID) {
  //      Serial.print((const char*)addresses[ROOM_ID][i]);
  //      Serial.print(";");
  //      radio.openReadingPipe(j, addresses[ROOM_ID][i]);
  //      j++;
  //    }
  //  }
  //radio.startListening();
  errorCount = 0;
}

void loop() {

  wdt_reset();//I am still alive!!
  unsigned long inTime = millis();
  if (previousMillis > inTime) {
    sampleTime = 0;
    submitTime = 0;
  }
  previousMillis = inTime;

  doSample = ( inTime - sampleTime > SAMPLE_TIME);
  doSubmit = ( inTime - submitTime > SUBMIT_INTERVAL);

  //  boot wait sensor warm up time
  if (warming) {
#ifdef DEBUG
    Serial.println(co2ppm);
#endif
    if (millis() - bootWait < 30000 && (co2ppm < 0 || co2ppm == 410)) {
      co2ppm = mhz19.getPPM();
#ifdef HVLCD
      lcd.setCursor(0, 1);
      lcd.print((millis() - bootWait) / 1000); lcd.print("s    ");
#endif
      int runtime = (millis() - inTime);
      if (runtime < 1000) delay(1000 - runtime);
      return;
    } else { //exit warming
#ifdef DEBUG
      Serial.print("exit warming...");  Serial.print((inTime - bootWait) / 1000); Serial.println('s');
#endif
      warming = false;
#ifdef HVLCD
      lcd.clear();
#endif
      doSample = doSubmit = true;
    }
  }

  if (doSample) {
    co2ppm = mhz19.getPPM();
#ifdef DEBUG
    Serial.print("CO2: "); Serial.print(co2ppm); Serial.println("ppm");
#endif
#ifdef HVLCD
    lcd.setCursor(0, 0);
    lcd.print("CO2:"); lcd.print(co2ppm); lcd.print("ppm ");
#endif
    if (co2ppm > 0) {
      if (co2Min > co2ppm) {
        co2Min = co2ppm;
      }
      if (co2Max < co2ppm) {
        co2Max = co2ppm;
      }
      if (measure < 120) {
        simpleSum += co2ppm;
        measure++;
      }  else {
        co2Avg += simpleSum * 119 / 120 + co2ppm;
      }
    }
    co2Avg = simpleSum / measure;
#ifdef HVLCD
    lcd.setCursor(0, 1);
    lcd.print(co2Min); lcd.print('|'); lcd.print(co2Avg); lcd.print('|'); lcd.print(co2Max); lcd.print("       ");
#endif
  }

  if (doSubmit) {
    //radio.powerUp();
    Serial.print(F("Now sending.."));
    myData.id = RADIO_ID;
    myData._micros = micros();
    myData.type = SENSOR_TYPE_CO2;
    myData.value1 = co2ppm;
    myData.value2 = co2ppm;

#ifdef DEBUG
    Serial.print(myData._micros);
    Serial.print(F(" : "));
    Serial.print(myData.id); Serial.print(F(" | "));
    Serial.print(myData.type); Serial.print(F(" | "));
    Serial.print(myData.value1);
#endif
    radio.stopListening();
#ifdef HVLCD
    lcd.setCursor(13, 0); lcd.print("C:");
#endif
    if (radio.write( &myData, sizeof(myData))) {
      Serial.println(F("..Done"));
#ifdef HVLCD
      lcd.print("Y");
#endif
      submitTime = inTime;
    } else {
      Serial.println(F("..Fail"));
#ifdef HVLCD
      lcd.print("FAIL");
#endif
      error = error | E_SD;
    }
    radio.startListening();
    //radio.powerDown();
  }

  if (error == 0) {
    if (doSample) sampleTime = inTime;
    errorCount = 0;
  } else if (error != 0) {
    Serial.println(errorCount);
    errorCount++;
    error = 0;
    delay(2000);
  }
  if (errorCount > 100) {
    //resetFunc();
  }
}
