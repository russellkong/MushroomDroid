/*new branch from garduino
   2.8 include CO2 sensor linked logic
   2.9 hardware mode switch, periodic resubmission of switch status
   2.91 fine tune logic, prevent alt. switch between HiRH and HiTp.
   2.92 use of sht environment sensor, mollier intake temp adj, CSC SD SPI check(XXX on 2.93)
   2.93 changed mollier constants
   2.94 updated error code and sd error lighting; fixed SD reboot error; updated info lcd page
   2.95 increase ventilation to suppress infection
   2.96 reduced hitemp humidity; changed wetting interval to time
   3.00 aircon energy saving; midday light off; CO2 on info scn;
*/
#ifndef conf_h
#define conf_h
//#define DEBUG

//Equipment List
//#define CFAN
//#define HEATER
//#define MH_LCD
#define SHT_SENSOR
#define RECEIVER
#define VERSION "v3.01"

//User Defined Conf
//#define MIDDAY_LIGHT_OFF

//System Parameters
//LCD
#define I2C_ADDR 0x3F // Define I2C Address for controller
#define BACKLIGHT 3
#define LCD_SIZE_X 20
#define LCD_SIZE_Y 4
//SD card
#define CONF_FILE "CONF.TXT"
//Joystick
#define JS_TRIGGER 200

//Operation constants
#define LOOP_TIME 1000 //the min. duration of a loop
#define SPLIT '|'
#define LCD_OFF_DELAY 20000 //idle loop to turn off LCD backlight
#define REACT_TIME 500 //reaction time of joystick action
#define MAX_CTL_HOLD_TIME 30000
#define RADIO_GAP 300

#define MAX_RUNTIME 20*60000 //max. time(ms) of program execution
#define GOAL_COUNT 20  //reading before advance step in program
#define SAMPLE_INTERVAL 2000 //interval to read local sensor
#define SUMMARY_INTERVAL 5*60000
#define RESUBMIT_INTERVAL 3*60000 //interval to resubmit switch instruction
#define DATA_TIMEOUT 120000 //time to expire received data
//Display variables
#define SCN_NUM 7
#define SCN_ID_INFO 0
#define SCN_ID_ENV 1
#define SCN_ID_SW 2
#define SCN_ID_MODE 3
#define SCN_ID_STAT 4
#define SCN_ID_CONF 5
#define SCN_ID_SYS 6

#define CO2_NORMAL 400
#define CO2_MAX 600

#define RADIO_ID 0

//flag for error led
#define OK 0
#define E_SENSOR 1
#define E_SD 2
#define E_RF 3
#define TENT_MODE_COUNT 3
#define TENT_MODE_OFF 0
#define TENT_MODE_INCUBATION 1
#define TENT_MODE_FRUIT 2
#endif

#ifndef PIN_H_INCLUDED
#define PIN_H_INCLUDED

//Environment Sensor
#ifndef SHT_SENSOR
#define DHT_PIN 8 //digital pin
#endif

//SD card
const uint8_t SOFT_MISO_PIN = 40;
const uint8_t SOFT_MOSI_PIN = 41;
const uint8_t SOFT_SCK_PIN  = 42;
const uint8_t SD_CHIP_SELECT_PIN = 43;

//RADIO
#define R_TRAN_PIN 2
#define R_REV_PIN 3
#define RF_CS_PIN 48
#define RF_CE_PIN 49
//Joystick
#define SW_PIN 4
// digital pin connected to switch output
#define X_PIN 14 // analog pin connected to X output
#define Y_PIN 15 // analog pin connected to Y output
#define JS_ORIENTATION 1 //vertical positioning of joystick

////CO2
#ifdef CO2_SENSOR
////CO2 MHZ19B
#define MHZ19B_RX 19  //Serial rx pin no
#define MHZ19B_TX 18 //Serial tx pin no
#endif
//DP-water temp
#define ONE_WIRE_BUS 9
#define DS18B20_LPIN 8
#define DS18B20_HPIN 10
//LED
#define RED_PIN 11
#define GREEN_PIN 12
#define BLUE_PIN 13
#endif

#include <Wire.h>
#include <DS3231.h> //Clock
#include <LCD.h>
#include <LiquidCrystal_I2C.h> // F Malpartida's NewLiquidCrystal library
#include "SHTSensor.h"
#include <SPI.h>
#include <SdFat.h> //SD card, softSpi
#include <DigitalIO.h>
#ifdef RECEIVER
//#include <RF24.h> //2.4GHz radio
#endif
#include <RCSwitch.h> //433 radio

#include <math.h>
#include <avr/wdt.h> //watchdog
#include <common.h>

//Tent program
typedef enum  {IDLE = 0, VENT, WET, RH_HIGH, RH_LOW, TEMP_HIGH, ERR } PROG;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
#ifdef RECEIVER
RF24 radio(RF_CE_PIN, RF_CS_PIN);
#endif
RCSwitch mySwitch = RCSwitch();

//Clock
DS3231  rtc(SDA, SCL);
Time curTime;
LiquidCrystal_I2C lcd(I2C_ADDR, BACKLIGHT, POSITIVE);
SHTSensor sht;

const PROGMEM char STR_CHG_MODE[] = "CM|%d";
const PROGMEM char STR_STS[] = "ST|%d|%d|%d|%d|%d|%d";
const PROGMEM char STR_CONF[] = "M%d|H:%d-%d-%d|T:%d-%d-%d|V:%d,%d|L:%d-%d|W:%d,%d|A:%c";
const PROGMEM char STR_CONF_LINE[] = "%2d%2d%2d%2d%2d%2d%2d%2d%2d%2d%2d%2d%c";

char logFilename[] = "xxxx.LOG\0";
char tmpLog[200];

RfDataObj myData;

int jsXRestPoint, jsYRestPoint;
unsigned long inTime = 0;
unsigned long previousMillis = 0;
unsigned long lastUserActionTime = 0;
unsigned long sampleTime = 0;
boolean doSample = true; //sampling of local sensors
unsigned long summaryTime = 0;
boolean doSummary = true;
unsigned long resubmitTime = 0;

boolean skipSd = false;
byte error = B00000000;
byte errorCnt = 0;

//Display variables
byte scn = 0; //display 0:time, 1:temp, 2:Relay status, 3: operation mode, 4: tent stat, 6: system opr, 5: tent conf
byte frame = 0;
byte previousScn, previousFrame;
byte scn_mode = 0;
byte x = 0, y = 0;

//Tent Operation
unsigned long decisionTime = 0;//start time of program suggestion
byte confirmCount = 0;
byte tentMode = 0;
PROG tentProg = PROG::IDLE;
PROG suggestion = PROG::IDLE;
byte tentStep = 0;
unsigned long tentProgTime = 0; //start time of prog execution
byte goalCount = 0;
unsigned long tentLastFanTime = 0;
unsigned long tentLastWetTime = 0;

//Switch Register
byte switchStatus = B00000000;
const uint8_t sysID = 0;
const uint8_t mistID = 1;
const uint8_t vFanID = 2;
const uint8_t lightID = 3;
const uint8_t cFanID = 4;
const uint8_t heatID = 5;

//Radio Switch Code
unsigned long vfan[2] = {17100, 17118};
unsigned long mist[2] = {17200, 17218};
//byte cooler = 5;
unsigned long light[2] = {1330303, 1331818};
unsigned long cfan[2] = {17400, 17418};
unsigned long heat[2] = {17500, 17518};

//Statistic
unsigned long lastSurvayTime = 0; //time to calculate usage statistic
int ttlOprMin[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int ttlOprSec[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//Tent parameters (define in conf. file (SD Card))
uint8_t humidMax = 90; uint8_t humidMid = 85; uint8_t humidMin = 83;
uint8_t tempMax = 25; uint8_t tempMid = 23; uint8_t tempMin = 16;
uint8_t ventInv = 30; uint8_t ventDur = 5;
uint8_t lightStart = 7; uint8_t lightEnd = 19;
uint8_t wetInv = 12; uint8_t wetDur = 30;
boolean airCon = false;

//SD card
/* CATALEX SD CARD READER SOFT-SPI PART */
// Here are the pins for the soft-spi-bus defined
// You can select every pin you want, just don't put them on an existing hardware SPI pin.
SdFatSoftSpi<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> sd;
SdFile inFile, confFile, logFile;

//Sensors
float envTemp = 0, workingTemp = 0, sensorTemp[RADIO_COUNT] = {0, 0, 0};
float envRh = 0, workingRh = 0, sensorRh[RADIO_COUNT] = {0, 0, 0};
int workingCO2 = 0, sensorCO2 = 0;
long lastRevTime[RADIO_COUNT] = {0, 0, 0};
long lastRevMinTime = 0;

void printTentEnv(byte env);

void switchMister(boolean newState, boolean force = false) ;
void switchVFan(boolean newState, boolean force = false) ;
void switchLight(boolean newState, boolean force = false) ;
void switchCFan(boolean newState, boolean force = false) ;
void switchHeater(boolean newState, boolean force = false) ;
float mollierTemp(float inTemp, float inRh);

void timeOverflowReset() {
  tentProgTime = 0;
  tentLastFanTime = 0;
  tentLastWetTime = 0;
  lastSurvayTime = 0;
  for (int i = 0; i < RADIO_COUNT; i++) {
    lastRevTime[i] = 0;
  }
  lastRevMinTime = 0;
  sampleTime = 0;
  summaryTime = 0;
  decisionTime = 0;
  lastUserActionTime = 0;
  resubmitTime = 0;
}

void(* rebootFunc) (void) = 0;

void setup() {

  inTime = millis();
  Serial.begin(9600);
  Serial.println(F("System start.."));
  //watchdog
  wdt_enable(WDTO_8S);

  // put your setup code here, to run once:
  lcd.begin (LCD_SIZE_X, LCD_SIZE_Y); // initialize the lcd
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("Controller"));
  lcd.setCursor(0, 1);
  lcd.print(VERSION);

  // Initialize the rtc object
  rtc.begin();
  // The following lines can be uncommented to set the date and time
  //    rtc.setDOW(SUNDAY);     // Set Day-of-Week to SUNDAY
  //    rtc.setTime(0, 4, 30);     // Set the time to 12:00:00 (24hr format)
  //    rtc.setDate(19, 8, 2018);   // Set the date to January 1st, 2014

  //Joystick
  pinMode(SW_PIN, INPUT_PULLUP);
  //digitalWrite(SW_PIN, HIGH);
  //INIT SPI modules
  SPI.begin();

  //int SD card
  initSd();

  lcd.setCursor(0, 2); lcd.print(rtc.getDateStr());
  curTime = rtc.getTime();
  openLog();
  sprintf_P(tmpLog, PSTR("SY|Boot|%s"), VERSION); addLog(tmpLog);
#ifdef RECEIVER
  Serial.print(F("Init sensor radio..."));
  //Init 2.4G radio listening for data
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_MAX);
  radio.setCRCLength( RF24_CRC_16 ) ;
  radio.setPayloadSize(sizeof(myData));
  // Open a writing and reading pipe on each radio, with opposite addresses
  Serial.print("Radio ID: "); Serial.println((const char*)addresses[RADIO_ID]);
  //radio.openWritingPipe(addresses[RADIO_ID]);
  //myData.id = RADIO_ID;
  int j = 1;
  Serial.print(F("Listen ID: "));
  for (int i = 0; i < RADIO_COUNT; i++) {
    if (i != RADIO_ID) {
      Serial.print((const char*)addresses[i]);
      Serial.print(";");
      radio.openReadingPipe(j, addresses[i]);
      j++;
    }
  }
  radio.startListening();
  Serial.println(F("Done"));
#endif
  Serial.print(F("Enable Switch Radio... "));
  //Init Switch 433MHz Radio
  mySwitch.enableTransmit(R_TRAN_PIN);
  Serial.println("Done");
  //Init sensors
#ifdef SHT_SENSOR
  Serial.print(F("Init SHT Sensor... "));
  sht.init();
  Serial.println(F("Done"));
#endif

  //Init error led
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);

  //All switch set to off
  switchReset();

  //Get hardware default mode
  Serial.print(F("Check Hardware Mode... "));
  if (checkMode() != tentMode) {
    changeMode(checkMode());
  }
  Serial.println("Done");

  jsXRestPoint = analogRead(X_PIN);
  jsYRestPoint = analogRead(Y_PIN);

  closeLog();
  if (millis() - inTime < 2000)  delay (2000 - millis() + inTime);
  Serial.println(F("Setup Done"));
  refreshScn();
}

void loop() {
#ifdef DEBUG
  Serial.println("Start loop..");
#endif
  wdt_reset();//I am still alive!!
  inTime = millis();
  curTime = rtc.getTime();

  //loop time and overflow control
  if (previousMillis > inTime) {
    timeOverflowReset();
  }
  previousMillis = inTime;

  doSample = ( inTime - sampleTime > SAMPLE_INTERVAL || sampleTime == 0);
  doSummary = (inTime - summaryTime > SUMMARY_INTERVAL || summaryTime == 0);

  //vvvvvvvvvvvvvvvvStart loop logicvvvvvvvvvvvvvvvvvvvvvvv//
  openLog();
  //================Update Sensors============================//
  revData();
  if (doSample) {
    readEnv();
    sampleTime = inTime;
  }
  //================Process Serial Input======================//
  processSerial();
  if (checkMode() != tentMode) {
    changeMode(checkMode());
  }

  //===============Process JS Action============================//
  readCtl();
  if (scn_mode == 1) ctlSettingScn();
  refreshScn();
  if (inTime - lastUserActionTime > LCD_OFF_DELAY) {
    lcd.noBacklight();
    scn_mode = 0; //exit edit mode
  }
  //===============Process tent operation======================//
  ////vvvvvvvvvvvvvvvv Start Enabled tent logic  vvvvvvvvvvvvvv//
  loadTentEnv();
  if (tentMode > 0) {
    autoLighting();
#ifdef CFAN
    autoCfan();
#endif
#ifdef HEATER
    autoHeater();
#endif
    //end any previous program on time out
    //    if (tentProg != PROG::IDLE
    //        && inTime - tentProgTime > MAX_RUNTIME) progEnd();
    //select program on idle
    if (tentProg == PROG::IDLE || tentProg == PROG::ERR ) {
      selectProgram(); // tent is idle
    }
    if (tentProg == PROG::IDLE
        || tentStep > 0 //confirmed execution
        || confirmProgram()
       ) {
      executeProgram();
    }

    resubmitSwitch();
  }
  ////^^^^^^^^^^^^^^^^ End tent logic      ^^^^^^^^^^^^^^^^^^^^//

  //============Gather statistic=================//
  gatherStat();
  //=============Logging=======================//
  if (doSummary) {
    printTentEnv();
    sprintf_P(tmpLog, STR_STS,
              ttlOprMin[0], ttlOprMin[1], ttlOprMin[2], ttlOprMin[3], ttlOprMin[4], ttlOprMin[5]); addLog(tmpLog);
    summaryTime = inTime;
  }
  //=====================Error handling=========//
  digitalWrite(BLUE_PIN, (error == 0) ? HIGH : LOW);
  //Red light alert on 3mins no connection|| internal sensor error|| received wrong data(radio crashed)
  digitalWrite(RED_PIN, bitRead(error, E_SENSOR) || (inTime - lastRevMinTime > 180000) || tentProg == PROG::ERR);
  //Green light alert on SD error || disabled
  digitalWrite(GREEN_PIN, bitRead(error, E_SD) || skipSd);

  if (tentProg == PROG::ERR) {
    errorCnt++;
  }
  //==================ERROR reboot; when radio lost, reboot the system
  boolean reboot = false;

  if (inTime - lastRevMinTime > 3600000 ||  errorCnt > 100) {
    sprintf_P(tmpLog, PSTR("[Reboot] Radio Timeout")); addLog(tmpLog);
    reboot = true;
  }

  //^^^^^^^^^^^^^^^^^^Finish Loop core logic^^^^^^^^^^^^^^^^^^^//
  closeLog();
  //=====================Finish Loop============================//
#ifdef DEBUG
  Serial.print(F("Loop end in ")); Serial.println((millis() - inTime));
#endif
  if (millis() - inTime < LOOP_TIME)  delay (LOOP_TIME - millis() + inTime);

  if (reboot) rebootFunc();
}

//==========================Control logic============================//
void processSerial() {
#ifdef DEBUG
  Serial.println("[IN] processSerial..");
#endif
  if (Serial.available() > 0) {
    char inChar = Serial.read();
    if (inChar == '?') Serial.println("Help");
    if (inChar >= 'A' && inChar <= 'Z') inChar = inChar + 32;
    if (inChar >= 'a' && inChar <= 'z') {
      switch (inChar) {
        case 'r':
          switchReset();
          break;
        case 'a':
          printTentEnv();
          break;
        case 't':
          Serial.print("Temp:");
          while (!Serial.available());
          float temp = Serial.parseFloat();
          Serial.print("Current RH:");
          while (!Serial.available());
          float inhumid = Serial.parseFloat();
          Serial.print("Target RH:");
          while (!Serial.available());
          float outhumid = Serial.parseFloat();
          Serial.print("Mollier Wetbulb:");
          Serial.println(wetTemp(temp, inhumid, outhumid));
      }
    }
    else if (inChar >= '0' && inChar <= '9') {
      Serial.println("on[o] or off [x], press other to skip?");
      while (!Serial.available());
      char inMode = Serial.read();
      if (inMode == 'o' || inMode == 'x') {
        byte inState = inMode == 'o' ? LOW : HIGH;
        switch (inChar) {
          case mistID+48:
            switchMister(inState, true); break;
          case vFanID+48:
            switchVFan(inState, true); break;
          case lightID+48:
            switchLight(inState, true); break;
          case cFanID+48:
            switchCFan(inState, true); break;
          case heatID+48:
            switchHeater(inState, true); break;
        }
      }
    }
  }
}

boolean isActive() {
  return inTime - lastUserActionTime < LCD_OFF_DELAY;
}
void readCtl() {
#ifdef DEBUG
  Serial.println("[IN] readCtl..");
#endif
  if (digitalRead(SW_PIN) == LOW) { //selection
    if (!isActive()) {
      lcdOn();
    } else if (scn_mode == 0) {
      //==============Enter Setting================//
      if (scn == SCN_ID_SW || scn == SCN_ID_MODE || scn == SCN_ID_SYS || scn == SCN_ID_CONF) { //screens with setting
        scn_mode = 1; //change to setting mode
        x = 0;  y = 0;
        refreshScn();
        btnDelay();// hold control until release
      }
    } else if (scn_mode == 1 && x == 0 && y == 0) {
      //==========Quit Setting=============//
      scn_mode = 0;
      refreshScn();
      btnDelay();// hold control until release
    }
  }
  else if (digitalRead(SW_PIN) == HIGH && isActive()) { //navigation
    //switch screen on display mode only
    if (scn_mode == 0) {
      unsigned long holdTime = millis();
      while (jsMove() && millis() - holdTime < MAX_CTL_HOLD_TIME) {
        wdt_reset();
        lcdOn();
        if ((analogRead(X_PIN) - jsXRestPoint)*JS_ORIENTATION < -JS_TRIGGER) scn--;
        else if ((analogRead(X_PIN) - jsXRestPoint)*JS_ORIENTATION > JS_TRIGGER)  scn++;
        if ((analogRead(Y_PIN) - jsYRestPoint) < -JS_TRIGGER)  frame--;
        else if ((analogRead(Y_PIN) - jsYRestPoint) > JS_TRIGGER) frame++;
        if (scn >= SCN_NUM) scn = 0;
        refreshScn();
        delay(REACT_TIME);
      }
    }
  }
}

void ctlSettingScn() {
#ifdef DEBUG
  Serial.println("[IN] ctlSettingScn..");
#endif
  //=============Cursor movement===============//
  unsigned long holdTime = millis();
  while (jsMove() && millis() - holdTime < MAX_CTL_HOLD_TIME) {
    wdt_reset();
    lcdOn();
    if ((analogRead(X_PIN) - jsXRestPoint)*JS_ORIENTATION < -JS_TRIGGER) x--;
    else if ((analogRead(X_PIN) - jsXRestPoint)*JS_ORIENTATION > JS_TRIGGER)  x++;
    if ((analogRead(Y_PIN) - jsYRestPoint) < -JS_TRIGGER)  y--;
    else if ((analogRead(Y_PIN) - jsYRestPoint) > JS_TRIGGER)  y++;
    if (x >= LCD_SIZE_X) x = 0;
    if (y >= LCD_SIZE_Y) y = 0;
    lcd.setCursor(x, y);
    delay(REACT_TIME);
  }
  //=========Screen Operation==============//
  if (scn == SCN_ID_SW) { //register control
    if (digitalRead(SW_PIN) == LOW && x < 10) {
      if (y == 1) {
        if (x < 6) {
          switchMister(!bitRead(switchStatus, mistID));
        } else {
          switchVFan(!bitRead(switchStatus, vFanID));
        }
      } else if (y == 2) {
        if (x < 6) {
          switchLight(!bitRead(switchStatus, lightID));
        } else {
          switchCFan(!bitRead(switchStatus, cFanID));
        }
      }
      refreshScn();
      btnDelay();// hold control until release
    }
  } else if (scn == SCN_ID_MODE) { //mode selection
    if (digitalRead(SW_PIN) == LOW && y == 1) {
      //int oldMode = tentMode;
      if (!changeMode((tentMode + 1) % TENT_MODE_COUNT)) {
        lcd.setCursor(0, 0); lcd.print(F("FAIL"));
        //tentMode = oldMode; //won't fail on zero(off mode)
        delay(1000);
      }
      bitWrite(switchStatus, sysID, tentMode == TENT_MODE_OFF ? 1 : 0);
      refreshScn();
      btnDelay();// hold control until release
    }
  } else if (scn == SCN_ID_SYS) {
    if (digitalRead(SW_PIN) == LOW && x > 0 ) {
      if (y == 0) {
        closeLog();
        skipSd = true;
      } else if (y == 1) {
        airCon = !airCon;
      }
      scn_mode = 0;
      refreshScn();
      btnDelay();// hold control until release
    }
  } else if (scn == SCN_ID_CONF) {
    editTentConf();
  }
}
boolean changeMode(byte newMode) {
#ifdef DEBUG
  Serial.println("[IN] changeMode..");
#endif
  boolean result = true;
  sprintf_P(tmpLog, STR_CHG_MODE, newMode); addLog(tmpLog);
  if (newMode != TENT_MODE_OFF) {
    result = loadConf(newMode); //0:off, 1:spawn running, 2:fruiting
  }
  progEnd(); //reset tent operation
  tentMode = newMode;
  return result;
}
byte changeValue(byte value) {
#ifdef DEBUG
  Serial.println("[IN] changeValue..");
#endif
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print('X'); lcd.print("CHG VAL:");
  lcd.setCursor(0, 1);
  lcd.print(value);
  btnDelay();
  unsigned long holdTime = millis();
  while (digitalRead(SW_PIN) != LOW && millis() - holdTime < MAX_CTL_HOLD_TIME) {
    if ((analogRead(Y_PIN) - jsYRestPoint) < -JS_TRIGGER)  value--;
    else if ((analogRead(Y_PIN) - jsYRestPoint) > JS_TRIGGER)  value++;
    lcd.setCursor(0, 1); lcd.print("   ");
    lcd.print(value); lcd.setCursor(0, 1);
    wdt_reset();
    delay(REACT_TIME);
  }
  return value;
}
void editTentConf() {
#ifdef DEBUG
  Serial.println("[IN] editTentConf..");
#endif
  if (digitalRead(SW_PIN) == LOW && y == 1) {
    switch (frame % 5) {
      case 0:
        if (x < 3 ) tempMax = changeValue(tempMax);
        else if (x < 6) tempMid = changeValue(tempMid);
        else if (x < 9) tempMin = changeValue(tempMin);
        break;
      case 1:
        if (x < 3 ) humidMax = changeValue(humidMax);
        else if (x < 6) humidMid = changeValue(humidMid);
        else if (x < 9) humidMin = changeValue(humidMin);
        break;
      case 2:
        if (x < 4 ) ventInv = changeValue(ventInv);
        else if (x < 8) ventDur = changeValue(ventDur);
        break;
      case 3:
        if (x < 4 ) lightStart = changeValue(lightStart);
        else if ( x < 8) lightEnd = changeValue(lightEnd);
        break;
      case 4:
        if (x < 4 ) wetInv = changeValue(wetInv);
        else if (x < 8) wetDur = changeValue(wetDur);
        break;
    }
    refreshScn();
    btnDelay();// hold control until release
  }
}

//=====================Screen Dispaly Content====================//
void displayScn() {
  if (scn == SCN_ID_INFO) {
    displayInfo();
  } else if (scn == SCN_ID_ENV) {
    displayEnv();
  } else if (scn == SCN_ID_SW) {
    displaySwitch();
  } else if (scn == SCN_ID_MODE) {
    lcd.setCursor(0, 0);
    lcd.print(F("Mode:"));
    lcd.setCursor(0, 1);
    switch (tentMode) {
      case 0:
        lcd.print('X');
        break;
      default:
        lcd.print(tentMode);
        break;
    }
  } else if (scn == SCN_ID_STAT) {
    displayTentStat();
  } else if (scn == SCN_ID_SYS) {
    lcd.setCursor(0, 0); lcd.print(skipSd ? " X" : " Y");
    lcd.setCursor(8, 0); lcd.print(F("USE SD"));
    lcd.setCursor(0, 1); lcd.print(airCon ? " Y" : " X");
    lcd.setCursor(8, 1); lcd.print(F("AIR CON"));
  } else if (scn == SCN_ID_CONF) {
    displayTentConf();
  }
}
void refreshScn() {
  if (previousScn != scn || previousFrame != frame) {
    lcd.clear();
    previousScn = scn;
    previousFrame = frame;
  }
  displayScn();
  if (scn_mode == 1) {
    lcd.blink();
    lcd.setCursor(0, 0);
    lcd.print('X');
    lcd.setCursor(x, y);
  } else {
    lcd.noBlink();
  }
}
void displayInfo() {
  //lcd.setCursor(0, 0); lcd.print(F("Date:")); lcd.print(rtc.getDateStr());lcd.print(F("Time:"));
  lcd.setCursor(0, 0); lcd.print(rtc.getDateStr()); lcd.print(F("  ")); lcd.print(rtc.getTimeStr());
  lcd.setCursor(0, 1); lcd.print(F("I:")); lcd.print(workingTemp); lcd.print(F("C|")); lcd.print(workingRh); lcd.print(F("%|")); lcd.print(workingCO2);
  lcd.setCursor(0, 2); lcd.print(F("Mode:   "));
  lcd.setCursor(6, 2); switch (tentMode) {
    case 0:
      lcd.print("X");
      break;
    default:
      lcd.print(tentMode);
      break;
  }
  lcd.setCursor(8, 2); lcd.print(F("OPER:"));
  switch (tentProg) {
    case IDLE:
      lcd.print("IDLE  ");
      break;
    case VENT:
      lcd.print("VENT "); lcd.print(tentStep);
      break;
    case  WET:
      lcd.print("HYDR "); lcd.print(tentStep);
      break;
    case  RH_HIGH:
      lcd.print("HiRH "); lcd.print(tentStep);
      break;
    case  RH_LOW:
      lcd.print("LoRH "); lcd.print(tentStep);
      break;
    case TEMP_HIGH:
      lcd.print("HiTP "); lcd.print(tentStep);
      break;
    case ERR:
      lcd.print("ERR  ");
  }

  lcd.setCursor(0, 3);
  lcd.print("M:"); lcd.print(bitRead(switchStatus, mistID));
  lcd.print("|VF:"); lcd.print(bitRead(switchStatus, vFanID));
  lcd.print("|L:"); lcd.print(bitRead(switchStatus, lightID));
  lcd.print("|CF:"); lcd.print(bitRead(switchStatus, cFanID));
}
void displaySwitch() {
  lcd.setCursor(0, 0);
  lcd.print(F("Switch: On[0]/Off[1]"));
  lcd.setCursor(0, 1);
  lcd.print("M: "); lcd.print(bitRead(switchStatus, mistID)); lcd.print(" |VF: "); lcd.print(bitRead(switchStatus, vFanID));
  lcd.setCursor(0, 2);
  lcd.print("L: "); lcd.print(bitRead(switchStatus, lightID)); lcd.print(" |CF: "); lcd.print(bitRead(switchStatus, cFanID));
}
void displayEnv() {
  lcd.setCursor(0, 0);
  lcd.print(F("Sensors Reading:"));
  if (frame % 2 == 0) {
    for (int i = 0; i < RADIO_COUNT; i++) {
      lcd.setCursor(0, i + 1);
      int lastbeat = (inTime - lastRevTime[i]) / 1000;
      lcd.print(i + 1); lcd.print(SPLIT);
      if ((inTime - lastRevTime[i]) < DATA_TIMEOUT) {
        lcd.print("G|");
      } else {
        lcd.print("!|");
      }
      if (lastbeat > 999 || lastRevTime[i] == 0) {
        lcd.print("ABSENT");
      } else {
        lcd.print(lastbeat); lcd.print("s|");
        if (CO2_SENSOR_ID == i + 1) {
          lcd.print(sensorCO2); lcd.print("ppm  ");
        } else {
          lcd.print(sensorTemp[i]); lcd.print("C"); lcd.print(SPLIT); lcd.print(sensorRh[i]); lcd.print("%  ");
        }
      }
    }
  } else {
    lcd.setCursor(0, 1);
    lcd.print("E|"); lcd.print(envTemp); lcd.print("C"); lcd.print(SPLIT); lcd.print(envRh); lcd.print("%  ");
  }
}
void displayTentStat() {
  lcd.setCursor(0, 0);
  lcd.print(F("Sy:"));  lcd.print(ttlOprMin[sysID] / 60); lcd.print('h'); lcd.print(ttlOprMin[sysID] % 60); lcd.print('m');
  lcd.setCursor(10, 0);
  lcd.print(F("Mt:"));  lcd.print(ttlOprMin[mistID] / 60); lcd.print('h'); lcd.print(ttlOprMin[mistID] % 60); lcd.print('m');
  lcd.setCursor(0, 1);
  lcd.print(F("VF:"));  lcd.print(ttlOprMin[vFanID] / 60); lcd.print('h'); lcd.print(ttlOprMin[vFanID] % 60); lcd.print('m');
  lcd.setCursor(10, 1);
  lcd.print(F("LED:")); lcd.print(ttlOprMin[lightID] / 60); lcd.print('h'); lcd.print(ttlOprMin[lightID] % 60); lcd.print('m');
  lcd.setCursor(0, 2);
  lcd.print(F("CF:")); lcd.print(ttlOprMin[cFanID] / 60); lcd.print('h'); lcd.print(ttlOprMin[cFanID] % 60); lcd.print('m');
  lcd.setCursor(10, 2);
  lcd.print(F("Ht:")); lcd.print(ttlOprMin[heatID] / 60); lcd.print('h'); lcd.print(ttlOprMin[heatID] % 60); lcd.print('m');
}
void displayTentConf() {
  lcd.setCursor(0, 0);
  switch (frame % 5) {
    case 0:
      lcd.print(F("Temp[H|M|L]"));
      lcd.setCursor(0, 1);
      lcd.print(tempMax); lcd.print(SPLIT); lcd.print(tempMid); lcd.print(SPLIT); lcd.print(tempMin);
      break;
    case 1:
      lcd.print(F("RH[H|M|L]"));
      lcd.setCursor(0, 1);
      lcd.print(humidMax); lcd.print(SPLIT); lcd.print(humidMid); lcd.print(SPLIT); lcd.print(humidMin);
      break;
    case 2:
      lcd.print(F("Vent[I|D]"));
      lcd.setCursor(0, 1);
      lcd.print(ventInv); lcd.print(F("m|")); lcd.print(ventDur); lcd.print('m');
      break;
    case 3:
      lcd.print(F("Light[S|E]"));
      lcd.setCursor(0, 1);
      lcd.print(lightStart); lcd.print(SPLIT); lcd.print(lightEnd);
      break;
    case 4:
      lcd.print(F("Wet[I|D]"));
      lcd.setCursor(0, 1);
      lcd.print(wetInv); lcd.print(F("h|")); lcd.print(wetDur); lcd.print('m');
      break;
  }
}

//================================Program Selection ====================================//
void suggestProgram() {
#ifdef DEBUG
  Serial.println("[IN] suggestProgram..");
#endif
  //Select Program
  // 2. hydration
  //if (curTime.hour == wetHour &&  curTime.min < wetDur) {
  if (inTime - tentLastWetTime > wetInv * 60 * 60000) {
    suggestion = PROG::WET;
    return;
  }
  //1. Regular ventilation
  //without sensor follow default schedule
  //with sensor, only ventilate when CO2 over limit
  if ((workingCO2 <= 0 && inTime - tentLastFanTime > ventInv * 60000)
      || workingCO2 > CO2_MAX) {
    suggestion = PROG::VENT;
    return;
  }
  if (workingTemp > 0) {
    if (workingTemp > tempMax
        && (!airCon
            || (airCon && wetTemp(envTemp, envRh, humidMax) < workingTemp))) {
      //5. High temperature
      //without airCon, excute anyway
      //with airCon, only execute when wetball temp lower than indoor temp
      suggestion = PROG::TEMP_HIGH;
      return;
    } else if (workingRh < humidMin) {
      //4. Low humidity
      suggestion = PROG::RH_LOW;
      return;
    } else if (workingRh > humidMax
               && workingRh > envRh
               && wetTemp(envTemp, envRh, humidMax) < tempMax) {
      //3. High humidity
      //outside RH must be lower
      //reduction of RH should not raise temp over tempMax
      suggestion = PROG::RH_HIGH;
      return;
    }
    suggestion = PROG::IDLE;
  } else {
    suggestion = PROG::ERR;
  }
  return;
}
void selectProgram() {
#ifdef DEBUG
  Serial.println("[IN] selectProgram..");
#endif
  suggestProgram();
  tentProg = suggestion;
  if (tentProg != PROG::IDLE) {
    tentStep = 0;
    confirmCount = 0;
    decisionTime = inTime;
    sprintf_P(tmpLog, PSTR("PS|%d"), tentProg); addLog(tmpLog);
    printTentEnv();
  }
}
boolean confirmProgram() {
#ifdef DEBUG
  Serial.println("[IN] confirmProgram..");
#endif
  suggestProgram();
  //byte prog = suggestion;
  (tentProg == suggestion) ?  confirmCount++ : confirmCount--;
  //30s decision time to hold execution for more reading
  if ((confirmCount > GOAL_COUNT && inTime - decisionTime > 30000)
      || suggestion == PROG::VENT || suggestion == PROG::WET) {
    sprintf_P(tmpLog, PSTR("PY|%d"), tentProg); addLog(tmpLog);
    tentProgTime = inTime;
    printTentEnv();
    return true;
  } else if (confirmCount == 0 ) {
    sprintf_P(tmpLog, PSTR("PX|%d"), tentProg); addLog(tmpLog);
    printTentEnv();
    tentProg = PROG::IDLE;
  }
  return false;
}

//============================Program Execution=======================================//
void executeProgram() {
#ifdef DEBUG
  Serial.println("[IN] executeProgram..");
#endif
  switch (tentProg) {
    case PROG::RH_LOW://1. Low humidity
      runHumidLo();
      break;
    case PROG::RH_HIGH://2. High humidity
      runHumidHi();
      break;
    case PROG::TEMP_HIGH://4. High temperature
      runTempHi();
      break;
    case PROG::VENT://5. Regular ventilation
      runRegVent();
      break;
    case PROG::WET: //6. hydration
      runHydration();
      break;
    case PROG::IDLE:
    default: //case IDLE
      runIdle();
  }
}
void runTempHi() {
#ifdef DEBUG
  Serial.println("[IN] runTempHi..");
#endif
  if (tentStep == 0) {
    tentStep++;
    logStepChg();
    goalCount = 0;
    //if (workingRh > humidMin)
    switchVFan(LOW);
    if (workingRh < humidMax) switchMister(LOW);
  } else if (tentStep == 1) {
    if (workingRh >= 98) {
      switchMister(HIGH);
    } else if (workingRh < humidMax) {
      switchMister(LOW);
    }
    (workingTemp < 0 || workingTemp <= tempMid
     || (airCon && workingTemp <= wetTemp(envTemp, envRh, humidMax))) ? goalCount++ : goalCount = 0;
    if (goalCount > GOAL_COUNT || inTime - tentProgTime > MAX_RUNTIME) progEnd();
  }
}
void runHumidHi() {
#ifdef DEBUG
  Serial.println("[IN] runHumidHi..");
#endif
  //P1. start fan until humidity < humidMid
  //C1. the result should not cause overheat (inhale air too hot)
  if (tentStep == 0) {
    tentStep++;
    logStepChg();
    goalCount = 0;
    switchVFan(LOW);
  } else if (tentStep == 1) {
    //((inTime - tentProgTime) % 300000 < 240000) ? switchVFan(LOW) : switchVFan(HIGH); //left time for air mixing
    if (workingRh < 0
        || workingRh <= humidMid
        || envRh > workingRh
        || wetTemp(envTemp, envRh, workingRh - 0.5) > tempMax) {
      goalCount++;
    } else {
      goalCount = 0;
    }
    if (goalCount > GOAL_COUNT || inTime - tentProgTime > MAX_RUNTIME) progEnd();
  }
}

void runHumidLo() {
#ifdef DEBUG
  Serial.println("[IN] runHumidLo..");
#endif
  //P1. start mister until workingRh > humidMid
  if (tentStep == 0) {
    tentStep++;
    logStepChg();
    goalCount = 0;
    switchMister(LOW);
  } else if (tentStep == 1) {
    //((inTime - tentProgTime) % 4*60000 < 2*60000) ?  switchMister(LOW) : switchMister(HIGH);  //Alt switch for detection
    (workingRh < 0 || workingRh > humidMid) ?  goalCount++ : goalCount = 0;
    if (goalCount > GOAL_COUNT || inTime - tentProgTime > MAX_RUNTIME) progEnd();
  }
}
void runRegVent() {
#ifdef DEBUG
  Serial.println("[IN] runRegVent..");
#endif
  //1. start fan and fogger until defined duration
  //In temp extreme, stop ventilation once normal CO2 level reached
  if (tentStep == 0) {
    tentStep++;
    logStepChg();
    switchVFan(LOW);
  } else if (tentStep == 1) {
    if (workingRh < 0 || workingRh < humidMid) {
      switchMister(LOW);
    } else if (workingRh > humidMax) {
      switchMister(HIGH);
    }
    if ((inTime - tentProgTime > ventDur * 60000)
        || (workingCO2 > 0 && workingCO2 <= CO2_NORMAL
            && (airCon || envTemp < tempMin || envTemp > tempMax)
           )
       ) {
      tentLastFanTime = inTime;
      progEnd();
    }
  }
}
void runHydration() {
#ifdef DEBUG
  Serial.println("[IN] runHydration..");
#endif
  if (tentStep == 0) {
    tentStep++;
    logStepChg();
    switchMister(LOW);
  } else if (tentStep == 1) {
    if (inTime - tentProgTime > wetDur * 60000) {
      tentLastWetTime = inTime;
      progEnd();
    }
  }
}
/** execute on idle status
    maximize air ventiliation on ideal external air
*/
void runIdle() {
#ifdef DEBUG
  Serial.println("[IN] runIdle..");
#endif
  if ((envTemp > tempMin + 1 && envTemp < tempMax - 1 &&
       envRh > humidMin + 1 && envRh < humidMax - 1)
      || abs(wetTemp(envTemp, envRh, humidMid) - tempMid) < abs(workingTemp - tempMid)
     ) {
    switchVFan(LOW);
  } else if (!((envTemp > tempMin && envTemp < tempMax &&
                envRh > humidMin && envRh < humidMax)
               || abs(wetTemp(envTemp, envRh, humidMid) - tempMid) < abs(workingTemp - tempMid)
              )) {
    switchVFan(HIGH);
  }
}

void logStepChg() {
#ifdef DEBUG
  Serial.println("[IN] logStepChg..");
#endif
  sprintf_P(tmpLog, PSTR("PU|%d|%d|%dm"),  tentProg, tentStep, (inTime - tentProgTime) / 60000); addLog(tmpLog);
  printTentEnv();
}
void progEnd() {
#ifdef DEBUG
  Serial.println("[IN] progEnd..");
#endif
  sprintf_P(tmpLog, PSTR("PF|%d|%dm"),  tentProg, (inTime - tentProgTime) / 60000); addLog(tmpLog);
  printTentEnv();
  switchMister(HIGH);
  switchVFan(HIGH);
  if (tentMode == TENT_MODE_OFF) {
    switchLight(HIGH);
    switchCFan(HIGH);
    switchHeater(HIGH);
  }
  tentProg = PROG::IDLE;
  tentStep = 0;
  tentProgTime = 0;
  goalCount = 0;
}

//===================================Smart equipments=====================================//
void autoLighting() {
  if (tentMode == 0) return;
  byte decision = NULL;
  if (lightStart < lightEnd) {
    if (curTime.hour >= lightStart && curTime.hour < lightEnd
#ifdef MIDDAY_LIGHT_OFF
        && curTime.hour != 12
#endif
       ) {
      decision = LOW;
    } else {
      decision = HIGH;
    }
  } else if (lightStart > lightEnd) {
    if (curTime.hour >= lightStart || curTime.hour < lightEnd) {
      decision = LOW;
    } else {
      decision = HIGH;
    }
  }
  switchLight(decision);
}
#ifdef CFAN
void autoCfan() {
  (tentMode != 0) ? switchCfan(LOW);
}
#endif
#ifdef HEATER
void autoHeater() {
  if (tentMode == 0 || workingTemp < 0) {
    switchHeater(HIGH);
  } else if (workingTemp < tempMin) {
    if (inTime % 300000 < 240000) {
      switchHeater(LOW);
    } else {
      switchHeater(HIGH);
    }
  } else {
    switchHeater(HIGH);
  }
}
#endif
//========================================SD============================================//

void initSd() {
  Serial.print(F("Init SD.."));
  // see if the card is present and can be initialized:
  if (!sd.begin(SD_CHIP_SELECT_PIN, SPI_FULL_SPEED)) {
    Serial.println(F("Failed...Skip SD"));
    bitWrite(error, E_SD, 1);
    skipSd = true;
    // don't do anything more:
    return;
  }
  Serial.println(F("Done"));
}
//format:[TENT_ID(1)][TENT_MODE(1)][HUMID_HI(2)][HUMID_MI(2)][HUMID_LO(2)][TEMP_HI(2)][TEMP_MI(2)][TEMP_LO(2)][VENT_INV_MIN(2)][ventDur_MIN(2)][EOL]
byte loadConf(byte inMode) {
  if (skipSd) {
    return 0;
  }
  if (inMode == 0) return 0;
  boolean result = false;
#ifdef DEBUG
  Serial.print(F("Scan conf for:"));  Serial.println(inMode);
#endif
  if (confFile.open(CONF_FILE, O_READ)) {
    //sprintf_P(tmpLog, PSTR("SY|CONF|%s"), CONF_FILE); addLog(tmpLog);
    String content;
    // read from the file until there's nothing else in it:
    char tmp; byte mode;
    while (confFile.available()) {
      content = "";
      do {
        tmp = confFile.read();
        content += tmp;
      } while (tmp != '\n');
#ifdef DEBUG
      Serial.print(F("Content:")); Serial.println(content);
#endif
      int i = 1;
      mode = content.substring(1, 2).toInt();//1-2
      if (inMode == mode) {
        humidMin =  content.substring(2, 4).toInt();
        humidMid =  content.substring(4, 6).toInt();
        humidMax =  content.substring(6, 8).toInt();
        tempMin =  content.substring(8, 10).toInt();
        tempMid =  content.substring(10, 12).toInt();
        tempMax =  content.substring(12, 14).toInt();
        ventInv = content.substring(14, 16).toInt();
        ventDur = content.substring(16, 18).toInt();
        lightStart =  content.substring(18, 20).toInt();
        lightEnd =  content.substring(20, 22).toInt();
        wetInv =  content.substring(22, 24).toInt();
        wetDur =  content.substring(24, 26).toInt();
        airCon = content.substring(26, 27).equals("Y");
        result = true;
        sprintf_P(tmpLog, STR_CONF,  mode, humidMin, humidMid, humidMax, tempMin, tempMid, tempMax,
                  ventInv, ventDur, lightStart, lightEnd, wetInv, wetDur, airCon ? 'Y' : 'N'); addLog(tmpLog);
        break;
      }
    }
  } else {
    sd.errorPrint(F("E] Conf file read"));
    result = false;
    bitWrite(error, E_SD, 1);
  }
  confFile.close();
  return result ? 0 : 1;
}

//=================================LOG============================================//
void openLog() {
#ifdef DEBUG
  Serial.println("[IN] openLog..");
#endif
  if (skipSd) return;
  if (curTime.mon < 10)
    logFilename[0] = 48;
  else
    logFilename[0] = char((curTime.mon / 10) + 48);
  logFilename[1] = char((curTime.mon % 10) + 48);
  if (curTime.date < 10)
    logFilename[2] = 48;
  else
    logFilename[2] = char((curTime.date / 10) + 48);
  logFilename[3] = char((curTime.date % 10) + 48);

  if (!logFile.open(logFilename, O_WRITE | O_CREAT | O_AT_END) ) {
    sd.errorPrint(&Serial, F("E] Log file open"));
    bitWrite(error, E_SD, 1);
  }
}

void closeLog() {
#ifdef DEBUG
  Serial.println("[IN] closeLog..");
#endif
  if (!skipSd) {
    logFile.close();
  }
}
void addLog(char *msg) {
#ifdef DEBUG
  Serial.println("[IN] addLog..");
#endif
  if (skipSd) return;
  //    if (!logFile.open(logFilename, O_WRITE | O_CREAT | O_AT_END) ) {
  //      sd.errorPrint(F("open failed"));
  //    }
  //else
  Serial.println(msg);
  if ( !logFile.print(rtc.getDateStr())
       || !logFile.print(SPLIT)
       || !logFile.print(rtc.getTimeStr())
       || !logFile.print(SPLIT)
       || !logFile.println(msg)) {
    sd.errorPrint(&Serial, F("E] Log file write "));
    bitWrite(error, E_SD, 1);
  }
  //msg[0] = '\0';
  //    if (!logFile.sync() || logFile.getWriteError()) {
  //      sd.errorPrint(&Serial, F("E] Log write"));
  //    }
  //    logFile.close();
}

//===================================Switch=============================//
void switchMister(boolean newState, boolean force) {
  if (bitRead(switchStatus, mistID) != newState || force) {
    if (!force) {
      sprintf_P(tmpLog, PSTR("SW|Mist|%d"), newState);
      addLog(tmpLog);
    }
    bitWrite(switchStatus, mistID, newState);
    mySwitch.send(mist[newState], 24);
    delay(RADIO_GAP);
  }
  //if (newState == LOW) switchCfan(newState);
  //  Serial.println(millis() - swTime);
}
void switchVFan(boolean newState, boolean force) {
  if (bitRead(switchStatus, vFanID) != newState || force) {
    if (!force) {
      sprintf_P(tmpLog, PSTR("SW|VFan|%d"), newState);
      addLog(tmpLog);
    }
    bitWrite(switchStatus, vFanID, newState);
    mySwitch.send(vfan[newState], 24);
    delay(RADIO_GAP);
  }
}
void switchLight(boolean newState, boolean force) {
  if (bitRead(switchStatus, lightID) != newState || force) {
    if (!force) {
      sprintf_P(tmpLog, PSTR("SW|LED|%d"), newState);
      addLog(tmpLog);
    }
    bitWrite(switchStatus, lightID, newState);
    mySwitch.send(light[newState], 24);
    delay(RADIO_GAP);
  }
}
void switchCFan(boolean newState, boolean force) {
  if (bitRead(switchStatus, cFanID) != newState || force) {
    if (!force) {
      sprintf_P(tmpLog, PSTR("SW|CFan|%d"), newState);
      addLog(tmpLog);
    }
    bitWrite(switchStatus, cFanID, newState);
    mySwitch.send(cfan[newState], 24);
    delay(RADIO_GAP);
  }
}
/* with protection logic to prevent long period heating */
void switchHeater(boolean newState, boolean force) {
  if (bitRead(switchStatus, heatID) != newState || force) {
    if (!force) {
      sprintf_P(tmpLog, PSTR("SW|Heat|%d"), newState);
      addLog(tmpLog);
    }
    bitWrite(switchStatus, heatID, newState);
    mySwitch.send(heat[newState], 24);
    delay(RADIO_GAP);
  }
}
//void switchCooler(boolean newState, boolean force=false) {
//  registerWrite(cooler, newState);
//}
//void switchPump(boolean newState, boolean force=false) {
//  registerWrite(pump, newState);
//}
void switchReset() {
  switchMister(HIGH);
  switchCFan(HIGH);
  switchVFan(HIGH);
  switchLight(HIGH);
  switchHeater(HIGH);
}
void resubmitSwitch() {
#ifdef DEBUG
  Serial.println("[IN] resubmitSwitch..");
#endif
  if (inTime - resubmitTime > RESUBMIT_INTERVAL) {
    switchMister(bitRead(switchStatus, mistID), true);
    switchVFan(bitRead(switchStatus, vFanID), true);
    //switchLight(bitRead(switchStatus, lightID), true);
    resubmitTime = inTime;
  }
}
//============================SENSOR & RADIO================================//
void gatherStat() {
  if (inTime - lastSurvayTime > 10000) {
    int timePassed = (inTime - lastSurvayTime) / 1000;
    for (int i = 0; i < 8; i++) {
      if (bitRead(switchStatus, i) == LOW) ttlOprSec[i] += timePassed;
      ttlOprMin[i] += ttlOprSec[i] / 60;
      ttlOprSec[i] = ttlOprSec[i] % 60;
    }
    lastSurvayTime = inTime;
  }
}
void printTentEnv() {
  char tempStr[7], humidStr[8];
  //print environment
  dtostrf(envTemp, 6, 2, tempStr);
  dtostrf(envRh, 7, 2, humidStr);
  sprintf_P(tmpLog, PSTR("EE|%s|%s"), tempStr, humidStr);  addLog(tmpLog);

  loadTentEnv();
  dtostrf(workingTemp, 6, 2, tempStr);
  dtostrf(workingRh, 7, 2, humidStr);
  sprintf_P(tmpLog, PSTR("EI|%s|%s|%d"),  tempStr, humidStr, workingCO2);  addLog(tmpLog);
}
void loadTentEnv() {
  workingTemp = 0;
  workingRh = 0;
  workingCO2 = 0;
  byte count = 0;
  for (byte i = 0; i < RADIO_COUNT; i++) {
    if (inTime - lastRevTime[i] < DATA_TIMEOUT
        && sensorTemp[i] > 0 && sensorRh[i] > 0) {
      workingTemp += sensorTemp[i];
      workingRh += sensorRh[i];
      count++;
    }
  }
  if (count > 0) {
    workingTemp = workingTemp / count;
    workingRh = workingRh / count;
  } else {
    workingTemp = workingRh = -1;
  }

  if (inTime - lastRevTime[CO2_SENSOR_ID - 1] < DATA_TIMEOUT
      && sensorCO2 > 0) {
    workingCO2 = sensorCO2;
  } else {
    workingCO2 = -1;
  }

}
void revData() {
#ifdef DEBUG
  Serial.println("[IN] revData..");
#endif
#ifdef RECEIVER
  if ( radio.available()) {
    // Variable for the received timestamp
    //while (radio.available()) {
    // While there is data ready
    radio.read(&myData, sizeof(myData));             // Get the payload
    //}
#ifdef DEBUG
    Serial.print(F("Received: "));
    Serial.print(myData.id); Serial.print(F(" | ")); Serial.print(myData.type); Serial.print(F(" | "));
    Serial.print(myData.value1); Serial.print(F(" | ")); Serial.println(myData.value2);
#endif
    int sensorId = myData.id;
    if (myData.type == SENSOR_TYPE_TEMP_RH) {
      sensorTemp[sensorId - 1] = myData.value1;
      sensorRh[sensorId - 1] = myData.value2;
    } else if (myData.type == SENSOR_TYPE_CO2) {
      sensorCO2 = (int) myData.value1;
    }
    lastRevTime[sensorId - 1] = inTime;
    lastRevMinTime = inTime;
  }
#endif
}
void readEnv() {
#ifdef SHT_SENSOR
  if (sht.readSample()) {
#else
  if (false) {
#endif
    envTemp = sht.getTemperature();
    envRh = sht.getHumidity();
    bitWrite(error, E_SENSOR, 0);
  } else {
    Serial.println(F("E]SHT"));
    envTemp = rtc.getTemp();
    if (envTemp > 35) envRh = 60; else envRh = 70; //assumed
    bitWrite(error, E_SENSOR, 1);
  }
}

//===========================Misc.============================//
void lcdOn() {
  lastUserActionTime = inTime;
  lcd.backlight();
}
void btnDelay() {
  unsigned long holdTime = millis();
  while (digitalRead(SW_PIN) == LOW && millis() - holdTime < 60000) {
    wdt_reset();
    delay(100); // hold control until release
  }
}
byte checkMode() {
  if (analogRead(A0) < 100) {
    return 0;
  } else if (analogRead(A1) < 100) {
    return 1;
  } else if (analogRead(A2) < 100) {
    return 2;
  }
  return 0;
}
boolean jsMove() {
  if (analogRead(X_PIN) - jsXRestPoint < -JS_TRIGGER || analogRead(X_PIN) - jsXRestPoint > JS_TRIGGER
      || analogRead(Y_PIN) - jsYRestPoint < -JS_TRIGGER || analogRead(Y_PIN) - jsYRestPoint > JS_TRIGGER) {
    lcdOn();
    return true;
  }
  return false;
}

/** function for approximate wet temperature by mollier chart to specific humidity

**/
float wetTemp(const float inTemp, const float inHumid, const float outHumid) {
  if (inHumid > outHumid) return inTemp; //no temp drop on dehydration
  float wetbulb = mollierTemp(inTemp, inHumid);
  float tailTemp = mollierTemp(inTemp, outHumid);
  return wetbulb + (inTemp - tailTemp);
}
/** function to wet blub temperature

*/
float mollierTemp(const float inTemp, const float inHumidity) {
  float valRV = (inHumidity / 100 ) * calPSValue(inTemp);

  //            CALCULATION OF NB FUNCTION
  float n;
  float nh = inTemp;
  float nl = -273.15;
  float pw1;
  float ps ;
  for (int k = 1; k <= 30; k++)
  {
    n = (nh + nl) / 2;
    //alert(n);
    ps = calPSValue(n);

    pw1 = ps - 66.374 * (1 - 0.00112 * n) * (inTemp - n);

    if (pw1 < valRV)
    {
      nl = n;
    } else {
      nh = n;
    }
  }
  return n;
}

float calPSValue(const float val) {
  float f1 = (1 - (273.15 + val) / 647.3);
  if (val >= 0)
  {
    return 22120000 * (exp(((-7.691234564 * f1
                             - 26.08023696 * pow(f1, 2)
                             - 168.1706546 * pow(f1, 3)
                             + 64.23285504 * pow(f1, 4)
                             - 118.9646225 * pow(f1, 5)) /
                            ((273.15 + val) / 647.3) /
                            (1 + 4.16711732 * f1 + 20.9750676 * pow(f1, 2)) - f1 / (1000000000 * pow(f1, 2) + 6))));
  } else {
    return 100 * exp(-20.947 * (273.15 / (273.15 + val) - 1) - 3.56654 * log(273.15 / (273.15 + val)) + 2.01889 * (1 - (273.15 + val) / 273.15) + 0.78584);
  }
}
