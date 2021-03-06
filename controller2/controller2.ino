//2.8 include CO2 sensor linked logic
//3.0
//Rewrite environment control execution logic

//#define DEBUG
#define VERSION "v3.0"

//Equipment List
//#define CFAN
//#define HEATER
#define MH_LCD

#include "controller.h"
#include "pin.h"

#include <Wire.h>
#include <DS3231.h> //Clock
#include <LCD.h>
#include <LiquidCrystal_I2C.h> // F Malpartida's NewLiquidCrystal library
//#include "SHTSensor.h"
#include <SPI.h>
#include <SdFat.h> //SD card, softSpi
#include <RF24.h> //2.4GHz radio
#include <RCSwitch.h> //433 radio
#include <SimpleDHT.h> //self-tailored made DHT22 lib (removed DHT11)
#include <avr/wdt.h> //watchdog
#include <common.h>

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
#define LCD_OFF_DELAY 15000 //idle loop to turn off LCD backlight
#define REACT_TIME 500 //reaction time of joystick action
#define MAX_CTL_HOLD_TIME 30000
#define RADIO_GAP 300

#define SAMPLE_TIME 5000 //interval to read local sensor
//Display variables
#define SCN_NUM 7
#define SCN_ID_INFO 0
#define SCN_ID_ENV 1
#define SCN_ID_RELAY 2
#define SCN_ID_MODE 3
#define SCN_ID_STAT 4
#define SCN_ID_CONF 5
#define SCN_ID_SYS 6

#define CO2_NORMAL 450
#define CO2_MAX 2500

#define RADIO_ID 0

//flag for error led
#define OK B00000000
#define E_SENSOR B00000001
#define E_SD B00000010

//Tent program
enum PROG {
  AUTO, VENT, WET
};
#define TENT_MODE_COUNT 3
#define TENT_MODE_OFF 0
#define TENT_MODE_INCUBATION 1
#define TENT_MODE_FRUIT 2

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(RF_CE_PIN, RF_CS_PIN);
RCSwitch mySwitch = RCSwitch();
//Clock
DS3231  rtc(SDA, SCL);
Time curTime;
LiquidCrystal_I2C lcd(I2C_ADDR, BACKLIGHT, POSITIVE);

const PROGMEM char STR_CHG_MODE[] = "CM|%d";
const PROGMEM char STR_STS[] = "ST|%d|%d|%d|%d|%d|%d";
const PROGMEM char STR_CONF[] = "M%d|H:%d-%d-%d|T:%d-%d-%d|V:%d,%d|L:%d-%d|W:%d,%d";

static char logFilename[] = "xxxx.LOG\0";
static char tmpLog[80];

int jsXRestPoint, jsYRestPoint;
unsigned long inTime = 0;
unsigned long previousMillis = 0;
unsigned long lastUserActionTime = 0;
unsigned long sampleTime = 0;
boolean doSample = true;
unsigned long summaryTime = 0;
boolean doSummary = true;
unsigned long resubmitTime=0;

boolean skipSd = false;
byte error = B00000000;

//Display variables
byte scn = 0; //display 0:time, 1:temp, 2:Relay status, 3: operation mode, 4: tent stat, 6: system opr, 5: tent conf
byte frame = 0;
byte scn_mode = 0;
byte x = 0, y = 0;

//Tent Operation
byte tentMode = 0;
PROG tentProg = PROG::AUTO;
byte tentStep = 0;
unsigned long tentProgTime = 0; //start time of prog execution
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
byte humidHi = 98; byte humidMid = 90; byte humidLo = 87;
byte tempHi = 28; byte tempMid = 25; byte tempLo = 15;
byte ventInv = 15; byte ventDur = 3;
byte lightStart = 0; byte lightEnd = 0;
byte wetInv = 6; byte wetDur = 10;

//SD card
/* CATALEX SD CARD READER SOFT-SPI PART */
// Here are the pins for the soft-spi-bus defined
// You can select every pin you want, just don't put them on an existing hardware SPI pin.
SdFatSoftSpi<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> sd;
SdFile inFile, confFile, logFile;

//Sensors
SimpleDHT22 dht22;
float envTemp = 0, workingTemp = 0, sensorTemp[RADIO_COUNT] = {0, 0, 0};
float envRh = 0, workingRh = 0, sensorRh[RADIO_COUNT] = {0, 0, 0};
int workingCO2 = 0, sensorCO2 = 0;
long lastRevTime[RADIO_COUNT] = {0, 0, 0};

void timeOverflowReset() {
  tentProgTime = 0;
  tentLastFanTime = 0;
  tentLastWetTime = 0;
  lastSurvayTime = 0;
  for (int i = 0; i < RADIO_COUNT; i++) {
    lastRevTime[i] = 0;
  }
  sampleTime = 0;
  summaryTime = 0;
  lastUserActionTime = 0;
  resubmitTime=0;
}

void(* rebootFunc) (void) = 0;

void setup() {
  inTime = millis();
  Serial.begin(9600);
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
  curTime = rtc.getTime();
  // The following lines can be uncommented to set the date and time

  //  rtc.setDOW(WEDNESDAY);     // Set Day-of-Week to SUNDAY
  //  rtc.setTime(15, 46, 30);     // Set the time to 12:00:00 (24hr format)
  //  rtc.setDate(8, 8, 2018);   // Set the date to January 1st, 2014

  //Joystick
  pinMode(SW_PIN, INPUT);
  digitalWrite(SW_PIN, HIGH);
  //INIT SPI modules
  SPI.begin();

  //int SD card
  initSd();
  openLog();
  sprintf_P(tmpLog, PSTR("SY|Boot|%s"), VERSION); addLog(tmpLog);
  //Init 2.4G radio listening for data
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_MAX);
  // Open a writing and reading pipe on each radio, with opposite addresses
  Serial.print("Radio ID: "); Serial.println((const char*)addresses[RADIO_ID]);
  radio.openWritingPipe(addresses[RADIO_ID]);
  myData.id = RADIO_ID;
  int j = 1;
  Serial.print("Listen ID: ");
  for (int i = 0; i < RADIO_COUNT; i++) {
    if (i != RADIO_ID) {
      Serial.print((const char*)addresses[i]);
      Serial.print(";");
      radio.openReadingPipe(j, addresses[i]);
      j++;
    }
  }
  radio.startListening();

  //Init Switch 433MHz Radio
  mySwitch.enableTransmit(R_TRAN_PIN);

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
#ifdef DEBUG
  Serial.print("Default mode from analog pin:");
  Serial.print(analogRead(A0)); Serial.print('|');
  Serial.print(analogRead(A1)); Serial.print('|');
  Serial.println(analogRead(A2));
#endif
  tentMode = 0;
  if (analogRead(A1) < 100) {
    tentMode = 1;
  } else if (analogRead(A2) < 100) {
    tentMode = 2;
  }
  loadConf(tentMode);

  jsXRestPoint = analogRead(X_PIN);
  jsYRestPoint = analogRead(Y_PIN);

  closeLog();
  if (millis() - inTime < 2000)  delay (2000 - millis() + inTime);
}

void loop() {
  wdt_reset();//I am still alive!!
  error = 0;

  inTime = millis();
  curTime = rtc.getTime();

  //loop time and overflow control
  if (previousMillis > inTime) {
    timeOverflowReset();
  }
  previousMillis = inTime;

  doSample = ( inTime - sampleTime > SAMPLE_TIME || sampleTime == 0);
  doSummary = (inTime - summaryTime > 10 * 60000 || summaryTime == 0);

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
  if (tentMode > 0) {
    loadTentEnv();
    autoLighting();
#ifdef CFAN
    autoCfan();
#endif
#ifdef HEATER
    autoHeater();
#endif
    if (tentProg == PROG::AUTO) selectProgram();
    else if (tentStep > 0 || confirmProgram())  executeProgram();

    if (tentProg == PROG::AUTO) {
      autoVFan();
      autoMister();
    }
  }
  ////^^^^^^^^^^^^^^^^ End tent logic      ^^^^^^^^^^^^^^^^^^^^//
  resubmitSwitch();
  //============Gather statistic=================//
  usageStat();
  //=============Logging=======================//
  if (doSummary) {
    printTentEnv(255);
    printTentEnv(0);
    sprintf_P(tmpLog, STR_STS,
              ttlOprMin[0], ttlOprMin[1], ttlOprMin[2], ttlOprMin[3], ttlOprMin[4], ttlOprMin[5]); addLog(tmpLog);
    summaryTime = inTime;
  }
  //=====================Error handling=========//
  if (error == 0) {
    digitalWrite(BLUE_PIN, HIGH);
  } else {
    digitalWrite(BLUE_PIN, LOW);
  }
  digitalWrite(RED_PIN, bitRead(error, 0));
  digitalWrite(GREEN_PIN, bitRead(error, 1));

  //^^^^^^^^^^^^^^^^^^Finish Loop core logic^^^^^^^^^^^^^^^^^^^//
  closeLog();

  //==================ERROR reboot; when all radio last rev time ran out, reboot the system
  boolean reboot = true;
  for (int i = 0; i < RADIO_COUNT; i++) {
    if (inTime - lastRevTime[i] < 300000) reboot = false;
  }
  if (reboot) {
    rebootFunc();
  }
  //=====================Finish Loop============================//
#ifdef DEBUG
  Serial.print(F("Loop end in ")); Serial.println((millis() - inTime));
#endif
  if (millis() - inTime < 1000)  delay (LOOP_TIME - millis() + inTime);
}


void resubmitSwitch() {
  if (inTime - resubmitTime > 60000) {
    //  switch (inTime % 5) {
    //    case mistID:
    switchMister(bitRead(switchStatus, mistID), true);// break;
    //    case vFanID:
    switchVFan(bitRead(switchStatus, vFanID), true); //break;
    //    case lightID:
    switchLight(bitRead(switchStatus, lightID), true); //break;
    //    case cFanID:
    //      switchCFan(bitRead(switchStatus, cFanID), true); break;
    //    case heatID:
    //      switchHeater(bitRead(switchStatus, heatID), true); break;
    //  }
    resubmitTime = inTime;
  }
}
void processSerial() {
  if (Serial.available() > 0) {
    char inChar = Serial.read();
    int bitToSet;
    if (inChar == '?') Serial.println("Help");
    if (inChar >= 'A' && inChar <= 'Z') inChar = inChar + 32;
    if (inChar >= 'a' && inChar <= 'z') {
      switch (inChar) {
        case 'r':
          switchReset();
          break;
        case 'a':
          printTentEnv(255);
          printTentEnv(0);
          break;
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
            switchMister(inState); break;
          case vFanID+48:
            switchVFan(inState); break;
          case lightID+48:
            switchLight(inState); break;
          case cFanID+48:
            switchCFan(inState); break;
          case heatID+48:
            switchHeater(inState); break;
        }
      }
    }
  }
}

void readCtl() {
  //switch screen on display mode only
  if (digitalRead(SW_PIN) == HIGH) { //navigation
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
  } else { //selection
    lcdOn();
    if (scn_mode == 0) {
      //==============Enter Setting================//
      if (scn == SCN_ID_RELAY || scn == SCN_ID_MODE || scn == SCN_ID_SYS || scn == SCN_ID_CONF) { //screens with setting
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
}
boolean jsMove() {
  return (analogRead(X_PIN) - jsXRestPoint < -JS_TRIGGER || analogRead(X_PIN) - jsXRestPoint > JS_TRIGGER
          || analogRead(Y_PIN) - jsYRestPoint < -JS_TRIGGER || analogRead(Y_PIN) - jsYRestPoint > JS_TRIGGER);
}
void ctlSettingScn() {
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
  if (scn == SCN_ID_RELAY) { //register control
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
      tentMode = (tentMode + 1) % TENT_MODE_COUNT;
      byte result = 0;
      if (tentMode != TENT_MODE_OFF) {
        result = loadConf(tentMode); //0:off, 1:spawn running, 2:fruiting
      }
      if (tentMode == TENT_MODE_OFF || result == 0) {
        sprintf_P(tmpLog, STR_CHG_MODE, tentMode); addLog(tmpLog);
        progEnd(); //reset tent operation
      } else {
        lcd.setCursor(0, 0); lcd.print(F("FAIL"));
        //tentMode = oldMode; //won't fail on zero(off mode)
        delay(1000);
      }
      bitWrite(switchStatus, sysID, tentMode == TENT_MODE_OFF ? 1 : 0);
      refreshScn();
      btnDelay();// hold control until release
    }
  } else if (scn == SCN_ID_SYS) {
    if (digitalRead(SW_PIN) == LOW && x > 0 && y == 0) {
      closeLog();
      skipSd = true;
      scn_mode = 0;
      refreshScn();
      btnDelay();// hold control until release
    }
  } else if (scn == SCN_ID_CONF) {
    editTentConf();
  }
}

byte previousScn, previousFrame;
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

void displayScn() {
  if (scn == SCN_ID_INFO) {
    displayInfo();
  } else if (scn == SCN_ID_ENV) {
    displayEnv();
  } else if (scn == SCN_ID_RELAY) {
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
    lcd.setCursor(5, 0); lcd.print(F("USE SD"));

  } else if (scn == SCN_ID_CONF) {
    displayTentConf();
  }
}
void lcdOn() {
  lastUserActionTime = inTime;
  lcd.backlight();
}
void displayInfo() {
  lcd.setCursor(0, 0); lcd.print(F("Date:")); lcd.print(rtc.getDateStr());
  lcd.setCursor(0, 1); lcd.print(F("Time:")); lcd.print(rtc.getTimeStr());
  lcd.setCursor(0, 2); lcd.print(workingTemp); lcd.print(F("C|")); lcd.print(workingRh); lcd.print(F("% "));
  lcd.setCursor(0, 3); lcd.print(F("Mode:"));
  switch (tentMode) {
    case 0:
      lcd.print('X');
      break;
    default:
      lcd.print(tentMode);
      break;
  }
  lcd.setCursor(8, 3); lcd.print(F("STAT:"));
  switch (tentProg) {
    case AUTO:
      lcd.print("AUTO");
      break;
    case VENT:
      lcd.print("VENT "); lcd.print(tentStep);
      break;
    case  WET:
      lcd.print("HYDR "); lcd.print(tentStep);
      break;
  }
}
void displaySwitch() {
  lcd.setCursor(0, 0);
  lcd.print(F("Switch: On[0]/Off[1]"));
  lcd.setCursor(0, 1);
  lcd.print("M: "); lcd.print(bitRead(switchStatus, mistID)); lcd.print(" |V.F: "); lcd.print(bitRead(switchStatus, vFanID));
  lcd.setCursor(0, 2);
  lcd.print("L: "); lcd.print(bitRead(switchStatus, lightID)); lcd.print(" |C.F: "); lcd.print(bitRead(switchStatus, cFanID));
}
void displayEnv() {
  //  frame = frame % sizeof(DHT22_PIN);
  //  //updateSensor(frame);
  //  lcd.setCursor(0, 0);
  //  lcd.print(frame); lcd.print(F(".T:")); lcd.print(sensorTemp[frame]); lcd.print(F(" *C "));
  //  lcd.setCursor(0, 1);
  //  lcd.print(F("H:")); lcd.print(sensorRh[frame]); lcd.print(F(" RH% "));
  lcd.setCursor(0, 0);
  lcd.print(F("Sensors Reading:"));
  if (frame % 2 == 0) {
    for (int i = 0; i < RADIO_COUNT; i++) {
      lcd.setCursor(0, i + 1);
      int lastbeat = (inTime - lastRevTime[i]) / 1000;
      lcd.print(i + 1); lcd.print(SPLIT);
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
    lcd.print(envTemp); lcd.print("C"); lcd.print(SPLIT); lcd.print(envRh); lcd.print("%  ");
  }
}
void displayTentStat() {
  lcd.setCursor(0, 0);
  lcd.print(F("Sy:"));  lcd.print(ttlOprMin[sysID] / 60); lcd.print('h'); lcd.print(ttlOprMin[sysID] % 60); lcd.print('m'); //lcd.print(ttlOprSec[mistID]); lcd.print('s');
  lcd.setCursor(10, 0);
  lcd.print(F("Mt:"));  lcd.print(ttlOprMin[mistID] / 60); lcd.print('h'); lcd.print(ttlOprMin[mistID] % 60); lcd.print('m'); //lcd.print(ttlOprSec[mistID]); lcd.print('s');
  lcd.setCursor(0, 1);
  lcd.print(F("VF:"));  lcd.print(ttlOprMin[vFanID] / 60); lcd.print('h'); lcd.print(ttlOprMin[vFanID] % 60); lcd.print('m'); //lcd.print(ttlOprSec[vFanID]); lcd.print('s');
  lcd.setCursor(10, 1);
  lcd.print(F("LED:")); lcd.print(ttlOprMin[lightID] / 60); lcd.print('h'); lcd.print(ttlOprMin[lightID] % 60); lcd.print('m'); //lcd.print(ttlOprSec[lightID]); lcd.print('s');
  lcd.setCursor(0, 2);
  lcd.print(F("CF:")); lcd.print(ttlOprMin[cFanID] / 60); lcd.print('h'); lcd.print(ttlOprMin[cFanID] % 60); lcd.print('m'); //lcd.print(ttlOprSec[cFanID]); lcd.print('s');
  lcd.setCursor(10, 2);
  lcd.print(F("Ht:")); lcd.print(ttlOprMin[heatID] / 60); lcd.print('h'); lcd.print(ttlOprMin[heatID] % 60); lcd.print('m'); //lcd.print(ttlOprSec[heatID]); lcd.print('s');
}

void displayTentConf() {
  lcd.setCursor(0, 0);
  switch (frame % 5) {
    case 0:
      lcd.print(F("Temp[H|M|L]"));
      lcd.setCursor(0, 1);
      lcd.print(tempHi); lcd.print(SPLIT); lcd.print(tempMid); lcd.print(SPLIT); lcd.print(tempLo);
      break;
    case 1:
      lcd.print(F("RH[H|M|L]"));
      lcd.setCursor(0, 1);
      lcd.print(humidHi); lcd.print(SPLIT); lcd.print(humidMid); lcd.print(SPLIT); lcd.print(humidLo);
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
byte changeValue(byte value) {
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
  if (digitalRead(SW_PIN) == LOW && y == 1) {
    switch (frame % 5) {
      case 0:
        if (x < 3 ) tempHi = changeValue(tempHi);
        else if (x < 6) tempMid = changeValue(tempMid);
        else if (x < 9) tempLo = changeValue(tempLo);
        break;
      case 1:
        if (x < 3 ) humidHi = changeValue(humidHi);
        else if (x < 6) humidMid = changeValue(humidMid);
        else if (x < 9) humidLo = changeValue(humidLo);
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

//================================Scheduler Program====================================//
PROG suggestProgram() {
  //Select Program
  // 6. hydration
  if (inTime - tentLastWetTime > wetInv * 60 * 60000) {
    return PROG::WET;
  }
  //5. Regular ventilation (enable while without CO2 sensor reading)
  if ((workingCO2 <= 0 && inTime - tentLastFanTime > ventInv * 60000)) {
    return PROG::VENT;
  }

  return PROG::AUTO;
}
void selectProgram() {
  tentProg = suggestProgram();
  if (tentProg != PROG::AUTO) {
    tentStep = 0;
    sprintf_P(tmpLog, PSTR("PS|%d"), tentProg); addLog(tmpLog);
    printTentEnv(0);
  }
}
boolean confirmProgram() {
  byte prog = suggestProgram();
  if (prog == PROG::VENT || prog == PROG::WET) { //decision time to hold execution for more reading
    sprintf_P(tmpLog, PSTR("PY|%d"), tentProg); addLog(tmpLog);
    printTentEnv(0);
    return true;
  }
  return false;
}

void executeProgram() {
  switch (tentProg) {
    case PROG::VENT://5. Regular ventilation
      runRegVent();
      break;
    case PROG::WET: //6. hydration
      runHydration();
      break;
  }
}
void runRegVent() {
  //1. start fan and fogger until defined duration
  if (tentStep == 0) {
    tentStep++;
    tentProgTime = inTime;
    logStepChg();
    switchVFan(LOW);
    switchMister(HIGH);
  } else if (tentStep == 1) {
    if (workingRh < 0 || workingRh < humidMid) {
      switchMister(LOW);
    } else if (workingRh > humidHi) {
      switchMister(HIGH);
    }
    if (inTime - tentProgTime > ventDur * 60000
        || (workingCO2 > 0 && workingCO2 < CO2_NORMAL)) {
      tentLastFanTime = inTime;
      progEnd();
    }
  }
}
void runHydration() {
  if (tentStep == 0) {
    tentStep++;
    tentProgTime = inTime;
    logStepChg();
    switchMister(LOW);
    switchVFan(HIGH);
  } else if (tentStep == 1) {
    if (inTime - tentProgTime > wetDur * 60000) {
      tentLastWetTime = inTime;
      progEnd();
    }
  }
}

void logStepChg() {
  sprintf_P(tmpLog, PSTR("PU|%d|%d|%dm"),  tentProg, tentStep, (inTime - tentProgTime) / 60000); addLog(tmpLog);
  printTentEnv(0);
}
void progEnd() {
  sprintf_P(tmpLog, PSTR("PF|%d|%dm"),  tentProg, (inTime - tentProgTime) / 60000); addLog(tmpLog);
  printTentEnv(0);
  switchMister(HIGH);
  switchVFan(HIGH);
  if (tentMode == TENT_MODE_OFF) {
    switchLight(HIGH);
    switchCFan(HIGH);
    switchHeater(HIGH);
  }
  tentProg = PROG::AUTO;
  tentStep = 0;
  tentProgTime = 0;
}
//===================================Smart equipments=====================================//
void autoLighting() {
  if (tentMode == 0) return;
  if (lightStart < lightEnd) {
    if (curTime.hour >= lightStart && curTime.hour < lightEnd) {
      switchLight(LOW);
    } else {
      switchLight(HIGH);
    }
  } else if (lightStart > lightEnd) {
    if (curTime.hour >= lightStart || curTime.hour < lightEnd) {
      switchLight(LOW);
    } else {
      switchLight(HIGH);
    }
  }
}
#ifdef CFAN
void autoCfan() {
  if (tentMode != 0)  switchCfan(LOW);
}
#endif
/*Simplified limited range mollier apporximation
   around 25~35C 70RH*/
float mollierTemp(float targetRh) {
  float wetBulb = envTemp;
  float wetRh = envRh;
  float stepRh = 0;
  if (wetRh < 40 && targetRh > wetRh) {
    if (targetRh >= 40) stepRh = 40; else stepRh = targetRh;
    wetBulb = wetBulb - (stepRh - wetRh) / 10 * 3;
    wetRh = 40;
  }
  if (wetRh < 50 && targetRh > wetRh) {
    if (targetRh >= 50) stepRh = 50; else stepRh = targetRh;
    wetBulb = wetBulb - (stepRh - wetRh) / 10 * 2.5;
    wetRh = 50;
  }
  if (wetRh < 70 && targetRh > wetRh) {
    if (targetRh >= 70) stepRh = 70; else stepRh = targetRh;
    wetBulb = wetBulb - (stepRh - wetRh) / 10 * 2;
    wetRh = 70;
  }
  if (wetRh < 100 && targetRh > wetRh) {
    if (targetRh >= 100) stepRh = 100; else stepRh = targetRh;
    wetBulb = wetBulb - (stepRh - wetRh) / 10 * 1.5;
    wetRh = 100;
  }
  return wetBulb;
}
/* keep
    1. CO2 <= Normal
    2. lo<temperature<mid
    3. Rh  < hi
*/
void autoVFan() {
  if (tentMode == 0 || tentProg != PROG::AUTO) return;
  byte vote = HIGH;
  if (workingCO2 > CO2_MAX) {//critical CO2 level
    vote = LOW;
  } else if (workingTemp > tempHi) { //critical temperature
    if (workingRh > humidLo
        && mollierTemp() < workingTemp)
      vote = LOW;
  } else if (workingTemp < tempLo) {
    if (workingRh > humidLo
        && envTemp > workingTemp)
      vote = LOW;
  } else if (workingCO2 > CO2_NORMAL) { //fine tuning
    if (workingRh > humidLo
        && mollierTemp() < tempHi)
      vote = LOW;
  } else if (workingTemp > tempMid) { //fine tuning
    if (workingRh > humidLo
        && mollierTemp() < workingTemp)
      vote = LOW;
  } else if (workingRh > humidHi) { //least concerned Rh tuning
    if (envRh < workingRh
        && mollierTemp(humidMid)  < tempMid
        && mollierTemp(humidMid)  > tempLo) {
      vote = LOW;
    }
  }
  if (vote == LOW) tentLastFanTime = inTime;
  switchVFan(vote);
}
void autoMister() {
  if (tentMode == 0 || tentProg != PROG::AUTO) return;
  byte vote = HIGH;
  if (workingRh < humidMid) {
    vote = LOW;
  } else if (workingTemp > tempMid) {
      if (workingRh < 99) {
      vote = LOW;
    }
  }
  switchMister(vote);
}
#ifdef HEATER
void autoHeater() {
  if (tentMode == 0 || workingTemp < 0) {
    switchHeater(HIGH);
  } else if (workingTemp < tempLo) {
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
//========================================SD Logger============================================//
void initSd() {
  Serial.print(F("Init SD.."));
  // see if the card is present and can be initialized:
  if (!sd.begin(SD_CHIP_SELECT_PIN)) {
    Serial.println(F("XX"));
    // don't do anything more:
    return;
  }
  Serial.println(F("OK"));
}
//void readLog() {
//  if (inFile.open(logFilename, O_READ)) {
//    while (inFile.available()) {
//      Serial.write(inFile.read());
//    }
//    inFile.close();
//  } else {
//    Serial.print(F("E]R.log"));
//    error = E_SD;
//  }
//}
//format:[TENT_ID(1)][TENT_MODE(1)][HUMID_HI(2)][HUMID_MI(2)][HUMID_LO(2)][TEMP_HI(2)][TEMP_MI(2)][TEMP_LO(2)][VENT_INV_MIN(2)][ventDur_MIN(2)][EOL]
byte loadConf(byte inMode) {
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
      mode = content.substring(1, 2).toInt();
      if (inMode == mode) {
        humidLo =  content.substring(2, 4).toInt();
        humidMid =  content.substring(4, 6).toInt();
        humidHi =  content.substring(6, 8).toInt();
        tempLo =  content.substring(8, 10).toInt();
        tempMid =  content.substring(10, 12).toInt();
        tempHi =  content.substring(12, 14).toInt();
        ventInv = content.substring(14, 16).toInt();
        ventDur = content.substring(16, 18).toInt();
        lightStart =  content.substring(18, 20).toInt();
        lightEnd =  content.substring(20, 22).toInt();
        wetInv =  content.substring(22, 24).toInt();
        wetDur =  content.substring(24, 26).toInt();
        result = true;
        sprintf_P(tmpLog, STR_CONF,  mode, humidHi, humidMid, humidLo, tempHi, tempMid, tempLo,
                  ventInv, ventDur, lightStart, lightEnd, wetInv, wetDur); addLog(tmpLog);
        break;
      }
    }
  } else {
    sd.errorPrint(F("E]R.conf"));
    result = false;
    error = error | E_SD;
  }
  confFile.close();
  return result ? 0 : 1;
}

//=================================Misc.============================================//
void openLog() {
  if (skipSd) return;
  if (!logFile.open(logFilename, O_WRITE | O_CREAT | O_AT_END) ) {
    sd.errorPrint(F("open failed"));
  }
}
void closeLog() {
  if (skipSd) return;
  if (!logFile.sync() || logFile.getWriteError()) {
    sd.errorPrint("write error");
  }
  logFile.close();
}
void addLog(char *msg) {
  Serial.print(rtc.getTimeStr()); Serial.print(SPLIT); Serial.println(msg);
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

  if (!skipSd) {
    //    if (!logFile.open(logFilename, O_WRITE | O_CREAT | O_AT_END) ) {
    //      sd.errorPrint(F("open failed"));
    //    }
    //else
    if ( !logFile.print(rtc.getDateStr())
         || !logFile.print(SPLIT)
         || !logFile.print(rtc.getTimeStr())
         || !logFile.print(SPLIT)
         || !logFile.println(msg)) {
      sd.errorPrint(F("E]W.log"));
      error = error | E_SD;
    }
    msg[0] = 0;
    //    if (!logFile.sync() || logFile.getWriteError()) {
    //      sd.errorPrint("write error");
    //    }
    //    logFile.close();
  }
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
    //if (newState == LOW) switchCfan(newState);
    //  Serial.println(millis() - swTime);
  }
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
void usageStat() {
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

void printTentEnv(byte env) {
  char tempStr[6], humidStr[7];
  if (env == 255) { //print environment
    dtostrf(envTemp, 5, 2, tempStr);
    dtostrf(envRh, 6, 2, humidStr);
    sprintf_P(tmpLog, PSTR("EO|%s|%s"), tempStr, humidStr);  addLog(tmpLog);
  } else {
    loadTentEnv();
    dtostrf(workingTemp, 5, 2, tempStr);
    dtostrf(workingRh, 6, 2, humidStr);
    sprintf_P(tmpLog, PSTR("EI|%s|%s|%d"),  tempStr, humidStr, workingCO2);  addLog(tmpLog);
  }
}
void loadTentEnv() {
  workingTemp = 0;
  workingRh = 0;
  workingCO2 = 0;
  byte count = 0;
  for (byte i = 0; i < RADIO_COUNT; i++) {
    if (inTime - lastRevTime[i] < 60000
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
  if (inTime - lastRevTime[CO2_SENSOR_ID - 1] < 60000
      && sensorCO2 > 0) {
    workingCO2 = sensorCO2;
  } else {
    workingCO2 = -1;
  }
}

void revData() {
  if ( radio.available()) {
    // Variable for the received timestamp
    while (radio.available()) {
      // While there is data ready
      radio.read( &myData, sizeof(myData) );             // Get the payload
    }

    Serial.print(F("Received: "));
    Serial.print(myData.id); Serial.print(F(" | "));
    Serial.print(myData.type); Serial.print(F(" | "));
    Serial.print(myData.value1);
    Serial.print(F(" | "));
    Serial.println(myData.value2);

    int sensorId = myData.id;
    if (myData.type == 1) {
      sensorTemp[sensorId - 1] = myData.value1;
      sensorRh[sensorId - 1] = myData.value2;
    } else if (myData.type == 2) {
      sensorCO2 = (int) myData.value1;
    }
    lastRevTime[sensorId - 1] = inTime;

    //    radio.stopListening();
    //    radio.write(&myData, sizeof(myData) );             // Send the final one back.
    //    radio.startListening();                              // Now, resume listening so we catch the next packets.

    //    if (! rt) {
    //      Serial.print(F("failed :"));
    //      Serial.println(rt);
    //    }
  }
#ifdef DEBUG
  if (myReceiver.available()) {

    int value = myReceiver.getReceivedValue();

    if (value == 0) {
      Serial.print("Unknown encoding");
    } else {
      Serial.print("Received ");
      Serial.print( myReceiver.getReceivedValue() );
      Serial.print(" / ");
      Serial.print( myReceiver.getReceivedBitlength() );
      Serial.print("bit ");
      Serial.print("Protocol: ");
      Serial.println( myReceiver.getReceivedProtocol() );
    }

    myReceiver.resetAvailable();
  }
#endif
}

void readEnv() {
  int err = SimpleDHTErrSuccess;
  int count = 0;
  error = error & ~E_SENSOR;
  if ((err = dht22.read2(DHT_PIN, &envTemp, &envRh, NULL)) != SimpleDHTErrSuccess) {
    envTemp = rtc.getTemp();
    Serial.print(F("E]DHT")); Serial.println(err);
    //    delay(500);
    //    if (count > 3) {
    //      envTemp = envRh = -1;
    //      error = error | E_SENSOR;
    //      break;
    //    }
  }
  envRh = 70; //WROKAROUND for inaccurate DHT22
}
void btnDelay() {
  unsigned long holdTime = millis();
  while (digitalRead(SW_PIN) == LOW && millis() - holdTime < 60000) {
    wdt_reset();
    delay(100); // hold control until release
  }
}
