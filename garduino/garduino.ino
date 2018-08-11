// 1.5 (2017-12-21)
// tent control with lighting (scn display, sdcard conf)
// heat by fan cirulation
// use DHT0,1 for tent 0
//log file by [date][month].lg
//2.0 (2018-1-3)
//Add phased control by sd card parameter
// mode updated from enable/disable to off|spawn running|fruiting
//reduce ventilation interval by 1.5 on temperature low
//2.1 (2018-01-15)
//Add hydration period
//2.2 (2018-01-22)
// reduce to single tent
// mega, co2 sensor, MH support flag
// hydration changed to periodic style
// enable functioning without sensor
// allocated senor 0 for external env.
//2.3(2018-2-11)
//interchange between DHT & SHT sensor
//heater option
//register and joystick orientation swap flag
//led indicator for sensor/sd error
//refined DHT sensing frequency (randomly update 1 sensor per loop)
//2.4 (2018-02-20)
//add water sensor
//2.6
//change program selection priority
//2.8
//millis overflow logic

//#define MEGA
#define MH_LCD
//#define BIG_LCD
//#define CO2_SENSOR
//#define COOLER
//#define PUMP
#define DISABLE_JS

#define SHT_SENSOR 1
#define ENV_SENSOR 0
#define VERSION "v2.9"

#include <SimpleDHT.h> //self-tailored made DHT22 lib (removed DHT11)
//#define DEBUG 0
#include <Wire.h>
#include <DS3231.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h> // F Malpartida's NewLiquidCrystal library
#include <SD.h>
#include <avr/wdt.h>

#include "pin.h"

#ifdef SHT_SENSOR
#include "SHTSensor.h"
#endif

//LCD
#ifdef MH_LCD
#define I2C_ADDR 0x27 // Define I2C Address for controller
#else
#define I2C_ADDR 0x3F
#endif
#define REG_SIZE 8 //digital pin
//DHT
#define DHTTYPE DHT22 // DHT22  (AM2302)
#define TENT_SENSORS_NUM 2
//LCD
#define BACKLIGHT 3
#ifdef BIG_LCD
#define LCD_SIZE_X 20
#define LCD_SIZE_Y 4
#else
#define LCD_SIZE_X 16
#define LCD_SIZE_Y 2
#endif
//SD card
#define CONF_FILE "CONF.TXT"
//Joystick
#define JS_TRIGGER 200

//flag for error led
#define OK B00000000
#define E_SENSOR B00000001
#define E_SD B00000010

//Operation constants
#define LOOP_TIME 1000 //the min. duration of a loop
#define SPLIT '|'
#define MAX_RUNTIME 300000 //max. time(ms) of program execution
#define REACT_TIME 500 //reaction time of joystick action
#define GOAL_COUNT 10  //reading before advance step in program
#define SUMMARY_PRINT_CYCLE 600 //loop interval to print environment
#define LIGHT_CYCLE 15 //idle loop to turn off LCD backlight
#define SAMPLING_CYCLE_TIME 300000
#define SAMPLE_TIME 2000

//System variables
const PROGMEM char STR_CHG_MODE[] = "CM|%d";
const PROGMEM char STR_STS[] = "ST|%d|%d|%d|%d|%d|%d|%d|%d";
const PROGMEM char STR_CONF[] = "M%d|H:%d-%d-%d|T:%d-%d-%d|V:%d,%d|L:%d-%d|W:%d,%d";
int jsXRestPoint, jsYRestPoint;
unsigned int loopCount = 0;
unsigned long inTime = 0;
unsigned long previousMillis = 0;
static char tmpLog[80];
byte lcdCountdown = 0;
File logFile;
static char logFilename[] = "xxxx.LOG\0";
byte error = OK;
boolean skipSd = false;
unsigned long offset = 0;
long sampleTime = 0;
boolean doSample = true;

//Clock
DS3231  rtc(SDA, SCL);
Time curTime;

//Register
byte relayStatus = B11111111;
byte cfan = 6;
byte mist = 2;
byte heat = 1;
byte cooler = 4;
byte fan = 7;
byte light = 0;
byte pump = 3;

//Sensors
const byte SENSORS_PIN[TENT_SENSORS_NUM] = {DHT_ENV_PIN, 0};
SimpleDHT22 dht22;
//const byte sensor[TENT_SENSORS_NUM] = {1,1};
float workingTemp, sensorTemp[TENT_SENSORS_NUM];
float workingRh, sensorRh[TENT_SENSORS_NUM];

#ifdef SHT_SENSOR
SHTSensor sht;
#endif

//Display variables
LiquidCrystal_I2C lcd(I2C_ADDR, BACKLIGHT, POSITIVE);
#define SCN_NUM 7
#define SCN_ID_STAT 0
#define SCN_ID_ENV 1
#define SCN_ID_RELAY 2
#define SCN_ID_MODE 3
#define SCN_ID_STS 4
#define SCN_ID_CONF 5
#define SCN_ID_SYS 6
byte scn = 0; //display 0:time, 1:temp, 2:Relay status, 3: operation mode, 4: tent stat, 6: system opr, 5: tent conf
byte frame = 0;
byte scn_mode = 0;
byte x = 0, y = 0;
enum PROG {
  IDLE, VENT, WET, RH_HIGH, RH_LOW, TEMP_HIGH
};
#define TENT_MODE_COUNT 3
#define TENT_MODE_OFF 0
#define TENT_MODE_INCUBATION 1
#define TENT_MODE_FRUIT 2

//Tent program
byte confirmCount = 0;
byte tentMode = 0;
PROG tentProg = PROG::IDLE;
byte tentStep = 0;
unsigned long tentProgTime = 0;
byte goalCount = 0;
unsigned long tentLastFanTime = 0;
unsigned long tentLastWetTime = 0;
unsigned long heaterST = 0;

//Statistic record
unsigned long lastSurvayTime = 0;
unsigned int ttlOprMin[REG_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};
byte ttlOprSec[REG_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};

//Tent parameters (define in conf. file (SD Card))
byte humidHi = 98;
byte humidMid = 90;
byte humidLo = 87;
byte tempHi = 28;
byte tempMid = 25;
byte tempLo = 15;
byte ventInv = 10;
byte ventDur = 3;
byte lightStart = 0;
byte lightEnd = 0;
byte wetInv = 6;
byte wetDur = 5;

void timeOverflowReset() {
  tentProgTime = 0;
  tentLastFanTime = 0;
  tentLastWetTime = 0;
  heaterST = 0;
  lastSurvayTime = 0;
  sampleTime = 0;
}

void setup() {
  // Setup Serial connection
  Serial.begin(9600);
#ifdef MEGA
  Serial1.begin(9600);
#endif
  //watchdog
  wdt_enable(WDTO_8S);
  inTime = millis();

  // Initialize the rtc object
  rtc.begin();
  curTime = rtc.getTime();
  // The following lines can be uncommented to set the date and time
  //rtc.setDOW(FRIDAY);     // Set Day-of-Week to SUNDAY
  //rtc.setTime(16, 41, 30);     // Set the time to 12:00:00 (24hr format)
  //rtc.setDate(23, 2, 2018);   // Set the date to January 1st, 2014

  lcd.begin (LCD_SIZE_X, LCD_SIZE_Y); // initialize the lcd
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("Hi"));
  lcd.setCursor(0, 1);
  lcd.print(VERSION);
  lcdCountdown = LIGHT_CYCLE;

  //Joystick
  pinMode(SW_PIN, INPUT);
  digitalWrite(SW_PIN, HIGH);

  //Reg
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);

  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);

#ifdef CO2_SENSOR
  mhz19.begin(MHZ19B_RX, MHZ19B_TX);
  mhz19.setAutoCalibration(false);
#endif

#ifdef MEGA
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
#endif

  initSd();
  openLog();

#ifdef SHT_SENSOR
  sht.init();
#endif

  registerReset();

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

  inTime = millis();
  curTime = rtc.getTime();
  if (previousMillis > inTime) {
    timeOverflowReset();
  }
  previousMillis = inTime;
  doSample = ( inTime - sampleTime > SAMPLE_TIME || sampleTime == 0);
  //vvvvvvvvvvvvvvvvStart loop logicvvvvvvvvvvvvvvvvvvvvvvv//
  openLog();
  //================Read DHT Sensors============================//
  if (doSample) {
    updateSensor();
    sampleTime = inTime;
  }

  //================Serial Ouput=================================//
  //================Process Serial Control======================//
#ifdef MEGA
  processSerial();
#endif
  //===============Process  Control============================//

  readCtl();
  if (scn_mode == 1) ctlSettingScn();
  refreshScn();
  if (lcdCountdown <= 0) {
    lcd.noBacklight();
    scn_mode = 0; //exit edit mode
  } else {
    lcdCountdown--;
  }
  //===============Process tent operation======================//
  if (tentMode > 0) { //skip disabled tent
    loadTentEnv();
    autoLighting();
    autoCfan();
#ifdef HEATER
    autoHeater();
#endif
    ////vvvvvvvvvvvvvvvv Start Enabled tent logic  vvvvvvvvvvvvvv//
    if (tentProg == PROG::IDLE) {
      if ((inTime - offset) % (SAMPLING_CYCLE_TIME) > 30000 && (inTime - offset) % (SAMPLING_CYCLE_TIME) < 60000) selectProgram(); // tent is idle
    }
    else if (tentStep > 0 || confirmProgram())  executeProgram();
    ////^^^^^^^^^^^^^^^^ End tent logic      ^^^^^^^^^^^^^^^^^^^^//
  }
  //============Gather statistic=================//
  usageStat();
  //=============Logging=======================//
  if (loopCount % SUMMARY_PRINT_CYCLE == 0) {
    sprintf_P(tmpLog, STR_STS,
              ttlOprMin[0], ttlOprMin[1], ttlOprMin[2], ttlOprMin[3],
              ttlOprMin[4], ttlOprMin[5], ttlOprMin[6], ttlOprMin[7]); addLog(tmpLog);
    printTentEnv(255);
    printTentEnv(0);
  }
  //^^^^^^^^^^^^^^^^^^Finish Loop core logic^^^^^^^^^^^^^^^^^^^//
  closeLog();
  //=====================Extra error indicator on MEGA=========//
#ifdef MEGA
  if (error == 0) {
    digitalWrite(BLUE_PIN, HIGH);
  } else {
    digitalWrite(BLUE_PIN, LOW);
  }
  digitalWrite(RED_PIN, bitRead(error, 0));
  digitalWrite(GREEN_PIN, bitRead(error, 1));
#endif
  //=====================Finish Loop============================//
  loopCount++;
#ifdef DEBUG
  Serial.print(F("Loop end in ")); Serial.println((millis() - inTime));
#endif
  if (millis() - inTime < 1000)  delay (LOOP_TIME - millis() + inTime);
}

#ifdef MEGA
void processSerial() {
  if (Serial.available() > 0) {
    char inChar = Serial.read();
    Serial.println(inChar);
    int bitToSet;
    switch (inChar) {
      case 'r':
        registerReset();
        break;
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
        bitToSet = inChar - 48;
        if (bitToSet >= 0 && bitToSet < 8)
          registerWrite(bitToSet, !bitRead(relayStatus, bitToSet));
        break;
      case 'a':
        printTentEnv(255);
        printTentEnv(0);
        break;
    }
  }
}
#endif

void readCtl() {
  //switch screen on display mode only
  if (digitalRead(SW_PIN) == HIGH) { //navigation
    if (scn_mode == 0) {
      while (jsMove()) {
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
#ifdef DISABLE_JS
  return false;
#else
  return (analogRead(X_PIN) - jsXRestPoint < -JS_TRIGGER || analogRead(X_PIN) - jsXRestPoint > JS_TRIGGER
          || analogRead(Y_PIN) - jsYRestPoint < -JS_TRIGGER || analogRead(Y_PIN) - jsYRestPoint > JS_TRIGGER);
#endif
}
void ctlSettingScn() {
  //=============Cursor movement===============//
  while (jsMove()) {
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
    if (digitalRead(SW_PIN) == LOW && x < 8 && y == 1) {
      registerWrite(x, !bitRead(relayStatus, x));
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
  if (scn == SCN_ID_STAT) {
    lcd.setCursor(0, 0); lcd.print(rtc.getTimeStr());
#ifndef BIG_LCD
    lcd.print('|'); lcd.print(tentMode); lcd.print('|'); lcd.print(tentProg); lcd.print('|'); lcd.print(tentStep);
#endif
    lcd.setCursor(0, 1);
    // lcd.print(rtc.getDateStr());
    lcd.print(workingTemp); lcd.print(F("C|")); lcd.print(workingRh); lcd.print(F("% "));
#ifdef BIG_LCD
    lcd.setCursor(0, 2); lcd.print(F("Mode:"));
    switch (tentMode) {
      case 0:
        lcd.print('X');
        break;
      default:
        lcd.print(tentMode);
        break;
    }
    lcd.setCursor(8, 2); lcd.print(F("STAT:"));
    switch (tentProg) {
      case IDLE:
        lcd.print("IDLE");
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
    }
#endif
  } else if (scn == SCN_ID_ENV) {
    displayEnv();
  } else if (scn == SCN_ID_RELAY) {
    displayRelayStatus();
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
  } else if (scn == SCN_ID_STS) {
    displayTentStat();
  } else if (scn == SCN_ID_SYS) {
    lcd.setCursor(1, 0); lcd.print(skipSd ? 'X' : 'Y');
    lcd.setCursor(5, 0);  lcd.print(F("SD"));
  } else if (scn == SCN_ID_CONF) {
    displayTentConf();
  }
}
void lcdOn() {
  if (lcdCountdown <= 0) lcd.backlight();
  lcdCountdown = LIGHT_CYCLE;
}
void displayRelayStatus() {
  lcd.setCursor(0, 0);
  lcd.print(F("Relay:"));
  lcd.setCursor(0, 1);
  for (int i = 0; i <= 7; i++) {
    lcd.print(!bitRead(relayStatus, i));
  }
}
void displayEnv() {
  frame = frame % TENT_SENSORS_NUM;
  //updateSensor(frame);
  lcd.setCursor(0, 0);
  lcd.print(frame); lcd.print(F(".T:")); lcd.print(sensorTemp[frame]); lcd.print(F("C "));
  lcd.setCursor(0, 1);
  lcd.print(F("H:")); lcd.print(sensorRh[frame]); lcd.print(F("% "));
}
void displayTentStat() {
  frame = frame % 2;
  switch (frame) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print(F("CF:"));
      lcd.print(ttlOprMin[cfan]); lcd.print('m'); lcd.print(ttlOprSec[cfan]); lcd.print('s');
      lcd.setCursor(0, 1);
      lcd.print(F("VF:"));
      lcd.print(ttlOprMin[fan]); lcd.print('m'); lcd.print(ttlOprSec[fan]); lcd.print('s');
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print(F("Mt:"));
      lcd.print(ttlOprMin[mist]); lcd.print('m'); lcd.print(ttlOprSec[mist]); lcd.print('s');
      lcd.setCursor(0, 1);
      lcd.print(F("Ht:"));
      lcd.print(ttlOprMin[heat]); lcd.print('m'); lcd.print(ttlOprSec[heat]); lcd.print('s');
      break;
  }
}
void displayTentConf() {
  lcd.setCursor(0, 0);
  switch (frame % 5) {
    case 0:
      lcd.print(F("T[H|M|L]"));
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
      lcd.print(F("LED[S|E]"));
      lcd.setCursor(0, 1);
      lcd.print(lightStart); lcd.print(SPLIT); lcd.print(lightEnd);
      break;
    case 4:
      lcd.print(F("Wet[I|D]"));
      lcd.setCursor(0, 1);
      lcd.print(wetInv); lcd.print(F("h|"));  lcd.print(wetDur); lcd.print('m');
      break;
  }
}
byte changeValue(byte value) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print('2');
  lcd.setCursor(0, 1);
  lcd.print(value);
  btnDelay();
  while (digitalRead(SW_PIN) != LOW) {
    wdt_reset();
    if ((analogRead(Y_PIN) - jsYRestPoint) < -JS_TRIGGER)  value--;
    else if ((analogRead(Y_PIN) - jsYRestPoint) > JS_TRIGGER)  value++;
    lcd.setCursor(0, 1); lcd.print(value); lcd.print("   "); lcd.setCursor(0, 1);
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
//================================Register Control===================================//
void registerReset() {
  relayStatus = B11111111;
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, relayStatus);
  digitalWrite(LATCH_PIN, HIGH);
#ifdef DEBUG
  Serial.print(F("Reset switches:"));  Serial.println(relayStatus, BIN);
#endif
}

void registerWrite(int whichPin, int whichState) {
  digitalWrite(LATCH_PIN, LOW);
  bitWrite(relayStatus, whichPin, whichState);
#ifdef REVERSE_REG
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, relayStatus);
#else
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, relayStatus);
#endif
  digitalWrite(LATCH_PIN, HIGH);
#ifdef DEBUG
  Serial.print(F("Updated switches:"));  Serial.println(relayStatus, BIN);
#endif
}

//================================Select program====================================//
PROG suggestProgram() {
  //Select Program
  // 6. hydration
  if (inTime - tentLastWetTime > wetInv * 60 * 60000) {
    return PROG::WET;
  }
  //5. Regular ventilation
  else if (inTime - tentLastFanTime > ventInv * 60000) {
    if (workingTemp < 0
        || workingTemp > tempLo
        || inTime - tentLastFanTime < ventInv * 60000 * 1.5) {
      return PROG::VENT;
    }
  }
  if (workingTemp > 0) {
    if (workingTemp > tempHi) {
      //4. High temperature
      return PROG::TEMP_HIGH;
    } else if (workingRh < humidLo) {
      //1. Low humidity
      return PROG::RH_LOW;
    } else if (workingRh > humidHi) {
      //2. High humidity
      return PROG::RH_HIGH;
    }
  }
  return PROG::IDLE;
}
void selectProgram() {
  tentProg = suggestProgram();
  if (tentProg != PROG::IDLE) {
    tentStep = 0;
    confirmCount = 0;
    sprintf_P(tmpLog, PSTR("PS|%d"), tentProg); addLog(tmpLog);
    printTentEnv(0);
  }
}
boolean confirmProgram() {
  byte prog = suggestProgram();
  (tentProg == prog) ?  confirmCount++ : confirmCount--;
  if (confirmCount > GOAL_COUNT || prog == PROG::VENT || prog == PROG::WET) {
    sprintf_P(tmpLog, PSTR("PY|%d"), tentProg); addLog(tmpLog);
    printTentEnv(0);
    return true;
  } else if (confirmCount == 0 ) {
    sprintf_P(tmpLog, PSTR("PX|%d"), tentProg); addLog(tmpLog);
    printTentEnv(0);
    tentProg = PROG::IDLE;
  }
  return false;
}
//============================Execute program=======================================//
void executeProgram() {
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
  }
}
void runTempHi() {
  switchHeater(HIGH); //ensure heater is off
  if (tentStep == 0) {
    tentStep++;
    tentProgTime = inTime;
    logStepChg();
    goalCount = 0;
    switchVFan(LOW);
    switchMister(LOW);
  } else if (tentStep == 1) {
    if (workingRh < 0 || workingRh < humidLo) {
      switchVFan(HIGH);
    } else if (workingRh > humidMid) {
      switchVFan(LOW);
    }
    if (workingRh >= 100) {
      switchMister(HIGH); //Prevent dripping, no extra evaperate cooling after satruation
    } else if (workingRh < 99) {
      switchMister(LOW);
    }
    (workingTemp < 0 || workingTemp <= tempMid) ? goalCount++ : goalCount = 0;
    if (goalCount > GOAL_COUNT || inTime - tentProgTime > MAX_RUNTIME) progEnd();
  }
}
void runHumidHi() {
  //P1. start fan until humidity < humidMid
  if (tentStep == 0) {
    tentStep++;
    tentProgTime = inTime;
    logStepChg();
    goalCount = 0;
    switchVFan(LOW);
  } else if (tentStep == 1) {
    ((inTime - tentProgTime) % 60000 < 30000) ? switchVFan(LOW) : switchVFan(HIGH);
    (workingRh < 0 || workingRh < humidMid) ? goalCount++ : goalCount = 0;
    if (goalCount > GOAL_COUNT || inTime - tentProgTime > MAX_RUNTIME) progEnd();
  }
}
void runHumidLo() {
  //P1. start mister until workingRh > humidMid
  if (tentStep == 0) {
    tentStep++;
    tentProgTime = inTime;
    logStepChg();
    goalCount = 0;
    switchMister(LOW);
  } else if (tentStep == 1) {
    ((inTime - tentProgTime) % 60000 < 30000) ?  switchMister(LOW) : switchMister(HIGH);
    (workingRh < 0 || workingRh > humidMid) ?  goalCount++ : goalCount = 0;
    if (goalCount > GOAL_COUNT || inTime - tentProgTime > MAX_RUNTIME) progEnd();
  }
}
void runRegVent() {
  //1. start fan and fogger until defined duration
  if (tentStep == 0) {
    tentStep++;
    tentProgTime = inTime;
    logStepChg();
    switchVFan(LOW);
  } else if (tentStep == 1) {
    if (workingRh < 0 || workingRh < humidLo || workingRh < 0) {
      switchMister(LOW);
    } else if (workingRh > 0 && workingRh > humidHi) {
      switchMister(HIGH);
    }
    if (inTime - tentProgTime > ventDur * 60000) {
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
  } else if (tentStep == 1) {
    if (inTime - tentProgTime > wetDur * 60000) {
      tentStep++;
      logStepChg();
      switchMister(HIGH);
    }
  } else if (tentStep == 2) {
    if (inTime - tentProgTime > (wetDur * 60000) + 180000) {//give 3 mins for mist to condensate
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
    switchCfan(HIGH);
    switchHeater(HIGH);
  }
  tentProg = PROG::IDLE;
  tentStep = 0;
  tentProgTime = 0;
  goalCount = 0;
  offset = inTime % SAMPLING_CYCLE_TIME;
}
//===================================Lighting=====================================//
void autoLighting() {
  if (tentMode == 0) {
    switchLight(HIGH);
    return;
  }
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
void autoCfan() {
  if (tentProg == PROG::IDLE) {
    ((inTime - offset) % SAMPLING_CYCLE_TIME < 60000) ? switchCfan(LOW) : switchCfan(HIGH); //sampling cycle with offset from endProg
  } else {
    (tentMode == 0) ? switchCfan(HIGH) : switchCfan(LOW);
  }
}
void autoHeater() {
  //  if (tentMode == 0 || workingTemp < 0) {
  //    switchHeater(HIGH);
  //    return;
  //  } else if (workingTemp < tempLo) {
  //    if (heaterST == 0) { //Was off
  //      heaterST = inTime;
  //      sprintf_P(tmpLog, PSTR("HO")); addLog(tmpLog);
  //    }
  //  } else if (workingTemp >= tempMid) {
  //    if (heaterST > 0) { //Was off
  //      sprintf_P(tmpLog, PSTR("HX")); addLog(tmpLog);
  //    }
  //    heaterST = 0;
  //  }
  //  if (heaterST > 0) {
  //    if (heaterST > inTime) { // Start time in future (resting)
  //      switchHeater(HIGH);
  //    } else if (inTime - heaterST < 15 * 60000) { //In range operation time (15mins)
  //      switchHeater(LOW);
  //    } else { //force to heater to rest for 3mins
  //      switchHeater(HIGH);
  //      heaterST = inTime + (3 * 60000);
  //      sprintf_P(tmpLog, PSTR("HRT")); addLog(tmpLog);
  //    }
  //  } else {
  //    switchHeater(HIGH);
  //  }
}

//========================================SD Logger============================================//
void initSd() {
#ifdef DEBUG
  Serial.print(F("Init SD.."));
#endif
  // see if the card is present and can be initialized:
  if (!SD.begin(CS_Pin)) {
    Serial.println(F("XX"));
    // don't do anything more:
    return;
  }
#ifdef DEBUG
  Serial.println(F("OK"));
#endif
}
//void readLog() {
//  File inFile = SD.open(logFilename);
//  if (inFile) {
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
  boolean result = false;
#ifdef DEBUG
  Serial.print(F("Scan conf for:"));  Serial.println(inMode);
#endif
  File confFile = SD.open(CONF_FILE, FILE_READ);
  if (confFile) {
    //sprintf_P(tmpLog, PSTR("CM|%d"), inMode); addLog(tmpLog);
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
#ifdef DEBUG
      Serial.print(F("Content for:")); Serial.println(mode);
#endif
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
    Serial.print(F("E]R.conf"));
    result = false;
    error = error | E_SD;
  }
  confFile.close();
  return result ? 0 : 1;
}

//=================================Misc.============================================//
void openLog() {
  if (skipSd) return;
  logFile = SD.open(logFilename, FILE_WRITE);
}
void closeLog() {
  if (skipSd) return;
  logFile.close();
}
void addLog(char *msg) {
  //Serial.print(rtc.getTimeStr()); Serial.print(SPLIT);
  Serial.println(msg);
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
    //logFile = SD.open(logFilename, FILE_WRITE);
    if (!logFile
        || !logFile.print(rtc.getDateStr())
         || !logFile.print(SPLIT)
         || !logFile.print(rtc.getTimeStr())
        || !logFile.print(SPLIT)
        || !logFile.println(msg)) {
#ifdef DEBUG
      Serial.println(F("E]W.log"));
#endif
      error = error | E_SD;
    }
    msg[0] = 0;
    //logFile.close();
  }
}

void switchMister(boolean newState) {
  //if (bitRead(relayStatus, mist) == newState) return;
  registerWrite(mist, newState);
  if (newState == LOW) switchCfan(newState);
}
void switchVFan(boolean newState) {
  //if (bitRead(relayStatus, fan) == newState) return;
  registerWrite(fan, newState);
}
void switchCfan(boolean newState) {
  //if (bitRead(relayStatus, mist) == newState) return;
  registerWrite(cfan, newState);
}
void switchLight(boolean newState) {
  //if (bitRead(relayStatus, light) == newState) return;
  registerWrite(light, newState);
}
/* with protection logic to prevent long period heating */
void switchHeater(boolean newState) {
  registerWrite(heat, newState);
}
#ifdef COOLER
void switchCooler(boolean newState) {
  registerWrite(cooler, newState);
}
#endif
#ifdef PUMP
void switchPump(boolean newState) {
  registerWrite(pump, newState);
}
#endif
void usageStat() {
  if (inTime - lastSurvayTime > 10000) {
    int timePassed = (inTime - lastSurvayTime) / 1000;
    for (int i = 0; i < REG_SIZE; i++) {
      if (bitRead(relayStatus, i) == LOW) ttlOprSec[i] += timePassed;
      ttlOprMin[i] += ttlOprSec[i] / 60;
      ttlOprSec[i] = ttlOprSec[i] % 60;
    }
    lastSurvayTime = inTime;
  }
}

void btnDelay() {
  while (digitalRead(SW_PIN) == LOW) delay(100); // hold control until release
}

void printTentEnv(byte env) {
  char tempStr[6], humidStr[7];
  if (env == 255) { //print environment
    dtostrf(sensorTemp[0], 5, 2, tempStr);
    dtostrf(sensorRh[0], 6, 2, humidStr);
    sprintf_P(tmpLog, PSTR("EV|%s|%s"), tempStr, humidStr);  addLog(tmpLog);
  } else {
    loadTentEnv();
    dtostrf(workingTemp, 5, 2, tempStr);
    dtostrf(workingRh, 6, 2, humidStr);
    sprintf_P(tmpLog, PSTR("EN|%s|%s"),  tempStr, humidStr);  addLog(tmpLog);
  }
}
void loadTentEnv() {
  workingTemp = 0;
  workingRh = 0;
  byte count = 0;
  for (byte i = 0; i < TENT_SENSORS_NUM; i++) {
    if (i != ENV_SENSOR && sensorTemp[i] != -1 && sensorRh[i] != -1) {
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
}
void updateSensor() {
  byte sensor_id = 0;
  for (sensor_id; sensor_id < TENT_SENSORS_NUM; sensor_id++) {
#ifdef SHT_SENSOR
    if (sensor_id == SHT_SENSOR) continue; //replace the last sensor
#endif
    int err = SimpleDHTErrSuccess;
    int retry = 0;
    error = error & ~E_SENSOR;
    if ((err = dht22.read2(SENSORS_PIN[sensor_id], &sensorTemp[sensor_id], &sensorRh[sensor_id], NULL)) != SimpleDHTErrSuccess) {
      retry++;
      delay(200);
      while (retry > 3) {
        Serial.print(F("E]DHT")); Serial.print(sensor_id); Serial.print(SPLIT); Serial.println(err);
        sensorTemp[sensor_id] = sensorRh[sensor_id] = -1;
        error = error | E_SENSOR;
        break;
      }
    }
#ifdef DEBUG
    Serial.print(F("DHT")); Serial.print(sensor_id); Serial.print(SPLIT); Serial.print(sensorTemp[sensor_id]); Serial.print(SPLIT); Serial.println(sensorRh[sensor_id]);
#endif
  }
#ifdef SHT_SENSOR
  sensor_id = SHT_SENSOR;
  if (sht.readSample()) {
    sensorTemp[sensor_id] = sht.getTemperature();
    sensorRh[sensor_id] = sht.getHumidity();
  } else {
    Serial.println(F("E]SHT"));
    sensorTemp[sensor_id] = sensorRh[sensor_id] = -1;
    error = error | E_SENSOR;
  }
#endif
#ifdef CO2_SENSOR
  int co2ppm = mhz19.getPPM();
  Serial.print("uart co2: "); Serial.println(co2ppm);
#endif
}
