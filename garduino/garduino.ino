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

//#define MEGA
#define MH_LCD
//#define BIG_LCD
//#define CO2_SENSOR
#define SHT_SENSOR
#define VERSION "v2.7b"

#include <SimpleDHT.h> //self-tailored made DHT22 lib (removed DHT11)
//#define DEBUG 0
#include <Wire.h>
#include <DS3231.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h> // F Malpartida's NewLiquidCrystal library
#include <SD.h>

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
#define TENT_SENSORS_NUM 1
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
#define JS_MID 500
#define JS_TRIGGER 200

//flag for error led
#define OK B00000000
#define E_SENSOR B00000001
#define E_SD B00000010

//Operation constants
#define LOOP_TIME 1000 //the min. duration of a loop
#define SPLIT '|'
#define MAX_RUNTIME 300000 //max. time(ms) of program execution
#define REACT_TIME 750 //reaction time of joystick action
#define GOAL_COUNT 10  //reading before advance step in program
#define ENV_PRINT_CYCLE 600 //loop interval to print environment
#define STS_PRINT_CYCLE 600 //loop interval to print environment
#define STS_CYCLE 5 //loop interval to gather stat
#define LIGHT_CYCLE 15 //idle loop to turn off LCD backlight
#define SAMPLING_CYCLE_TIME 300000


//System variables
const PROGMEM char STR_STS[] = "SY|STS|%d|%d|%d|%d|%d|%d|%d|%d";
const PROGMEM char STR_CONF[] = "M%d|H:%d-%d-%d|T:%d-%d-%d|V:%d,%d|L:%d-%d|W:%d,%d";
unsigned int loopCount = 0;
unsigned long inTime;
static char tmpLog[80];
byte lcdCountdown = 0;
File logFile;
static char logFilename[] = "xxxx.LOG\0";
byte error = OK;
boolean skipSd = false;
unsigned long offset = 0;

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
const byte DHT22_PIN[TENT_SENSORS_NUM  + 1] = {DHT_ENV_PIN, DHT1_PIN}; //plus one env. sensor
SimpleDHT22 dht22;
const byte sensor[TENT_SENSORS_NUM] = {1};//plus one env. sensor
float workingTemp, sensorTemp[TENT_SENSORS_NUM + 1];
float workingRH, sensorRh[TENT_SENSORS_NUM + 1];

#ifdef SHT_SENSOR
SHTSensor sht;
#endif

//Display variables
LiquidCrystal_I2C lcd(I2C_ADDR, BACKLIGHT, POSITIVE);
#define SCN_NUM 7
#define SCN_ID_TIME 0
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
PROG tentProg = 0;
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

void setup() {
  inTime = millis();
  // Setup Serial connection
  Serial.begin(9600);
#ifdef MEGA
  Serial1.begin(9600);
#endif
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
  registerReset();
  tentLastWetTime = tentLastFanTime = millis();
  tentMode = 0;
#ifdef CO2_SENSOR
  mhz19.begin(MHZ19B_RX, MHZ19B_TX);
  mhz19.setAutoCalibration(false);
#endif
#ifdef SHT_SENSOR
  sht.init();
#endif
#ifdef MEGA
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
#endif
  //restore parameters from sd card
  initSd();
  if (millis() - inTime < 2000)  delay (2000 - millis() + inTime);
}
void loop() {
  inTime = millis();
  curTime = rtc.getTime();
  //================Read DHT Sensors============================//
  updateSensor();
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
    lcd.backlight();
  }
  //===============Process tent operation======================//
  if (tentMode > 0) { //skip disabled tent
    loadTentEnv();
    autoLighting();
    autoCfan();
    autoHeater();
    ////vvvvvvvvvvvvvvvv Start Enabled tent logic  vvvvvvvvvvvvvv//
    if (tentProg == PROG::IDLE) {
      if ((millis() - offset) % (SAMPLING_CYCLE_TIME) > 30000 && (millis() - offset) % (SAMPLING_CYCLE_TIME) < 60000) selectProgram(); // tent is idle
    }
    else if (tentStep > 0 || confirmProgram())  executeProgram();
    ////^^^^^^^^^^^^^^^^ End tent logic      ^^^^^^^^^^^^^^^^^^^^//
  }
  //============Gather statistic=================//
  usageStat();
  //=============Logging=======================//
  if (loopCount % ENV_PRINT_CYCLE == 0) {
    printTentEnv(255);
    printTentEnv(0);
  }
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
        lcdOn();
        if ((analogRead(X_PIN) - JS_MID)*JS_ORIENTATION < -JS_TRIGGER) scn--;
        else if ((analogRead(X_PIN) - JS_MID)*JS_ORIENTATION > JS_TRIGGER)  scn++;
        if ((analogRead(Y_PIN) - JS_MID) < -JS_TRIGGER)  frame--;
        else if ((analogRead(Y_PIN) - JS_MID) > JS_TRIGGER) frame++;
        if (scn < 0) scn = SCN_NUM - 1;
        scn = scn % SCN_NUM;
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
  return (analogRead(X_PIN) - JS_MID < -JS_TRIGGER || analogRead(X_PIN) - JS_MID > JS_TRIGGER
          || analogRead(Y_PIN) - JS_MID < -JS_TRIGGER || analogRead(Y_PIN) - JS_MID > JS_TRIGGER);
}
void ctlSettingScn() {
  //=============Cursor movement===============//
  while (jsMove()) {
    lcdOn();
    if ((analogRead(X_PIN) - JS_MID)*JS_ORIENTATION < -JS_TRIGGER) x--;
    else if ((analogRead(X_PIN) - JS_MID)*JS_ORIENTATION > JS_TRIGGER)  x++;
    if ((analogRead(Y_PIN) - JS_MID) < -JS_TRIGGER)  y--;
    else if ((analogRead(Y_PIN) - JS_MID) > JS_TRIGGER)  y++;
    if (x < 0)x = 0; x = x % LCD_SIZE_X;
    if (y < 0)y = 0; y = y % LCD_SIZE_Y;
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
      //if (x < 3 ) {
      int oldMode = tentMode;
      tentMode = (tentMode + 1) % TENT_MODE_COUNT;
      byte result = 0;
      if (tentMode != TENT_MODE_OFF) {
        result = loadConf(tentMode); //0:off, 1:spawn running, 2:fruiting
      }
      if (tentMode == TENT_MODE_OFF || result == 0) {
        sprintf_P(tmpLog, PSTR("CTL|%d"), tentMode); addLog(tmpLog);
        progEnd(); //reset tent operation
      } else {
        lcd.setCursor(0, 0); lcd.print(F("FAIL"));
        //tentMode = oldMode; //won't fail on zero(off mode)
        delay(1000);
      }
      refreshScn();
      btnDelay();// hold control until release
      //}
    }
  } else if (scn == SCN_ID_SYS) {
    if (digitalRead(SW_PIN) == LOW && x > 0 && y == 0) {
      if (y == 0) skipSd = true;
      //if (x == 15 && y == 1) printTentEnv(0); printTentEnv(1);
      scn_mode = 0;
      refreshScn();
      btnDelay();// hold control until release
    }
  } else if (scn == SCN_ID_CONF) {
    editTentConf();
  }
}

void refreshScn() {
  lcd.clear();
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
  if (scn == SCN_ID_TIME) {
    lcd.setCursor(0, 0); lcd.print(rtc.getTimeStr());
    lcd.setCursor(0, 1); lcd.print(rtc.getDateStr());
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
    lcd.setCursor(0, 0);
    lcd.print(F(" SD:"));
    lcd.setCursor(15, 0);
    lcd.print(skipSd ? 'X' : 'Y');
  } else if (scn == SCN_ID_CONF) {
    displayTentConf();
  }
}
void lcdOn() {
  lcdCountdown = 20;
  lcd.backlight();
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
  frame = frame % sizeof(DHT22_PIN);
  //updateSensor(frame);
  lcd.setCursor(0, 0);
  lcd.print(frame); lcd.print(F(".T:")); lcd.print(sensorTemp[frame]); lcd.print(F(" *C "));
  lcd.setCursor(0, 1);
  lcd.print(F("H:")); lcd.print(sensorRh[frame]); lcd.print(F(" RH% "));
}
void displayTentStat() {
  frame = frame % 2;
  switch (frame) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print(F("CFan:"));
      lcd.print(ttlOprMin[cfan]); lcd.print('m'); lcd.print(ttlOprSec[cfan]); lcd.print('s');
      lcd.setCursor(0, 1);
      lcd.print(F("VFan:"));
      lcd.print(ttlOprMin[fan]); lcd.print('m'); lcd.print(ttlOprSec[fan]); lcd.print('s');
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print(F("Mist:"));
      lcd.print(ttlOprMin[mist]); lcd.print('m'); lcd.print(ttlOprSec[mist]); lcd.print('s');
      lcd.setCursor(0, 1);
      lcd.print(F("Heat:"));
      lcd.print(ttlOprMin[heat]); lcd.print('m'); lcd.print(ttlOprSec[heat]); lcd.print('s');
      break;
  }
}
void displayTentConf() {
  if (frame < 0) frame = 0;
  frame = frame % 4;
  byte scn = frame % 4;
  lcd.setCursor(0, 0);
  switch (scn) {
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
      lcd.print(F("Light[S|E]"));
      lcd.setCursor(0, 1);
      lcd.print(lightStart); lcd.print(SPLIT); lcd.print(lightEnd); ;
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
    if ((analogRead(Y_PIN) - JS_MID) < -JS_TRIGGER)  value--;
    else if ((analogRead(Y_PIN) - JS_MID) > JS_TRIGGER)  value++;
    lcd.setCursor(0, 1);
    lcd.print(value);
    delay(300);
  }
  return value;
}
void editTentConf() {
  if (digitalRead(SW_PIN) == LOW && y == 1) {
    if (frame < 0) frame = 0;
    frame = frame % 4;
    byte scn = frame % 4;
    switch (scn) {
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
        if (x < 4 ) wetInv = changeValue(&wetInv);
        else if (x < 8) wetDur = changeValue(&wetDur);
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
  if (millis() - tentLastWetTime > wetInv * 60 * 60000) {
    return PROG::WET;
  }
  //5. Regular ventilation
  else if (millis() - tentLastFanTime > ventInv * 60000) {
    if (workingTemp < 0
        || workingTemp > tempLo
        || millis() - tentLastFanTime < ventInv * 60000 * 1.5) {
      return PROG::VENT;
    }
  }
  if (workingTemp > 0) {
    if (workingTemp > tempHi) {
      //4. High temperature
      return PROG::TEMP_HIGH;
    } else if (workingRH < humidLo) {
      //1. Low humidity
      return PROG::RH_LOW;
    } else if (workingRH > humidHi) {
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
    tentProg = 0;
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
    tentProgTime = millis();
    logStepChg();
    goalCount = 0;
    switchFan(LOW);
    switchMister(LOW);
  } else if (tentStep == 1) {
    if (workingRH < 0 || workingRH < humidLo) {
      switchFan(HIGH);
    } else if (workingRH > humidLo + 2) {
      switchFan(LOW);
    }
    switchMister(workingRH > 99.9 ? HIGH : LOW);
    (workingTemp < 0 || workingTemp <= tempMid) ? goalCount++ : goalCount = 0;
    if (goalCount > GOAL_COUNT || millis() - tentProgTime > MAX_RUNTIME) progEnd();
  }
}
void runHumidHi() {
  //P1. start fan until humidity < humidMid
  if (tentStep == 0) {
    tentStep++;
    tentProgTime = millis();
    logStepChg();
    goalCount = 0;
    switchFan(LOW);
  } else if (tentStep == 1) {
    ((millis() - tentProgTime) % 60000 < 30000) ? switchFan(LOW) : switchFan(HIGH);
    (workingRH < 0 || workingRH < humidMid) ? goalCount++ : goalCount = 0;
    if (goalCount > GOAL_COUNT || millis() - tentProgTime > MAX_RUNTIME) progEnd();
  }
}
void runHumidLo() {
  //P1. start mister until workingRH > humidMid
  if (tentStep == 0) {
    tentStep++;
    tentProgTime = millis();
    logStepChg();
    goalCount = 0;
    switchMister(LOW);
  } else if (tentStep == 1) {
    ((millis() - tentProgTime) % 60000 < 30000) ?  switchMister(LOW) : switchMister(HIGH);
    (workingRH < 0 || workingRH > humidMid) ?  goalCount++ : goalCount = 0;
    if (goalCount > GOAL_COUNT || millis() - tentProgTime > MAX_RUNTIME) progEnd();
  }
}
void runRegVent() {
  //1. start fan and fogger until defined duration
  if (tentStep == 0) {
    tentStep++;
    tentProgTime = millis();
    logStepChg();
    switchFan(LOW);
  } else if (tentStep == 1) {
    if (workingRH < 0 || workingRH < humidLo || workingRH < 0) {
      switchMister(LOW);
    } else if (workingRH > 0 && workingRH > humidHi) {
      switchMister(HIGH);
    }
    if (millis() - tentProgTime > ventDur * 60000) {
      tentLastFanTime = millis();
      progEnd();
    }
  }
}
void runHydration() {
  if (tentStep == 0) {
    tentStep++;
    tentProgTime = millis();
    logStepChg();
    switchMister(LOW);
  } else if (tentStep == 1) {
    if (millis() - tentProgTime > wetDur * 60000) {
      tentStep++;
      logStepChg();
      switchMister(HIGH);
    }
  } else if (tentStep == 2) {
    if (millis() - tentProgTime > (wetDur * 60000) + 60000 * 3) {//give 3 mins for mist to condensate
      tentLastWetTime = millis();
      progEnd();
    }
  }
}

void logStepChg() {
  sprintf_P(tmpLog, PSTR("PU|%d|%d|%dm"),  tentProg, tentStep, (millis() - tentProgTime) / 60000); addLog(tmpLog);
  printTentEnv(0);
}
void progEnd() {
  sprintf_P(tmpLog, PSTR("PF|%d|%dm"),  tentProg, (millis() - tentProgTime) / 60000); addLog(tmpLog);
  printTentEnv(0);
  switchMister(HIGH);
  switchFan(HIGH);
  if (tentMode == TENT_MODE_OFF) {
    switchLight(HIGH);
    switchCfan(HIGH);
    switchHeater(HIGH);
  }
  tentProg = PROG::IDLE;
  tentStep = 0;
  tentProgTime = 0;
  goalCount = 0;
  offset = millis() % SAMPLING_CYCLE_TIME;
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
    ((millis() - offset) % SAMPLING_CYCLE_TIME < 60000) ? switchCfan(LOW) : switchCfan(HIGH); //sampling cycle with offset from endProg
  } else {
    (tentMode == 0) ? switchCfan(HIGH) : switchCfan(LOW);
  }
}
void autoHeater() {
  if (tentMode == 0 || workingTemp < 0) {
    switchHeater(HIGH);
    return;
  } else if (workingTemp < tempLo) {
    if (heaterST == 0) { //Was off
      heaterST = millis();
      sprintf_P(tmpLog, PSTR("HO")); addLog(tmpLog);
    }
  } else if (workingTemp >= tempMid) {
    if (heaterST > 0) { //Was off
      sprintf_P(tmpLog, PSTR("HX")); addLog(tmpLog);
    }
    heaterST = 0;
  }
  if (heaterST > 0) {
    if (heaterST > millis()) { // Start time in future (resting)
      switchHeater(HIGH);
    } else if (millis() - heaterST < 15 * 60000) { //In range operation time (15mins)
      switchHeater(LOW);
    } else { //force to heater to rest for 3mins
      switchHeater(HIGH);
      heaterST = millis() + (3 * 60000);
      sprintf_P(tmpLog, PSTR("HRT")); addLog(tmpLog);
    }
  } else {
    switchHeater(HIGH);
  }
}

//========================================SD Logger============================================//
void initSd() {
  Serial.print(F("Init SD.."));
  // see if the card is present and can be initialized:
  if (!SD.begin(CS_Pin)) {
    Serial.println(F("XX"));
    // don't do anything more:
    return;
  }
  Serial.println(F("OK"));
}
void readLog() {
  File inFile = SD.open(logFilename);
  if (inFile) {
    while (inFile.available()) {
      Serial.write(inFile.read());
    }
    inFile.close();
  } else {
    Serial.print(F("E]R.log"));
    error = E_SD;
  }
}
//format:[TENT_ID(1)][TENT_MODE(1)][HUMID_HI(2)][HUMID_MI(2)][HUMID_LO(2)][TEMP_HI(2)][TEMP_MI(2)][TEMP_LO(2)][VENT_INV_MIN(2)][ventDur_MIN(2)][EOL]
byte loadConf(byte inMode) {
  boolean result = false;
#ifdef DEBUG
  Serial.print(F("Scan conf for:"));  Serial.println(inMode);
#endif
  File confFile = SD.open(CONF_FILE, FILE_READ);
  if (confFile) {
    sprintf_P(tmpLog, PSTR("SY|CONF|%s"), CONF_FILE); addLog(tmpLog);
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
    logFile = SD.open(logFilename, FILE_WRITE);
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
    logFile.close();
  }
}

void switchMister(boolean newState) {
  //if (bitRead(relayStatus, mist) == newState) return;
  registerWrite(mist, newState);
  if (newState == LOW) switchCfan(newState);
}
void switchFan(boolean newState) {
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
#ifdef MEGA
void switchCooler(boolean newState) {
  registerWrite(cooler, newState);
}
void switchPump(boolean newState) {
  registerWrite(pump, newState);
}
#endif
void usageStat() {
  if (loopCount % STS_CYCLE == 0) {
    long curTime = millis();
    int timePassed = (curTime - lastSurvayTime) / 1000;
    lastSurvayTime = curTime;
    for (int i = 0; i < REG_SIZE; i++) {
      if (bitRead(relayStatus, i) == LOW) ttlOprSec[i] += timePassed;
      ttlOprMin[i] += ttlOprSec[i] / 60;
      ttlOprSec[i] = ttlOprSec[i] % 60;
    }
  }
  if (loopCount % STS_PRINT_CYCLE == 0) { //dump to log every ~10mins (est. by cycles)
    sprintf_P(tmpLog, STR_STS,
              ttlOprMin[0], ttlOprMin[1], ttlOprMin[2], ttlOprMin[3],
              ttlOprMin[4], ttlOprMin[5], ttlOprMin[6], ttlOprMin[7]); addLog(tmpLog);
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
    dtostrf(workingRH, 6, 2, humidStr);
    sprintf_P(tmpLog, PSTR("EN|%s|%s"),  tempStr, humidStr);  addLog(tmpLog);
  }
}
void loadTentEnv() {
  workingTemp = 0;
  workingRH = 0;
  byte count = 0;
  for (byte i = 0; i < sizeof(sensor); i++) {
    if (sensorTemp[sensor[i]] != -1 && sensorRh[sensor[i]] != -1) {
      workingTemp += sensorTemp[sensor[i]];
      workingRH += sensorRh[sensor[i]];
      count++;
    }
  }
  if (count > 0) {
    workingTemp = workingTemp / count;
    workingRH = workingRH / count;
  } else {
    workingTemp = workingRH = -1;
  }
}
void updateSensor() {
  byte sensor_id = 0;
  //for (sensor_id; sensor_id < sizeof(DHT22_PIN); sensor_id++) {
#ifdef SHT_SENSOR
  sensor_id = millis() % (sizeof(DHT22_PIN) - 1); //replace the last sensor
#else
  sensor_id = millis() % sizeof(DHT22_PIN);
#endif
  int err = SimpleDHTErrSuccess;
  int count = 0;
  error = error & ~E_SENSOR;
  if ((err = dht22.read2(DHT22_PIN[sensor_id], &sensorTemp[sensor_id], &sensorRh[sensor_id], NULL)) != SimpleDHTErrSuccess) {
    //    count++;
    //    delay(500);
    //    if (count > 3) {
    Serial.print(F("E]DHT")); Serial.print(sensor_id); Serial.print(SPLIT); Serial.println(err);
    sensorTemp[sensor_id] = sensorRh[sensor_id] = -1;
    error = error | E_SENSOR;
    //      break;
    //    }
  }
#ifdef DEBUG
  Serial.print(F("DHT")); Serial.print(sensor_id); Serial.print(SPLIT); Serial.print(sensorTemp[sensor_id]); Serial.print(SPLIT); Serial.println(sensorRh[sensor_id]);
#endif
  //}
#ifdef SHT_SENSOR
  sensor_id = sizeof(DHT22_PIN) - 1;
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
