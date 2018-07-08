

#include "pin.h"

#include <Wire.h>
#include <DS3231.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h> // F Malpartida's NewLiquidCrystal library
//#include "SHTSensor.h"
#include <SPI.h>
#include <SdFat.h>
#include "RF24.h"
#include <RCSwitch.h>
#include <SimpleDHT.h> //self-tailored made DHT22 lib (removed DHT11)

#define DEBUG
#define MEGA
#define MH_LCD
#define BIG_LCD
#define VERSION "v2.7b"

#define I2C_ADDR 0x3F // Define I2C Address for controller
#define BACKLIGHT 3
#define LCD_SIZE_X 20
#define LCD_SIZE_Y 4
//SD card
#define CONF_FILE "CONF.TXT"
//Joystick
#define JS_MID 500
#define JS_TRIGGER 200

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

//Display variables
#define SCN_NUM 7
#define SCN_ID_TIME 0
#define SCN_ID_ENV 1
#define SCN_ID_RELAY 2
#define SCN_ID_MODE 3
#define SCN_ID_STS 4
#define SCN_ID_CONF 5
#define SCN_ID_SYS 6

//flag for error led
#define OK B00000000
#define E_SENSOR B00000001
#define E_SD B00000010

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(RF_CE_PIN, RF_CS_PIN);
RCSwitch mySwitch = RCSwitch();
//Clock
DS3231  rtc(SDA, SCL);
Time curTime;


const PROGMEM char STR_STS[] = "SY|STS|%d|%d|%d|%d|";
const PROGMEM char STR_CONF[] = "M%d|H:%d-%d-%d|T:%d-%d-%d|V:%d,%d|L:%d-%d|W:%d,%d";
unsigned int loopCount = 0;
unsigned long inTime;
static char tmpLog[80];
byte lcdCountdown = 0;
static char logFilename[] = "xxxx.LOG\0";
boolean skipSd = false;
unsigned long offset = 0;

//Display variables
LiquidCrystal_I2C lcd(I2C_ADDR, BACKLIGHT, POSITIVE);
byte scn = 0; //display 0:time, 1:temp, 2:Relay status, 3: operation mode, 4: tent stat, 6: system opr, 5: tent conf
byte frame = 0;
byte scn_mode = 0;
byte x = 0, y = 0;

//Tent program
enum PROG {
  IDLE, VENT, WET, RH_HIGH, RH_LOW, TEMP_HIGH
};
#define TENT_MODE_COUNT 3
#define TENT_MODE_OFF 0
#define TENT_MODE_INCUBATION 1
#define TENT_MODE_FRUIT 2
byte confirmCount = 0;
byte tentMode = 0;
PROG tentProg = 0;
byte tentStep = 0;
unsigned long tentProgTime = 0;
byte goalCount = 0;
unsigned long tentLastFanTime = 0;
unsigned long tentLastWetTime = 0;
unsigned long heaterST = 0;

//Switch Register
byte switchStatus = B11111111;
const uint8_t sysID = 0;
const uint8_t mistID = 1;
const uint8_t vFanID = 2;
const uint8_t lightID = 3;
const uint8_t cFanID = 4;
const uint8_t heatID = 5;
//Radio Switch Code
int vfan[2] = {17100, 17118};
int mist[2] = {17200, 17218};
//byte cooler = 5;
int cfan[2] = {17300, 17318};
int light[2] = {17400, 17418};
int heat[2] = {17500, 17518};
//Switch Statistic
unsigned long lastSurvayTime = 0;
uint8_t ttlOprMin[6] = {0, 0, 0, 0, 345, 0};
uint8_t ttlOprSec[6] = {0, 0, 0, 0, 0, 0};

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

//Radio
int radioNumber = 0;
#define RADIO_COUNT 3
byte addresses[RADIO_COUNT][6] = {"1Node", "2Node", "3Node"};
struct dataStruct {
  unsigned long _micros;
  int id;
  float value_temp;
  float value_rh;
} myData;

//SD card
/* CATALEX SD CARD READER SOFT-SPI PART */
// Here are the pins for the soft-spi-bus defined
// You can select every pin you want, just don't put them on an existing hardware SPI pin.
SdFatSoftSpi<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> sd;
SdFile inFile, confFile, logFile;

//Sensors
SimpleDHT22 dht22;
float envTemp, workingTemp, sensorTemp[RADIO_COUNT]={0,0,0};
float envRh, workingRh, sensorRh[RADIO_COUNT]={0,0,0};
long lastReceive[RADIO_COUNT]={0,0,0};

int sensorId = 0;
long simpleTime = 0;
boolean doSimple = true;


byte error = B00000000;


void setup() {
  inTime = millis();
  Serial.begin(9600);

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
  //  rtc.setDOW(FRIDAY);     // Set Day-of-Week to SUNDAY
  //  rtc.setTime(12, 46, 30);     // Set the time to 12:00:00 (24hr format)
  //  rtc.setDate(6, 7, 2018);   // Set the date to January 1st, 2014

  //Joystick
  pinMode(SW_PIN, INPUT);
  digitalWrite(SW_PIN, HIGH);

  //INIT SPI modules
  SPI.begin();
  //init radio
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_MAX);
  // Open a writing and reading pipe on each radio, with opposite addresses
  Serial.print("Radio ID: "); Serial.println((const char*)addresses[radioNumber]);
  radio.openWritingPipe(addresses[radioNumber]);
  myData.id = radioNumber;
  int j = 1;
  Serial.print("Listen ID: ");
  for (int i = 0; i < RADIO_COUNT; i++) {
    if (i != radioNumber) {
      Serial.print((const char*)addresses[i]);
      Serial.print(";");
      radio.openReadingPipe(j, addresses[i]);
      j++;
    }
  }
  initSd();
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  // Start the radio listening for data
  radio.startListening();
  mySwitch.enableTransmit(R_TRAN_PIN);

  //init data
  lcdCountdown = LIGHT_CYCLE;

  tentLastWetTime = tentLastFanTime = millis();
  tentMode = 0;
  //switch
  switchReset();
  if (millis() - inTime < 2000)  delay (2000 - millis() + inTime);
}

void loop() {
  error = 0;
  inTime = millis();
  curTime = rtc.getTime();
  //================Read DHT Sensors============================//
  revData();
  readEnv();
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
    //autoCfan();
    //autoHeater();
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
        if (scn >= SCN_NUM) scn = SCN_NUM - 1;
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
    if (x >= LCD_SIZE_X) x = LCD_SIZE_X - 1;
    if (y >= LCD_SIZE_Y) y = LCD_SIZE_Y - 1;
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
      bitWrite(switchStatus, sysID, tentMode == TENT_MODE_OFF ? 1 : 0);
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
    lcd.setCursor(0, 0); lcd.print(F("Date:")); lcd.print(rtc.getDateStr());
    lcd.setCursor(0, 1); lcd.print(F("Time:")); lcd.print(rtc.getTimeStr());
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
  for (int i = 0; i < RADIO_COUNT; i++) {
    lcd.setCursor(0, i + 1);
    lcd.print(i); lcd.print("|"); lcd.print((millis() - lastReceive[i]) / 1000); lcd.print("s|"); lcd.print(sensorTemp[i]); lcd.print("C"); lcd.print("|"); lcd.print(sensorRh[i]); lcd.print("%");
  }
}
void displayTentStat() {
  lcd.setCursor(0, 0);
  lcd.print(F("Sy:"));  lcd.print(ttlOprMin[0] / 60); lcd.print('h'); lcd.print(ttlOprMin[0] % 60); lcd.print('m'); //lcd.print(ttlOprSec[mistID]); lcd.print('s');
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
  if (frame < 0) frame = 0;
  frame = frame % 4;
  byte scn = frame % 4;
  lcd.setCursor(0, 0);
  switch (scn) {
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
    switchVFan(LOW);
    switchMister(LOW);
  } else if (tentStep == 1) {
    if (workingRh < 0 || workingRh < humidLo) {
      switchVFan(HIGH);
    } else if (workingRh > humidLo + 2) {
      switchVFan(LOW);
    }
    switchMister(workingRh > 99.9 ? HIGH : LOW);
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
    switchVFan(LOW);
  } else if (tentStep == 1) {
    ((millis() - tentProgTime) % 60000 < 30000) ? switchVFan(LOW) : switchVFan(HIGH);
    (workingRh < 0 || workingRh < humidMid) ? goalCount++ : goalCount = 0;
    if (goalCount > GOAL_COUNT || millis() - tentProgTime > MAX_RUNTIME) progEnd();
  }
}
void runHumidLo() {
  //P1. start mister until workingRh > humidMid
  if (tentStep == 0) {
    tentStep++;
    tentProgTime = millis();
    logStepChg();
    goalCount = 0;
    switchMister(LOW);
  } else if (tentStep == 1) {
    //((millis() - tentProgTime) % 60000 < 30000) ?  switchMister(LOW) : switchMister(HIGH);  //Alt switch for detection
    (workingRh < 0 || workingRh > humidMid) ?  goalCount++ : goalCount = 0;
    if (goalCount > GOAL_COUNT || millis() - tentProgTime > MAX_RUNTIME) progEnd();
  }
}
void runRegVent() {
  //1. start fan and fogger until defined duration
  if (tentStep == 0) {
    tentStep++;
    tentProgTime = millis();
    logStepChg();
    switchVFan(LOW);
  } else if (tentStep == 1) {
    if (workingRh < 0 || workingRh < humidLo || workingRh < 0) {
      switchMister(LOW);
    } else if (workingRh > 0 && workingRh > humidHi) {
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
  switchVFan(HIGH);
  if (tentMode == TENT_MODE_OFF) {
    switchLight(HIGH);
    //    switchCFan(HIGH);
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
//void autoCfan() {
//  if (tentProg == PROG::IDLE) {
//    ((millis() - offset) % SAMPLING_CYCLE_TIME < 60000) ? switchCfan(LOW) : switchCfan(HIGH); //sampling cycle with offset from endProg
//  } else {
//    (tentMode == 0) ? switchCfan(HIGH) : switchCfan(LOW);
//  }
//}
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
  if (!sd.begin(SD_CHIP_SELECT_PIN)) {
    Serial.println(F("XX"));
    // don't do anything more:
    return;
  }
  Serial.println(F("OK"));
}
void readLog() {
  if (inFile.open(logFilename, O_READ)) {
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
  if (confFile.open(CONF_FILE, O_READ)) {
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
    sd.errorPrint(F("E]R.conf"));
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
    if (!logFile.open(logFilename, O_WRITE | O_CREAT | O_AT_END) ) {
      sd.errorPrint(F("open failed"));
    }
    else if ( !logFile.print(rtc.getDateStr())
              || !logFile.print(SPLIT)
              || !logFile.print(rtc.getTimeStr())
              || !logFile.print(SPLIT)
              || !logFile.println(msg)) {
      sd.errorPrint(F("E]W.log"));
      error = error | E_SD;
    }
    msg[0] = 0;
    if (!logFile.sync() || logFile.getWriteError()) {
      sd.errorPrint("write error");
    }
    logFile.close();
  }
}
//===================================Switch=============================//
void switchMister(boolean newState) {
  if (bitRead(switchStatus, mistID) != newState) {
    sprintf_P(tmpLog, PSTR("SW|Mist|%d"), newState); addLog(tmpLog);
  }
  bitWrite(switchStatus, mistID, newState);
  mySwitch.send(mist[newState], 24);
  //if (newState == LOW) switchCfan(newState);
}
void switchVFan(boolean newState) {
  if (bitRead(switchStatus, vFanID) != newState) {
    sprintf_P(tmpLog, PSTR("SW|VFan|%d"), newState); addLog(tmpLog);
  }
  bitWrite(switchStatus, vFanID, newState);
  mySwitch.send(vfan[newState], 24);
}

void switchLight(boolean newState) {
  if (bitRead(switchStatus, lightID) != newState) {
    sprintf_P(tmpLog, PSTR("SW|Light|%d"), newState); addLog(tmpLog);
  }
  bitWrite(switchStatus, lightID, newState);
  mySwitch.send(light[newState], 24);
}
void switchCFan(boolean newState) {
  if (bitRead(switchStatus, cFanID) != newState) {
    sprintf_P(tmpLog, PSTR("SW|CFan|%d"), newState); addLog(tmpLog);
  }
  bitWrite(switchStatus, cFanID, newState);
  mySwitch.send(cfan[newState], 24);
}
/* with protection logic to prevent long period heating */
void switchHeater(boolean newState) {
  if (bitRead(switchStatus, heatID) != newState) {
    sprintf_P(tmpLog, PSTR("SW|Heat|%d"), newState); addLog(tmpLog);
  }
  bitWrite(switchStatus, heatID, newState);
  mySwitch.send(heat[newState], 24);
}
//void switchCooler(boolean newState) {
//  registerWrite(cooler, newState);
//}
//void switchPump(boolean newState) {
//  registerWrite(pump, newState);
//}
void switchReset() {
  switchMister(HIGH);
  switchVFan(HIGH);
  switchLight(HIGH);
  switchHeater(HIGH);
}
void usageStat() {
  if (loopCount % STS_CYCLE == 0) {
    long curTime = millis();
    int timePassed = (curTime - lastSurvayTime) / 1000;
    lastSurvayTime = curTime;
    for (int i = 0; i < 6; i++) {
      if (bitRead(switchStatus, i) == LOW) ttlOprSec[i] += timePassed;
      ttlOprMin[i] += ttlOprSec[i] / 60;
      ttlOprSec[i] = ttlOprSec[i] % 60;
    }
  }
  if (loopCount % STS_PRINT_CYCLE == 0) { //dump to log every ~10mins (est. by cycles)
    sprintf_P(tmpLog, STR_STS,
              ttlOprMin[0], ttlOprMin[1], ttlOprMin[2], ttlOprMin[3], ttlOprMin[4], ttlOprMin[5]); addLog(tmpLog);
  }
}

void printTentEnv(byte env) {
  char tempStr[6], humidStr[7];
  if (env == 255) { //print environment
    dtostrf(envTemp, 5, 2, tempStr);
    dtostrf(envRh, 6, 2, humidStr);
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
  for (byte i = 0; i < RADIO_COUNT; i++) {
    if (sensorTemp[i] != -1 && sensorRh[i] != -1) {
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

void revData() {
  if ( radio.available()) {
    // Variable for the received timestamp
    while (radio.available()) {
      // While there is data ready
      radio.read( &myData, sizeof(myData) );             // Get the payload
    }

    sensorId = myData.id;
    sensorTemp[sensorId - 1] = myData.value_temp;
    sensorRh[sensorId - 1] = myData.value_rh;
    lastReceive[sensorId - 1] = millis();

    Serial.print(F("Received: "));
    Serial.print(myData.id); Serial.print(F(" | "));
    Serial.print(myData.value_temp);
    Serial.print(F(" | "));
    Serial.println(myData.value_rh);

    //    radio.stopListening();
    //    radio.write(&myData, sizeof(myData) );             // Send the final one back.
    //    radio.startListening();                              // Now, resume listening so we catch the next packets.

    //    if (! rt) {
    //      Serial.print(F("failed :"));
    //      Serial.println(rt);
    //    }
  }
}

void readEnv() {
  int err = SimpleDHTErrSuccess;
  int count = 0;
  error = error & ~E_SENSOR;
  if ((err = dht22.read2(DHT_PIN, &envTemp, &envRh, NULL)) != SimpleDHTErrSuccess) {
    Serial.print(F("E]DHT")); Serial.println(err);
    envTemp = envRh = -1;
    error = error | E_SENSOR;
    //      break;
    //    }
  }
}
void btnDelay() {
  while (digitalRead(SW_PIN) == LOW) delay(100); // hold control until release
}
