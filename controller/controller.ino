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
   3.11 update idle handling; update ventilation humidity control; update wet period to time in clock
*/

#include "conf.h"
#include "pin.h"
#include "controller.h"


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
  vfanOnSectionTime = 0;
  lastSwitchTimeVFan = 0;
  lastSwitchTimeMist = 0;
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
  lcd.print(F("Controller "));lcd.print(char(ROOM_ID+65));
  lcd.setCursor(0, 1);
  lcd.print(VERSION);

  // Initialize the rtc object
  rtc.begin();
  // The following lines can be uncommented to set the date and time
//          rtc.setDOW(3);     // Set Day-of-Week to SUNDAY
//          rtc.setTime(00, 32, 30);     // Set the time to 12:00:00 (24hr format)
//          rtc.setDate(22, 5, 2024);   // Set the date to January 1st, 2014

  //Joystick
  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  //digitalWrite(SW_PIN, HIGH);
  
  //Init error led
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);

  //Flip switch pins - once go LOW, flip the switch
  pinMode(ML_LIGHT_ON, INPUT_PULLUP);
  pinMode(ML_MIST_ON,INPUT_PULLUP);
  pinMode(ML_VFAN_ON,INPUT_PULLUP);
  pinMode(ML_LIGHT_OFF, INPUT_PULLUP);
  pinMode(ML_MIST_OFF,INPUT_PULLUP);
  pinMode(ML_VFAN_OFF,INPUT_PULLUP);
  
  //INIT SPI modules
  SPI.begin();

  //int SD card
  initSd();
  lcd.setCursor(0, 2); lcd.print(rtc.getDateStr());
  Serial.print(F("Get time..."));
  curTime = rtc.getTime();
  openLog();
  sprintf_P(tmpLog, PSTR("SY|Boot|%s"), VERSION); addLog(tmpLog);
#ifdef RECEIVER
  Serial.print(F("Init sensor radio..."));
  //Init 2.4G radio listening for data
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  //  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(RF_CHANNEL);
  //  radio.setCRCLength( RF24_CRC_16 ) ;
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  //radio.setPayloadSize(sizeof(myData));
  // Open a writing and reading pipe on each radio, with opposite addresses
  //Serial.print("Radio ID: "); Serial.println((const char*)addresses[ROOM_ID][RADIO_ID]);
  //radio.openWritingPipe(addresses[ROOM_ID][RADIO_ID]);
  //myData.id = RADIO_ID;
  int j = 1;
  //Serial.print(F("Listen ID: "));
  for (int i = 0; i < RADIO_COUNT; i++) {
    if (i != RADIO_ID) {
      //Serial.print((const char*)addresses[ROOM_ID][i]);
      //Serial.print(";");
      radio.openReadingPipe(j, addresses[ROOM_ID][i]);
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


  //All switch set to off
  switchReset();

  //Get hardware default mode
  Serial.print(F("Check Hardware Mode... "));
  if (checkMode() != tentMode) {
    changeMode(checkMode());
  }
  Serial.println("Done");

  jsXRestPoint = 512;//analogRead(X_PIN);
  jsYRestPoint = 512;//analogRead(Y_PIN);

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
#ifdef DEBUG2
  Serial.print(F("Cursor:"));
  Serial.print(analogRead(X_PIN));
  Serial.print(';');
  Serial.println(analogRead(Y_PIN));
#endif
  if (scn_mode == 1) ctlSettingScn();
  refreshScn();
  if (inTime - lastUserActionTime > LCD_OFF_DELAY) {
    lcd.noBacklight();
    scn_mode = 0; //exit edit mode
  }
  //===============Process tent operation======================//
  loadTentEnv();
  ////vvvvvvvvvvvvvvvv Start Enabled tent logic  vvvvvvvvvvvvvv//
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
    } else if (tentStep > 0 //confirmed execution
               || confirmProgram()
              ) {
      executeProgram();
    }
    if (tentProg == PROG::IDLE) {
      runIdle();
    }

    resubmitSwitch();
  }
  //Read pins to send radio signal
  switchRadio();
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
  digitalWrite(RED_PIN, bitRead(error, E_SENSOR) || (inTime - lastRevMinTime > 5 * 60000) || tentProg == PROG::ERR || !shtSensorHealth);
  //Green light alert on SD error || disabled
  digitalWrite(GREEN_PIN, bitRead(error, E_SD) || skipSd);

  if (tentProg == PROG::ERR || !shtSensorHealth) {
    errorCnt++;
  } else {
    errorCnt == 0;
  }
  //==================ERROR reboot; when radio lost, reboot the system
  boolean reboot = false;

  if (inTime - lastRevMinTime > 30 * 60000 ||  errorCnt > 30 * 60) {
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
  return inTime - lastUserActionTime < LCD_OFF_DELAY && lastUserActionTime > 0;
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
        if (x < 2) {
          switchMister(!bitRead(switchStatus, mistID), true);
        } else if (x >= 2) {
          switchVFan(!bitRead(switchStatus, vFanID), true);
        }
      } else if (y == 2) {
        if (x < 2) {
          switchLight(!bitRead(switchStatus, lightID), true);
        } else if (x >= 2) {
          switchCFan(!bitRead(switchStatus, cFanID), true);
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
  progEnd(); //reset tent operation
  if (newMode != TENT_MODE_OFF) {
    result = loadConf(newMode); //0:off, 1:spawn running, 2:fruiting
  }
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
        if (x <= 1 ) wetHour = changeValue(wetHour);
        else if (x <= 3) wetInv = changeValue(wetInv);
        else if (x <= 5) wetDur = changeValue(wetDur);
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
  lcd.setCursor(0, 0); //lcd.print(rtc.getDateStr()); lcd.print(F("  ")); lcd.print(rtc.getTimeStr());
  lcd.print(F("ROOM ")); lcd.print(char(ROOM_ID+65)); lcd.print(F("    "));lcd.print(rtc.getTimeStr());
  lcd.setCursor(0, 1); //lcd.print(F("I:")); 12.51C|99.9%|10000
  lcd.print(workingTemp); lcd.print(F("C|")); lcd.print(workingRh); lcd.setCursor(11, 1); lcd.print(F("%|")); lcd.print(workingCO2); if (workingCO2 < 10000)lcd.print(F(" ")); if (workingCO2 < 1000)lcd.print(F(" "));
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
    case EXCG:
      lcd.print("EXCG "); lcd.print(tentStep);
      break;
    case ERR:
      lcd.print("ERR   ");
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
  lcd.print("M|VF: "); lcd.print(bitRead(switchStatus, mistID)); lcd.print("|"); lcd.print(bitRead(switchStatus, vFanID));
  lcd.setCursor(0, 2);
  lcd.print("L|CF: "); lcd.print(bitRead(switchStatus, lightID)); lcd.print("|"); lcd.print(bitRead(switchStatus, cFanID));
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
      lcd.print(ventInv); lcd.print(F("h|")); lcd.print(ventDur); lcd.print('m');
      break;
    case 3:
      lcd.print(F("Light[S|E]"));
      lcd.setCursor(0, 1);
      lcd.print(lightStart); lcd.print(SPLIT); lcd.print(lightEnd);
      break;
    case 4:
      lcd.print(F("Wet[S|I|D]"));
      lcd.setCursor(0, 1);
      lcd.print(F("S|I|D"));
      lcd.setCursor(0, 2);
      lcd.print(wetHour); lcd.print(F("h|")); lcd.print(wetInv); lcd.print(F("h|")); lcd.print(wetDur); lcd.print('m');
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
  if ( curTime.hour % wetInv == wetHour &&  curTime.min < wetDur) {
    //if (inTime - tentLastWetTime > wetInv * 60 * 60000) {
    suggestion = PROG::WET;
    return;
  }
  //1. Regular ventilation
  //without sensor follow default schedule
  //with sensor, only ventilate when CO2 over limit
  if (//(workingCO2 <= 0 && inTime - tentLastFanTime > ventInv * 60000)
    inTime - tentLastFanTime > ventInv * 60000
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
               && !airCon
               //&& workingRh > envRh
               //&& wetTemp(envTemp, envRh, humidMax) < tempMax
              ) {
      //3. High humidity
      //outside RH must be lower
      //reduction of RH should not raise temp over tempMax
      suggestion = PROG::RH_HIGH;
      return;
    }
    //    else if ((envTemp > tempMin && envTemp < tempMax &&
    //                envRh > humidMin && envRh < humidMax )
    //               //|| abs(wetTemp(envTemp, envRh, humidMid) - tempMid) < abs(workingTemp - tempMid)
    //              ) {
    //      suggestion = PROG::EXCG;
    //      return;
    //    }
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
  if (tentProg != PROG::IDLE && tentProg != PROG::ERR) {
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
    //    case PROG::EXCG:
    //      runExchange();
    //      break;
    //case PROG::IDLE:
    default: //case IDLE
      break;
      //runIdle();
  }
}
/* tempHi()
    Increase ventilation and reduce humidity on temperature  high
    reduce contamination and increase evaparation
    enable speckler or water screen when available
*/
void runTempHi() {
#ifdef DEBUG
  Serial.println("[IN] runTempHi..");
#endif

  if (tentStep == 0) {
    tentStep++;
    logStepChg();
    goalCount = 0;
  } else if (tentStep == 1) {
    if (airCon) {
      //intake air able to carry heat away
      if (workingTemp > wetTemp(envTemp, envRh, humidMax) || workingCO2 > CO2_MAX) {
        switchVFan(LOW);
      }
      else if (workingTemp < wetTemp(envTemp, envRh, humidMax) - 2 && workingCO2 < CO2_NORMAL) {
        switchVFan(HIGH);
      }
      if (workingRh < humidMin) {
        switchMister(LOW);
      } else if (workingRh > humidMid) {
        switchMister(HIGH);
      }
    } else {
      //if (workingRh < humidMin ) switchMister(LOW);
      switchVFan(LOW);
      //switchMister(LOW);
      //dry intake air allow evaporation, but too dry will dry out mushroom
      if ( workingRh < humidMin || envRh < humidMin-10) {//
        switchMister(LOW);
      }
      else if (workingRh > humidMid) {//workingRh > humidMax ||
        switchMister(HIGH);
      }
    }
#ifdef DEBUG
    Serial.print("goalCount=");
    Serial.print(goalCount);
    Serial.print("|runTime=");
    Serial.println(inTime - tentProgTime);
#endif
    (workingTemp < 0 || workingTemp <= tempMid) ? goalCount++ : goalCount = 0;
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
    switchVFan(LOW);
    if (workingRh < 0
        || workingRh <= humidMid
        || envRh > workingRh
        || wetTemp(envTemp, envRh, workingRh - 0.5) > tempMax
       ) {
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
//    if (workingCO2 > 0 && workingCO2 > CO2_MAX) {
//      switchVFan(LOW);
//    } else if (workingCO2 < CO2_MAX - 100) {
//      switchVFan(HIGH);
//    }
    switchMister(LOW);
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
    //2020-01-26 determine by intake air Rh instead of internal air
    //if (workingRh < 0 || workingRh < humidMid) {
    switchVFan(LOW);
    if (envRh < 0 || envRh <= humidMid || workingRh < humidMid) {
      switchMister(LOW);
    } else if (envRh > humidMax || workingRh > humidMax) {
      switchMister(HIGH);
    }
    if ((inTime - tentProgTime > ventDur * 60000)
        || (workingCO2 > 0 && workingCO2 <= CO2_NORMAL) ||(workingTemp <tempMin)
        //&& (airCon || envTemp < tempMin || envTemp > tempMax)
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
    switchMister(LOW);
    if (inTime - tentProgTime > wetDur * 60000) {
      tentLastWetTime = inTime;
      progEnd();
    }
  }
}
//void runExchange() {
//#ifdef DEBUG
//  Serial.println("[IN] runExchange..");
//#endif
//  if (tentStep == 0) {
//    tentStep++;
//    logStepChg();
//    switchVFan(LOW);
//  } else if (tentStep == 1) {
//    switchVFan(LOW);
//    if (inTime - tentProgTime > 300000) {
//      progEnd();
//    }
//  }
//}
/** execute on idle status
    maximize air ventiliation on ideal external air
*/
void runIdle() {
#ifdef DEBUG
  Serial.println("[IN] runIdle..");
#endif
  //  if ((envTemp > tempMin + 1 && envTemp < tempMax - 1 &&
  //       envRh > humidMin + 1 && envRh < humidMax - 1)
  //      || abs(wetTemp(envTemp, envRh, humidMid) - tempMid) < abs(workingTemp - tempMid)
  //     ) {
  //    switchVFan(LOW);
  //  } else if (!((envTemp > tempMin && envTemp < tempMax &&
  //                envRh > humidMin && envRh < humidMax)
  //               || abs(wetTemp(envTemp, envRh, humidMid) - tempMid) < abs(workingTemp - tempMid)
  //              )) {
  //    switchVFan(HIGH);
  //  }
  if ((envTemp > tempMin + 1 && envTemp < tempMax - 1 &&
       envRh > humidMin + 1 )//&& envRh < humidMax - 1
     ) {
    switchVFan(LOW);
  } else if (!((envTemp >= tempMin && envTemp <= tempMax &&
                envRh >= humidMin )//&& envRh <= humidMax
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
    //switchLight(HIGH);
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
  if (tentMode == 0 ) return;//|| lightStart == lightEnd
  byte decision = HIGH;
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
#ifdef USE_SD
  // see if the card is present and can be initialized:
  if (!sd.begin(SD_CHIP_SELECT_PIN, SPI_FULL_SPEED)) {
    Serial.println(F("Failed...Skip SD"));
    bitWrite(error, E_SD, 1);
    skipSd = true;
    // don't do anything more:
    return;
  }
#endif
  Serial.println(F("Done"));
}
//format:[TENT_ID(1)][TENT_MODE(1)][HUMID_HI(2)][HUMID_MI(2)][HUMID_LO(2)][TEMP_HI(2)][TEMP_MI(2)][TEMP_LO(2)][VENT_INV_MIN(2)][ventDur_MIN(2)][EOL]
byte loadConf(byte inMode) {
  if (skipSd) {
    return 0;
  }
  if (inMode == 0) return 0;

  boolean result = false;
#ifdef USE_SD
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
        wetHour =  content.substring(22, 24).toInt();
        wetInv =  content.substring(24, 26).toInt();
        wetDur =  content.substring(26, 28).toInt();
        airCon = content.substring(28, 29).equals("Y");
        result = true;
        sprintf_P(tmpLog, STR_CONF,  mode, humidMin, humidMid, humidMax, tempMin, tempMid, tempMax,
                  ventInv, ventDur, lightStart, lightEnd, wetHour, wetInv, wetDur, airCon ? 'Y' : 'N'); addLog(tmpLog);
        break;
      }
    }
  } else {
    sd.errorPrint(F("E] Conf file read"));
    result = false;
    bitWrite(error, E_SD, 1);
  }
  confFile.close();
#endif
  return result ? 0 : 1;
}

//=================================LOG============================================//
void openLog() {
#ifdef USE_SD
#ifdef DEBUG
  Serial.print("[IN] openLog..");
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
#ifdef DEBUG
  Serial.println("Done");
#endif
#endif
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
//for receiver (receiver skip msg submitted closely)
long lastRadioSendTime = millis();
void delayRadio() {
  if ((millis() - lastRadioSendTime) < RADIO_GAP) {
    delay(RADIO_GAP - (millis() - lastRadioSendTime));
  }
  lastRadioSendTime = millis();
}
void switchMister(boolean newState, boolean force) {
  if ((bitRead(switchStatus, mistID) != newState && millis() - lastSwitchTimeMist > SWITCH_INTERVAL_MIN) || force ) {
    if (!force) {
      lastSwitchTimeMist = millis();
      sprintf_P(tmpLog, PSTR("SW|Mist|%d"), newState);
      addLog(tmpLog);
    }
    bitWrite(switchStatus, mistID, newState);
    delayRadio();
    mySwitch.send(mist[ROOM_ID][newState], 24);
    //delay(RADIO_GAP);
  }
  //if (newState == LOW) switchCfan(newState);
  //  Serial.println(millis() - swTime);
}
void switchVFan(boolean newState, boolean force) {
  //  if (bitRead(switchStatus, vFanID) != newState || force) {
  //    sprintf_P(tmpLog, PSTR("SW|VFan|%d"), newState);
  //    addLog(tmpLog);
  //  }
  //fan operation purse (temp solution to over ventilate)
  if (tentMode > 0 && !force) {
    if (bitRead(switchStatus, vFanID) == 0 || newState == LOW) { //fan already on [
#ifdef DEBUG
      Serial.print("[VFan]On");
#endif
//      if (vfanOnSectionTime == 0) {
//        vfanOnSectionTime = millis();
//      }
//      if (millis() > vfanOnSectionTime + VFAN_OP_TIME + VFAN_REST_TIME) { //next start
//#ifdef DEBUG
//        Serial.print("[VFan]Resume");
//#endif
//        vfanOnSectionTime = millis();
//      } else if (millis() > vfanOnSectionTime + VFAN_OP_TIME) { //ontime over, start cooldown
//#ifdef DEBUG
//        Serial.print("[VFan]CoolDown");
//#endif
//        newState = HIGH;
//      }
    } else {
#ifdef DEBUG
      Serial.print("[VFan]Off");
#endif
      vfanOnSectionTime = 0;
    }
  }
#ifdef DEBUG
  Serial.println(".");
#endif
  //&& millis() - lastSwitchTimeVFan > SWITCH_INTERVAL_MIN
  if ((bitRead(switchStatus, vFanID) != newState ) || force) {
//    lastSwitchTimeVFan = millis();
    sprintf_P(tmpLog, PSTR("SW|VFan|%d"), newState);
    addLog(tmpLog);

    bitWrite(switchStatus, vFanID, newState);
    delayRadio();
    mySwitch.send(vfan[ROOM_ID][newState], 24);
  }
  //delay(RADIO_GAP);

}
void switchLight(boolean newState, boolean force) {
  if (bitRead(switchStatus, lightID) != newState || force) {
    if (!force) {
      sprintf_P(tmpLog, PSTR("SW|LED|%d"), newState);
      addLog(tmpLog);
    }
    bitWrite(switchStatus, lightID, newState);
    delayRadio();
    mySwitch.send(light[ROOM_ID][newState], 24);
  }
}
void switchCFan(boolean newState, boolean force) {
  if (bitRead(switchStatus, cFanID) != newState || force) {
    if (!force) {
      sprintf_P(tmpLog, PSTR("SW|CFan|%d"), newState);
      addLog(tmpLog);
    }
    bitWrite(switchStatus, cFanID, newState);
    delayRadio();
    mySwitch.send(cfan[ROOM_ID][newState], 24);
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
    mySwitch.send(heat[ROOM_ID][newState], 24);
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

byte rotation;
void resubmitSwitch() {
  if (inTime - resubmitTime > RESUBMIT_INTERVAL) {
#ifdef DEBUG
    Serial.println("[IN] resubmitSwitch..");
#endif

    rotation++;
    switch (rotation % 2) {
      case 0:
        switchMister(bitRead(switchStatus, mistID), true);
        break;
      case 1:
        switchVFan(bitRead(switchStatus, vFanID), true);
        break;
    }
    //switchLight(bitRead(switchStatus, lightID), true);
    resubmitTime = inTime;
  }
}
//Manual trigger to send radio signal to switch
void switchRadio(){
  if(digitalRead(ML_VFAN_ON)==LOW){
    mySwitch.send(vfan[ROOM_ID][0], 24);
  }
  if(digitalRead(ML_VFAN_OFF)==LOW){
    mySwitch.send(vfan[ROOM_ID][1], 24);
  }
  if(digitalRead(ML_LIGHT_ON)==LOW){
    mySwitch.send(light[ROOM_ID][0], 24);
    
  }
  if(digitalRead(ML_LIGHT_OFF)==LOW){
    mySwitch.send(light[ROOM_ID][1], 24);
  }
  if(digitalRead(ML_MIST_ON)==LOW){
    mySwitch.send(mist[ROOM_ID][0], 24);
  }
  if(digitalRead(ML_MIST_OFF)==LOW){
    mySwitch.send(mist[ROOM_ID][1], 24);
  }
  delayRadio();
  return;
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
  float _workingTemp = 0;
  float _workingRh = 0;
  float _workingCO2 = 0;
  byte count = 0;
  for (byte i = 0; i < RADIO_COUNT; i++) {
    if (inTime - lastRevTime[i] < DATA_TIMEOUT
        && sensorTemp[i] > 0 && sensorRh[i] > 0) {
      _workingTemp += sensorTemp[i];
      _workingRh += sensorRh[i];
      count++;
    }
  }
  if (count > 0) {
    workingTemp = _workingTemp / count;
    workingRh = _workingRh / count;
    shtSensorHealth = true;
  } else {
    if(inTime - lastRevTime[0] > DATA_TIMEOUT){
      //workingTemp = workingRh = -1;
      //#ifdef TEMP_DOWN
      workingTemp = envTemp;
      workingRh = envRh;
      shtSensorHealth = false;
      //#endif
    }
  }

  if (inTime - lastRevTime[CO2_SENSOR_ID - 1] < DATA_TIMEOUT
      ) {//&& sensorCO2 > 0
    workingCO2 = sensorCO2;
  } else {
    workingCO2 = -2;
  }

}

void revData() {
#ifdef DEBUG_RF
  Serial.println("[IN] revData..");
#endif
#ifdef RECEIVER
  while ( radio.available()) {
    // Variable for the received timestamp
    //while (radio.available()) {
    // While there is data ready
    radio.read(&myData, sizeof(myData));             // Get the payload
    //}
#ifdef DEBUG_RF
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
