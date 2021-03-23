#include <Wire.h>
#include <DS3231.h> //Clock
#include <LCD.h>
#include <LiquidCrystal_I2C.h> // F Malpartida's NewLiquidCrystal library
#include "SHTSensor.h"
#include <SPI.h>
#include <SdFat.h> //SD card, softSpi

#ifdef RECEIVER
#include <RF24.h> //2.4GHz radio
#endif
#include <RCSwitch.h> //433 radio

#include <math.h>
#include <avr/wdt.h> //watchdog
#include <common.h>

#include "conf.h"
#include "pin.h"

//Tent program
typedef enum  {IDLE = 0, VENT, WET, RH_HIGH, RH_LOW, TEMP_HIGH, EXCG, ERR } PROG;

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
const PROGMEM char STR_CONF[] = "M%d|H:%d-%d-%d|T:%d-%d-%d|V:%d,%d|L:%d-%d|W:%d,%d,%d|A:%c";
const PROGMEM char STR_CONF_LINE[] = "%2d%2d%2d%2d%2d%2d%2d%2d%2d%2d%2d%2d%c";

char logFilename[] = "xxxx.LOG\0";
char tmpLog[200];

//RfDataObj myData;

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

//Radio Switch Code [0:On, 1:Off]
unsigned long vfan[2] = {1117011, 1117810};
unsigned long mist[2] = {1717200, 1817218};
//byte cooler = 5;
unsigned long light[2] = {1330303, 1331818};
unsigned long cfan[2] = {17400, 17418};
unsigned long heat[2] = {17500, 17518};

//Statistic
unsigned long lastSurvayTime = 0; //time to calculate usage statistic
int ttlOprMin[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int ttlOprSec[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//Tent parameters (define in conf. file (SD Card))
uint8_t humidMax = 95; uint8_t humidMid = 90; uint8_t humidMin = 87;
uint8_t tempMax = 28; uint8_t tempMid = 23; uint8_t tempMin = 16;
uint8_t ventInv = 30; uint8_t ventDur = 10;
uint8_t lightStart = 10; uint8_t lightEnd = 20;
uint8_t wetHour = 0; uint8_t wetInv = 2; uint8_t wetDur = 10; 
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
long vfanOnSectionTime = 0; //time count of vfan sectional operation time
long vfanCooldownTime = 0;
void printTentEnv(byte env);
long lastSwitchTimeVFan = 0;
long lastSwitchTimeMist = 0;

void switchMister(boolean newState, boolean force = false) ;
void switchVFan(boolean newState, boolean force = false) ;
void switchLight(boolean newState, boolean force = false) ;
void switchCFan(boolean newState, boolean force = false) ;
void switchHeater(boolean newState, boolean force = false) ;
float mollierTemp(float inTemp, float inRh);
