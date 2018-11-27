//#define MEGA
#define MH_LCD
//#define BIG_LCD
//#define CO2_SENSOR
//#define COOLER
//#define PUMP
//#define DISABLE_JS

#define SHT_SENSOR 1
#define ENV_SENSOR 0
#define VERSION "v2.91"

//#include <SimpleDHT.h> //self-tailored made DHT22 lib (removed DHT11)

#include <Wire.h>
#include <DS3231.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h> // F Malpartida's NewLiquidCrystal library
#include <SD.h>
#include <avr/wdt.h>

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

#define TENT_MODE_COUNT 3
#define TENT_MODE_OFF 0
#define TENT_MODE_INCUBATION 1
#define TENT_MODE_FRUIT 2

#define SCN_NUM 7
#define SCN_ID_INFO 0
#define SCN_ID_ENV 1
#define SCN_ID_SW 2
#define SCN_ID_MODE 3
#define SCN_ID_STAT 4
#define SCN_ID_CONF 5
#define SCN_ID_SYS 6



//Clock
DS3231  rtc(SDA, SCL);
Time curTime;
#ifdef SHT_SENSOR
SHTSensor sht(SHTSensor::SHT3X);
SHTSensor envSht(SHTSensor::SHT3X_ALT);
#endif
LiquidCrystal_I2C lcd(I2C_ADDR, BACKLIGHT, POSITIVE);

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
float tentTemp, envTemp;
float tentRh, envRh;

byte scn = 0; //display 0:time, 1:temp, 2:Relay status, 3: operation mode, 4: tent stat, 6: system opr, 5: tent conf
byte frame = 0;
byte previousScn, previousFrame;
byte scn_mode = 0;
byte x = 0, y = 0;

typedef enum  {IDLE = 0, VENT, WET, RH_HIGH, RH_LOW, TEMP_HIGH } PROG;

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
