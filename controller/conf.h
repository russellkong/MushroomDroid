#ifndef conf_h
#define conf_h
//#define DEBUG

//Equipment List
//#define CFAN
//#define HEATER
#define MH_LCD
#define SHT_SENSOR

#define VERSION "v2.93"

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

#define MAX_RUNTIME 15*60000 //max. time(ms) of program execution
#define GOAL_COUNT 15  //reading before advance step in program
#define SAMPLE_INTERVAL 5000 //interval to read local sensor
#define SUMMARY_INTERVAL 5*60000
#define RESUBMIT_INTERVAL 60000 //interval to resubmit switch instruction
#define DATA_TIMEOUT 90000 //time to expire received data
//Display variables
#define SCN_NUM 7
#define SCN_ID_INFO 0
#define SCN_ID_ENV 1
#define SCN_ID_SW 2
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
#define TENT_MODE_COUNT 3
#define TENT_MODE_OFF 0
#define TENT_MODE_INCUBATION 1
#define TENT_MODE_FRUIT 2
#endif
