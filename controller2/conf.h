#ifndef conf_h
#define conf_h
//#define DEBUG
//#define DEBUG_RF
//#define TEMP_DOWN

//Equipment List
//#define CFAN
//#define HEATER
//#define MH_LCD
#define SHT_SENSOR
#define RECEIVER
#define USE_SD
#define VERSION "v4.0"

#define ROOM_ID 0  //[0;1]
#define RADIO_ID 0

//User Defined Conf
//#define MIDDAY_LIGHT_OFF

//System Parameters
//LCD
#ifdef MH_LCD
#define I2C_ADDR 0x27
#else
#define I2C_ADDR 0x3F // Define I2C Address for controller
#endif

#define BACKLIGHT 3
#define LCD_SIZE_X 20
#define LCD_SIZE_Y 4
//SD card
#define CONF_FILE "CONF.TXT"
//Joystick
#define JS_TRIGGER 300

//Operation constants
#define LOOP_TIME 1000 //the min. duration of a loop
#define SPLIT '|'
#define LCD_OFF_DELAY 30000 //idle loop to turn off LCD backlight
#define REACT_TIME 450 //reaction time of joystick action
#define MAX_CTL_HOLD_TIME 30000
#define RADIO_GAP 1000

#define MAX_RUNTIME 30*60000 //max. time(ms) of program execution
#define GOAL_COUNT 15  //reading before advance step in program
#define SAMPLE_INTERVAL 2000 //interval to read local sensor
#define SUMMARY_INTERVAL 5*60000
#define RESUBMIT_INTERVAL 1*60000 //interval to resubmit switch instruction
#define DATA_TIMEOUT 5*60000 //time to expire received data
#define VFAN_REST_TIME 6*60000
#define VFAN_OP_TIME 4*60000
#define SWITCH_INTERVAL_MIN 60000
//Display variables
#define SCN_NUM 7
#define SCN_ID_INFO 0
#define SCN_ID_ENV 1
#define SCN_ID_SW 2
#define SCN_ID_MODE 3
#define SCN_ID_STAT 4
#define SCN_ID_CONF 5
#define SCN_ID_SYS 6

#define CO2_NORMAL 500
#define CO2_MAX 1000


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
