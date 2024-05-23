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
