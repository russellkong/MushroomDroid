#ifndef PIN_H_INCLUDED
#define PIN_H_INCLUDED

//vvvvvvvvvvvvvvvvvvvvvvvvSTART BROAD SPEC CONF vvvvvvvvvvvvvvvvvvvvvvvvvv//
#ifdef MEGA //====================MEGA============================
//DHT
#define DHT_ENV_PIN 6 //digital pin
#define DHT1_PIN 7 //digital pin
//SD card
#define CS_Pin 53 //dihgital pin (clock pin)
//Joystick
#define SW_PIN 2 // digital pin connected to switch output
#define X_PIN A4 // analog pin connected to X output
#define Y_PIN A3 // analog pin connected to Y output
#define JS_ORIENTATION -1 //vertical positioning of joystick
//Register
#define DATA_PIN 3 //digital pin
#define LATCH_PIN 4 //digital pin
#define CLOCK_PIN 5 //digital pin
#define REVERSE_REG //reversed register to relay pin connection
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

#else   //=====================UNO R3==============================
#define DHT_ENV_PIN 6 //digital pin
#define DHT1_PIN 7 //digital pin
//SD card
#define CS_Pin 10 //digital pin (clock pin)
//Joystick
#define SW_PIN 2 // digital pin connected to switch output
#define X_PIN A3 // analog pin connected to X output
#define Y_PIN A4 // analog pin connected to Y output
#define JS_ORIENTATION 1
//Register
#define DATA_PIN 3 //digital pin
#define LATCH_PIN 4 //digital pin
#define CLOCK_PIN 5 //digital pin
//CO2
#ifdef CO2_SENSOR
//CO2 MHZ19B
#define MHZ19B_RX 11  //Serial rx pin no
#define MHZ19B_TX 12  //Serial tx pin no
#endif
#endif
//^^^^^^^^^^^^^^^^^^^^^^^^^^END BROAD SPEC CONF^^^^^^^^^^^^^^^^^^^^^^^^^^^//
#endif
