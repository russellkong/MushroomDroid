
#include <LCD.h>
#include <LiquidCrystal_I2C.h> // F Malpartida's NewLiquidCrystal library
//#include "SHTSensor.h"
#include <SPI.h>
#include "RF24.h"
#include <RCSwitch.h>



#define I2C_ADDR 0x3F // Define I2C Address for controller
#define BACKLIGHT 3
#define LCD_SIZE_X 20
#define LCD_SIZE_Y 4

//Display variables
LiquidCrystal_I2C lcd(I2C_ADDR, BACKLIGHT, POSITIVE);
//SHTSensor sht;
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(5, 4);
RCSwitch mySwitch = RCSwitch();

int radioNumber = 0;
#define RADIO_COUNT 3
byte addresses[RADIO_COUNT][6] = {"1Node", "2Node", "3Node"};

int sensorId = 0;
float sensorTemp = 0;
float sensorRh = 0;
long simpleTime = 0;
boolean doSimple = true;

struct dataStruct {
  unsigned long _micros;
  int id;
  float value_temp;
  float value_rh;
} myData;

byte error = B00000000;
//flag for error led
#define OK B00000000
#define E_SENSOR B00000001
#define E_SD B00000010

byte curFog = HIGH, curFan = HIGH;
long lastReceive;
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  lcd.begin (LCD_SIZE_X, LCD_SIZE_Y); // initialize the lcd
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("Controller"));

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
  // Start the radio listening for data
  radio.startListening();

  mySwitch.enableTransmit(2);
}

void loop() {
  error = 0;

  if ( radio.available()) {
    // Variable for the received timestamp
    while (radio.available()) {                          // While there is data ready
      radio.read( &myData, sizeof(myData) );             // Get the payload
    }
    sensorId = myData.id;
    sensorTemp = myData.value_temp;
    sensorRh = myData.value_rh;
    lastReceive = millis();

    radio.stopListening();                               // Increment the float value
    int rt = radio.write( &myData, sizeof(myData) );             // Send the final one back.
    radio.startListening();                              // Now, resume listening so we catch the next packets.
    Serial.print(F("Sent response "));
    
    Serial.print(myData._micros);
    Serial.print(F(" : "));
    Serial.print(myData.id);Serial.print(F(" | "));
    Serial.print(myData.value_temp);
    Serial.print(F(" | "));
    Serial.println(myData.value_rh);
    if (! rt) {
      Serial.print(F("failed :"));
      Serial.println(rt);
    }
  }
  if (sensorId == 1) {
    lcd.setCursor(0, 1);
    lcd.print(sensorId); lcd.print("|"); lcd.print((millis() - lastReceive) / 1000); lcd.print("s|"); lcd.print(sensorTemp); lcd.print("C"); lcd.print("|"); lcd.print(sensorRh); lcd.print("%");
  }
  if (sensorId == 2) {
    lcd.setCursor(0, 2);
    lcd.print(sensorId); lcd.print("|"); lcd.print((millis() - lastReceive) / 1000); lcd.print("s|"); lcd.print(sensorTemp); lcd.print("C"); lcd.print("|"); lcd.print(sensorRh); lcd.print("%");
  }
  lcd.setCursor(0, 3);
  lcd.print("FOG: "); lcd.print(curFog); lcd.print("  FAN: "); lcd.print(curFan);

  if (sensorRh > 80 && curFog == LOW) {
    Serial.println("OFF");
    mySwitch.send(17393, 24);
    curFog = HIGH;
  }
  if (sensorRh < 70 && curFog == HIGH) {
    Serial.println("ON");
    mySwitch.send(17393, 24);
    curFog = LOW;
  }
  //  if (doSimple && error == 0) {
  //    simpleTime = millis();
  //  }
}
