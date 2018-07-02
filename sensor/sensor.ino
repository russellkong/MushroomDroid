
#include <LCD.h>
#include <LiquidCrystal_I2C.h> // F Malpartida's NewLiquidCrystal library
#include "SHTSensor.h"
#include <SPI.h>
#include "RF24.h"

#define SIMPLE_TIME 5000

#define I2C_ADDR 0x3F // Define I2C Address for controller
#define BACKLIGHT 3
#define LCD_SIZE_X 20
#define LCD_SIZE_Y 4

//Display variables
LiquidCrystal_I2C lcd(I2C_ADDR, BACKLIGHT, POSITIVE);
SHTSensor sht;
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(5, 4);
int radioNumber = 1;
#define RADIO_COUNT 3
byte addresses[RADIO_COUNT][6] = {"1Node", "2Node", "3Node"};

float sensorTemp = 0;
float sensorRh = 0;
long simpleTime = millis();
boolean doSimple = true;
unsigned long   started_waiting_at  = micros();           // Set up a timeout period, get the current microseconds
boolean pongFlg = true;

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
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

  sht.init();

  lcd.begin (LCD_SIZE_X, LCD_SIZE_Y); // initialize the lcd
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("Sensor"));

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
  radio.startListening();
  simpleTime = millis();
}
void loop() {
  doSimple = ( millis() - simpleTime > SIMPLE_TIME);
  error = 0;
  // put your main code here, to run repeatedly:
  if (doSimple) {
    if (sht.readSample()) {
      sensorTemp = sht.getTemperature();
      sensorRh = sht.getHumidity();
    } else {
      Serial.println(F("E]SHT"));
      error = error | E_SENSOR;
    }
  }

  if (radio.available() ) {                            // While nothing is received
    // Grab the response, compare, and send to debugging spew
    while (radio.available()) {                          // While there is data ready
      radio.read( &myData, sizeof(myData) );             // Get the payload
    }
    if (radioNumber == myData.id) {
      unsigned long time = micros();
      lcd.setCursor(9, 1); lcd.print("CONN: "); lcd.print("OK  ");
      // Spew it
      Serial.print(F("Got response "));
      Serial.print(myData._micros);
      Serial.print(F(", delay "));
      Serial.print(time - myData._micros);
      Serial.println(F(" ms"));
      pongFlg = true;
      radio.stopListening();
    }
  }

  if (!pongFlg && micros() - started_waiting_at > 200000 ) {           // If waited longer than 200ms, indicate timeout and exit while loop
    lcd.setCursor(9, 1);   lcd.print("CONN: "); lcd.print("FAIL");
  }

  if (doSimple && error == 0) {

    Serial.print(F("Now sending.."));
    myData.id = radioNumber;
    myData._micros = micros();
    myData.value_temp = sensorTemp;
    myData.value_rh = sensorRh;

    Serial.print(myData._micros);
    Serial.print(F(" : "));
    Serial.print(myData.id); Serial.print(F(" | "));
    Serial.print(myData.value_temp);
    Serial.print(F(" | "));
    Serial.print(myData.value_rh);

    radio.stopListening();
    int rt = radio.write( &myData, sizeof(myData));
    radio.startListening();
    if (rt) {
      Serial.print(F("..Sent, "));
    } else {
      Serial.print(F("failed :"));
      Serial.println(rt);
    }
    started_waiting_at = micros();
    pongFlg = false;
  }

  boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not

  lcd.setCursor(0, 1);
  lcd.print("ID: "); lcd.print(radioNumber);
  lcd.setCursor(0, 2);
  lcd.print("TEMP: "); lcd.print(sensorTemp);
  lcd.setCursor(0, 3);
  lcd.print("RH: "); lcd.print(sensorRh);


  if (doSimple && error == 0) {
    simpleTime = millis();

  }
}
