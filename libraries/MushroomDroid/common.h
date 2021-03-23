
#define RADIO_COUNT 3

#define SENSOR_TYPE_TEMP_RH 1
#define SENSOR_TYPE_CO2 2

#define CONTROLLER 0
#define CO2_SENSOR_ID 2
#define RF_CHANNEL 30

byte addresses[][6] = {"1Node", "2Node", "3Node", "4Node"};


struct RfDataObj {
  unsigned long _micros;
  int id;
  int type;//1:SHT, 2:CO2
  float value1;//temp;CO2
  float value2;//rh;
} myData;