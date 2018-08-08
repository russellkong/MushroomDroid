
#define RADIO_COUNT 3
#define CO2_SENSOR_ID 2

byte addresses[3][6] = {"1Node", "2Node", "3Node"};


struct dataStruct {
  unsigned long _micros;
  int id;
  int type;//1:SHT, 2:CO2
  float value1;//temp;CO2
  float value2;//rh;
} myData;