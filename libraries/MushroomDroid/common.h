
#define RADIO_COUNT 3

#define SENSOR_TYPE_TEMP_RH 1
#define SENSOR_TYPE_CO2 2

#define CONTROLLER 0
#define CO2_SENSOR_ID 2
#define RF_CHANNEL 30

uint64_t addresses[2][5] = {{0xF0F0F0E0AA, 0xF0F0F0E0BB, 0xF0F0F0E0CC, 0xF0F0F0E0DD,0xF0F0F0E0EE},
{0xF0F0F0F0AA, 0xF0F0F0F0BB, 0xF0F0F0F0CC, 0xF0F0F0F0DD,0xF0F0F0F0EE}};


struct RfDataObj {
  unsigned long _micros;
  int id;
  int type;//1:SHT, 2:CO2
  float value1;//temp;CO2
  float value2;//rh;
} myData;