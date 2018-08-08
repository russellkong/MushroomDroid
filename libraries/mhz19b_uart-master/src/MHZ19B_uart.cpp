/*
  MHZ19B_uart.cpp - MH-Z19 CO2 sensor library for ESP-WROOM-02/32(ESP8266/ESP32) or Arduino
  version 0.3
  
  License MIT
*/

#include "MHZ19B_uart.h"
#include "Arduino.h"


#define WAIT_READ_TIMES	100
#define WAIT_READ_DELAY	10

// public

MHZ19B_uart::MHZ19B_uart(){
}
MHZ19B_uart::MHZ19B_uart(int rx, int tx){
	begin(rx,tx);
}

MHZ19B_uart::~MHZ19B_uart(){
}

#ifdef ARDUINO_ARCH_ESP32
void MHZ19B_uart::begin(int rx, int tx, int s){
	_rx_pin = rx;
	_tx_pin = tx;
	_serialno = s;
}
#else
void MHZ19B_uart::begin(int rx, int tx){
	_rx_pin = rx;
	_tx_pin = tx;
}
#endif

void MHZ19B_uart::setAutoCalibration(boolean autocalib){
	writeCommand( autocalib ? autocalib_on : autocalib_off );
}

void MHZ19B_uart::calibrateZero() {
	writeCommand( zerocalib );
}

void MHZ19B_uart::calibrateSpan(int ppm) {
	if( ppm < 1000 )	return;

	uint8_t com[MHZ19B_uart::REQUEST_CNT];
	for(int i=0; i<MHZ19B_uart::REQUEST_CNT; i++) {
		com[i] = spancalib[i];
	}
	com[3] = (uint8_t)(ppm/256);
	com[4] = (uint8_t)(ppm%256);
	writeCommand( com );
}

void MHZ19B_uart::setRange(int ppm){
	//if(ppm != 2000 && ppm != 5000) return;
	uint8_t com[MHZ19B_uart::REQUEST_CNT];
	for(int i=0; i<MHZ19B_uart::REQUEST_CNT; i++) {
		com[i] = setrange[i];
	}
	com[3] = (uint8_t)(ppm/256);
	com[4] = (uint8_t)(ppm%256);
	writeCommand( com );
}

int MHZ19B_uart::getPPM() {
	return getSerialData(PPM);
}

//protected
void MHZ19B_uart::writeCommand(uint8_t cmd[]) {
	writeCommand(cmd,NULL);
}

void MHZ19B_uart::writeCommand(uint8_t cmd[], uint8_t* response) {
	#ifdef ARDUINO_ARCH_ESP32
		HardwareSerial hserial(_serialno);
		hserial.begin(9600, SERIAL_8N1, _rx_pin, _tx_pin);
	#else
		SoftwareSerial hserial(_rx_pin, _tx_pin);
		hserial.begin(9600);
	#endif
    hserial.write(cmd, REQUEST_CNT);
	hserial.write(mhz19b_checksum(cmd));
	hserial.flush();
	
	if (response != NULL) {
		int i = 0;
		hserial.listen();
		while(hserial.available() <= 0) {
			if( ++i > WAIT_READ_TIMES ) {
				Serial.println("error: can't get MH-Z19B response.");
				return;
			}
			yield();
			delay(WAIT_READ_DELAY);
		}
		//Serial.println(String("debug:")+String(i));
		hserial.readBytes(response, MHZ19B_uart::RESPONSE_CNT);
	}
}

//private

int MHZ19B_uart::getSerialData(MHZ19B_DATA flg) {
	uint8_t buf[MHZ19B_uart::RESPONSE_CNT];
	for( int i=0; i<MHZ19B_uart::RESPONSE_CNT; i++){
		buf[i]=0x0;
	}

	writeCommand(getppm, buf);
	int co2 = 0;

	// parse
	if (buf[0] == 0xff && buf[1] == 0x86 && mhz19b_checksum(buf) == buf[MHZ19B_uart::RESPONSE_CNT-1]) {
		co2 = buf[2] * 256 + buf[3];
	} else {
		co2 = -1;
	}

	switch(flg) {
		case MHZ19B_DATA::PPM:
		default:
			return co2;
			break;
	}
}	

uint8_t MHZ19B_uart::mhz19b_checksum( uint8_t com[] ) {
	uint8_t sum = 0x00;
	for ( int i = 1; i < MHZ19B_uart::REQUEST_CNT; i++) {
		sum += com[i];
	}
	sum = 0xff - sum + 0x01;
	return sum;
}
