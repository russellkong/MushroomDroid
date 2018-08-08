/*
 *  Copyright (c) 2016, Sensirion AG <andreas.brauchli@sensirion.com>
 *  Copyright (c) 2015-2016, Johannes Winkelmann <jw@smts.ch>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <inttypes.h>
#include <Wire.h>
#include <Arduino.h>

#include "SHTSensor.h"


//
// class SHTSensorDriver
//

SHTSensorDriver::~SHTSensorDriver()
{
}

bool SHTSensorDriver::readSample()
{
  return false;
}


//
// class SHTI2cSensor
//

const uint8_t SHTI2cSensor::CMD_SIZE            = 2;
const uint8_t SHTI2cSensor::EXPECTED_DATA_SIZE  = 6;

bool SHTI2cSensor::readFromI2c(uint8_t i2cAddress,
                               const uint8_t *i2cCommand,
                               uint8_t commandLength, uint8_t *data,
                               uint8_t dataLength)
{
  Wire.beginTransmission(i2cAddress);
  for (int i = 0; i < commandLength; ++i) {
    if (Wire.write(i2cCommand[i]) != 1) {
      return false;
    }
  }

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom(i2cAddress, dataLength);

  // check if the same number of bytes are received that are requested.
  if (Wire.available() != dataLength) {
    return false;
  }

  for (int i = 0; i < dataLength; ++i) {
    data[i] = Wire.read();
  }
  return true;
}

uint8_t SHTI2cSensor::crc8(const uint8_t *data, uint8_t len)
{
  // adapted from SHT21 sample code from
  // http://www.sensirion.com/en/products/humidity-temperature/download-center/

  uint8_t crc = 0xff;
  uint8_t byteCtr;
  for (byteCtr = 0; byteCtr < len; ++byteCtr) {
    crc ^= data[byteCtr];
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}


bool SHTI2cSensor::readSample()
{
  uint8_t data[EXPECTED_DATA_SIZE];
  uint8_t cmd[CMD_SIZE];

  cmd[0] = mI2cCommand >> 8;
  cmd[1] = mI2cCommand & 0xff;

  if (!readFromI2c(mI2cAddress, cmd, CMD_SIZE, data,
                   EXPECTED_DATA_SIZE)) {
    return false;
  }

  // -- Important: assuming each 2 byte of data is followed by 1 byte of CRC

  // check CRC for both RH and T
  if (crc8(&data[0], 2) != data[2] || crc8(&data[3], 2) != data[5]) {
    return false;
  }

  // convert to Temperature/Humidity
  uint16_t val;
  val = (data[0] << 8) + data[1];
  mTemperature = mA + mB * (val / mC);

  val = (data[3] << 8) + data[4];
  mHumidity = mX * (val / mY);

  return true;
}


//
// class SHT3xSensor
//

class SHT3xSensor : public SHTI2cSensor
{
private:
  static const uint16_t SHT3X_ACCURACY_HIGH    = 0x2c06;

public:
  static const uint8_t SHT3X_I2C_ADDRESS_44 = 0x44;

  SHT3xSensor(uint8_t i2cAddress = SHT3X_I2C_ADDRESS_44)
      : SHTI2cSensor(i2cAddress, SHT3X_ACCURACY_HIGH,
                     -45, 175, 65535, 100, 65535)
  {
  }

};

//
// class SHTSensor
//
const float SHTSensor::TEMPERATURE_INVALID = NAN;
const float SHTSensor::HUMIDITY_INVALID = NAN;

bool SHTSensor::init()
{
  if (mSensor != NULL) {
    cleanup();
  }
  mSensor = new SHT3xSensor();
  return (mSensor != NULL);
}

bool SHTSensor::readSample()
{
  if (!mSensor || !mSensor->readSample())
    return false;
  mTemperature = mSensor->mTemperature;
  mHumidity = mSensor->mHumidity;
  return true;
}

void SHTSensor::cleanup()
{
  if (mSensor) {
    delete mSensor;
    mSensor = NULL;
  }
}
