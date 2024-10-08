/*!
 * @file PM1006.cpp
 *
 * @mainpage Arduino PM1006K library for PM1006K sensors
 *
 * @section intro_sec Introduction
 *
 * Driver for the PM1006K PM2.5 and PM1.0 sensor.
 *
 * The sensor uses UART to communicate with a 5 Volt Logic Level. The
 * It provides inhalable particulate matter measures for 2.5um, 1.0um and 10um
 * size particles particles.
 * 
 * Documentation for the sensor can be found on Cubic's website: 
 * https://en.gassensor.com.cn/Product_files/Specifications/LED%20Particle%20Sensor%20PM1006K%20Specification.pdf
 *
 * @section author Author
 *
 * Written by Kevin Lutzer
 *
 * @section license License
 *
 * Apache license.
 * See the LICENSE file for details.
 *
 */

#include "PM1006K.h"
#include "Stream.h"


PM1006K::PM1006K(Stream *stream) {
  _stream = stream;
}

int PM1006K::getPM2_5() {
  return this->_lastPM2_5;
}

int PM1006K::getPM1_0() {
  return this->_lastPM1_0;
}

int PM1006K::getPM10() {
  return this->_lastPM10;
}

bool PM1006K::takeMeasurement() {
  uint8_t rxBuf[CMD_TAKE_MEASUREMENT_RESP_LEN];
  if(!this->sendCMD(CMD_TAKE_MEASUREMENT, CMD_TAKE_MEASUREMENT_LEN, rxBuf, CMD_TAKE_MEASUREMENT_RESP_LEN)) {
    return false;
  }
  
  if(!this->isValidMeasurementFrame(rxBuf)) {
    return false;
  }

  /*
    From page 7 of the data sheet Response is in the format of:
    [0x16, 0x0D, 0x02, DF1, DF2, DF3, DF4, DF5, DF6, DF7, DF8, DF9, DF10, DF11, DF12, CS]
    
    PM2.5(μg/m³)= DF3*256 + DF4
    PM1.0(μg/m³) = DF7*256 + DF8
    PM10(μg/m³) = DF11*256 + DF12
  */
  this->_lastPM2_5 = (rxBuf[5] << 8) | rxBuf[6];
  this->_lastPM1_0 = (rxBuf[9] << 8) | rxBuf[10];
  this->_lastPM10 = (rxBuf[13] << 8) | rxBuf[14];

  return true;
}

bool PM1006K::isValidMeasurementFrame(uint8_t * buf) {
    return buf[0] == 0x16 && buf[1] == 0x11 && buf[2] == 0x0B;
}

bool PM1006K::sendCMD(const unsigned char * txBuf, uint8_t txLen, unsigned char * rxBuf, uint8_t rxLen) {

  // Clear the RX buffer
  while (this->_stream->available()) {
    this->_stream->read();
  }

  // Send the command to get the measurement
  this->_stream->write(txBuf, txLen);

  int i = 0;
  unsigned long start = millis();
  while (((millis() - start) < CMD_TIMEOUT) && i < rxLen) {
    while (this->_stream->available()) {
      rxBuf[i] = this->_stream->read();
      i++;
    }
    yield();
  }

  // Failed because of timeout
  if (i < rxLen) {
    return false;
  }

  return true;
}
