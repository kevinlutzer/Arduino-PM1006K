/*!
 * @file PM1006.h
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
#ifndef __PM1006K_H__
#define __PM1006K_H__

#include "Stream.h"
#include "Arduino.h"

class PM1006K {

public:

  static const int BAUD_RATE = 9600;
  const int CMD_TIMEOUT = 1000;

  const uint8_t CMD_TAKE_MEASUREMENT_LEN = 5;
  const unsigned char CMD_TAKE_MEASUREMENT[5] = {0x11, 0x02, 0x0B, 0x01, 0xE1};
  
  // From the datasheet there is only one command that is avaliable to take the
  // appropriate readings. The response is 16 bytes long including the header
  const uint8_t CMD_TAKE_MEASUREMENT_RESP_LEN = 16;

  PM1006K(Stream *stream);
  ~PM1006K();

  bool takeMeasurement();

  int getPM2_5();
  int getPM1_0();
  int getPM10();

private:

  Stream *_stream = NULL;
  bool sendCMD(const unsigned char * txBuf, uint8_t txLen, unsigned char * rxBuf, uint8_t rxLen);
  bool isValidMeasurementFrame(uint8_t * buf);

  int _lastPM2_5 = -1;
  int _lastPM1_0 = -1;
  int _lastPM10 = -1;  
};

#endif
