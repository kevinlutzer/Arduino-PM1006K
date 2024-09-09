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

  /*!
    *  @brief class constructor for using hardware or software serial
    *  @param stream is the serial instance to use. Stream can be either of
    *  type HardwareSerial or Software Serial. The  
  */
  PM1006K(Stream *stream);

  /*!
    * @brief takes a measurement for PM2.5, PM1.0, and PM10 concentrations
    * @return true if successful, false if otherwise
  */
  bool takeMeasurement();

  /*!
    * @brief returns the last measured PM2.5 reading in ug/(m^3). 
    * @returns -1 if there has been no measurement taken or the value
    * from the last successful measurement.
  */
  int getPM2_5();

  /*!
    * @brief returns the last measured PM1.0 reading in ug/(m^3). 
    * @returns -1 if there has been no measurement taken or the value
    * from the last successful measurement.
  */
  int getPM1_0();

  /*!
    * @brief returns the last measured PM10 reading in ug/(m^3). 
    * @returns -1 if there has been no measurement taken or the value
    * from the last successful measurement.
  */
  int getPM10();

private:

  Stream *_stream = NULL;

  /*!
    * @brief sends a command to the PM1006K sensor and reads the returned data. This command will fail
    * and timeout if the sensor does not 
    * @param txBuf a buffer that contains the data to be sent to the sensor. 
    * @param txLen the length of the txBuf buffer.
    * @param rxBuf a buffer that will be filled with data that is return
    * @param rxLen the length of the rxBuf buffer.
    * @return true on success and false otherwise.
  */
  bool sendCMD(const unsigned char * txBuf, uint8_t txLen, unsigned char * rxBuf, uint8_t rxLen);

  /*!
    * @brief checks to see if the buffer paramter contains the correct start bytes
    * to signify a valid measurement frame.
    * @param buf is the data to be checked
  */
  bool isValidMeasurementFrame(uint8_t * buf);

  int _lastPM2_5 = -1;
  int _lastPM1_0 = -1;
  int _lastPM10 = -1;  
};

#endif
