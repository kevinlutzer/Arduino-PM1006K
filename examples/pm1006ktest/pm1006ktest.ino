/**
 * PM1006K Test
 * Author: Kevin Lutzer
*/

#include <PM1006K.h>
#include <HardwareSerial.h>

#define PM1006K_RX_PIN 4
#define PM1006K_TX_PIN 5
#define PM1006K_FAN_PIN 9

#define SAMPLE_RATE 2000 // ms

PM1006K * pm1006k;

void setup() {
    // Setup and turn on fan
    pinMode(PM1006K_FAN_PIN, OUTPUT);
    digitalWrite(PM1006K_FAN_PIN, HIGH);

    // Setup the serial logger
    Serial.begin(115200);

    // Setup and create instance of the PM1006K driver
    // The baud rate for the serial connection must be PM1006K::BAUD_RATE.
    Serial1.begin(PM1006K::BAUD_RATE, SERIAL_8N1, PM1006K_RX_PIN, PM1006K_TX_PIN);
    pm1006k = new PM1006K(&Serial1);
}

void loop() {
    if(!pm1006k->takeMeasurement()) {
        Serial.println("Failed to take measurement");
    } else {
        Serial.print("PM2.5 = ");
        Serial.print(pm1006k->getPM2_5());
        Serial.println(" μg/m³");

        Serial.print("PM1.0 = ");
        Serial.print(pm1006k->getPM1_0());
        Serial.println(" μg/m³");

        Serial.print("PM10 = ");
        Serial.print(pm1006k->getPM10());
        Serial.println(" μg/m³");
    }
    
    Serial.println();
    
    delay(SAMPLE_RATE);
}