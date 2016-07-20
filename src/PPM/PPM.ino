/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is derived from deviationTx project for Arduino.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 see <http://www.gnu.org/licenses/>
*/

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <EEPROM.h>

#include "common.h"
#include "utils.h"
#include "RCRcvrPWM.h"

#define PIN_BEAT    13
#define PIN_LED1    A0
#define PIN_LED2    A1
#define PIN_LED3    A2

static RCRcvrPWM mRcvr;

static void showLED(u8 color)
{
    digitalWrite(PIN_LED1, color & 0x01);
    digitalWrite(PIN_LED2, color & 0x02);
    digitalWrite(PIN_LED3, color & 0x04);
}

void setup()
{
    pinMode(PIN_BEAT, OUTPUT);
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);
    digitalWrite(PIN_BEAT, LOW);
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PIN_LED2, LOW);
    digitalWrite(PIN_LED3, LOW);
    Serial.begin(57600);
    mRcvr.init();
}

char buf[100];
u8   beatStatus = 0;

void loop()
{
     sprintf(buf, "%4d %4d %4d %4d %4d %4d %4d %4d\n", mRcvr.getRC(0), mRcvr.getRC(1), mRcvr.getRC(2), mRcvr.getRC(3), mRcvr.getRC(4),
        mRcvr.getRC(5), mRcvr.getRC(6), mRcvr.getRC(7), mRcvr.getRC(8));
     Serial.print(buf);

    beatStatus = !beatStatus;
    digitalWrite(PIN_BEAT, beatStatus);
    delay(500);
}

