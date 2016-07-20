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
#include "SerialProtocol.h"
#include "RCRcvrPWM.h"
#include "RCRcvrPPM.h"
#include "Sound.h"

//#define __DEBUG__   1

#define DUR_2   (1000 / 2)
#define DUR_4   (1000 / 4)
#define DUR_8   (1000 / 8)


static const u16 NOTE1[] PROGMEM = {
    NOTE_C4, DUR_4, NOTE_G3, DUR_8, NOTE_G3, DUR_8, NOTE_A3, DUR_4, NOTE_G3, DUR_4, NOTE_NON, DUR_4, NOTE_B3, DUR_4, NOTE_C4, DUR_4
};

static const u16 NOTE_START[] PROGMEM = {
    NOTE_C4, DUR_4, NOTE_E3, DUR_4, NOTE_G3, DUR_4, NOTE_C5, DUR_2
};

enum {
    STATE_INIT = 0,
    STATE_AP_CONNECT,
    STATE_DISCOVERY,
    STATE_DISCOVERY_ACK,
    STATE_CONFIG,
    STATE_WORK,
};

#define PIN_SOUND   A3
#define PIN_LED1    A0
#define PIN_LED2    A1
#define PIN_LED3    A2

#define FW_VERSION  0x0120

static Sound            mSound(PIN_SOUND);
static SerialProtocol   mSerial;
static RCRcvrPPM       *mRcvr = NULL;
static u16              mNote[32];

static void showLED(u8 color)
{
    digitalWrite(PIN_LED1, color & 0x01);
    digitalWrite(PIN_LED2, color & 0x02);
    digitalWrite(PIN_LED3, color & 0x04);
}

static void initReceiver(void)
{
    // receiver
    if (mRcvr) {
        mRcvr->close();
        delete mRcvr;
        mRcvr = NULL;
    }
    mRcvr = new RCRcvrPPM();
    if (mRcvr) {
        mRcvr->init();
    }
}

u32 serialCallback(u8 cmd, u8 *data, u8 size)
{
    u32 id;
    u16 ram;
    u8  ret = 0;
    u8  buf[5];
    u8  sz = 0;

    switch (cmd) {
        case SerialProtocol::CMD_GET_VERSION:
            ram = FW_VERSION;
            mSerial.sendResponse(true, cmd, (u8*)&ram, sizeof(ram));
            break;

        case SerialProtocol::CMD_GET_FREE_RAM:
            ram = freeRam();
            mSerial.sendResponse(true, cmd, (u8*)&ram, sizeof(ram));
            break;

        case SerialProtocol::CMD_SET_STATE:
            showLED(*data);
            break;

        case SerialProtocol::CMD_PLAY_NOTE:
            memcpy(mNote, data, size);
            mSound.play(mNote, size);
            mSerial.sendString("PLAY : %d\n !!!", size);
            break;
    }
    return ret;
}

void setup()
{
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PIN_LED2, LOW);
    digitalWrite(PIN_LED3, LOW);

    mSerial.begin(57600);
    mSerial.setCallback(serialCallback);
    initReceiver();
    mSound.play(NOTE_START, sizeof(NOTE_START));

#ifdef __DEBUG__
    mSerial.sendString("READY !!!");
#endif
}

#ifdef __DEBUG__
static char buf[255];
static void handleKey(void)
{
    int size;

    size = mSerial.read((u8*)buf);
    if (size > 0) {
        u8 ch = (u8)buf[0];

        switch (ch) {
            case '1' : mSound.play(NOTE1, sizeof(NOTE1));
            break;

            case '2' : mSound.play(NOTE_START, sizeof(NOTE_START));
            break;
        }
    }
}
#endif

void loop()
{
#ifdef __DEBUG__
    if (mRcvr) {
        static u32 lastTS;
        u32 ts = millis();
        if (ts - lastTS > 200) {
            sprintf(buf, "T:%4d R:%4d E:%4d A:%4d %4d %4d %4d %4d\n", mRcvr->getRC(0), mRcvr->getRC(1), mRcvr->getRC(2), mRcvr->getRC(3), mRcvr->getRC(4),
                mRcvr->getRC(5), mRcvr->getRC(6), mRcvr->getRC(7), mRcvr->getRC(8));
            mSerial.sendString(buf);
            lastTS = ts;
        }
    }
    handleKey();
#else
    mSerial.handleRX();
    if (mRcvr) {
        static u32 lastTS;
        u32 ts = millis();
        if (ts - lastTS > 20) {
            mSerial.sendCmd(SerialProtocol::CMD_SET_RC, (u8*)mRcvr->getRCs(), mRcvr->getChCnt() * 2);
            lastTS = ts;
        }
    }
#endif
    mSound.handleSound();
}

int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
