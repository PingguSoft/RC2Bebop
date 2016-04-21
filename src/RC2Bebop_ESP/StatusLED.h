/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 see <http://www.gnu.org/licenses/>
*/

#ifndef _STATUS_LED_H_
#define _STATUS_LED_H_

#include <Arduino.h>
#include <stdarg.h>
#include "SerialProtocol.h"


class StatusLED
{
public:
    enum {
        LED_RED     = 1,
        LED_GREEN   = 2,
        LED_YELLOW  = 3,
        LED_BLUE    = 4,
        LED_PURPLE  = 5,
        LED_CYAN    = 6,
        LED_WHITE   = 7
    };

    StatusLED(SerialProtocol serial) { 
        mColor = 0; 
        mInterval = 0; 
        mLastTS = 0;
        mOn  = false;
        mSerial = serial;
     }
    
    ~StatusLED() { }

    void set(u8 color, u32 interval) {
        mColor    = color; 
        mInterval = interval;
        mLastTS   = millis();
        mOn       = true;
        mSerial.sendCmd(SerialProtocol::CMD_SET_STATE, &mColor, 1);
    }

    void process(void) {
        u32 ts = millis();

        if (mInterval == 0)
            return;

        if (ts - mLastTS >= mInterval) {
            mOn = !mOn;
            if (mOn) {
                mSerial.sendCmd(SerialProtocol::CMD_SET_STATE, &mColor, 1);
            } else {
                u8  color = 0;
                mSerial.sendCmd(SerialProtocol::CMD_SET_STATE, &color, 1);
            }
            mLastTS = ts;
        }
    }

private:
    u8      mColor;
    u32     mInterval;
    u32     mLastTS;
    bool    mOn;
    SerialProtocol  mSerial;
};

#endif
