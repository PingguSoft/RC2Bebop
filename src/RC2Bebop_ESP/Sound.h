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

#ifndef _SOUND_H_
#define _SOUND_H_

#include <Arduino.h>
#include <stdarg.h>
#include "SerialProtocol.h"
#include "pitches.h"

class Sound
{
public:

    Sound(SerialProtocol serial) {
        mSerial = serial;
     }

    ~Sound() { }

    void play(u16 *note, u8 size) {
        mSerial.sendCmd(SerialProtocol::CMD_PLAY_NOTE, (u8*)note, size);
    }

private:
    SerialProtocol  mSerial;
};

#endif
