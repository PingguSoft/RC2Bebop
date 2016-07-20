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
#include <avr/pgmspace.h>
#include "Common.h"
#include "pitches.h"

class Sound
{
private:
    void _play(u16 *note, u8 size) {
        mNotePtr  = note;
        mNoteSize = size / (sizeof(u16) * 2);
        mNoteIdx  = 0;
        mNextTime = 0;
        mLastTS = 0;
        mBoolPlaying = TRUE;
        handleSound();
    }

public:
    Sound(u8 pin) {
        mPin = pin;
        pinMode(mPin, OUTPUT);
        mBoolPlaying = FALSE;
    }

    void handleSound() {
        if (!mBoolPlaying)
            return;

        u32 diff = millis() - mLastTS;
        if (diff >= mNextTime) {
            if (mNoteIdx < mNoteSize) {
                noTone(mPin);

                u16 note;
                u16 duration;
                if (mFlash) {
                    note = pgm_read_word(mNotePtr + mNoteIdx * 2);
                    duration = pgm_read_word(mNotePtr + mNoteIdx * 2 + 1);
                } else {
                    note = mNotePtr[mNoteIdx * 2];
                    duration = mNotePtr[mNoteIdx * 2 + 1];
                }

                if (note == 0xffff && duration == 0xffff) {
                    mNoteIdx = 0;
                } else {
                    tone(mPin, note, duration);
                    mNextTime = duration * 1.30;
                    mLastTS = millis();
                    mNoteIdx++;
                }
            } else {
                noTone(mPin);
                mBoolPlaying = FALSE;
            }
        }
    }

    void play(const u16 *note, u8 size) {
        mFlash  = TRUE;
        _play((u16*)note, size);
    }

    void play(u16 *note, u8 size) {
        mFlash  = FALSE;
        _play((u16*)note, size);
    }

    void stop(void) {
        noTone(mPin);
        mBoolPlaying = FALSE;
    }

private:
    u8   mPin;

    bool mFlash;
    bool mBoolPlaying;
    u32  mLastTS;
    u32  mNextTime;

    u16  *mNotePtr;
    u8   mNoteSize;
    u16  mNoteIdx;
};

#endif
