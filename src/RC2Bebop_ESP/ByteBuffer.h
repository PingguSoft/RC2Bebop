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

#ifndef _BYTEBUFFER_H_
#define _BYTEBUFFER_H_
#include <Arduino.h>
#include "Common.h"


class ByteBuffer
{
public:
    ByteBuffer(u8 *buf, int size)
    {
        mBuf = buf;
        mBufIdx = 0;
        mBufSize = size;
    }

    inline void reset(void)
    {
        mBufIdx = 0;
    }

    inline void put8(u8 v)
    {
        mBuf[mBufIdx++] = v;
    }

    inline void put16(u16 v)
    {
        mBuf[mBufIdx++] = v & 0xff;
        mBuf[mBufIdx++] = (v >> 8) & 0xff;
    }

    inline void put32(u32 v)
    {
        mBuf[mBufIdx++] = v & 0xff;
        mBuf[mBufIdx++] = (v >>  8) & 0xff;
        mBuf[mBufIdx++] = (v >> 16) & 0xff;
        mBuf[mBufIdx++] = (v >> 24) & 0xff;
    }

    inline u8 get8(void)
    {
        return mBuf[mBufIdx++];
    }

    inline u16 get16(void)
    {
        u16 v;

        v  = mBuf[mBufIdx + 1] << 8;
        v |= mBuf[mBufIdx];
        mBufIdx += 2;

        return v;
    }

    inline u32 get32(void)
    {
        u32 v;

        v  = (mBuf[mBufIdx + 3] << 24);
        v |= (mBuf[mBufIdx + 2] << 16);
        v |= (mBuf[mBufIdx + 1] << 8);
        v |= (mBuf[mBufIdx]);
        mBufIdx += 4;

        return v;
    }

    inline void putfloat(float v)
    {
        memcpy(&mBuf[mBufIdx], &v, sizeof(float));
        mBufIdx += sizeof(float);
    }

    inline float getfloat(void)
    {
        float v;

        memcpy(&v, &mBuf[mBufIdx], sizeof(float));
        mBufIdx += sizeof(float);
        return v;
    }

    inline void putdouble(double v)
    {
        memcpy(&mBuf[mBufIdx], &v, sizeof(double));
        mBufIdx += sizeof(double);
    }

    inline double getdouble(void)
    {
        double v;

        memcpy(&v, &mBuf[mBufIdx], sizeof(double));
        mBufIdx += sizeof(double);
        return v;
    }

    inline void putstr(char *v)
    {
        int len = strlen(v) + 1;
        memcpy(&mBuf[mBufIdx], v, len);
        mBufIdx += len;
    }

    inline char *getstr(void)
    {
        int i;
        for (i = 0; i < mBufSize; i++) {
            if (!mBuf[mBufIdx + i])
                break;
        }

        char *ptr = (char*)&mBuf[mBufIdx];
        mBufIdx += i;
        return ptr;
    }

private:
    int     mBufIdx;
    int     mBufSize;
    u8      *mBuf;

};


#endif
