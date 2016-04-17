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

#ifndef _UTILS_H_
#define _UTILS_H_
#include <Arduino.h>
#include <stdarg.h>
#include "Common.h"

// Bit vector from bit position
#define BV(bit) (1 << (bit))

#define PRINT_FUNC  //Serial.printf("-------------------------> %s\n", __func__)

class Utils
{
public:
    static inline int put8(u8 *buf, u8 v)
    {
        buf[0] = v;
        return sizeof(v);
    }

    static inline int put16(u8 *buf, u16 v)
    {
        buf[0] = v & 0xff;
        buf[1] = (v >> 8) & 0xff;
        return sizeof(u16);
    }

    static inline int put32(u8 *buf, u32 v)
    {
        buf[0] = v & 0xff;
        buf[1] = (v >>  8) & 0xff;
        buf[2] = (v >> 16) & 0xff;
        buf[3] = (v >> 24) & 0xff;
        return sizeof(u32);
    }

    static inline u8 get8(u8 *buf)
    {
        return buf[0];
    }

    static inline u16 get16(u8 *buf)
    {
        u16 v;

        v  = buf[1] << 8;
        v |= buf[0];

        return v;
    }

    static inline u32 get32(u8 *buf)
    {
        u32 v;

        v  = (buf[3] << 24);
        v |= (buf[2] << 16);
        v |= (buf[1] << 8);
        v |= (buf[0]);

        return v;
    }

    static inline int putfloat(u8 *buf, float v)
    {
        memcpy(buf, &v, sizeof(float));
        return sizeof(float);
    }

    static inline float getfloat(u8 *buf)
    {
        float v;

        memcpy(&v, buf, sizeof(float));
        return v;
    }

    static inline int putdouble(u8 *buf, double v)
    {
        memcpy(buf, &v, sizeof(double));
        return sizeof(double);
    }

    static inline double getdouble(u8 *buf)
    {
        double v;

        memcpy(&v, buf, sizeof(double));
        return v;
    }

    static inline int putlonglong(u8 *buf, unsigned long long v)
    {
        memcpy(buf, &v, sizeof(unsigned long long));
        return sizeof(unsigned long long);
    }

    static inline int putstr(u8 *buf, char *v)
    {
        int len = strlen(v) + 1;
        memcpy(buf, v, len);
        return len;
    }

    static void dump(u8 *data, u16 cnt);
    static void printf(char *fmt, ... );

    static char *ftoa(char *s, float n);
    static char *dtoa(char *s, double n);

private:

    int     mBufIdx;
    u8      *mBuf;

};


#endif
