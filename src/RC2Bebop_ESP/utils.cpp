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
#include <Arduino.h>
#include <stdarg.h>
#include <Math.h>
#include "common.h"
#include "utils.h"

void Utils::dump(u8 *data, u16 cnt)
{
    u8  i;
    u8  b;
    u16 addr = 0;

//    Serial.printf("-- buf size : %d -- \n", cnt);
    while (cnt) {
        Serial.printf("%08x - ", addr);

        for (i = 0; (i < 16) && (i < cnt); i ++) {
            b = *(data + i);
            Serial.printf("%02x ", b);
        }

        Serial.printf(" : ");
        for (i = 0; (i < 16) && (i < cnt); i ++) {
            b = *(data + i);
            if ((b > 0x1f) && (b < 0x7f))
                Serial.printf("%c", b);
            else
                Serial.printf(".");
        }
        Serial.printf("\n");
        data += i;
        addr += i;
        cnt  -= i;
    }
}


void Utils::printf(char *fmt, ... )
{
#if 1
    char buf[256];
    va_list args;

    va_start (args, fmt );
    vsnprintf(buf, 256, fmt, args);
    va_end (args);
    Serial.print(buf);
#endif
}

/*
int Utils::power(int base, int exp){
    int result = 1;
    while(exp) { result *= base; exp--; }
    return result;
}

char* Utils::ftoa(float num, uint8_t decimals) {
  // float to string; no float support in esp8266 sdk printf
  // warning: limited to 15 chars & non-reentrant
  // e.g., dont use more than once per os_printf call
  char buf[32];
  int whole = num;
  int decimal = (num - whole) * power(10, decimals);
  if (decimal < 0) {
    // get rid of sign on decimal portion
    decimal -= 2 * decimal;
  }

  char pattern[32];
  sprintf(pattern, "%%d.%%0%dd", decimals);
  sprintf(buf, pattern, whole, decimal);
  return (char *)buf;
}

char* Utils::ftoa(double num, uint8_t decimals) {
  // float to string; no float support in esp8266 sdk printf
  // warning: limited to 15 chars & non-reentrant
  // e.g., dont use more than once per os_printf call
  char buf[32];
  int whole = num;
  int decimal = (num - whole) * power(10, decimals);
  if (decimal < 0) {
    // get rid of sign on decimal portion
    decimal -= 2 * decimal;
  }

  char pattern[32];
  sprintf(pattern, "%%d.%%0%dd", decimals);
  sprintf(buf, pattern, whole, decimal);
  return (char *)buf;
}
*/

static double PRECISIOND = 0.00000000000001;
static float  PRECISIONF = 0.001;
static int    MAX_NUMBER_STRING_SIZE = 32;

char* Utils::dtoa(char *s, double n) {
    // handle special cases
    if (isnan(n)) {
        strcpy(s, "nan");
    } else if (isinf(n)) {
        strcpy(s, "inf");
    } else if (n == 0.0) {
        strcpy(s, "0");
    } else {
        int digit, m, m1;
        char *c = s;
        int neg = (n < 0);
        if (neg)
            n = -n;
        // calculate magnitude
        m = log10(n);
        int useExp = (m >= 14 || (neg && m >= 9) || m <= -9);
        if (neg)
            *(c++) = '-';
        // set up for scientific notation
        if (useExp) {
            if (m < 0)
               m -= 1.0;
            n = n / pow(10.0, m);
            m1 = m;
            m = 0;
        }
        if (m < 1.0) {
            m = 0;
        }
        // convert the number
        while (n > PRECISIOND || m >= 0) {
            double weight = pow(10.0, m);
            if (weight > 0 && !isinf(weight)) {
                digit = floor(n / weight);
                n -= (digit * weight);
                *(c++) = '0' + digit;
            }
            if (m == 0 && n > 0)
                *(c++) = '.';
            m--;
        }
        if (useExp) {
            // convert the exponent
            int i, j;
            *(c++) = 'e';
            if (m1 > 0) {
                *(c++) = '+';
            } else {
                *(c++) = '-';
                m1 = -m1;
            }
            m = 0;
            while (m1 > 0) {
                *(c++) = '0' + m1 % 10;
                m1 /= 10;
                m++;
            }
            c -= m;
            for (i = 0, j = m-1; i<j; i++, j--) {
                // swap without temporary
                c[i] ^= c[j];
                c[j] ^= c[i];
                c[i] ^= c[j];
            }
            c += m;
        }
        *(c) = '\0';
    }
    return s;
}


char* Utils::ftoa(char *s, float n) {
    // handle special cases
    if (isnan(n)) {
        strcpy(s, "nan");
    } else if (isinf(n)) {
        strcpy(s, "inf");
    } else if (n == 0.0) {
        strcpy(s, "0");
    } else {
        int digit, m, m1;
        char *c = s;
        int neg = (n < 0);
        if (neg)
            n = -n;
        // calculate magnitude
        m = log10(n);
        int useExp = (m >= 14 || (neg && m >= 9) || m <= -9);
        if (neg)
            *(c++) = '-';
        // set up for scientific notation
        if (useExp) {
            if (m < 0)
               m -= 1.0;
            n = n / pow(10.0, m);
            m1 = m;
            m = 0;
        }
        if (m < 1.0) {
            m = 0;
        }
        // convert the number
        while (n > PRECISIONF || m >= 0) {
            double weight = pow(10.0, m);
            if (weight > 0 && !isinf(weight)) {
                digit = floor(n / weight);
                n -= (digit * weight);
                *(c++) = '0' + digit;
            }
            if (m == 0 && n > 0)
                *(c++) = '.';
            m--;
        }
        if (useExp) {
            // convert the exponent
            int i, j;
            *(c++) = 'e';
            if (m1 > 0) {
                *(c++) = '+';
            } else {
                *(c++) = '-';
                m1 = -m1;
            }
            m = 0;
            while (m1 > 0) {
                *(c++) = '0' + m1 % 10;
                m1 /= 10;
                m++;
            }
            c -= m;
            for (i = 0, j = m-1; i<j; i++, j--) {
                // swap without temporary
                c[i] ^= c[j];
                c[j] ^= c[i];
                c[i] ^= c[j];
            }
            c += m;
        }
        *(c) = '\0';
    }
    return s;
}