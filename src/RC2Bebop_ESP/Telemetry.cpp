/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Deviation is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Deviation.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "Common.h"
#include "Telemetry.h"
#include "utils.h"

Telemetry::Telemetry()
{
    mUpdateMask = 0;
    memset(&mVolt, 0, sizeof(mVolt));
    memset(&mTemp, 0, sizeof(mTemp));
    memset(&mRPM, 0, sizeof(mRPM));
    mRSSI = 0;
    mBattCap = 0;
    memset(&mGPS, 0, sizeof(mGPS));
}

Telemetry::~Telemetry()
{

}

//0[00] 18(0x12)
//1[01] 00
//2[02] Altitude MSB (Hex)
//3[03] Altitude LSB (Hex) 16bit signed integer, in 0.1m
//4[04] Max Altitude MSB (Hex)
//5[05] Max Altitude LSB (Hex) 16bit signed integer, in 0.1m
//6[05] Unknown
//7[07] Unknown
//8[08] Unknown
//9[09] Unknown
//10[0A] Unknown
//11[0B] Unknown
//12[0C] Unknown
//13[0D] Unknown
//14[0E] Unknown
//15[0F] Unknown
u8 Telemetry::buildAltInfo(u8 *buf)
{
    static u16 max_alt = 0;
    u16 alt = mBaroAlt / 10;

    if (alt > max_alt)
        max_alt = alt;

    memset(buf, 0, 18);
    buf[0] = 0x12;
    buf[1] = 0x00;
    buf[2] = (alt >> 8) & 0xff;
    buf[3] = alt & 0xff;
    buf[4] = (max_alt >> 8) & 0xff;
    buf[5] = max_alt & 0xff;

    return 18;
}

// 0 [00] 0x0A
// 1 [01] 00
// 2 [02] V1 MSB (Hex)
// 3 [03] V1 LSB (Hex) //In 0.01V
// 4 [04] V2 MSB (Hex)
// 5 [05] V2 LSB (Hex) //In 0.01V
// 6 [06] Cap1 MSB (Hex)
// 7 [07] Cap1 LSB (Hex) //In 1mAh
// 8 [08] Cap2 MSB (Hex)
// 9 [09] Cap2 LSB (Hex) //In 1mAh
//10 [0A] 00
//11 [0B] 00
//12 [0C] 00
//13 [0D] 00
//14 [0E] 00
//15 [0F] Alarm // The fist bit is alarm V1, the second V2, the third Capacity 1, the 4th capacity 2.

u8 Telemetry::buildPowerInfo(u8 *buf)
{
    u8  idx = 0;
    u16 mV;

    if (getVolt(idx) == 0) {
        idx++;
    }
    memset(buf, 0, 18);
    buf[0] = 0x0A;
    buf[1] = 0x00;

    mV     = getVolt(idx) / 10;         // 0.01V
    buf[2] = (mV >> 8) & 0xff;
    buf[3] = mV & 0xff;

    mV     = getVolt(idx + 1) / 10;
    buf[4] = (mV >> 8) & 0xff;
    buf[5] = mV & 0xff;
    buf[6] = 0x08;                      // ex:2200mAh
    buf[7] = 0x98;
    buf[8] = 0x08;
    buf[9] = 0x98;

    return 18;
}


//0[00] 22(0x16)
//1[01] 00
//2[02] Altitude LSB (Decimal) //In 0.1m
//3[03] Altitude MSB (Decimal)
//4[04] 1/100th of a degree second latitude (Decimal) (XX YY.SSSS)
//5[05] degree seconds latitude (Decimal)
//6[06] degree minutes latitude (Decimal)
//7[07] degrees latitude (Decimal)
//8[08] 1/100th of a degree second longitude (Decimal) (XX YY.SSSS)
//9[09] degree seconds longitude (Decimal)
//10[0A] degree minutes longitude (Decimal)
//11[0B] degrees longitude (Decimal)
//12[0C] Heading LSB (Decimal)
//13[0D] Heading MSB (Decimal) Divide by 10 for Degrees
//14[0E] Unknown
//15[0F] First bit for latitude: 1=N(+), 0=S(-);
//Second bit for longitude: 1=E(+), 0=W(-);
//Third bit for longitude over 99 degrees: 1=+-100 degrees
u8 Telemetry::buildGPSInfo(u8 *buf)
{
    memset(buf, 0, 18);
    buf[0] = 0x16;
    buf[1] = 0x00;
    buf[2] = (getGPS()->altitude >> 8) & 0xff;
    buf[3] = getGPS()->altitude & 0xff;

    s32 lat = getGPS()->latitude;
    buf[7] =  lat / 3600000;     // hour
    lat    =  lat % 3600000;
    buf[6] =  lat / 60000;       // min
    lat    =  lat % 60000;
    buf[5] =  lat / 6;           // sec
    lat    =  lat % 6;
    buf[4] =  lat;               // 1/100

    s32 lon = getGPS()->longitude;
    buf[11] = lon / 3600000;   // hour
    lon     = lon % 3600000;
    buf[10] = lon / 60000;     // min
    lon     = lon % 60000;
    buf[9]  = lon / 6;          // sec
    lon     = lon % 6;
    buf[8] =  lon;              // 1/100

    buf[12] = getGPS()->heading & 0xff;
    buf[13] = (getGPS()->heading >> 8) & 0xff;

    buf[15] = 0xC0;

    return 18;
}


//0[00] 7E or FE
//1[01] 00
//2[02] RPM MSB (Hex)
//3[03] RPM LSB (Hex) //RPM = 120000000 / number_of_poles(2, 4, ... 32) / gear_ratio(0.01 - 30.99) / Value
//4[04] Volt MSB (Hex)
//5[05] Volt LSB (Hex) //In 0.01V
//6[06] Temp MSB (Hex)
//7[07] Temp LSB (Hex) //Value (Decimal) is in Fahrenheit, for Celsius (Value (Decimal) - 32) * 5) / 9)
//8[08] Unknown
//9[09] Unknown
//10[0A] Unknown
//11[0B] Unknown
//12[0C] Unknown
//13[0D] Unknown	// Assan when 7E type is Rx RSSI
//14[0E] Unknown
//15[0F] Unknown
u8 Telemetry::buildTMInfo(u8 *buf)
{
    u8  idx = 0;

    if (getVolt(idx) == 0) {
        idx++;
    }

    memset(buf, 0, 18);
    buf[0] = 0x7E;
    buf[1] = 0x00;
    buf[2] = (getRPM(0) >> 8) & 0xff;
    buf[3] = getRPM(0)  & 0xff;

    u16 volt = getVolt(idx) / 10;
    buf[4] = (volt >> 8) & 0xff;
    buf[5] = volt & 0xff;

    buf[6] = (getTemp(idx) >> 8) & 0xff;
    buf[7] = getTemp(idx) & 0xff;

    return 18;
}


//0[00] 23(0x17)
//1[01] 00
//2[02] Speed LSB (Decimal)
//3[03] Speed MSB (Decimal) Divide by 10 for Knots. Multiply by 0.185 for Kph and 0.115 for Mph
//4[04] UTC Time LSB (Decimal) 1/100th sec. (HH:MM:SS.SS)
//5[05] UTC Time (Decimal) = SS
//6[06] UTC Time (Decimal) = MM
//7[07] UTC Time MSB (Decimal) = HH
//8[08] Number of Sats (Decimal)
//9[09] Altitude in 1000m (Decimal) Altitude = Value * 10000 + Altitude(0x16) (in 0.1m)
//10[0A]-15[0F] Unused (But contains Data left in buffer)


//Data type = 0x03 High Current Sensor
//0 [00] 03
//1 [01] 00
//2 [02] MSB (Hex) //16bit signed integer
//3 [03] LSB (Hex) //In 0.196791A
//4 [04] 00
//5 [05] 00
//6 [06] 00
//7 [07] 00
//8 [08] 00
//9 [09] 00
//10 [0A] 00
//11 [0B] 00
//12 [0C] 00
//13 [0D] 00
//14 [0E] 00
//15 [0F] 00


//Data type = 0x0A PowerBox Sensor
//0 [00] 0x0A
//1 [01] 00
//2 [02] V1 MSB (Hex)
//3 [03] V1 LSB (Hex) //In 0.01V
//4 [04] V2 MSB (Hex)
//5 [05] V2 LSB (Hex) //In 0.01V
//6 [06] Cap1 MSB (Hex)
//7 [07] Cap1 LSB (Hex) //In 1mAh
//8 [08] Cap2 MSB (Hex)
//9 [09] Cap2 LSB (Hex) //In 1mAh
//10 [0A] 00
//11 [0B] 00
//12 [0C] 00
//13 [0D] 00
//14 [0E] 00
//15 [0F] Alarm // The fist bit is alarm V1, the second V2, the third Capacity 1, the 4th capacity 2.


//Data type = 0x11 AirSpeed Sensor
//0[00] 17(0x11)
//1[01] 00
//2[02] Speed MSB (Hex)
//3[03] Speed LSB (Hex) //In 1 km/h
//4[04] Unknown
//5[05] Unknown
//6[06] Unknown
//7[07] Unknown
//8[08] Unknown
//9[09] Unknown
//10[0A] Unknown
//11[0B] Unknown
//12[0C] Unknown
//13[0D] Unknown
//14[0E] Unknown
//15[0F] Unknown


//Data type = 0x14 Gforce Sensor
//0[00] 20(0x14)
//1[01] 00
//2[02] x MSB (Hex, signed integer)
//3[03] x LSB (Hex, signed integer) //In 0.01g
//4[04] y MSB (Hex, signed integer)
//5[05] y LSB (Hex, signed integer) //In 0.01g
//6[06] z MSB (Hex, signed integer)
//7[07] z LSB (Hex, signed integer) //In 0.01g
//8[08] x max MSB (Hex, signed integer)
//9[09] x max LSB (Hex, signed integer) //In 0.01g
//10[0A] y max MSB (Hex, signed integer)
//11[0B] y max LSB (Hex, signed integer) //In 0.01g
//12[0C] z max MSB (Hex, signed integer)
//13[0D] z max LSB (Hex, signed integer) //In 0.01g
//14[0E] z min MSB (Hex, signed integer)
//15[0F] z min LSB (Hex, signed integer) //In 0.01g


//Data type = 0x15 JetCat Sensor
//0[00] 21(0x15)
//1[01] 00
//2[02] Status
//3[03] Throttle //up to 159% (the upper nibble is 0-f, the lower nibble 0-9)
//4[04] Pack_Volt LSB (Decimal)
//5[05] Pack_Volt MSB (Decimal) //In 0.01V
//6[06] Pump_Volt LSB (Decimal)
//7[07] Pump_Volt MSB (Decimal) //In 0.01V
//8[08] RPM LSB (Decimal) //Up to 999999rpm
//9[09] RPM Mid (Decimal)
//10[0A] RPM MSB (Decimal)
//11[0B] 00
//12[0C] TempEGT (Decimal) LSB
//13[0D] TempEGT (Decimal) MSB //used only lover nibble, up to 999°C
//14[0E] Off_Condition
//15[0F] 00


//Data type = 7F{TM1000} or FF{TM1100}
//0[00] 7F or FF
//1[01] 00
//2[02] A MSB (Hex)
//3[03] A LSB (Hex)
//4[04] B MSB (Hex)
//5[05] B LSB (Hex)
//6[06] L MSB (Hex)
//7[07] L LSB (Hex) //0xFFFF = NC (not connected)
//8[08] R MSB (Hex)
//9[09] R LSB (Hex) //0xFFFF = NC (not connected)
//10[0A] Frame loss MSB (Hex)
//11[0B] Frame loss LSB (Hex)
//12[0C] Holds MSB (Hex)
//13[0D] Holds LSB (Hex)
//14[0E] Receiver Volts MSB (Hex) //In 0.01V
//15[0F] Receiver Volts LSB (Hex)

//answer from TX module:
//header: 0xAA
//0x00 - no telemetry
//0x01 - telemetry packet present and telemetry block (TM1000, TM1100)
//answer in 16 bytes body from
//http://www.deviationtx.com/forum/protocol-development/1192-dsm-telemetry-support?start=60
//..
//0x1F - this byte also used as RSSI signal for receiving telemetry
//block. readed from CYRF in a TX module
//0x80 - BIND packet answer, receiver return his type
//aa bb cc dd ee......
//aa ORTX_USExxxx from main.h
//bb
//cc - max channel, AR6115e work only in 6 ch mode (0xA2)
//0xFF - sysinfo
//aa.aa.aa.aa - CYRF manufacturer ID
//bb.bb - firmware version
//	if(ortxRxBuffer[1] == 0x80){//BIND packet answer
//		ortxMode = ortxRxBuffer[2];
//		if(ortxMode & ORTX_USE_DSMX)
//			g_model.ppmNCH = DSM2_DSMX;
//		else
//			g_model.ppmNCH = DSM2only;
//		//ortxTxBuffer[3];//power
//		g_model.unused1[0] = ortxRxBuffer[4];//channels number

void Telemetry::frameDSM(u8 rssi, u8 *buf, u8 size)
{
    Serial1.write(0xAA);
    Serial1.write(rssi);
    for (u8 i = 0; i < size; i++)
        Serial1.write(buf[i]);
}

void Telemetry::update(void)
{
    u8 size;
    u8 rssi;

    if (isMasked(MASK_RSSI)) {
        rssi = getRSSI() >> 3;
    } else {
        rssi = 0x1;
    }

    if (isMasked(MASK_VOLT)) {
        LOG("VOLT - V1:%d, V2:%d, V3:%d\n", mVolt[0], mVolt[1], mVolt[2]);
        size = buildPowerInfo(mTeleBuf);
        frameDSM(rssi, mTeleBuf, size);
        clearMask(MASK_VOLT);
    }

    if (isMasked(MASK_TEMP)) {
        LOG("TEMP - T1:%d, T2:%d, T3:%d, T4:%d\n", mTemp[0], mTemp[1], mTemp[2], mTemp[3]);
        size = buildTMInfo(mTeleBuf);
        frameDSM(rssi, mTeleBuf, size);
        clearMask(MASK_TEMP);
    }

    if (isMasked(MASK_RPM)) {
        LOG("RPM  - R1:%d, R2:%d, R3:%d\n", mRPM[0], mRPM[1], mRPM[2]);
        size = buildTMInfo(mTeleBuf);
        frameDSM(rssi, mTeleBuf, size);
        clearMask(MASK_RPM);
    }

    if (isMasked(MASK_BARO_ALT)) {
        LOG("ALT  - %d\n", mBaroAlt);
        size = buildAltInfo(mTeleBuf);
        frameDSM(rssi, mTeleBuf, size);
        clearMask(MASK_BARO_ALT);
    }

    if (isMasked(MASK_VELOCITY)) {

    }

    if (isMasked(MASK_GPS)) {
        size = buildGPSInfo(mTeleBuf);
        frameDSM(rssi, mTeleBuf, size);
        clearMask(MASK_GPS);
    }
}

u8 Telemetry::handleTX(u8 *data)
{
    return 0;
}

