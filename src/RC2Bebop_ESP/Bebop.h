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

#ifndef _BEBOP_H_
#define _BEBOP_H_

#include <Arduino.h>
#include <stdarg.h>
#include <string.h>
#include "Common.h"
#include "Utils.h"

// http://robotika.cz/robots/katarina/en#150202
// https://github.com/robotika/katarina
// https://github.com/Parrot-Developers/libARCommands/blob/master/Xml/ARDrone3_commands.xml
// https://github.com/Parrot-Developers/libARCommands/blob/master/gen/Includes/libARCommands/ARCOMMANDS_Ids.h

enum {
    FRAME_TYPE_ACK = 0x1,
    FRAME_TYPE_DATA = 0x2,
    FRAME_TYPE_DATA_LOW_LATENCY = 0x3,
    FRAME_TYPE_DATA_WITH_ACK = 0x4,
};

enum {
    BUFFER_ID_PING = 0,
    BUFFER_ID_PONG = 1,

    BUFFER_ID_C2D_PCMD = 10,            // Non ack data (periodic commands for piloting and camera orientation)
    BUFFER_ID_C2D_SETTINGS = 11,        // Ack data (Events, settings ...)
    BUFFER_ID_C2D_EMERGENCY = 12,       // Emergency data (Emergency command only)
    BUFFER_ID_C2D_VID_ACK = 13,         // ARStream video acks
    
    BUFFER_ID_D2C_RPT = 127,            // Non ack data (periodic reports from the device)
    BUFFER_ID_D2C_ACK_SETTINGS = 126,   // Ack data (Events, settings ...)
    BUFFER_ID_D2C_VID = 125,            // ARStream video data
};

enum {
    PROJECT_ARDRONE3 = 1,
    PROJECT_ARDRONE3DEBUG = 129,
    PROJECT_JUMPINGSUMO = 3,
    PROJECT_JUMPINGSUMODEBUG = 131,
    PROJECT_MINIDRONE = 2,
    PROJECT_MINIDRONEDEBUG = 130,
    PROJECT_SKYCONTROLLER = 4,
    PROJECT_SKYCONTROLLERDEBUG = 132,
    PROJECT_COMMON = 0,
    PROJECT_COMMONDEBUG = 128,
    PROJECT_PRO = 7,
};

enum frameid {
    FRAME_NO_ACK  = 10,
    FRAME_ACK_REQ = 11,
};

enum {
    ARDRONE3_CLASS_PILOTING = 0,
    ARDRONE3_CLASS_ANIMATIONS = 5,
    ARDRONE3_CLASS_CAMERA = 1,
    ARDRONE3_CLASS_MEDIARECORD = 7,
    ARDRONE3_CLASS_MEDIARECORDSTATE = 8,
    ARDRONE3_CLASS_MEDIARECORDEVENT = 3,
    ARDRONE3_CLASS_PILOTINGSTATE = 4,
    ARDRONE3_CLASS_PILOTINGEVENT = 34,
    ARDRONE3_CLASS_NETWORK = 13,
    ARDRONE3_CLASS_NETWORKSTATE = 14,
    ARDRONE3_CLASS_PILOTINGSETTINGS = 2,
    ARDRONE3_CLASS_PILOTINGSETTINGSSTATE = 6,
    ARDRONE3_CLASS_SPEEDSETTINGS = 11,
    ARDRONE3_CLASS_SPEEDSETTINGSSTATE = 12,
    ARDRONE3_CLASS_NETWORKSETTINGS = 9,
    ARDRONE3_CLASS_NETWORKSETTINGSSTATE = 10,
    ARDRONE3_CLASS_SETTINGS = 15,
    ARDRONE3_CLASS_SETTINGSSTATE = 16,
    ARDRONE3_CLASS_DIRECTORMODE = 17,
    ARDRONE3_CLASS_DIRECTORMODESTATE = 18,
    ARDRONE3_CLASS_PICTURESETTINGS = 19,
    ARDRONE3_CLASS_PICTURESETTINGSSTATE = 20,
    ARDRONE3_CLASS_MEDIASTREAMING = 21,
    ARDRONE3_CLASS_MEDIASTREAMINGSTATE = 22,
    ARDRONE3_CLASS_GPSSETTINGS = 23,
    ARDRONE3_CLASS_GPSSETTINGSSTATE = 24,
    ARDRONE3_CLASS_CAMERASTATE = 25,
    ARDRONE3_CLASS_ANTIFLICKERING = 29,
    ARDRONE3_CLASS_ANTIFLICKERINGSTATE = 30,
    ARDRONE3_CLASS_GPSSTATE = 31,
    ARDRONE3_CLASS_PROSTATE = 32,
};

enum {
    COMMON_CLASS_NETWORK = 0,
    COMMON_CLASS_NETWORKEVENT = 1,
    COMMON_CLASS_SETTINGS = 2,
    COMMON_CLASS_SETTINGSSTATE = 3,
    COMMON_CLASS_COMMON = 4,
    COMMON_CLASS_COMMONSTATE = 5,
    COMMON_CLASS_OVERHEAT = 6,
    COMMON_CLASS_OVERHEATSTATE = 7,
    COMMON_CLASS_CONTROLLER = 8,
    COMMON_CLASS_WIFISETTINGS = 9,
    COMMON_CLASS_WIFISETTINGSSTATE = 10,
    COMMON_CLASS_MAVLINK = 11,
    COMMON_CLASS_MAVLINKSTATE = 12,
    COMMON_CLASS_CALIBRATION = 13,
    COMMON_CLASS_CALIBRATIONSTATE = 14,
    COMMON_CLASS_CAMERASETTINGSSTATE = 15,
    COMMON_CLASS_GPS = 16,
    COMMON_CLASS_FLIGHTPLANSTATE = 17,
    COMMON_CLASS_FLIGHTPLANEVENT = 19,
    COMMON_CLASS_ARLIBSVERSIONSSTATE = 18,
    COMMON_CLASS_AUDIO = 20,
    COMMON_CLASS_AUDIOSTATE = 21,
    COMMON_CLASS_HEADLIGHTS = 22,
    COMMON_CLASS_HEADLIGHTSSTATE = 23,
    COMMON_CLASS_ANIMATIONS = 24,
    COMMON_CLASS_ANIMATIONSSTATE = 25,
    COMMON_CLASS_ACCESSORY = 26,
    COMMON_CLASS_ACCESSORYSTATE = 27,
    COMMON_CLASS_CHARGER = 28,
    COMMON_CLASS_CHARGERSTATE = 29,
    COMMON_CLASS_RUNSTATE = 30,
};


#define PACK_CMD(prj, cls, cmd) ((prj << 24) | (cls << 16) | (cmd))
#define PACK_PRJ_CLS(prj, cls)  ((prj << 24) | (cls << 16))

#define GET_PRJ_CLS(id)         ((id) & 0xffff0000)
#define GET_PRJ(id)             ((id >> 24) & 0xff)
#define GET_CLS(id)             ((id >> 16) & 0xff)
#define GET_CMD(id)             ((id) & 0xffff)

class Bebop {

public:
//    0          1       2      3 4 5 6       7              8        9 10     11
// frametype, frameid, seqid, payloadlen+7    payload...
//                                            prj,           cls,     cmd,     args

    static int buildCmd(u8 *buf, u8 ft, u8 fi, const char *fmt, ...)
    {
        u8      vu8;
        u16     vu16;
        u32     vu32;
        float   vf;
        int     idx;
        u32     size;
        char    *str;
        u64     vu64;

        buf[0]  = ft;
        buf[1]  = fi;
        buf[2]  = mSeqID[fi]++;

        idx     = 7;
        size    = 0;

        if (fmt) {
            va_list ap;

            va_start(ap, fmt);
            for (int i = 0; i < strlen(fmt); i++) {
                switch (fmt[i]) {
                    case 'b':
                    case 'B':
                        buf[idx] = va_arg(ap, u32);
                        size = 1;
                        break;

                   case 'h':
                   case 'H':
                        vu16 = va_arg(ap, u32);
                        size = Utils::put16(&buf[idx], vu16);
                        break;

                   case 'i':
                   case 'I':
                   case 'l':
                   case 'L':
                        vu32 = va_arg(ap, u32);
                        size = Utils::put32(&buf[idx], vu32);
                        break;

                   case 'q':
                   case 'Q':
                        vu64 = va_arg(ap, u64);
                        size = Utils::putlonglong(&buf[idx], vu64);
                        break;

                   case 'f':
                        vf   = va_arg(ap, float);
                        size = Utils::putfloat(&buf[idx], vf);
                        break;

                   case 's':
                   case 'S':
                        str  = va_arg(ap, char*);
                        size = Utils::putstr(&buf[idx], str);
                        break;

                   case 'p':
                   case 'P':
                        size = va_arg(ap, u32);
                        str  = va_arg(ap, char*);
                        memcpy(&buf[idx], str, size);
                        break;

                   default:
                        size = 0;
                        Serial.printf("ABNORMAL FORMAT !!!  : %c\n", fmt[i]);
                        break;
                }
                idx += size;
            }
            va_end(ap);
        }
        Utils::put32(&buf[3], idx);

        return idx;
    }

private:
    static u8  mSeqID[256];
};
#endif
