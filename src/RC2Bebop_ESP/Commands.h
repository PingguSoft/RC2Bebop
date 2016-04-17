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

#ifndef _COMMANDS_H_
#define _COMMANDS_H_

#include <WiFiUdp.h>
#include "Common.h"
#include "Bebop.h"


// http://robotika.cz/robots/katarina/en#150202
// https://github.com/robotika/katarina
// https://github.com/Parrot-Developers/libARCommands/blob/master/Xml/ARDrone3_commands.xml


//    0          1       2      3 4 5 6       7
// frametype, frameid, seqid, payloadlen+7    payload

class Commands
{
public:
    Commands(char *host, int port);
    ~Commands();
    void sendto(u8 *data, int size);

    void takeOff(void);
    void land(void);
    void emergency(void);
    void trim(void);
    void requestSettings(void);
    void requestStates(void);
    void resetHome(void);

    void move(u8 enRollPitch, s8 roll, s8 pitch, s8 yaw, s8 gaz);
    void enableVideoAutoRecording(u8 enable, u8 storage = 0);
    void takePicture(u8 storage = 0);
    void recordVideo(u8 enable, u8 storage = 0);
    void setDate(void);
    void setTime(void);
    void enableVideoStreaming(u8 enable);
    void moveCamera(s8 tilt, s8 pan);
    bool config(void);
    void process(u8 *dataAck, int size);

    s8   getRoll(void)  { return mRoll;     }
    s8   getPitch(void) { return mPitch;    }
    s8   getYaw(void)   { return mYaw;      }
    s8   getGaz(void)   { return mGaz;      }

private:
    u8  mBuf[1024];
    WiFiUDP mUDP;
    Bebop   mBebop;

    char *mStrHost;
    int  mPort;
    long mLastTS;

    u8   mEnRollPitch;
    s8   mRoll;
    s8   mPitch;
    s8   mYaw;
    s8   mGaz;

    u8   mCfgIdx;

    u8   mPCMDSeq;
};

#endif
