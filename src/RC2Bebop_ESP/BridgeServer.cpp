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
#include <string.h>
#include <stdlib.h>
#include "BridgeServer.h"
#include "Utils.h"
#include "ByteBuffer.h"

BridgeServer::BridgeServer(char *name, int portServer)
{
    mName       = name;
    mBypass     = false;
}

BridgeServer::~BridgeServer()
{
    mUDPHost.stop();
}

void BridgeServer::sendto(u8 *data, int size)
{
    //Utils::printf("<<< TX : %s to (%s:%d)\n", 
    //    mName, mHostIP.toString().c_str(), mHostPort);

    if (mHostIP[0] == 0) {
        Utils::printf("<<< TX ERROR : no dest\n");
        return;
    }

    mUDPHost.beginPacket(mHostIP, mHostPort);
    mUDPHost.write(data, size);
    mUDPHost.endPacket();
}

int BridgeServer::preProcess(u8 *data, u32 size, u8 *dataAck)
{
    if (mBypass) {
        if (mHostPort != 0) {
            sendto(mBuffer, mPayloadLen);
        } else {
            Utils::printf("HOST PORT IS ZERO !!!\n");
        }
        return -mPayloadLen;;
    }
}

int BridgeServer::kick(void)
{
    long ts = millis();
    int  diff = ts - mLastTS;
    int  size = 0;

    if (diff >= 25) {
        u8  flag = 0;
        u32 tsPCMD = (mPCMDSeq++ << 24) | (millis() & 0x00ffffff);
        u8  buf[40];

        size = Bebop::buildCmd(buf, FRAME_TYPE_DATA, BUFFER_ID_C2D_PCMD, "BBHBbbbbI", PROJECT_ARDRONE3, ARDRONE3_CLASS_PILOTING, 2,
            flag, 0, 0, 0, 0, tsPCMD);
        sendto(buf, size);
        mLastTS = ts;
    }
    return size;
}

