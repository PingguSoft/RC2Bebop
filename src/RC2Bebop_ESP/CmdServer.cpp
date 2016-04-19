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
#include "CmdServer.h"
#include "Utils.h"
#include "ByteBuffer.h"

CmdServer::CmdServer(int port)
{
    mPort       = port;
    mNextState  = STATE_HEADER;
    mPayloadLen = 0;
}

CmdServer::~CmdServer()
{
    mUDP.stop();
}

int CmdServer::recv(u8 *data, int size)
{
    int cb = mUDP.parsePacket();
    if (!cb || mUDP.available() < size)
        return 0;

    return mUDP.read(data, size);
}

//    0          1       2      3 4 5 6       7
// frametype, frameid, seqid, payloadlen+7    payload

int CmdServer::parseFrame(u8 *data, u32 size)
{
    ByteBuffer   ba(data, size);
    u32         cmdID;

    switch (mFrameType) {
        case FRAME_TYPE_DATA:
            if (mFrameID == 0x10) {
                cmdID = PACK_CMD(ba.get8(), ba.get8(), ba.get16());
                if (cmdID == PACK_CMD(PROJECT_ARDRONE3, ARDRONE3_CLASS_PILOTING, 2)) {
                    Utils::dump(data, size);
                }
            }
            break;
    }

    return size;
}

void CmdServer::begin(void)
{
    mUDP.begin(mPort);
    Utils::printf("Local port : %d\n", mUDP.localPort());
}

int CmdServer::process(void)
{
    int cb = mUDP.parsePacket();
    int len = 0;

    switch (mNextState) {
        case STATE_HEADER:
        {
            if (!cb || mUDP.available() < HEADER_LEN)
                return len;

            mUDP.read(mBuffer, HEADER_LEN);
            u8 *data = mBuffer;

            Utils::printf("-------------------------RX START ---------------------\n");
            Utils::dump(data, HEADER_LEN);

            ByteBuffer   ba(data, HEADER_LEN);
            mFrameType  = ba.get8();
            mFrameID    = ba.get8();
            mFrameSeqID = ba.get8();
            mPayloadLen = ba.get32();
            mNextState = STATE_BODY;
        }

        case STATE_BODY:
        {
            u32 bodylen = mPayloadLen - HEADER_LEN;
            
            if (mUDP.available() < bodylen)
                return len;

            mUDP.read(&mBuffer[HEADER_LEN], bodylen);
            Utils::dump(&mBuffer[HEADER_LEN], bodylen);
            len = parseFrame(&mBuffer[HEADER_LEN], bodylen);
            Utils::printf("-------------------------RX END -----------------------\n\n");

            mNextState = STATE_HEADER;
        }
        break;
    }

    return len;
}
