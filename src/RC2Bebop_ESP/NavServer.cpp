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
#include "NavServer.h"
#include "Utils.h"
#include "ByteBuffer.h"

NavServer::NavServer()
{
    mPort       = 0;
    mNextState  = STATE_HEADER;
    mPayloadLen = 0;
}

NavServer::NavServer(int port)
{
    mPort       = port;
    mNextState  = STATE_HEADER;
    mPayloadLen = 0;
}

NavServer::~NavServer()
{
    mUDP.stop();
}

int NavServer::recv(u8 *data, int size)
{
    int cb = mUDP.parsePacket();
    if (!cb || mUDP.available() < size)
        return 0;

    return mUDP.read(data, size);
}

//    0          1       2      3 4 5 6       7
// frametype, frameid, seqid, payloadlen+7    payload

static const char *TBL_FSTATES[]  = {"landed", "takingoff", "hovering", "flying", "landing", "emergency"};
static const char *TBL_ASTATES[]  = {"none/No alert", "user/User emergency alert", "cut_out/Cut out alert", "critical_battery", "low_battery", "too_much_angle"};
static const char *TBL_HSTATES[]  = {"available", "inProgress", "unavailable", "pending", "low_battery", "too_much_angle"};
static const char *TBL_HREASONS[] = {"userRequest", "connectionLost", "lowBattery", "finished", "stopped", "disabled", "enabled"};
static const char *TBL_VSTATES[]  = {"stopped", "started", "failed", "autostopped"};
static const char *TBL_VSSTATES[] = {"enabled", "disabled", "error"};

int NavServer::parseFrame(u8 *data, u32 size, u8 *dataAck)
{
    ByteBuffer   ba(data, size);
    char        buf[32];
    u32         cmdID;
    u16         cmd;
    int         len = 0;

    len = preProcess(data, size, dataAck);
    if (len < 0)
        return -len;

    switch (mFrameType) {
        case FRAME_TYPE_ACK:
            if (mPayloadLen == 8 && mFrameID == 0x8b) {
                Utils::printf(">> ACKACK       : %d\n", *data);
                len = Bebop::buildCmd(dataAck, FRAME_TYPE_ACK, 0xFE, "B", mFrameSeqID);
            }
            return len;

        case FRAME_TYPE_DATA_LOW_LATENCY:
            if (mPayloadLen >= 12 && mFrameID == BUFFER_ID_D2C_VID) {
                u16 frameNo      = ba.get16();
                u8  frameFlags   = ba.get8();
                u8  fragNo       = ba.get8();
                u8  fragPerFrame = ba.get8();
                static u64 ackLow;
                static u64 ackHigh;

                if (frameNo != mVidFrameNo) {
                    ackLow  = 0;
                    ackHigh = 0;
                    mVidFrameNo = frameNo;
                }

                if (fragNo < 64)
                    ackLow |= (1 << fragNo);
                else
                    ackHigh |= (1 << (fragNo - 64));

                Utils::printf(">> VIDEO        : %05d, %02X, %03d, %03d\n", frameNo, frameFlags, fragNo, fragPerFrame);
                len = Bebop::buildCmd(dataAck, FRAME_TYPE_DATA, 13, "HQQ", frameNo, ackHigh, ackLow);
            }
            return len;

        case FRAME_TYPE_DATA_WITH_ACK:
            Utils::printf(">> ACK REQUIRED : %d %d %d\n", mFrameType, mFrameID, mFrameSeqID);
            len = Bebop::buildCmd(dataAck, FRAME_TYPE_ACK, 0x80 | mFrameID, "B", mFrameSeqID);
            return len;
    }

    switch(mFrameID) {
        case BUFFER_ID_PING:
            Utils::printf(">> Ping Stamp   : %d.%d\n", ba.get32(), ba.get32() / 1000000000);
            len = Bebop::buildCmd(dataAck, FRAME_TYPE_DATA, BUFFER_ID_PONG, "P", size, data);
            break;

        case BUFFER_ID_D2C_RPT:
            cmdID = PACK_CMD(ba.get8(), ba.get8(), ba.get16());
            cmd   = GET_CMD(cmdID);

            if (cmdID == PACK_CMD(PROJECT_COMMON, COMMON_CLASS_COMMONSTATE, 7)) {
                Utils::printf(">> RSSI         : %5d\n", ba.get16());
            } else if (cmdID == PACK_CMD(PROJECT_ARDRONE3, ARDRONE3_CLASS_CAMERASTATE, 0)) {
                Utils::printf(">> CAM          : %d %d\n", ba.get8(), ba.get8());
            } else if (GET_PRJ_CLS(cmdID) == PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_PILOTINGSTATE)) {
                switch(cmd) {
                    case 4:
                        Utils::printf(">> POS          : %s %s %s\n", Utils::dtoa(buf, ba.getdouble()),
                            Utils::dtoa(buf, ba.getdouble()), Utils::dtoa(buf, ba.getdouble()));
                        break;

                    case 5:
                        Utils::printf(">> SPEED        : %s %s %s\n", Utils::ftoa(buf, ba.getfloat()),
                            Utils::ftoa(buf, ba.getfloat()), Utils::ftoa(buf, ba.getfloat()));
                        break;

                    case 6:
                        Utils::printf(">> ANGLE        : %s %s %s\n", Utils::ftoa(buf, ba.getfloat()),
                            Utils::ftoa(buf, ba.getfloat()), Utils::ftoa(buf, ba.getfloat()));
                        break;

                    case 8:
                        Utils::printf(">> ALT          : %s\n", Utils::dtoa(buf, ba.getdouble()));
                        break;
                }
            } else {
                Utils::printf(">> UNKNOWN      : %08x\n", cmdID);
            }
            break;

        case BUFFER_ID_D2C_ACK_SETTINGS:
            cmdID = PACK_CMD(ba.get8(), ba.get8(), ba.get16());
            cmd   = GET_CMD(cmdID);

            switch(GET_PRJ_CLS(cmdID)) {
                case PACK_PRJ_CLS(PROJECT_COMMON, COMMON_CLASS_SETTINGSSTATE):
                    switch (cmd) {
                        case 0:
                            Utils::printf(">> All Settings - Done\n");
                            break;
                            
                        case 2:
                            Utils::printf(">> Product Name : %s\n", ba.getstr());
                            break;

                        case 3:
                            Utils::printf(">> Product Ver  : %s\n", ba.getstr());
                            break;

                        case 4:
                            Utils::printf(">> Product SerH : %s\n", ba.getstr());
                            break;

                        case 5:
                            Utils::printf(">> Product SerL : %s\n", ba.getstr());
                            break;
                            
                        case 6:
                            Utils::printf(">> Country      : %s\n", ba.getstr());
                            break;

                        case 7:
                            Utils::printf(">> AutoCountry  : %s\n", ba.getstr());
                            break;

                        default:
                            Utils::printf(">> UNKNOWN      : %08x\n", cmdID);
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_COMMON, COMMON_CLASS_COMMONSTATE):
                    switch (cmd) {
                        case 1:
                            Utils::printf(">> Battery      : %d\n", ba.get8());
                            break;
                            
                        case 2:
                            Utils::printf(">> Date         : %s\n", ba.getstr());
                            break;
                            
                        case 5:
                            Utils::printf(">> Time         : %s\n", ba.getstr());
                            break;
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_COMMON, COMMON_CLASS_WIFISETTINGSSTATE):
                    if (cmd == 0) {
                        Utils::printf(">> WiFi Outdoor : %d\n", ba.get8());
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_COMMON, COMMON_CLASS_CALIBRATIONSTATE):
                    switch (cmd) {
                        case 0:
                            Utils::printf(">> Mag Cal      : %d %d %d %d\n", ba.get8(), ba.get8(), ba.get8(), ba.get8());
                            break;

                        case 1:
                            Utils::printf(">> Mag Cal Req  : %d\n", ba.get8());
                            break;
                            
                        case 3:
                            Utils::printf(">> Mag Cal Start: %d\n", ba.get8());
                            break;

                        default:
                            Utils::printf(">> Calibration  : %d\n", cmd);
                            break;
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_PILOTINGSTATE):
                    switch (cmd) {
                        case 0:
                            Utils::printf(">> FlatTrim Done:\n");
                            break;

                        case 1:
                            Utils::printf(">> Flying State : %s\n", TBL_FSTATES[ba.get32()]);
                            break;

                        case 2:
                            Utils::printf(">> Alert  State : %s\n", TBL_ASTATES[ba.get32()]);
                        break;

                        case 3:
                            Utils::printf(">> Navigate Home: %s, %s\n", TBL_HSTATES[ba.get32()], TBL_HREASONS[ba.get32()]);
                            break;

                        default:
                            Utils::printf(">> UNKNOWN PILOT: %08x\n", cmdID);
                            break;
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_PILOTINGSETTINGSSTATE):
                    switch (cmd) {
                        case 0:
                            Utils::printf(">> Max Alt      : %f %f %f\n", ba.getfloat(), ba.getfloat(), ba.getfloat());
                            break;
                            
                        case 1:
                            Utils::printf(">> Max Tilt     : %f %f %f\n", ba.getfloat(), ba.getfloat(), ba.getfloat());
                            break;

                        case 2:
                            Utils::printf(">> Absolute Ctrl: %d\n", ba.get8());
                            break;

                        default:
                            Utils::printf(">> UNKNOWN PILOT: %08x\n", cmdID);
                            break;
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_MEDIARECORDSTATE):
                    switch (cmd) {
                        case 0:
                            Utils::printf(">> Pictue State : %d %d\n", ba.get8(), ba.get8());
                            break;
                            
                        case 1:
                            Utils::printf(">> Video  State : %s %d\n", TBL_VSTATES[ba.get32()], ba.get8());
                            break;

                        default:
                            Utils::printf(">> UNKNOWN MEDIA: %08x\n", cmdID);
                            break;
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_SPEEDSETTINGSSTATE):
                    Utils::printf(">> SPEED State  : %08x\n", cmdID);
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_SETTINGSSTATE):
                    switch (cmd) {
                        case 4:
                            Utils::printf(">> Motor Flight : %d %d %d\n", ba.get16(), ba.get16(), ba.get32());
                            break;
                        
                        case 5:
                            Utils::printf(">> Motor LastErr: %d\n", ba.get32());
                            break;

                        default:
                            Utils::printf(">> Setting State: %08x\n", cmdID);
                            break;
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_PICTURESETTINGSSTATE):
                    if (cmd == 5) {
                        Utils::printf(">> VideoRec Stat: %d %d\n", ba.get8(), ba.get8());
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_MEDIASTREAMINGSTATE):
                    if (cmd == 0) {
                        Utils::printf(">> VideoStm Stat: %s\n", TBL_VSSTATES[ba.get32()]);
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_GPSSETTINGSSTATE):
                    switch (cmd) {
                        case 0:
                            Utils::printf(">> Home Changed : %f %f\n", ba.getdouble(), ba.getdouble());
                            break;

                        case 2:
                            Utils::printf(">> GPS Fix stat : %d\n", ba.get8());
                            break;
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3DEBUG, 3):
                    if (cmd == 0) {
                        Utils::printf(">> GPS NumSat   : %d\n", ba.get8());
                    }
                    break;

                default:
                    if (GET_PRJ(cmdID) == PROJECT_ARDRONE3DEBUG) {
                        Utils::printf(">> DEBUG        : %08x\n", cmdID);
                    }
                    break;
            }
            break;

        default:
            Utils::printf(">> UNKNOWN TYPE : %d %d\n", mFrameType, mFrameID);
            break;
    }

    return len;
}


void NavServer::begin(void)
{
    if (mPort == 0) {
        Utils::printf("NO PORT !!!\n");
        return;
    }
    mUDP.begin(mPort);
    Utils::printf("Local port : %d\n", mUDP.localPort());
}

int NavServer::process(u8 *dataAck)
{
    int cb = mUDP.parsePacket();
    int len = 0;
    int size = 0;

    cb = mUDP.available();
   
    while (cb > 0) {
        switch (mNextState) {
            case STATE_HEADER:
            {
                if (cb < HEADER_LEN)
                    return size;

                mUDP.read(mBuffer, HEADER_LEN);
                u8 *data = mBuffer;

                //Utils::printf(">> RX --- \n");
                //Utils::dump(data, HEADER_LEN);

                ByteBuffer   ba(data, HEADER_LEN);
                mFrameType  = ba.get8();
                mFrameID    = ba.get8();
                mFrameSeqID = ba.get8();
                mPayloadLen = ba.get32();
                mNextState = STATE_BODY;
                cb -= HEADER_LEN;
            }
            // no break

            case STATE_BODY:
            {
                u32 bodylen = mPayloadLen - HEADER_LEN;
                
                if (cb < bodylen)
                    return len;

                mUDP.read(&mBuffer[HEADER_LEN], bodylen);
                //Utils::dump(&mBuffer[HEADER_LEN], bodylen);
                len = parseFrame(&mBuffer[HEADER_LEN], bodylen, dataAck);
                dataAck += len;
                size    += len;

                mNextState = STATE_HEADER;
                cb -= bodylen;
            }
            break;
        }
    }

    return size;
}
