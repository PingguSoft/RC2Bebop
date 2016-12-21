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
    mTM         = NULL;
}

NavServer::NavServer(int port)
{
    mPort       = port;
    mNextState  = STATE_HEADER;
    mPayloadLen = 0;
    mTM         = NULL;
}

NavServer::NavServer(int port, Telemetry *tm)
{
    mPort       = port;
    mNextState  = STATE_HEADER;
    mPayloadLen = 0;
    mTM         = tm;
}

NavServer::~NavServer()
{
    mUDP.stop();
}

int NavServer::recv(u8 *data, int size)
{
    int cb = mUDP.parsePacket();
    cb = mUDP.available();

    if (cb < size)
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

int NavServer::preProcess(u8 *data, u32 size, u8 *dataAck)
{
    return 0;
}

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
                LOG(">> ACKACK       : %d\n", *data);
            }
            return len;

        case FRAME_TYPE_DATA_LOW_LATENCY:
            if (mPayloadLen >= 12 && mFrameID == BUFFER_ID_D2C_VID) {
                u16 frameNo      = ba.get16();
                u8  frameFlags   = ba.get8();
                u8  fragNo       = ba.get8();
                u8  fragPerFrame = ba.get8();

                if (frameNo != mVidFrameNo) {
                    mAckLow  = 0;
                    mAckHigh = 0;
                    mVidFrameNo = frameNo;
                }

                if (fragNo < 64)
                    mAckLow |= (1 << fragNo);
                else
                    mAckHigh |= (1 << (fragNo - 64));

                LOG(">> VIDEO        : %05d, %02X, %03d, %03d\n", frameNo, frameFlags, fragNo, fragPerFrame);
                len = Bebop::buildCmd(dataAck, FRAME_TYPE_DATA, BUFFER_ID_C2D_VID_ACK, "HQQ", frameNo, mAckHigh, mAckLow);
                return len;
            }
            LOG(">> VIDEO        : ERROR !!!\n");
            return 0;

        case FRAME_TYPE_DATA:
            if (mFrameID == BUFFER_ID_PING) {
                LOG(">> Ping Stamp   : %d.%d\n", ba.get32(), ba.get32() / 1000000000);
                len = Bebop::buildCmd(dataAck, FRAME_TYPE_DATA, BUFFER_ID_PONG, "P", size, data);
                return len;
            }
            break;

        case FRAME_TYPE_DATA_WITH_ACK:
            //LOG(">> ACK REQUIRED : %d %d %d\n", mFrameType, mFrameID, mFrameSeqID);
            len = Bebop::buildCmd(dataAck, FRAME_TYPE_ACK, 0x80 | mFrameID, "B", mFrameSeqID);
            break;
            // no return
    }

    switch(mFrameID) {
        case BUFFER_ID_D2C_NAV:
            cmdID = PACK_CMD(ba.get8(), ba.get8(), ba.get16());
            cmd   = GET_CMD(cmdID);

            if (cmdID == PACK_CMD(PROJECT_COMMON, COMMON_CLASS_COMMONSTATE, 7)) {
                u16 rssi = ba.get16();
                LOG(">> RSSI         : %5d\n", rssi);
                if (mTM) {
                    mTM->setRSSI(rssi >> 8);
                }
            } else if (cmdID == PACK_CMD(PROJECT_ARDRONE3, ARDRONE3_CLASS_CAMERASTATE, 0)) {
                LOG(">> CAM          : %d %d\n", ba.get8(), ba.get8());
            } else if (GET_PRJ_CLS(cmdID) == PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_PILOTINGSTATE)) {
                switch(cmd) {
                    case 4:
                        LOG(">> POS          : %s %s %s\n", Utils::dtoa(buf, ba.getdouble()),
                            Utils::dtoa(buf, ba.getdouble()), Utils::dtoa(buf, ba.getdouble()));
                        break;

                    case 5:
                        LOG(">> SPEED        : %s %s %s\n", Utils::ftoa(buf, ba.getfloat()),
                            Utils::ftoa(buf, ba.getfloat()), Utils::ftoa(buf, ba.getfloat()));
                        break;

                    case 6:
                        LOG(">> ANGLE        : %s %s %s\n", Utils::ftoa(buf, ba.getfloat()),
                            Utils::ftoa(buf, ba.getfloat()), Utils::ftoa(buf, ba.getfloat()));
                        break;

                    case 8:
                        double alt = ba.getdouble();
                        if (mTM) {
                            mTM->setBaroAlt(alt * 10);
                        }
                        LOG(">> ALT          : %s\n", Utils::dtoa(buf, alt));
                        break;
                }
            } else {
                LOG(">> UNKNOWN      : %08x\n", cmdID);
            }
            break;

        case BUFFER_ID_D2C_EVENT:
            cmdID = PACK_CMD(ba.get8(), ba.get8(), ba.get16());
            cmd   = GET_CMD(cmdID);

            switch(GET_PRJ_CLS(cmdID)) {
                case PACK_PRJ_CLS(PROJECT_COMMON, COMMON_CLASS_SETTINGSSTATE):
                    switch (cmd) {
                        case 0:
                            LOG(">> All Settings - Done\n");
                            break;

                        case 2:
                            LOG(">> Product Name : %s\n", ba.getstr());
                            break;

                        case 3:
                            LOG(">> Product Ver  : %s\n", ba.getstr());
                            break;

                        case 4:
                            LOG(">> Product SerH : %s\n", ba.getstr());
                            break;

                        case 5:
                            LOG(">> Product SerL : %s\n", ba.getstr());
                            break;

                        case 6:
                            LOG(">> Country      : %s\n", ba.getstr());
                            break;

                        case 7:
                            LOG(">> AutoCountry  : %s\n", ba.getstr());
                            break;

                        default:
                            LOG(">> UNKNOWN (0,3): %08x\n", cmdID);
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_COMMON, COMMON_CLASS_COMMONSTATE):
                    switch (cmd) {
                        case 1:
                            mBatt = ba.get8();
                            LOG(">> Battery      : %d\n", mBatt);
                            if (mTM) {
                                mTM->setVolt(0, mBatt * 126 / 100);
                            }
                            break;

                        case 4:
                            LOG(">> Date         : %s\n", ba.getstr());
                            break;

                        case 5:
                            LOG(">> Time         : %s\n", ba.getstr());
                            break;
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_COMMON, COMMON_CLASS_WIFISETTINGSSTATE):
                    if (cmd == 0) {
                        LOG(">> WiFi Outdoor : %d\n", ba.get8());
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_COMMON, COMMON_CLASS_CALIBRATIONSTATE):
                    switch (cmd) {
                        case 0:
                            LOG(">> Mag Cal      : %d %d %d %d\n", ba.get8(), ba.get8(), ba.get8(), ba.get8());
                            break;

                        case 1:
                            LOG(">> Mag Cal Req  : %d\n", ba.get8());
                            break;

                        case 3:
                            LOG(">> Mag Cal Start: %d\n", ba.get8());
                            break;

                        default:
                            LOG(">> Calibration  : %d\n", cmd);
                            break;
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_PILOTINGSTATE):
                    switch (cmd) {
                        case 0:
                            LOG(">> FlatTrim Done:\n");
                            break;

                        case 1:
                            LOG(">> Flying State : %s\n", TBL_FSTATES[ba.get32()]);
                            break;

                        case 2:
                            LOG(">> Alert  State : %s\n", TBL_ASTATES[ba.get32()]);
                        break;

                        case 3:
                            LOG(">> Navigate Home: %s, %s\n", TBL_HSTATES[ba.get32()], TBL_HREASONS[ba.get32()]);
                            break;

                        default:
                            LOG(">> UNKNOWN PILOT: %08x\n", cmdID);
                            break;
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_PILOTINGSETTINGSSTATE):
                    switch (cmd) {
                        case 0:
                            LOG(">> Max Alt      : %s %s %s\n", Utils::ftoa(buf, ba.getfloat()), Utils::ftoa(buf, ba.getfloat()), Utils::ftoa(buf, ba.getfloat()));
                            break;

                        case 1:
                            LOG(">> Max Tilt     : %s %s %s\n", Utils::ftoa(buf, ba.getfloat()), Utils::ftoa(buf, ba.getfloat()), Utils::ftoa(buf, ba.getfloat()));
                            break;

                        case 2:
                            LOG(">> Absolute Ctrl: %d\n", ba.get8());
                            break;

                        default:
                            LOG(">> UNKNOWN PILOT: %08x\n", cmdID);
                            break;
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_MEDIARECORDSTATE):
                    switch (cmd) {
                        case 0:
                            LOG(">> Pictue State : %d %d\n", ba.get8(), ba.get8());
                            break;

                        case 1:
                            LOG(">> Video  State : %s %d\n", TBL_VSTATES[ba.get32()], ba.get8());
                            break;

                        default:
                            LOG(">> UNKNOWN MEDIA: %08x\n", cmdID);
                            break;
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_SPEEDSETTINGSSTATE):
                    LOG(">> SPEED State  : %08x\n", cmdID);
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_SETTINGSSTATE):
                    switch (cmd) {
                        case 4:
                            LOG(">> Motor Flight : %d %d %d\n", ba.get16(), ba.get16(), ba.get32());
                            break;

                        case 5:
                            LOG(">> Motor LastErr: %d\n", ba.get32());
                            break;

                        default:
                            LOG(">> Setting State: %08x\n", cmdID);
                            break;
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_PICTURESETTINGSSTATE):
                    if (cmd == 5) {
                        LOG(">> VideoAutoRec : %d %d\n", ba.get8(), ba.get8());
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_MEDIASTREAMINGSTATE):
                    if (cmd == 0) {
                        LOG(">> VideoStm Stat: %s\n", TBL_VSSTATES[ba.get32()]);
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_GPSSETTINGSSTATE):
                    switch (cmd) {
                        case 0:
                            LOG(">> Home Changed : %s %s\n", Utils::dtoa(buf, ba.getdouble()), Utils::dtoa(buf, ba.getdouble()));
                            break;

                        case 2:
                            LOG(">> GPS Fix stat : %d\n", ba.get8());
                            break;
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3DEBUG, 3):
                    if (cmd == 0) {
                        LOG(">> GPS NumSat   : %d\n", ba.get8());
                    }
                    break;

                default:
                    if (GET_PRJ(cmdID) == PROJECT_ARDRONE3DEBUG) {
                        LOG(">> DEBUG        : %08x\n", cmdID);
                    }
                    break;
            }
            break;

        default:
            LOG(">> UNKNOWN TYPE : %d %d\n", mFrameType, mFrameID);
            break;
    }

    return len;
}


void NavServer::begin(void)
{
    if (mPort == 0) {
        LOG("NavServer : NO PORT !!!\n");
        return;
    }
    mUDP.begin(mPort);
    LOG("Local port : %d\n", mUDP.localPort());
}

int NavServer::process(u8 *dataAck)
{
    int cb = mUDP.parsePacket();
    int len = 0;
    int size = 0;

    cb = mUDP.available();

    if (cb > 1024) {
        LOG("BIG DATA !!! : %d\n", cb);
    }

    while (cb > 0) {
        switch (mNextState) {
            case STATE_HEADER:
            {
                if (cb < HEADER_LEN)
                    return size;

                mUDP.read(mBuffer, HEADER_LEN);
                u8 *data = mBuffer;

                //LOG(">> RX --- \n");
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
