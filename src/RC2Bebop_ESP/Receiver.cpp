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
#include "Receiver.h"
#include "Utils.h"
#include "ByteBuffer.h"

Receiver::Receiver(int port)
{
    mPort       = port;
    mNextState  = STATE_HEADER;
    mPayloadLen = 0;
    mBodyLen    = 0;
}

Receiver::~Receiver()
{
    mUDP.stop();
}

int Receiver::recv(u8 *data, int size)
{
    int cb = mUDP.parsePacket();
    if (!cb || mUDP.available() < size)
        return 0;

    return mUDP.read(data, size);
}

//    0          1       2      3 4 5 6       7
// frametype, frameid, seqid, payloadlen+7    payload

int Receiver::parseFrame(u8 *data, u32 size, u8 *dataAck)
{
    ByteBuffer   ba(data, size);
    char        buf[32];
    u32         cmdID;
    u16         cmd;
    int         len = 0;

    switch (mFrameType) {
        case FRAME_TYPE_ACK:
            if (mPayloadLen == 8 && mFrameID == 0x8b) {
                Utils::printf(">> ACKACK       : %d\n", *data);
                len = Bebop::buildCmd(dataAck, FRAME_TYPE_ACK, 0xFE, "B", mFrameSeqID);
            }
            return len;

        case FRAME_TYPE_DATA_LOW_LATENCY:
            if (mPayloadLen >= 12 && mFrameID == 0x7d) {
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
            len = Bebop::buildCmd(dataAck, FRAME_TYPE_ACK, 0xFE, "B", mFrameSeqID);
            return len;
    }

    switch(mFrameID) {
        case BUFFER_ID_PING:
            Utils::printf(">> Ping Stamp   : %d.%d\n", ba.get32(), ba.get32() / 1000000000);
            len = Bebop::buildCmd(dataAck, FRAME_TYPE_DATA, BUFFER_ID_PONG, "P", size, data);
            break;

        case 0x7f:
            cmdID = PACK_CMD(ba.get8(), ba.get8(), ba.get16());
            cmd   = GET_CMD(cmdID);

            if (cmdID == PACK_CMD(PROJECT_COMMON, COMMON_CLASS_COMMONSTATE, 7)) {
                Utils::printf(">> RSSI         : %5d\n", ba.get16());
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
            } else if (cmdID == PACK_CMD(PROJECT_ARDRONE3, ARDRONE3_CLASS_CAMERASTATE, 0)) {
                Utils::printf(">> CAM          : %d %d\n", ba.get8(), ba.get8());
            } else {
                Utils::printf(">> UNKNOWN      : %08x\n", cmdID);
            }
            break;

        case 0x7e:
            cmdID = PACK_CMD(ba.get8(), ba.get8(), ba.get16());
            cmd   = GET_CMD(cmdID);

            switch(GET_PRJ_CLS(cmdID)) {
                case PACK_PRJ_CLS(PROJECT_COMMON, COMMON_CLASS_SETTINGSSTATE):
                    if (cmd == 0) {
                        Utils::printf(">> All Settings - Done\n");
                    } else if (cmd == 2) {
                        Utils::printf(">> Product Name : %s\n", ba.getstr());
                    } else if (cmd == 3) {
                        Utils::printf(">> Product Ver  : %s\n", ba.getstr());
                    } else if (cmd == 4) {
                        Utils::printf(">> Product SerH : %s\n", ba.getstr());
                    } else if (cmd == 5) {
                        Utils::printf(">> Product SerL : %s\n", ba.getstr());
                    } else if (cmd == 6) {
                        Utils::printf(">> Country      : %s\n", ba.getstr());
                    } else if (cmd == 7) {
                        Utils::printf(">> AutoCountry  : %s\n", ba.getstr());
                    } else {
                        Utils::printf(">> UNKNOWN      : %08x\n", cmdID);
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_COMMON, COMMON_CLASS_COMMONSTATE):
                    if (cmd == 1) {
                        Utils::printf(">> Battery      : %d\n", ba.get8());
                    } else if (cmd == 4) {
                        Utils::printf(">> Date         : %s\n", ba.getstr());
                    } else if (cmd == 5) {
                        Utils::printf(">> Time         : %s\n", ba.getstr());
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_COMMON, COMMON_CLASS_WIFISETTINGSSTATE):
                    if (cmd == 0) {
                        Utils::printf(">> WiFi Outdoor : %d\n", ba.get8());
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_COMMON, COMMON_CLASS_CALIBRATIONSTATE):
                    if (cmd == 0) {
                        Utils::printf(">> Mag Cal      : %d %d %d %d\n", ba.get8(), ba.get8(), ba.get8(), ba.get8());
                    } else if (cmd == 1) {
                        Utils::printf(">> Mag Cal Req  : %d\n", ba.get8());
                    } else if (cmd == 3) {
                        Utils::printf(">> Mag Cal Start: %d\n", ba.get8());
                    } else {
                        Utils::printf(">> Calibration  : %d\n", cmd);
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_PILOTINGSTATE):
                    if (cmd == 0) {
                        Utils::printf(">> FlatTrim Done:\n");
                    } else if (cmd == 1) {
                        const char *states[] = {"landed", "takingoff", "hovering", "flying", "landing", "emergency"};
                        Utils::printf(">> Flying State : %s\n", states[ba.get32()]);
                    } else if (cmd == 2) {
                        const char *states[] = {"none/No alert", "user/User emergency alert", "cut_out/Cut out alert", "critical_battery", "low_battery", "too_much_angle"};
                        Utils::printf(">> Alert  State : %s\n", states[ba.get32()]);
                    } else if (cmd == 3) {
                        const char *states[]  = {"available", "inProgress", "unavailable", "pending", "low_battery", "too_much_angle"};
                        const char *reasons[] = {"userRequest", "connectionLost", "lowBattery", "finished", "stopped", "disabled", "enabled"};
                        Utils::printf(">> Navigate Home: %s, %s\n", states[ba.get32()], reasons[ba.get32()]);
                    } else {
                        Utils::printf(">> UNKNOWN PILOT: %08x\n", cmdID);
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_PILOTINGSETTINGSSTATE):
                    if (cmd == 0) {
                        Utils::printf(">> Max Alt      : %f %f %f\n", ba.getfloat(), ba.getfloat(), ba.getfloat());
                    } else if (cmd == 1) {
                        Utils::printf(">> Max Tilt     : %f %f %f\n", ba.getfloat(), ba.getfloat(), ba.getfloat());
                    } else if (cmd == 2) {
                        Utils::printf(">> Absolute Ctrl: %d\n", ba.get8());
                    } else {
                        Utils::printf(">> UNKNOWN PILOT: %08x\n", cmdID);
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_MEDIARECORDSTATE):
                    if (cmd == 0) {
                        Utils::printf(">> Pictue State : %d %d\n", ba.get8(), ba.get8());
                    } else if (cmd == 1) {
                        const char *states[] = {"stopped", "started", "failed", "autostopped"};
                        Utils::printf(">> Video  State : %s %d\n", states[ba.get32()], ba.get8());
                    } else {
                        Utils::printf(">> UNKNOWN MEDIA: %08x\n", cmdID);
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_SPEEDSETTINGSSTATE):
                    Utils::printf(">> SPEED State  : %08x\n", cmdID);
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_SETTINGSSTATE):
                    if (cmd == 4) {
                        Utils::printf(">> Motor Flight : %d %d %d\n", ba.get16(), ba.get16(), ba.get32());
                    } else if (cmd == 5) {
                        Utils::printf(">> Motor LastErr: %d\n", ba.get32());
                    } else {
                        Utils::printf(">> Setting State: %08x\n", cmdID);
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_PICTURESETTINGSSTATE):
                    if (cmd == 5) {
                        Utils::printf(">> VideoRec Stat: %d %d\n", ba.get8(), ba.get8());
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_MEDIASTREAMINGSTATE):
                    if (cmd == 0) {
                        const char *states[] = {"enabled", "disabled", "error"};
                        Utils::printf(">> VideoStm Stat: %s\n", states[ba.get32()]);
                    }
                    break;

                case PACK_PRJ_CLS(PROJECT_ARDRONE3, ARDRONE3_CLASS_GPSSETTINGSSTATE):
                    if (cmd == 0) {
                        Utils::printf(">> Home Changed : %f %f\n", ba.getdouble(), ba.getdouble());
                    } else if (cmd == 2) {
                        Utils::printf(">> GPS Fix stat : %d\n", ba.get8());
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


void Receiver::begin(void)
{
    mUDP.begin(mPort);
    Utils::printf("Local port : %d\n", mUDP.localPort());
}

int Receiver::process(u8 *dataAck)
{
    int cb = mUDP.parsePacket();
    int len = 0;

    switch (mNextState) {
        case STATE_HEADER:
        {
            if (!cb || mUDP.available() < HEADER_LEN)
                return len;

            mUDP.read(mHeader, HEADER_LEN);
            u8 *data = mHeader;

            //Utils::printf("-------------------------RX START ---------------------\n");
            //Utils::dump(data, HEADER_LEN);

            ByteBuffer   ba(data, HEADER_LEN);
            mFrameType  = ba.get8();
            mFrameID    = ba.get8();
            mFrameSeqID = ba.get8();
            mPayloadLen = ba.get32();
            mBodyLen    = mPayloadLen - HEADER_LEN;
            mNextState = STATE_BODY;
        }

        case STATE_BODY:
        {
            if (mUDP.available() < mPayloadLen - HEADER_LEN)
                return len;

            mUDP.read(mBody, mBodyLen);
            //Utils::dump(mBody, mPayloadLen - HEADER_LEN);
            len = parseFrame(mBody, mBodyLen, dataAck);
            //Utils::printf("-------------------------RX END -----------------------\n\n");

            mNextState = STATE_HEADER;
        }
        break;
    }

    return len;
}
