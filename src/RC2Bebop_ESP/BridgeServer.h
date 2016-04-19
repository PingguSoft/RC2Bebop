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

#ifndef _BRIDGE_SERVER_H_
#define _BRIDGE_SERVER_H_

#include <WiFiUdp.h>
#include "Common.h"
#include "Bebop.h"

#define HEADER_LEN  7

// http://robotika.cz/robots/katarina/en#150202
// https://github.com/robotika/katarina
// https://github.com/Parrot-Developers/libARCommands/blob/master/Xml/ARDrone3_commands.xml


class BridgeServer
{
public:
    enum {
        STATE_HEADER = 0,
        STATE_BODY   = 1,
    };

    BridgeServer(char *name, int port);
    ~BridgeServer();

    void setHost(IPAddress hostIP, int hostport)    { mHostIP = hostIP; mHostPort = hostport; }
    void setBypass(bool bypass)                     { mBypass = bypass; }
    
    void sendto(u8 *data, int size);                // send to host ip/port
    int  recv(u8 *data, int size);                  // receive from server ip/port
    
    void begin(void);
    int  process(u8 *dataAck);
    u8   *getData(void)     { return mBuffer;       }
    u32  getDataSize(void)  { return mPayloadLen;   }
    int  kick(void);
    
private:
    int parseFrame(u8 *data, u32 size, u8 *dataAck);

    char *mName;
    WiFiUDP mUDPServer; // RX only
    int mServerPort;    // RX only

    WiFiUDP mUDPHost;   // TX only
    IPAddress mHostIP;  // TX only
    int mHostPort;

    u8  mBuffer[1124];
    u8  mNextState;

    bool mBypass;
    u8  mFrameType;
    u8  mFrameID;
    u8  mFrameSeqID;
    u32 mPayloadLen;

    u8  mPCMDSeq;
    u16 mVidFrameNo;
    u32 mLastTS;
};

#endif
