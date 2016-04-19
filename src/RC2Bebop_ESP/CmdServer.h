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

#ifndef _CMD_SERVER_H_
#define _CMD_SERVER_H_

#include <WiFiUdp.h>
#include "Common.h"
#include "Bebop.h"

#define HEADER_LEN  7

// http://robotika.cz/robots/katarina/en#150202
// https://github.com/robotika/katarina
// https://github.com/Parrot-Developers/libARCommands/blob/master/Xml/ARDrone3_commands.xml


class CmdServer
{
public:
    enum {
        STATE_HEADER = 0,
        STATE_BODY   = 1,
    };

    CmdServer(int port);
    ~CmdServer();

    int recv(u8 *data, int size);
    void begin(void);
    int  process(void);
    u8   *getData(void)     { return mBuffer;       }
    u32  getDataSize(void)  { return mPayloadLen;   }
// datetime.datetime.now().date().isoformat()                ==> '2016-04-15'              V
// datetime.datetime.now().time().isoformat()                ==> '18:55:34.756000'
// datetime.datetime.now().time().strftime("T%H%M%S+0000")   ==> 'T185603+0000'            V

private:
    int parseFrame(u8 *data, u32 size);


    WiFiUDP mUDP;

    u8  *mStrHost;
    int mPort;

    u8  mBuffer[1124];
    u8  mNextState;

    u8  mFrameType;
    u8  mFrameID;
    u8  mFrameSeqID;
    u32 mPayloadLen;
};

#endif
