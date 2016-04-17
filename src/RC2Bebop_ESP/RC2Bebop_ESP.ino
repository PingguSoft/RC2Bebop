#include <Arduino.h>
#include <stdarg.h>
#include <ESP8266WiFi.h>
#include "Commands.h"
#include "Receiver.h"
#include "ByteBuffer.h"
#include "SerialProtocol.h"

enum {
    STATE_INIT = 0,
    STATE_AP_CONNECT,
    STATE_DISCOVERY,
    STATE_DISCOVERY_ACK,
    STATE_CONFIG,
    STATE_WORK,
};

static char *HOST = "192.168.42.1";

static SerialProtocol   mSerial;
static WiFiClient       mClient;
static Commands         mControl(HOST, 54321);
static Receiver         mNavData(43210);
static u8               mNextState = STATE_INIT;

void WiFiEvent(WiFiEvent_t event) {
    Utils::printf("[WiFi-event] event: %d\n", event);

    switch(event) {
        case WIFI_EVENT_STAMODE_GOT_IP:
            Serial.println("WiFi connected");
            Utils::printf("IP address: %s\n", WiFi.localIP().toString().c_str());
            mNextState = STATE_DISCOVERY;
            mSerial.sendCmd(SerialProtocol::CMD_SET_STATE, &mNextState, 1);
            break;


        case WIFI_EVENT_STAMODE_DISCONNECTED:
            Utils::printf("WiFi lost connection\n");
            mNextState = STATE_INIT;
            mSerial.sendCmd(SerialProtocol::CMD_SET_STATE, &mNextState, 1);
            break;
    }
}


static u8 dataAck[1024];
static u8 recVideo = 0;
static s8 speed = 0;
static s8 roll = 0;
static s8 pitch = 0;
static s8 yaw = 0;
static s8 aux1 = 0;
static s8 aux2 = 0;
static s8 aux3 = 0;
static s8 aux4 = 0;

#if 0
void handleKey(void)
{
    int size;

    size = Serial.available();
    while(size--) {
        u8 ch = Serial.read();
        Utils::printf("***** : %c\n", ch);

        switch (ch) {
            case 'p' : mControl.takePicture();                                  break;
            case 'v' : recVideo = !recVideo;    mControl.recordVideo(recVideo); break;

            case 'a' : mControl.takeOff();                                      break;
            case 'z' : mControl.land();                                         break;

            case 's' :
                speed += (speed < 90) ? 10 : 0;
                Utils::printf("thr : %d\n", speed);
                break;

            case 'x' :
                speed += (speed > -90) ? -10 : 0;
                Utils::printf("thr : %d\n", speed);
                break;

            case 'i' :
                pitch += (pitch < 90) ? 10 : 0;
                Utils::printf("pitch : %d\n", pitch);
                break;

            case 'k' :
                pitch += (pitch > -90) ? -10 : 0;
                Utils::printf("pitch : %d\n", pitch);
                break;

            case 'j' :
                roll += (roll > -90) ? -10 : 0;
                Utils::printf("roll : %d\n", roll);
                break;

            case 'l' :
                roll += (roll < 90) ? 10 : 0;
                Utils::printf("roll : %d\n", roll);
                break;

            case 'u' :
                yaw += (yaw > -90) ? -10 : 0;
                Utils::printf("yaw : %d\n", yaw);
                break;

            case 'o' :
                yaw += (yaw < 90) ? 10 : 0;
                Utils::printf("yaw : %d\n", yaw);
                break;

            case ' ' :
                yaw = 0;
                roll = 0;
                pitch = 0;
                speed = 0;
                Utils::printf("reset pos\n");
                break;
        }
    }
    u8 flag = 0;
    if (roll != 0 || pitch != 0)
        flag = 1;
    mControl.move(flag, roll, pitch, yaw, speed);
}
#endif

void scanAndConnect(void)
{
    int n = WiFi.scanNetworks();

    if (n == 0) {
        Utils::printf("no networks found\n");
    } else {
        Utils::printf("%d networks found\n", n);
        for (int i = 0; i < n; i++) {
            Utils::printf("%d : %s (%d) %d\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.encryptionType(i));
            if (!strncmp(WiFi.SSID(i).c_str(), "BebopDrone", 10)) {
                WiFi.onEvent(WiFiEvent);
                WiFi.begin(WiFi.SSID(i).c_str(), "");
                mNextState = STATE_AP_CONNECT;
                Utils::printf("Connect to BebopDrone !!!\n");
                mSerial.sendCmd(SerialProtocol::CMD_SET_STATE, &mNextState, 1);
                break;
            }
        }
    }
}

void connectDiscovery(void)
{
    Utils::printf("Connect to discovery socket !!!\n");

    if (!mClient.connect(HOST, 44444)) {
        Utils::printf("Connection Failed !!!\n");
    } else {
        char *req = "{\"controller_type\":\"computer\", \"controller_name\":\"UniConTX\", \"d2c_port\":\"43210\"}";
        mClient.print(req);
        mClient.flush();
        mNextState = STATE_DISCOVERY_ACK;
        mSerial.sendCmd(SerialProtocol::CMD_SET_STATE, &mNextState, 1);
    }
}

void handleDiscovery(void)
{
    u8  buf[256];

    Utils::printf("Waiting response !!!\n");
    while (mClient.available()) {
        int len = mClient.read(buf, 256);
        if (len > 0) {
            Utils::printf("%d %s\n", len, (char*)buf);
            mNavData.begin();
            mNextState = STATE_CONFIG;
            mSerial.sendCmd(SerialProtocol::CMD_SET_STATE, &mNextState, 1);
        }
        //{ "status": 0, "c2d_port": 54321, "arstream_fragment_size": 65000, "arstream_fragment_maximum_number": 4, "arstream_max_ack_interval": -1, "c2d_update_port": 51, "c2d_user_port": 21 }
    }
}

s16 map(s16 v)
{
    return (-5 <= v && v <= 5) ? 0 : v;
}

u32 serialCallback(u8 cmd, u8 *data, u8 size)
{
    u8 flag = 0;
    u32 ret = 0;
    ByteBuffer bb(data, size);

    switch (cmd) {
        case SerialProtocol::CMD_SET_RC:
            speed = map((s16)bb.get16());
            yaw   = map((s16)bb.get16());
            pitch = map((s16)bb.get16());
            roll  = map((s16)bb.get16());
            aux1  = map((s16)bb.get16());
            aux2  = map((s16)bb.get16());
            aux3  = map((s16)bb.get16());
            aux4  = map((s16)bb.get16());

#if 0
            if (aux1 >= 50)
                mControl.takeOff();
            else if (aux1 < 50)
                mControl.land();

            if (roll != 0 || pitch != 0)
                flag = 1;

            mControl.move(flag, roll, pitch, yaw, speed);
#endif
            break;
    }
    return ret;
}


IPAddress apIP(192, 168, 70, 1);



void setup() {
    Serial.begin(57600);

//    WiFi.mode(WIFI_STA);
//    WiFi.disconnect();
//    delay(100);

    Utils::printf("Ready !!! : %08x\n", ESP.getChipId());
    mSerial.setCallback(serialCallback);

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("BebopDrone-E035114");
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    delay(100);
}

void testServer(void)
{
    if (mServer.hasClient()) {
        if (!mServerClient || !mServerClient.connected()) {
            if (mServerClient)
                mServerClient.stop();

            mServerClient = mServer.available();
            Utils::printf("New Client !!\n");
        }
    }

    if (mServerClient && mServerClient.connected()) {
        while (mServerClient.available()) {
            Serial.write(mServerClient.read());
        }
    }
}

bool serverStarted = false;
WiFiServer  mServer(12345);
WiFiClient  mServerClient;

void loop()
{
    u8  size;

    if (!serverStarted) {
        mServer.begin();
        serverStarted = true;
    }

    if (serverStarted) {
        testServer();
    }

#if 0
    switch (mNextState) {
        case STATE_INIT:
            scanAndConnect();
            break;

        case STATE_DISCOVERY:
            connectDiscovery();
            break;

        case STATE_DISCOVERY_ACK:
            handleDiscovery();
            break;

        case STATE_CONFIG:
            if (mControl.config()) {
                mNextState = STATE_WORK;
                mSerial.sendCmd(SerialProtocol::CMD_SET_STATE, &mNextState, 1);
            }

            size = mNavData.process(dataAck);
            if (size > 0)
                mControl.process(dataAck, size);
            break;

        case STATE_WORK:
            size = mNavData.process(dataAck);
            mControl.process(dataAck, size);
            break;
    }
    mSerial.handleRX();
#endif
}

