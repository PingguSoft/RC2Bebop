#include <Arduino.h>
#include <stdarg.h>
#include <ESP8266WiFi.h>
#include "Commands.h"
#include "NavServer.h"
#include "ByteBuffer.h"
#include "SerialProtocol.h"
#include "StatusLED.h"
#include "Sound.h"

enum {
    STATE_INIT = 0,
    STATE_AP_CONNECT,
    STATE_DISCOVERY,
    STATE_DISCOVERY_ACK,
    STATE_CONFIG,
    STATE_WORK,
};

#define DISCOVERY_PORT      44444
#define NAV_SERVER_PORT     52000

//static SerialProtocol   mSerial;
static WiFiClient       mBebopDiscoveryClient;
static Commands         mControl;
static NavServer        mNavServer(NAV_SERVER_PORT);
static u8               mNextState = STATE_INIT;
static bool             mBattWarn = false;

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

//static StatusLED    mLED(mSerial);
//static Sound        mSound(mSerial);

static void WiFiEvent(WiFiEvent_t event) {
    Utils::printf("[WiFi-event] event: %d\n", event);

    switch(event) {
        case WIFI_EVENT_STAMODE_GOT_IP:
            Serial.println("WiFi connected");
            Utils::printf("IP address: %s\n", WiFi.localIP().toString().c_str());
            mNextState = STATE_DISCOVERY;
            break;


        case WIFI_EVENT_STAMODE_DISCONNECTED:
            Utils::printf("WiFi lost connection\n");
            mNextState = STATE_INIT;
            break;
    }
}

static void handleKey(void)
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

            case 'r':
                mNextState = STATE_INIT;
                break;
        }
    }
    u8 flag = 0;
    if (roll != 0 || pitch != 0)
        flag = 1;
    mControl.move(flag, roll, pitch, yaw, speed);
}

static bool bebop_scanAndConnect(void)
{
    if (WiFi.isConnected()) {
        if (!strncmp(WiFi.SSID().c_str(), "BebopDrone", 10)) {
            mNextState = STATE_DISCOVERY;
            return true;
        } else {
            WiFi.disconnect();
        }
    }

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
                Utils::printf("Connect to BebopDrone !!!\n");
                return true;
            }
        }
    }
    return false;
}

static bool bebop_connectDiscovery(void)
{
    IPAddress hostIP = WiFi.localIP();
    hostIP[3] = 1;

//    WiFi.removeEvent(WiFiEvent);
    Utils::printf("bebop_connectDiscovery : %s, %d\n", hostIP.toString().c_str(), DISCOVERY_PORT);
    if (!mBebopDiscoveryClient.connect(hostIP, DISCOVERY_PORT)) {
        Utils::printf("Connection Failed !!!\n");
    } else {
        char req[200];
        sprintf(req,"{\"d2c_port\":%d, \"controller_name\":\"UniConTX\", \"controller_type\":\"computer\"}", NAV_SERVER_PORT);
        Utils::printf("to bebop : %s\n", req);
        mBebopDiscoveryClient.print(req);
        mBebopDiscoveryClient.flush();
        return true;
    }

    return false;
}

static bool bebop_handleDiscovery(void)
{
    u8      buf[256];
    char    sv[20];

    Utils::printf("Waiting response !!!\n");
    while (mBebopDiscoveryClient.available()) {
        int len = mBebopDiscoveryClient.read(buf, 256);
        if (len > 0) {
            Utils::printf("%d %s\n", len, (char*)buf);
            //{ "status": 0, "c2d_port": 54321, "arstream_fragment_size": 65000, "arstream_fragment_maximum_number": 4, "arstream_max_ack_interval": -1, "c2d_update_port": 51, "c2d_user_port": 21 }

            char *ptr = strstr((char*)buf, "\"c2d_port\":");
            if (ptr) {
                ptr += strlen("\"c2d_port\":");
                char *comma = strstr(ptr, ",");
                char szPort[20];

                strncpy(szPort, ptr, comma - ptr);
                szPort[comma - ptr] = 0;
                int port = atoi(szPort);
                Utils::printf("dev command (c2d_port):%d !!\n", port);
                mControl.setDest(mBebopDiscoveryClient.remoteIP(), port);
                mBebopDiscoveryClient.stop();
            }
            return true;
        }
    }
    return false;
}

static s16 map(s16 v)
{
    return (-5 <= v && v <= 5) ? 0 : v;
}

static u32 serialCallback(u8 cmd, u8 *data, u8 size)
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

//            Utils::printf("%3d %3d %3d %3d %3d %3d %3d %3d\n", speed, yaw, pitch, roll, aux1, aux2, aux3, aux4);
#if 1
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


#define DUR_2   (1000 / 2)
#define DUR_4   (1000 / 4)
#define DUR_8   (1000 / 8)

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(500);

//    mSerial.setCallback(serialCallback);
//    mLED.set(StatusLED::LED_GREEN, 300);
}

void loop()
{
    int  size;

    switch (mNextState) {
        case STATE_INIT:
            if (bebop_scanAndConnect()) {
                mNextState = STATE_AP_CONNECT;
            }
            break;

        case STATE_DISCOVERY:
            if (bebop_connectDiscovery()) {
                mNextState = STATE_DISCOVERY_ACK;
            }
            break;

        case STATE_DISCOVERY_ACK:
            if (bebop_handleDiscovery()) {
                mNavServer.begin();
                mNextState = STATE_CONFIG;
            }
            break;

        case STATE_CONFIG:
            if (mControl.config()) {
                mNextState = STATE_WORK;
//                mLED.set(StatusLED::LED_GREEN, 0);
            }
            size = mNavServer.process(dataAck);
            if (size > 0)
                mControl.process(dataAck, size);
            break;

        case STATE_WORK:
            size = mNavServer.process(dataAck);
            mControl.process(dataAck, size);
            break;
    }
//    handleKey();

    u8 batt = mNavServer.getBatt();
    if (!mBattWarn && batt != 0 && batt < 20) {
//        mLED.set(StatusLED::LED_PURPLE, 50);

        const u16 noteSiren[] = { 630, DUR_2, 315, DUR_2, 0xffff, 0xffff };
//        mSound.play((u16*)noteSiren, sizeof(noteSiren));
        mBattWarn = true;
    }

//    mSerial.handleRX();
//    mLED.process();
}

