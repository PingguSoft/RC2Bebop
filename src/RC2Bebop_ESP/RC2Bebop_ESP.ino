#include <Arduino.h>
#include <stdarg.h>
#include <ESP8266WiFi.h>
#include "Commands.h"
#include "NavServer.h"
#include "ByteBuffer.h"
#include "RCRcvrPPM.h"
#include "Telemetry.h"

#define FLAG_TAKE_OFF       0
#define FLAG_VIDEO_REC      1

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

static WiFiClient       mBebopDiscoveryClient;
static Commands         mControl;
static Telemetry        mTM;
static NavServer        mNavServer(NAV_SERVER_PORT, &mTM);
static u8               mNextState = STATE_INIT;
static RCRcvrPPM        mRcvr;

static u8  dataAck[1024];
static u8  recVideo = 0;
static s8  speed = 0;
static s8  roll = 0;
static s8  pitch = 0;
static s8  yaw = 0;
static s8  aux1 = 0;
static s8  aux2 = 0;
static s8  aux3 = 0;
static s8  aux4 = 0;
static u32 mFlag = 0;

static void handleKey(void)
{
    int size;

    size = Serial.available();
    while(size--) {
        u8 ch = Serial.read();
        LOG("***** : %c\n", ch);

        switch (ch) {
            case 'p' : mControl.takePicture();                                  break;
            case 'v' : recVideo = !recVideo;    mControl.recordVideo(recVideo); break;

            case 'a' : mControl.takeOff();                                      break;
            case 'z' : mControl.land();                                         break;

            case 's' :
                speed += (speed < 90) ? 10 : 0;
                LOG("thr : %d\n", speed);
                break;

            case 'x' :
                speed += (speed > -90) ? -10 : 0;
                LOG("thr : %d\n", speed);
                break;

            case 'i' :
                pitch += (pitch < 90) ? 10 : 0;
                LOG("pitch : %d\n", pitch);
                break;

            case 'k' :
                pitch += (pitch > -90) ? -10 : 0;
                LOG("pitch : %d\n", pitch);
                break;

            case 'j' :
                roll += (roll > -90) ? -10 : 0;
                LOG("roll : %d\n", roll);
                break;

            case 'l' :
                roll += (roll < 90) ? 10 : 0;
                LOG("roll : %d\n", roll);
                break;

            case 'u' :
                yaw += (yaw > -90) ? -10 : 0;
                LOG("yaw : %d\n", yaw);
                break;

            case 'o' :
                yaw += (yaw < 90) ? 10 : 0;
                LOG("yaw : %d\n", yaw);
                break;

            case ' ' :
                yaw = 0;
                roll = 0;
                pitch = 0;
                speed = 0;
                LOG("reset pos\n");
                break;

            case 'r':
                mNextState = STATE_INIT;
                break;

            case 't':
                mTM.setVolt(0, 100, 10);
                mTM.setVolt(1, 90, 10);
                mTM.setVolt(2, 80, 10);
                break;
        }
    }
    u8 flag = 0;
    if (roll != 0 || pitch != 0)
        flag = 1;
    mControl.move(flag, roll, pitch, yaw, speed);
}

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

static bool bebop_scanAndConnect(void)
{
    if (mNextState != STATE_INIT)
        return false;

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
        LOG("no networks found\n");
    } else {
        LOG("\n\n%d networks found\n", n);
        for (int i = 0; i < n; i++) {
            LOG("%d : %s (%d) %d\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.encryptionType(i));
            if (!strncmp(WiFi.SSID(i).c_str(), "BebopDrone", 10)) {
                LOG("Connect to BebopDrone !!!\n");
                mNextState = STATE_AP_CONNECT;

                WiFi.begin(WiFi.SSID(i).c_str(), "");
                WiFi.onEvent(WiFiEvent);
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
    LOG("bebop_connectDiscovery : %s, %d\n", hostIP.toString().c_str(), DISCOVERY_PORT);
    if (!mBebopDiscoveryClient.connect(hostIP, DISCOVERY_PORT)) {
        LOG("Connection Failed !!!\n");
    } else {
        char req[200];
        sprintf(req,"{\"d2c_port\":%d, \"controller_name\":\"UniConTX\", \"controller_type\":\"computer\"}", NAV_SERVER_PORT);
        LOG("to bebop : %s\n", req);
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

    LOG("Waiting response !!!\n");
    while (mBebopDiscoveryClient.available()) {
        int len = mBebopDiscoveryClient.read(buf, 256);
        if (len > 0) {
            LOG("%d %s\n", len, (char*)buf);
            //{ "status": 0, "c2d_port": 54321, "arstream_fragment_size": 65000, "arstream_fragment_maximum_number": 4, "arstream_max_ack_interval": -1, "c2d_update_port": 51, "c2d_user_port": 21 }

            char *ptr = strstr((char*)buf, "\"c2d_port\":");
            if (ptr) {
                ptr += strlen("\"c2d_port\":");
                char *comma = strstr(ptr, ",");
                char szPort[20];

                strncpy(szPort, ptr, comma - ptr);
                szPort[comma - ptr] = 0;
                int port = atoi(szPort);

                IPAddress hostIP = WiFi.localIP();
                hostIP[3] = 1;

                LOG("dev command to %s (c2d_port):%d !!\n", hostIP.toString().c_str() , port);
                mControl.setDest(hostIP, port);
                mBebopDiscoveryClient.stop();
            }
            return true;
        }
    }
    return false;
}

void setup() {
    Serial1.begin(115200);
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.persistent(false);
    delay(500);
}

#define AUX_SW_ON   50  //(CHAN_MID_VALUE + CHAN_MAX_VALUE / 2)

void loop()
{
    int  size;

#if 1
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
                mRcvr.init();
                mNextState = STATE_WORK;
            }
            size = mNavServer.process(dataAck);
            if (size > 0)
                mControl.process(dataAck, size);
            break;

        case STATE_WORK:
            {
                u8  flag = 0;

                roll  = mRcvr.getRC(CH_AILERON,  -100, 100);
                pitch = mRcvr.getRC(CH_ELEVATOR, -100, 100);
                yaw   = mRcvr.getRC(CH_RUDDER,   -100, 100);
                speed = mRcvr.getRC(CH_THROTTLE, -100, 100);
                aux1  = mRcvr.getRC(CH_AUX1,     -100, 100);
                aux2  = mRcvr.getRC(CH_AUX2,     -100, 100);
                aux3  = mRcvr.getRC(CH_AUX3,     -100, 100);
                aux4  = mRcvr.getRC(CH_AUX4,     -100, 100);

                if (BIT_IS_CLR(mFlag, FLAG_TAKE_OFF) && aux1 >= AUX_SW_ON) {
                    BIT_SET(mFlag, FLAG_TAKE_OFF);
                    mControl.takeOff();
                } else if (BIT_IS_SET(mFlag, FLAG_TAKE_OFF) && aux1 < AUX_SW_ON) {
                    BIT_CLR(mFlag, FLAG_TAKE_OFF);
                    mControl.land();
                }

                if (BIT_IS_CLR(mFlag, FLAG_VIDEO_REC) && aux2 >= AUX_SW_ON) {
                    BIT_SET(mFlag, FLAG_VIDEO_REC);
                    mControl.recordVideo(TRUE);
                } else if (BIT_IS_SET(mFlag, FLAG_VIDEO_REC) && aux2 < AUX_SW_ON) {
                    BIT_CLR(mFlag, FLAG_VIDEO_REC);
                    mControl.recordVideo(FALSE);
                }

                if (roll != 0 || pitch != 0)
                    flag = 1;

                mControl.move(flag, roll, pitch, yaw, speed);

//                LOG("T:%4d R:%4d E:%4d A:%4d %4d %4d %4d %4d [%4d %4d]\n", mRcvr.getRC(0), mRcvr.getRC(1), mRcvr.getRC(2), mRcvr.getRC(3), mRcvr.getRC(4),
//                    mRcvr.getRC(5), mRcvr.getRC(6), mRcvr.getRC(7), mRcvr.getRC(8), mRcvr.getRC(9), mRcvr.getRC(10));
            }
            size = mNavServer.process(dataAck);
            mControl.process(dataAck, size);
            mTM.update();
            break;
    }
#endif
}

