#include <Arduino.h>
#include <stdarg.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include "Commands.h"
#include "CmdServer.h"
#include "ByteBuffer.h"
#include "SerialProtocol.h"
#include "BridgeServer.h"

extern "C" {
#include "user_interface.h"
}

enum {
    STATE_INIT = 0,
    STATE_AP_CONNECT,
    STATE_DISCOVERY,
    STATE_DISCOVERY_ACK,
    STATE_CONFIG,
    STATE_WORK,
};

#define DISCOVERY_PORT      44444
#define BRG_CMD_SERVER_PORT 51000
#define BRG_NAV_SERVER_PORT 52000

static SerialProtocol   mSerial;
static WiFiClient       mBebopDiscoveryClient;

static u8               mNextState = STATE_INIT;
static char             mStrDiscovery2App[200];

static BridgeServer     mCmdBridge("CMD_BRG", BRG_CMD_SERVER_PORT);
static BridgeServer     mNavBridge("NAV_BRG", BRG_NAV_SERVER_PORT);

static u8          mac[20];
static WiFiServer  mAppDiscoveryServer(DISCOVERY_PORT);
static WiFiClient  mAppDiscoveryClient;


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


static u8 dataAck[4096];
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

bool bebop_scanAndConnect(void)
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
                Utils::printf("Connect to BebopDrone !!!\n");
                mSerial.sendCmd(SerialProtocol::CMD_SET_STATE, &mNextState, 1);
                return true;
            }
        }
    }
    return false;
}

bool bebop_connectDiscovery(void)
{
    IPAddress hostIP = WiFi.localIP();
    hostIP[3] = 1;

    WiFi.removeEvent(WiFiEvent);
    Utils::printf("bebop_connectDiscovery : %s, %d\n", hostIP.toString().c_str(), DISCOVERY_PORT);
    if (!mBebopDiscoveryClient.connect(hostIP, DISCOVERY_PORT)) {
        Utils::printf("Connection Failed !!!\n");
    } else {
        char req[200];
        sprintf(req,"{\"d2c_port\":%d, \"controller_name\":\"UniConTX\", \"controller_type\":\"computer\"}", BRG_NAV_SERVER_PORT);
        Utils::printf("to bebop : %s\n", req);
        mBebopDiscoveryClient.print(req);
        mBebopDiscoveryClient.flush();
        mSerial.sendCmd(SerialProtocol::CMD_SET_STATE, &mNextState, 1);
        return true;
    }

    return false;
}

bool bebop_handleDiscovery(void)
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
                mCmdBridge.setHost(mBebopDiscoveryClient.remoteIP(), port);

                memset(mStrDiscovery2App, 0, sizeof(mStrDiscovery2App));
                strncpy(mStrDiscovery2App, (char*)buf, ptr - (char*)&buf[0]);
                strcat(mStrDiscovery2App, itoa(BRG_CMD_SERVER_PORT, sv, 10));
                strcat(mStrDiscovery2App, comma);
                Utils::printf("prepare App discovery msg (c2d_port):%s !!\n", mStrDiscovery2App);
                mBebopDiscoveryClient.stop();
            }
            mSerial.sendCmd(SerialProtocol::CMD_SET_STATE, &mNextState, 1);
            return true;
        }
        
    }

    return false;
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

void setupNetwork(void)
{
    IPAddress   apIP(192, 168, 77, 1);
    
    sprintf(mStrDiscovery2App, 
        "{ \"status\": 0, \"c2d_port\": %d, \"arstream_fragment_size\": 65000, \"arstream_fragment_maximum_number\": 4, \"arstream_max_ack_interval\": -1, \"c2d_update_port\": 51, \"c2d_user_port\": 21 }", 
        BRG_CMD_SERVER_PORT);

    Utils::printf("\n\nReady !!! : %08x\n", ESP.getChipId());
    mSerial.setCallback(serialCallback);

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("BebopDrone-Bridge");
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    delay(500);

    wifi_get_macaddr(0, mac);
    Utils::printf("STA_MAC:%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    wifi_get_macaddr(1, mac);
    Utils::printf("AP_MAC :%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

#if 0
    mac[0] = 0x02;
    mac[1] = 0xAE;
    mac[2] = 0x5E;
    mac[3] = 0x03;
    mac[4] = 0x51;
    mac[5] = 0x14;
    wifi_set_macaddr(1, mac);
    Utils::printf("AP_MAC :%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif

    if (!MDNS.begin("arsdk-0901")) {
        Serial.println("Error setting up MDNS responder!");
        for(;;);
    } else {
        MDNS.setInstanceName("BebopDrone-E035114");
        MDNS.addService("arsdk-0901", "udp", DISCOVERY_PORT);
        MDNS.addServiceTxt("arsdk-0901", "udp" ,"{\"device_id\":\"PI040339AG5E035114\"}", " ");

        Utils::printf("mDNS started !!!\n");
        delay(100);
    }
}

void setup() {
    Serial.begin(57600);
    setupNetwork();
}

bool app_handleDiscovery(void)
{
    u8      buf[256];
    char    sv[20];
    
    if (mAppDiscoveryServer.hasClient()) {
        if (!mAppDiscoveryClient || !mAppDiscoveryClient.connected()) {
            if (mAppDiscoveryClient)
                mAppDiscoveryClient.stop();

            mAppDiscoveryClient = mAppDiscoveryServer.available();
            Utils::printf("New Client !!\n");
        }
    }

    if (mAppDiscoveryClient && mAppDiscoveryClient.connected()) {
        mAppDiscoveryClient.read(buf, mAppDiscoveryClient.available());
        char *ptr = strstr((char*)buf, "\"d2c_port\":");
        if (ptr) {
            ptr += strlen("\"d2c_port\":");
            char *comma = strstr(ptr, ",");
            char szPort[20];
            
            strncpy(szPort, ptr, comma - ptr);
            szPort[comma - ptr] = 0;
            int port = atoi(szPort);

            mNavBridge.setHost(mAppDiscoveryClient.remoteIP(), port);
            Utils::printf("app nav port (d2c_port):%d  %s!!\n", port, mStrDiscovery2App);
            mAppDiscoveryClient.print(mStrDiscovery2App);
            return true;
        }
    }
    return false;
}

void loop()
{
    int  size;

    switch (mNextState) {
        case STATE_INIT:
            if (bebop_scanAndConnect())
                mNextState = STATE_AP_CONNECT;
            break;

        case STATE_DISCOVERY:
            if (bebop_connectDiscovery()) {
                mNextState = STATE_DISCOVERY_ACK;
            }
            break;

        case STATE_DISCOVERY_ACK:
            if (bebop_handleDiscovery()) {
                mAppDiscoveryServer.begin();

                mNavBridge.begin();
                mNavBridge.setBypass(false);
                mCmdBridge.begin();
                mCmdBridge.setBypass(false);
                
                mNextState = STATE_CONFIG;
            }
            break;

        case STATE_CONFIG:
            if (app_handleDiscovery()) {
                mCmdBridge.setBypass(true);
                mNavBridge.setBypass(true);
                mNextState = STATE_WORK;
            }

            size = mNavBridge.process(dataAck);
            if (size > 0) {
                mCmdBridge.sendto(dataAck, size);
            }
            mCmdBridge.kick();
            
            break;

        case STATE_WORK:
            mCmdBridge.process(dataAck);
            mNavBridge.process(dataAck);
            break;

#if 0
        case STATE_CONFIG:
            if (mControl.config()) {
                mNextState = STATE_WORK;
                mSerial.sendCmd(SerialProtocol::CMD_SET_STATE, &mNextState, 1);
            }

            size = mNavBridge.process(dataAck);
            if (size > 0)
                mControl.process(dataAck, size);
            break;

        case STATE_WORK:
            size = mNavBridge.process(dataAck);
            mControl.process(dataAck, size);
            break;
#endif            
    }
    mSerial.handleRX();
}

