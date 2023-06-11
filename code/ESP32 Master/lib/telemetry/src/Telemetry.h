/*
 *
 *
 * 
*/

#ifndef telemetry_h
#define telemetry_h

#include <WiFiUdp.h>

class TelemetryManager{
    public:
    TelemetryManager();
    void StartUDP(uint16_t localPort);
    void SendData();

    private:
    WiFiUDP udp;
};

#endif