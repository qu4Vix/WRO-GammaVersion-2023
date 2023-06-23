/*
 *
 *
 * 
*/

#ifndef telemetry_h
#define telemetry_h

#include <WiFi.h>
#include <WiFiUdp.h>

class TelemetryManager{
    public:
    TelemetryManager(IPAddress ip, uint16_t port);
    void StartUDP(uint16_t localPort);
    void SendString(String dataString);
    void AddData(String data);
    void SendData();

    private:
    WiFiUDP _udp;
    IPAddress _receiversIP;
    uint16_t _receiversPort;
    String _dataString;
};

#endif