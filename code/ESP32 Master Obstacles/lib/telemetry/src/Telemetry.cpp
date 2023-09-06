/*
 *
 *
 * 
*/

#include "Telemetry.h"

TelemetryManager::TelemetryManager(IPAddress ip, uint16_t port) {
    _receiversIP = ip;
    _receiversPort = port;
}

void TelemetryManager::StartUDP(uint16_t localPort) {
    while(!_udp.begin(localPort)) {
        delay(100);
    }
}

void TelemetryManager::SendString(String dataString) {
    _udp.beginPacket(_receiversIP, _receiversPort);
    _udp.printf(dataString.c_str());
    _udp.endPacket();
}

void TelemetryManager::AddData(String data) {
    _dataString += data + ',';
}

void TelemetryManager::SendData() {
    SendString(_dataString);
    _dataString = "";
}