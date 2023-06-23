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
    _udp.begin(localPort);
}

void TelemetryManager::SendString(String dataString) {
    _udp.beginPacket(_receiversIP, _receiversPort);
    _udp.print(dataString);
    _udp.endPacket();
}

void TelemetryManager::AddData(String data) {
    _dataString += data + ',';
}

void TelemetryManager::SendData() {
    if (_dataString[sizeof(_dataString)] == ',') {
        _dataString[sizeof(_dataString)] = '\0';
    }
    SendString(_dataString);
}