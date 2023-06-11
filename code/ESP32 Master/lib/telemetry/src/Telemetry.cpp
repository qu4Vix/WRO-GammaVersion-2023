/*
 *
 *
 * 
*/

#include "Telemetry.h"

TelemetryManager::TelemetryManager() {

}

void TelemetryManager::StartUDP(uint16_t localPort) {
    udp.begin(localPort);
}

void TelemetryManager::SendData() {
    
}