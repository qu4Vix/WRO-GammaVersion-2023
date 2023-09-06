// LED pins
#define pinLED_rojo 12
#define pinLED_verde 13

// Battery level LED pins
#define pinLED_batRojo 27
#define pinLED_batAmarillo 26
#define pinLED_batVerde 25

// LIDAR pins (UART2 + PWM)
#define pinLIDAR_TX 16
#define pinLIDAR_RX 17
#define pinLIDAR_motor 5

// MPU pins (I2C)
#define pinMPU_SDA 21
#define pinMPU_SCL 22

// Interchip Communication pins (UART)
#define pinTX 2
#define pinRX 4

// Button pin
#define pinBoton 33

// Telemetry serial pins
#define telemetriaTX 19
#define telemetriaRX 23

// Pin Modes
void setPinModes() {
    pinMode(pinBoton, INPUT_PULLUP);
    pinMode(pinLED_rojo, OUTPUT);
    pinMode(pinLED_verde, OUTPUT);
    pinMode(pinLED_batRojo, OUTPUT);
    pinMode(pinLED_batAmarillo, OUTPUT);
    pinMode(pinLED_batVerde, OUTPUT);
    pinMode(pinLIDAR_motor, OUTPUT);
}