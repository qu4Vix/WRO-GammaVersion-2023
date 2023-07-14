/*
 * Motor.h - Library for controling an engine with Arduino Framework.
 *
 * Created by the Gamma Version Team, 2023
 * 
 * 
 */

#include "AdvancedMPU.h"

MPU::MPU() {
    
}

void MPU::BeginWire(byte pinSDA, byte pinSCL, uint32_t freq) {
    _pinSDA = pinSDA;
    _pinSCL = pinSCL;
    while (!Wire1.begin(int(_pinSDA), int(_pinSCL), freq)) {
        // Wire Connection failed
        delay(200);
        //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}

void MPU::Setup() {
    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    while(!_mpu.setup(0x68, setting, Wire1)) {  // change to your own address
        //Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
        delay(300);
        //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

    //loadCalibration();
}

void MPU::WorkOffset() {
    int num = 0;
    float tot = 0;
    while (num < 1000) {
        if (_mpu.update()) {
            num++;
            tot += _mpu.getGyroZ();
        }
        delay(5);
    }
    _offset = tot / num;
    Serial.println("Offset = " + String(_offset));
}

void MPU::UpdateAngle() {
    if (_mpu.update()) {
        unsigned long sampleDuration = millis() - _prev_ms_angle;
        float gyroZ = _mpu.getGyroZ();
        _prev_ms_angle = millis();
        _angle += ((gyroZ - _offset) * sampleDuration / 1000);
        //Serial.println(_angle);
    }
}

float MPU::GetAngle() {
    return _angle;
}

void MPU::measureFirstMillis() {
    if (!_firstMillisRead) {
        _prev_ms_angle = millis();
        _firstMillisRead = true;
    }
}