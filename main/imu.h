#pragma once

#include "freertos/idf_additions.h"

#include <cstddef>
#include <cstdint>

enum class ICMRegisterMap : uint8_t {
    WhoAmI = 0x72,

    DeviceConfig = 0x7f,
    GyroConfig = 0x1c,
    AccelConfig = 0x1b,
    FifoConfig0 = 0x1d,
    FifoConfig2 = 0x20,
    FifoConfig3 = 0x21,
    PwrMgmt0 = 0x10,
    IOCPadScenarioOvrd = 0x31,
    RtcConfig = 0x26,

    FifoCount = 0x12,
    FifoData = 0x14,
};
constexpr uint8_t EXPECTED_WHO_AM_I = 0xe9;

void init_imu(QueueHandle_t packetQueue);
void configure_imu();
void imu_read_task(void *userArg);
void signal_dropped_packet(QueueHandle_t packetQueue);
void signal_fifo_overrun(QueueHandle_t packetQueue);

void write_register(ICMRegisterMap reg, uint8_t value);
void write_register(ICMRegisterMap reg, uint8_t *data, size_t length);

uint8_t read_register(ICMRegisterMap reg);
void read_register(ICMRegisterMap reg, uint8_t *buffer, size_t length);
