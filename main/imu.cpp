#include "imu.h"

#include "driver/i2c_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "packets.h"
#include "portmacro.h"

#include <cstring>
#include <driver/i2c_master.h>

static const char *TAG = "IMU";

constexpr uint8_t ICM45_DEVICE_ADDRESS = 0x68;

i2c_master_bus_handle_t busHandle;
i2c_master_dev_handle_t deviceHandle;

void init_imu(QueueHandle_t packetQueue) {
    i2c_master_bus_config_t i2cBusConfig = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = static_cast<gpio_num_t>(CONFIG_SDA_PIN),
            .scl_io_num = static_cast<gpio_num_t>(CONFIG_SCL_PIN),
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags =
                    {
                            .enable_internal_pullup = false,
                    },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2cBusConfig, &busHandle));

    i2c_device_config_t i2cDeviceConfig = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = ICM45_DEVICE_ADDRESS,
            .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(busHandle,
                                              &i2cDeviceConfig,
                                              &deviceHandle));

    auto result = i2c_master_probe(busHandle, ICM45_DEVICE_ADDRESS, -1);
    if (result != ESP_OK) {
        result = i2c_master_probe(busHandle, ICM45_DEVICE_ADDRESS, -1);
    }
    ESP_ERROR_CHECK(result);

    uint8_t whoAmI = read_register(ICMRegisterMap::WhoAmI);
    if (whoAmI != EXPECTED_WHO_AM_I) {
        ESP_LOGE(TAG, "WhoAmI value didn't match! Received: %02x", whoAmI);
        abort();
    }

    ESP_LOGI(TAG, "IMU Found!");

    configure_imu();

    xTaskCreate(imu_read_task, "imu_read", 4096, &packetQueue, 5, nullptr);
}

void configure_imu() {
    write_register(ICMRegisterMap::DeviceConfig, 0x03); // Soft reset
    vTaskDelay(35 / portTICK_PERIOD_MS);

    write_register(ICMRegisterMap::IOCPadScenarioOvrd,
                   (0b1 << 2)              // Enable override
                           | (0b10 << 0)); // Pin 9 as CLKIN
    write_register(ICMRegisterMap::RtcConfig,
                   (0b1 << 6)              // Realign RTC
                           | (0b0 << 5)    // RTC not enabled
                           | (0b11 << 0)); // Reset value bits
    write_register(ICMRegisterMap::GyroConfig,
                   (0b0000 << 4)             // 4000dps
                           | (0b0111 << 0)); // 409.6Hz
    write_register(ICMRegisterMap::AccelConfig,
                   (0b000 << 4)              // 32g
                           | (0b1001 << 0)); // 102.4 Hz
    write_register(ICMRegisterMap::FifoConfig0,
                   (0b10 << 6)                 // Stop on full FIFO mode
                           | (0b011111 << 0)); // 8k bytes FIFO depth
    write_register(ICMRegisterMap::FifoConfig3,
                   (0b1 << 3)             // Enable hires
                           | (0b1 << 2)   // Enable gyro
                           | (0b1 << 1)   // Enable accel
                           | (0b1 << 0)); // Enable FIFO
    write_register(ICMRegisterMap::PwrMgmt0,
                   (0b11 << 2)             // Gyro in low-noise modde
                           | (0b11 << 0)); // Accel in low-noise mode
}

void imu_read_task(void *userArg) {
    QueueHandle_t packetQueue = *reinterpret_cast<QueueHandle_t *>(userArg);
    uint8_t fifoBuffer[IMU_PACKET_SIZE * 8];

    Packet packet;
    packet.packetId = SendCodes::Data;
    packet.data.data.dataSize = IMU_PACKET_SIZE;

    while (true) {
        uint16_t fifoCount;
        read_register(ICMRegisterMap::FifoCount,
                      reinterpret_cast<uint8_t *>(&fifoCount),
                      sizeof(fifoCount));

        if (fifoCount == 0) {
            continue;
        }

        if (fifoCount == 409) {
            signal_fifo_overrun(packetQueue);
            write_register(ICMRegisterMap::FifoConfig2,
                           (0b1 << 7)         // Flush FIFO
                                   | (0x20)); // Reset value
            continue;
        }

        if (fifoCount > 8) {
            fifoCount = 8;
        }

        read_register(ICMRegisterMap::FifoData,
                      fifoBuffer,
                      fifoCount * IMU_PACKET_SIZE);

        for (size_t i = 0; i < fifoCount; i++) {
            memcpy(packet.data.data.data,
                   &fifoBuffer[IMU_PACKET_SIZE * i],
                   IMU_PACKET_SIZE);

            auto result = xQueueSend(packetQueue, &packet, 0);
            if (result != pdPASS) {
                signal_dropped_packet(packetQueue);
                break;
            }
        }
    }
};

void signal_dropped_packet(QueueHandle_t packetQueue) {
    Packet packet;
    packet.packetId = SendCodes::Error;
    packet.data.error.errorCode = ErrorCodes::DroppedPackets;

    xQueueReset(packetQueue);
    xQueueSend(packetQueue, &packet, portMAX_DELAY);
}

void signal_fifo_overrun(QueueHandle_t packetQueue) {
    Packet packet;
    packet.packetId = SendCodes::Error;
    packet.data.error.errorCode = ErrorCodes::FIFOOverflow;

    xQueueReset(packetQueue);
    xQueueSend(packetQueue, &packet, portMAX_DELAY);
}

void write_register(ICMRegisterMap reg, uint8_t value) {
    write_register(reg, &value, sizeof(value));
}

void write_register(ICMRegisterMap reg, uint8_t *data, size_t length) {
    i2c_master_transmit_multi_buffer_info_t writeInfo[]{
            {
                    .write_buffer = reinterpret_cast<uint8_t *>(&reg),
                    .buffer_size = sizeof(reg),
            },
            {
                    .write_buffer = data,
                    .buffer_size = length,
            },
    };

    i2c_master_multi_buffer_transmit(deviceHandle,
                                     writeInfo,
                                     sizeof(writeInfo) / sizeof(*writeInfo),
                                     -1);
}

uint8_t read_register(ICMRegisterMap reg) {
    uint8_t result;
    read_register(reg, &result, sizeof(result));
    return result;
}

void read_register(ICMRegisterMap reg, uint8_t *buffer, size_t length) {
    i2c_master_transmit_receive(deviceHandle,
                                reinterpret_cast<uint8_t *>(&reg),
                                sizeof(reg),
                                buffer,
                                length,
                                -1);
}
