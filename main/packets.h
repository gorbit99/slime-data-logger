#pragma once

#include <cstddef>

constexpr uint8_t IMU_PACKET_SIZE = 20;

enum class SendCodes : uint8_t {
    Ping = 0x01,
    Data = 0x02,
    Error = 0x03,
};

enum class ReceiveCodes : uint8_t {
    Pong = 0x01,
};

struct DataPacket {
    uint8_t dataSize;
    uint8_t data[IMU_PACKET_SIZE];
};

enum class ErrorCodes : uint8_t {
    FIFOOverflow = 0x00,
    DroppedPackets = 0x01,
};

struct ErrorPacket {
    ErrorCodes errorCode;
};

union InnerData {
    DataPacket data;
    ErrorPacket error;
};

struct Packet {
    SendCodes packetId;
    InnerData data;
};
