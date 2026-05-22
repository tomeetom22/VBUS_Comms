#ifndef GATEWAY_BRIDGE_H
#define GATEWAY_BRIDGE_H

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include "air_quality_protocol.h"

namespace GatewayBridge
{

static const uint8_t MAGIC = 0xB7;
static const uint8_t VERSION = 1;
static const size_t MAX_BRIDGE_PAYLOAD_LEN = AirQuality::MAX_PACKET_LEN;

enum FrameType : uint8_t
{
  FRAME_RECORD = 1
};

enum SourceNetwork : uint8_t
{
  SOURCE_AIR_QUALITY = 1,
  SOURCE_MICROMET = 2
};

struct BridgeRecord
{
  uint8_t sourceNetwork;
  uint32_t receivedMs;
  uint8_t nodeId;
  uint32_t bootId;
  uint32_t sequence;
  int16_t rssi;
  int8_t snr;
  uint16_t payloadLength;
  uint8_t payload[MAX_BRIDGE_PAYLOAD_LEN];
};

static const size_t RECORD_HEADER_LEN = 1 + 1 + 1 + 1 + 4 + 1 + 4 + 4 + 2 + 1 + 2;
static const size_t RECORD_CRC_LEN = 2;
static const size_t MAX_RECORD_FRAME_LEN = RECORD_HEADER_LEN + MAX_BRIDGE_PAYLOAD_LEN + RECORD_CRC_LEN;

inline size_t encodeRecord(const BridgeRecord &record, uint8_t *buffer, size_t length)
{
  if (record.payloadLength > MAX_BRIDGE_PAYLOAD_LEN ||
      length < RECORD_HEADER_LEN + record.payloadLength + RECORD_CRC_LEN)
  {
    return 0;
  }

  size_t offset = 0;
  AirQuality::writeU8(buffer, offset, MAGIC);
  AirQuality::writeU8(buffer, offset, VERSION);
  AirQuality::writeU8(buffer, offset, FRAME_RECORD);
  AirQuality::writeU8(buffer, offset, record.sourceNetwork);
  AirQuality::writeU32(buffer, offset, record.receivedMs);
  AirQuality::writeU8(buffer, offset, record.nodeId);
  AirQuality::writeU32(buffer, offset, record.bootId);
  AirQuality::writeU32(buffer, offset, record.sequence);
  AirQuality::writeI16(buffer, offset, record.rssi);
  AirQuality::writeI8(buffer, offset, record.snr);
  AirQuality::writeU16(buffer, offset, record.payloadLength);
  memcpy(buffer + offset, record.payload, record.payloadLength);
  offset += record.payloadLength;
  AirQuality::writeU16(buffer, offset, AirQuality::crc16(buffer, offset));
  return offset;
}

inline bool decodeRecord(const uint8_t *buffer, size_t length, BridgeRecord &record)
{
  if (length < RECORD_HEADER_LEN + RECORD_CRC_LEN ||
      buffer[0] != MAGIC ||
      buffer[1] != VERSION ||
      buffer[2] != FRAME_RECORD)
  {
    return false;
  }

  uint16_t receivedCrc = (uint16_t)buffer[length - 2] | ((uint16_t)buffer[length - 1] << 8);
  if (receivedCrc != AirQuality::crc16(buffer, length - 2))
  {
    return false;
  }

  size_t offset = 3;
  if (!AirQuality::readU8(buffer, length, offset, record.sourceNetwork) ||
      !AirQuality::readU32(buffer, length, offset, record.receivedMs) ||
      !AirQuality::readU8(buffer, length, offset, record.nodeId) ||
      !AirQuality::readU32(buffer, length, offset, record.bootId) ||
      !AirQuality::readU32(buffer, length, offset, record.sequence) ||
      !AirQuality::readI16(buffer, length, offset, record.rssi) ||
      !AirQuality::readI8(buffer, length, offset, record.snr) ||
      !AirQuality::readU16(buffer, length, offset, record.payloadLength))
  {
    return false;
  }

  if (record.payloadLength > MAX_BRIDGE_PAYLOAD_LEN ||
      offset + record.payloadLength + RECORD_CRC_LEN != length)
  {
    return false;
  }

  memcpy(record.payload, buffer + offset, record.payloadLength);
  return true;
}

} // namespace GatewayBridge

#endif
