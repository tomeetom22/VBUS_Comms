#ifndef AIR_QUALITY_PROTOCOL_H
#define AIR_QUALITY_PROTOCOL_H

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

namespace AirQuality
{

static const uint8_t MAGIC = 0xA1;
static const uint8_t VERSION = 1;
static const uint16_t DEFAULT_NETWORK_ID = 0x5642; // "VB"

static const uint8_t NODE_COUNT = 10;
static const uint8_t CHANNEL_COUNT = 6;
static const uint8_t SAMPLE_COUNT = 10;
static const uint32_t FRAME_MS = 10000;
static const uint32_t SLOT_MS = 900;
static const uint32_t SLOT_GUARD_MS = 100;
static const uint32_t SYNC_TIMEOUT_MS = 30000;
static const uint8_t BACKFILL_FRAME_INTERVAL = 6;
static const int16_t MISSING_SAMPLE = INT16_MIN;

enum PacketType : uint8_t
{
  PACKET_BEACON = 1,
  PACKET_BATCH = 2,
  PACKET_ACK = 3
};

enum StatusFlags : uint16_t
{
  STATUS_SENSOR_FAULT = 1 << 0,
  STATUS_SENSOR_SATURATED = 1 << 1,
  STATUS_STALE_SAMPLE = 1 << 2,
  STATUS_WINDOWED_SAMPLE = 1 << 3,
  STATUS_SD_SAVE_FAILED = 1 << 4,
  STATUS_BACKFILL_BATCH = 1 << 5
};

struct BeaconPacket
{
  uint16_t networkId;
  uint32_t frameCounter;
  uint32_t gatewayMs;
  uint16_t frameMs;
  uint16_t slotMs;
};

struct BatchPacket
{
  uint16_t networkId;
  uint8_t nodeId;
  uint32_t bootId;
  uint32_t sequence;
  uint32_t sampleStartMs;
  uint16_t sampleIntervalMs;
  uint16_t batteryMv;
  uint16_t statusFlags;
  int16_t samples[SAMPLE_COUNT][CHANNEL_COUNT];
};

struct AckPacket
{
  uint16_t networkId;
  uint8_t nodeId;
  uint32_t bootId;
  uint32_t sequence;
  uint32_t gatewayFrameCounter;
  uint32_t gatewayMs;
  int16_t rssi;
  int8_t snr;
  uint8_t backfillHint;
};

static const size_t BEACON_PACKET_LEN = 1 + 1 + 1 + 2 + 4 + 4 + 2 + 2 + 2;
static const size_t BATCH_PACKET_LEN = 1 + 1 + 1 + 2 + 1 + 4 + 4 + 4 + 2 + 2 + 2 + (SAMPLE_COUNT * CHANNEL_COUNT * 2) + 2;
static const size_t ACK_PACKET_LEN = 1 + 1 + 1 + 2 + 1 + 4 + 4 + 4 + 4 + 2 + 1 + 1 + 2;
static const size_t MAX_PACKET_LEN = BATCH_PACKET_LEN;

inline uint16_t crc16(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (crc & 0x0001)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  return crc;
}

inline void writeU8(uint8_t *buffer, size_t &offset, uint8_t value)
{
  buffer[offset++] = value;
}

inline void writeI8(uint8_t *buffer, size_t &offset, int8_t value)
{
  buffer[offset++] = (uint8_t)value;
}

inline void writeU16(uint8_t *buffer, size_t &offset, uint16_t value)
{
  buffer[offset++] = (uint8_t)(value & 0xFF);
  buffer[offset++] = (uint8_t)(value >> 8);
}

inline void writeI16(uint8_t *buffer, size_t &offset, int16_t value)
{
  writeU16(buffer, offset, (uint16_t)value);
}

inline void writeU32(uint8_t *buffer, size_t &offset, uint32_t value)
{
  buffer[offset++] = (uint8_t)(value & 0xFF);
  buffer[offset++] = (uint8_t)((value >> 8) & 0xFF);
  buffer[offset++] = (uint8_t)((value >> 16) & 0xFF);
  buffer[offset++] = (uint8_t)((value >> 24) & 0xFF);
}

inline bool readU8(const uint8_t *buffer, size_t length, size_t &offset, uint8_t &value)
{
  if (offset + 1 > length)
  {
    return false;
  }
  value = buffer[offset++];
  return true;
}

inline bool readI8(const uint8_t *buffer, size_t length, size_t &offset, int8_t &value)
{
  uint8_t raw;
  if (!readU8(buffer, length, offset, raw))
  {
    return false;
  }
  value = (int8_t)raw;
  return true;
}

inline bool readU16(const uint8_t *buffer, size_t length, size_t &offset, uint16_t &value)
{
  if (offset + 2 > length)
  {
    return false;
  }
  value = (uint16_t)buffer[offset] | ((uint16_t)buffer[offset + 1] << 8);
  offset += 2;
  return true;
}

inline bool readI16(const uint8_t *buffer, size_t length, size_t &offset, int16_t &value)
{
  uint16_t raw;
  if (!readU16(buffer, length, offset, raw))
  {
    return false;
  }
  value = (int16_t)raw;
  return true;
}

inline bool readU32(const uint8_t *buffer, size_t length, size_t &offset, uint32_t &value)
{
  if (offset + 4 > length)
  {
    return false;
  }
  value = (uint32_t)buffer[offset] |
          ((uint32_t)buffer[offset + 1] << 8) |
          ((uint32_t)buffer[offset + 2] << 16) |
          ((uint32_t)buffer[offset + 3] << 24);
  offset += 4;
  return true;
}

inline bool hasValidEnvelope(const uint8_t *buffer, size_t length, PacketType expectedType)
{
  if (length < 7)
  {
    return false;
  }
  if (buffer[0] != MAGIC || buffer[1] != VERSION || buffer[2] != (uint8_t)expectedType)
  {
    return false;
  }

  uint16_t receivedCrc = (uint16_t)buffer[length - 2] | ((uint16_t)buffer[length - 1] << 8);
  return receivedCrc == crc16(buffer, length - 2);
}

inline size_t encodeBeacon(const BeaconPacket &packet, uint8_t *buffer, size_t length)
{
  if (length < BEACON_PACKET_LEN)
  {
    return 0;
  }
  size_t offset = 0;
  writeU8(buffer, offset, MAGIC);
  writeU8(buffer, offset, VERSION);
  writeU8(buffer, offset, PACKET_BEACON);
  writeU16(buffer, offset, packet.networkId);
  writeU32(buffer, offset, packet.frameCounter);
  writeU32(buffer, offset, packet.gatewayMs);
  writeU16(buffer, offset, packet.frameMs);
  writeU16(buffer, offset, packet.slotMs);
  writeU16(buffer, offset, crc16(buffer, offset));
  return offset;
}

inline bool decodeBeacon(const uint8_t *buffer, size_t length, BeaconPacket &packet)
{
  if (length != BEACON_PACKET_LEN || !hasValidEnvelope(buffer, length, PACKET_BEACON))
  {
    return false;
  }
  size_t offset = 3;
  return readU16(buffer, length, offset, packet.networkId) &&
         readU32(buffer, length, offset, packet.frameCounter) &&
         readU32(buffer, length, offset, packet.gatewayMs) &&
         readU16(buffer, length, offset, packet.frameMs) &&
         readU16(buffer, length, offset, packet.slotMs);
}

inline size_t encodeBatch(const BatchPacket &packet, uint8_t *buffer, size_t length)
{
  if (length < BATCH_PACKET_LEN)
  {
    return 0;
  }
  size_t offset = 0;
  writeU8(buffer, offset, MAGIC);
  writeU8(buffer, offset, VERSION);
  writeU8(buffer, offset, PACKET_BATCH);
  writeU16(buffer, offset, packet.networkId);
  writeU8(buffer, offset, packet.nodeId);
  writeU32(buffer, offset, packet.bootId);
  writeU32(buffer, offset, packet.sequence);
  writeU32(buffer, offset, packet.sampleStartMs);
  writeU16(buffer, offset, packet.sampleIntervalMs);
  writeU16(buffer, offset, packet.batteryMv);
  writeU16(buffer, offset, packet.statusFlags);
  for (uint8_t sample = 0; sample < SAMPLE_COUNT; sample++)
  {
    for (uint8_t channel = 0; channel < CHANNEL_COUNT; channel++)
    {
      writeI16(buffer, offset, packet.samples[sample][channel]);
    }
  }
  writeU16(buffer, offset, crc16(buffer, offset));
  return offset;
}

inline bool decodeBatch(const uint8_t *buffer, size_t length, BatchPacket &packet)
{
  if (length != BATCH_PACKET_LEN || !hasValidEnvelope(buffer, length, PACKET_BATCH))
  {
    return false;
  }
  size_t offset = 3;
  if (!readU16(buffer, length, offset, packet.networkId) ||
      !readU8(buffer, length, offset, packet.nodeId) ||
      !readU32(buffer, length, offset, packet.bootId) ||
      !readU32(buffer, length, offset, packet.sequence) ||
      !readU32(buffer, length, offset, packet.sampleStartMs) ||
      !readU16(buffer, length, offset, packet.sampleIntervalMs) ||
      !readU16(buffer, length, offset, packet.batteryMv) ||
      !readU16(buffer, length, offset, packet.statusFlags))
  {
    return false;
  }

  for (uint8_t sample = 0; sample < SAMPLE_COUNT; sample++)
  {
    for (uint8_t channel = 0; channel < CHANNEL_COUNT; channel++)
    {
      if (!readI16(buffer, length, offset, packet.samples[sample][channel]))
      {
        return false;
      }
    }
  }
  return true;
}

inline size_t encodeAck(const AckPacket &packet, uint8_t *buffer, size_t length)
{
  if (length < ACK_PACKET_LEN)
  {
    return 0;
  }
  size_t offset = 0;
  writeU8(buffer, offset, MAGIC);
  writeU8(buffer, offset, VERSION);
  writeU8(buffer, offset, PACKET_ACK);
  writeU16(buffer, offset, packet.networkId);
  writeU8(buffer, offset, packet.nodeId);
  writeU32(buffer, offset, packet.bootId);
  writeU32(buffer, offset, packet.sequence);
  writeU32(buffer, offset, packet.gatewayFrameCounter);
  writeU32(buffer, offset, packet.gatewayMs);
  writeI16(buffer, offset, packet.rssi);
  writeI8(buffer, offset, packet.snr);
  writeU8(buffer, offset, packet.backfillHint);
  writeU16(buffer, offset, crc16(buffer, offset));
  return offset;
}

inline bool decodeAck(const uint8_t *buffer, size_t length, AckPacket &packet)
{
  if (length != ACK_PACKET_LEN || !hasValidEnvelope(buffer, length, PACKET_ACK))
  {
    return false;
  }
  size_t offset = 3;
  return readU16(buffer, length, offset, packet.networkId) &&
         readU8(buffer, length, offset, packet.nodeId) &&
         readU32(buffer, length, offset, packet.bootId) &&
         readU32(buffer, length, offset, packet.sequence) &&
         readU32(buffer, length, offset, packet.gatewayFrameCounter) &&
         readU32(buffer, length, offset, packet.gatewayMs) &&
         readI16(buffer, length, offset, packet.rssi) &&
         readI8(buffer, length, offset, packet.snr) &&
         readU8(buffer, length, offset, packet.backfillHint);
}

inline uint8_t nodeSlot(uint8_t nodeId)
{
  if (nodeId == 0 || nodeId > NODE_COUNT)
  {
    return 0;
  }
  return nodeId - 1;
}

inline bool isCatchupFrame(uint32_t frameCounter)
{
  return frameCounter > 0 && (frameCounter % BACKFILL_FRAME_INTERVAL) == 0;
}

} // namespace AirQuality

#endif
