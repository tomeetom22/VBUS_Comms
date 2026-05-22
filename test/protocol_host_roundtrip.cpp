#include <assert.h>
#include <string.h>
#include "air_quality_protocol.h"
#include "gateway_bridge.h"

static void fillBatch(AirQuality::BatchPacket &batch)
{
  memset(&batch, 0, sizeof(batch));
  batch.networkId = AirQuality::DEFAULT_NETWORK_ID;
  batch.nodeId = 3;
  batch.bootId = 0x12345678;
  batch.sequence = 42;
  batch.sampleStartMs = 100000;
  batch.sampleIntervalMs = 1000;
  batch.batteryMv = 3720;
  batch.statusFlags = AirQuality::STATUS_WINDOWED_SAMPLE;

  for (uint8_t sample = 0; sample < AirQuality::SAMPLE_COUNT; sample++)
  {
    for (uint8_t channel = 0; channel < AirQuality::CHANNEL_COUNT; channel++)
    {
      batch.samples[sample][channel] = (int16_t)(sample * 100 + channel);
    }
  }
}

int main()
{
  uint8_t buffer[GatewayBridge::MAX_RECORD_FRAME_LEN];

  AirQuality::BeaconPacket beacon = {
      AirQuality::DEFAULT_NETWORK_ID,
      11,
      110040,
      AirQuality::FRAME_MS,
      AirQuality::SLOT_MS};
  AirQuality::BeaconPacket decodedBeacon;
  size_t beaconLen = AirQuality::encodeBeacon(beacon, buffer, sizeof(buffer));
  assert(beaconLen == AirQuality::BEACON_PACKET_LEN);
  assert(AirQuality::decodeBeacon(buffer, beaconLen, decodedBeacon));
  assert(decodedBeacon.frameCounter == beacon.frameCounter);
  assert(decodedBeacon.gatewayMs == beacon.gatewayMs);

  AirQuality::BatchPacket batch;
  AirQuality::BatchPacket decodedBatch;
  fillBatch(batch);
  size_t batchLen = AirQuality::encodeBatch(batch, buffer, sizeof(buffer));
  assert(batchLen == AirQuality::BATCH_PACKET_LEN);
  assert(AirQuality::decodeBatch(buffer, batchLen, decodedBatch));
  assert(decodedBatch.nodeId == batch.nodeId);
  assert(decodedBatch.sequence == batch.sequence);
  assert(decodedBatch.samples[9][5] == batch.samples[9][5]);

  buffer[10] ^= 0x01;
  assert(!AirQuality::decodeBatch(buffer, batchLen, decodedBatch));

  AirQuality::AckPacket ack = {
      AirQuality::DEFAULT_NETWORK_ID,
      3,
      0x12345678,
      42,
      12,
      120100,
      -73,
      8,
      1};
  AirQuality::AckPacket decodedAck;
  size_t ackLen = AirQuality::encodeAck(ack, buffer, sizeof(buffer));
  assert(ackLen == AirQuality::ACK_PACKET_LEN);
  assert(AirQuality::decodeAck(buffer, ackLen, decodedAck));
  assert(decodedAck.sequence == ack.sequence);
  assert(decodedAck.rssi == ack.rssi);

  GatewayBridge::BridgeRecord bridgeRecord;
  GatewayBridge::BridgeRecord decodedBridgeRecord;
  memset(&bridgeRecord, 0, sizeof(bridgeRecord));
  bridgeRecord.sourceNetwork = GatewayBridge::SOURCE_AIR_QUALITY;
  bridgeRecord.receivedMs = 120200;
  bridgeRecord.nodeId = batch.nodeId;
  bridgeRecord.bootId = batch.bootId;
  bridgeRecord.sequence = batch.sequence;
  bridgeRecord.rssi = -76;
  bridgeRecord.snr = 7;
  bridgeRecord.payloadLength = AirQuality::encodeBatch(batch, bridgeRecord.payload, sizeof(bridgeRecord.payload));
  assert(bridgeRecord.payloadLength == AirQuality::BATCH_PACKET_LEN);

  size_t bridgeLen = GatewayBridge::encodeRecord(bridgeRecord, buffer, sizeof(buffer));
  assert(bridgeLen > bridgeRecord.payloadLength);
  assert(GatewayBridge::decodeRecord(buffer, bridgeLen, decodedBridgeRecord));
  assert(decodedBridgeRecord.sourceNetwork == bridgeRecord.sourceNetwork);
  assert(decodedBridgeRecord.sequence == bridgeRecord.sequence);
  assert(decodedBridgeRecord.payloadLength == bridgeRecord.payloadLength);

  return 0;
}
