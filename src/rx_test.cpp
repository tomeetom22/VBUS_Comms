#include <Arduino.h>
#include <SX126x-Arduino.h>
#include "air_quality_protocol.h"
#include "gateway_bridge.h"

// ---------------------------------------------------------------------------
// LoRa P2P radio settings
// ---------------------------------------------------------------------------
static const uint32_t RF_FREQUENCY = 916000000;
static const uint8_t LORA_BANDWIDTH = 0;
static const uint8_t LORA_SPREADING_FACTOR = 7;
static const uint8_t LORA_CODING_RATE = 1;
static const int8_t TX_OUTPUT_POWER = 22;
static const uint16_t LORA_PREAMBLE_LENGTH = 8;
static const uint16_t LORA_SYMBOL_TIMEOUT = 0;
static const bool LORA_FIXED_LENGTH_PAYLOAD = false;
static const bool LORA_IQ_INVERSION = false;
static const uint32_t TX_TIMEOUT_MS = 5000;

// ---------------------------------------------------------------------------
// Gateway configuration
// ---------------------------------------------------------------------------
static const uint16_t NETWORK_ID = AirQuality::DEFAULT_NETWORK_ID;
static const uint32_t BEACON_OFFSET_MS = 40;
static const uint32_t ACK_REPLY_DELAY_MS = 120;
static const uint32_t RX_STATUS_INTERVAL_MS = 3000;
static const uint8_t DEDUP_CACHE_SIZE = 32;
static const bool REQUIRE_GATEWAY_SD_BEFORE_ACK = false; // Enable after SD wiring is known.
static const bool CELLULAR_UPLINK_GATEWAY = true; // Gateway A: true. Gateway B: false.
static const bool FORWARD_LOCAL_RECORDS_TO_CELL_GATEWAY = false; // Gateway B: true after local bridge wiring.
static const uint32_t CELL_UPLOAD_INTERVAL_MS = 60000;
static const uint8_t LOCAL_SOURCE_NETWORK = GatewayBridge::SOURCE_AIR_QUALITY;

static const char *CHANNEL_NAMES[AirQuality::CHANNEL_COUNT] = {
    "pm1", "pm25", "pm10", "temp_c_x100", "rh_x100", "gas_raw"};

static RadioEvents_t RadioEvents;

static volatile bool rxDone = false;
static volatile bool rxTimeout = false;
static volatile bool rxError = false;
static volatile bool txDone = false;
static volatile bool txTimeout = false;
static bool radioBusy = false;

static uint8_t rxBuffer[AirQuality::MAX_PACKET_LEN + 1];
static uint16_t rxSize = 0;
static int16_t rxRssi = 0;
static int8_t rxSnr = 0;
static uint8_t txBuffer[AirQuality::MAX_PACKET_LEN];

static uint32_t packetCount = 0;
static uint32_t duplicateCount = 0;
static uint32_t badPacketCount = 0;
static uint32_t savedCount = 0;
static uint32_t ackedCount = 0;
static uint32_t forwardedCount = 0;
static uint32_t bridgeReceivedCount = 0;
static uint32_t cellularStageCount = 0;
static uint32_t cellularUploadAttemptCount = 0;
static uint32_t lastStatusMs = 0;
static uint32_t lastCellUploadMs = 0;
static uint32_t lastBeaconFrame = UINT32_MAX;
static uint32_t pendingAckReadyMs = 0;

struct DedupRecord
{
  bool active;
  uint8_t nodeId;
  uint32_t bootId;
  uint32_t sequence;
};

static DedupRecord dedupCache[DEDUP_CACHE_SIZE];
static uint8_t dedupWriteIndex = 0;

static AirQuality::AckPacket pendingAck;
static bool hasPendingAck = false;
static uint8_t bridgeTxBuffer[GatewayBridge::MAX_RECORD_FRAME_LEN];

void onTxDone(void);
void onTxTimeout(void);
void onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void onRxTimeout(void);
void onRxError(void);
void startReceive(void);
void pollRadioEvents(void);
void pollBeacon(void);
void pollPendingAck(void);
void pollGatewayBridgeInput(void);
void sendBeacon(uint32_t frameCounter);
void handleBatch(const AirQuality::BatchPacket &batch);
bool isDuplicateBatch(const AirQuality::BatchPacket &batch);
void rememberBatch(const AirQuality::BatchPacket &batch);
bool appendGatewayBatchLog(const AirQuality::BatchPacket &batch, uint32_t receivedMs, int16_t rssi, int8_t snr);
bool appendForwardedGatewayLog(const GatewayBridge::BridgeRecord &record);
void handleAcceptedLocalBatch(const AirQuality::BatchPacket &batch);
bool buildBridgeRecordFromBatch(const AirQuality::BatchPacket &batch, GatewayBridge::BridgeRecord &record);
bool forwardBridgeRecord(const GatewayBridge::BridgeRecord &record);
bool stageCellularRecord(const GatewayBridge::BridgeRecord &record);
bool uploadStagedCellularRecords(void);
void queueAck(const AirQuality::BatchPacket &batch);
void sendAck(void);
uint32_t currentGatewayFrame(void);
uint32_t frameElapsedMs(void);
void printBatchSummary(const AirQuality::BatchPacket &batch, bool duplicate);
void blinkWakeupTest(void);
void flashPacketReceived(void);

void setup()
{
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);

  blinkWakeupTest();

  Serial.begin(115200);
  uint32_t serialStartMs = millis();
  while (!Serial && (millis() - serialStartMs < 5000))
  {
    delay(100);
    digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
  }
  digitalWrite(LED_GREEN, LOW);

  Serial.println();
  Serial.println("========================================");
  Serial.println("RAK4630 air-quality TDMA gateway");
  Serial.println("========================================");

  uint32_t initResult = lora_rak4630_init();
  Serial.printf("LoRa hardware init: %s\r\n", initResult == 0 ? "success" : "failed");

  RadioEvents.TxDone = onTxDone;
  RadioEvents.TxTimeout = onTxTimeout;
  RadioEvents.RxDone = onRxDone;
  RadioEvents.RxTimeout = onRxTimeout;
  RadioEvents.RxError = onRxError;
  RadioEvents.CadDone = NULL;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODING_RATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIXED_LENGTH_PAYLOAD,
                    true, 0, 0, LORA_IQ_INVERSION, TX_TIMEOUT_MS);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODING_RATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIXED_LENGTH_PAYLOAD,
                    0, true, 0, 0, LORA_IQ_INVERSION, true);

  Serial.printf("Network=0x%04X frame=%lu ms slot=%lu ms nodes=%u\r\n",
                NETWORK_ID, AirQuality::FRAME_MS, AirQuality::SLOT_MS,
                AirQuality::NODE_COUNT);
  Serial.printf("BATCH bytes=%u ACK bytes=%u BEACON bytes=%u\r\n",
                (unsigned)AirQuality::BATCH_PACKET_LEN,
                (unsigned)AirQuality::ACK_PACKET_LEN,
                (unsigned)AirQuality::BEACON_PACKET_LEN);
  Serial.println("Gateway SD append is currently a stub; set REQUIRE_GATEWAY_SD_BEFORE_ACK after SD wiring is implemented.");

  startReceive();
  lastStatusMs = millis();
}

void loop()
{
  pollRadioEvents();
  pollBeacon();
  pollPendingAck();
  pollGatewayBridgeInput();

  if (CELLULAR_UPLINK_GATEWAY && millis() - lastCellUploadMs >= CELL_UPLOAD_INTERVAL_MS)
  {
    lastCellUploadMs = millis();
    uploadStagedCellularRecords();
  }

  if (millis() - lastStatusMs >= RX_STATUS_INTERVAL_MS)
  {
    lastStatusMs = millis();
    Serial.printf("Gateway listening frame=%lu packets=%lu saved=%lu dup=%lu bad=%lu acked=%lu bridge_rx=%lu cell_stage=%lu\r\n",
                  currentGatewayFrame(), packetCount, savedCount, duplicateCount,
                  badPacketCount, ackedCount, bridgeReceivedCount, cellularStageCount);
    digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
  }
}

void pollRadioEvents(void)
{
  if (rxDone)
  {
    rxDone = false;
    packetCount++;
    flashPacketReceived();

    AirQuality::BatchPacket batch;
    if (AirQuality::decodeBatch(rxBuffer, rxSize, batch) && batch.networkId == NETWORK_ID)
    {
      handleBatch(batch);
    }
    else
    {
      badPacketCount++;
      Serial.printf("Bad/unknown packet len=%u rssi=%d snr=%d bad=%lu\r\n",
                    rxSize, rxRssi, rxSnr, badPacketCount);
    }
    startReceive();
  }

  if (txDone)
  {
    txDone = false;
    radioBusy = false;
    digitalWrite(LED_BLUE, LOW);
    startReceive();
  }

  if (txTimeout)
  {
    txTimeout = false;
    radioBusy = false;
    digitalWrite(LED_BLUE, LOW);
    Serial.println("Gateway TX timeout");
    startReceive();
  }

  if (rxTimeout)
  {
    rxTimeout = false;
    startReceive();
  }

  if (rxError)
  {
    rxError = false;
    badPacketCount++;
    startReceive();
  }
}

void pollBeacon(void)
{
  if (radioBusy || hasPendingAck)
  {
    return;
  }

  uint32_t frame = currentGatewayFrame();
  uint32_t elapsed = frameElapsedMs();
  if (elapsed >= BEACON_OFFSET_MS && elapsed < BEACON_OFFSET_MS + 80 && lastBeaconFrame != frame)
  {
    lastBeaconFrame = frame;
    sendBeacon(frame);
  }
}

void pollPendingAck(void)
{
  if (radioBusy || !hasPendingAck)
  {
    return;
  }

  if (millis() - pendingAckReadyMs < ACK_REPLY_DELAY_MS)
  {
    return;
  }

  sendAck();
}

void pollGatewayBridgeInput(void)
{
  // Future Gateway A wiring: read framed records from Gateway B over Serial1,
  // I2C, or SPI, then decode with GatewayBridge::decodeRecord().
  //
  // The intended handling is:
  //   1. appendForwardedGatewayLog(record)
  //   2. stageCellularRecord(record)
  //
  // This is intentionally a no-op until the local bridge hardware is chosen.
}

void sendBeacon(uint32_t frameCounter)
{
  AirQuality::BeaconPacket beacon;
  beacon.networkId = NETWORK_ID;
  beacon.frameCounter = frameCounter;
  beacon.gatewayMs = millis();
  beacon.frameMs = AirQuality::FRAME_MS;
  beacon.slotMs = AirQuality::SLOT_MS;

  size_t len = AirQuality::encodeBeacon(beacon, txBuffer, sizeof(txBuffer));
  if (len == 0)
  {
    Serial.println("BEACON encode failed");
    return;
  }

  radioBusy = true;
  digitalWrite(LED_BLUE, HIGH);
  Serial.printf("TX BEACON frame=%lu bytes=%u\r\n", frameCounter, (unsigned)len);
  Radio.Sleep();
  Radio.Send(txBuffer, len);
}

void handleBatch(const AirQuality::BatchPacket &batch)
{
  bool duplicate = isDuplicateBatch(batch);
  bool saved = true;
  if (duplicate)
  {
    duplicateCount++;
  }
  else
  {
    saved = appendGatewayBatchLog(batch, millis(), rxRssi, rxSnr);
    if (saved)
    {
      rememberBatch(batch);
      savedCount++;
      handleAcceptedLocalBatch(batch);
    }
  }

  printBatchSummary(batch, duplicate);

  if (saved || duplicate || !REQUIRE_GATEWAY_SD_BEFORE_ACK)
  {
    queueAck(batch);
  }
  else
  {
    Serial.printf("ACK withheld for node=%u boot=%lu seq=%lu because gateway SD save failed\r\n",
                  batch.nodeId, batch.bootId, batch.sequence);
  }
}

bool isDuplicateBatch(const AirQuality::BatchPacket &batch)
{
  for (uint8_t i = 0; i < DEDUP_CACHE_SIZE; i++)
  {
    if (dedupCache[i].active &&
        dedupCache[i].nodeId == batch.nodeId &&
        dedupCache[i].bootId == batch.bootId &&
        dedupCache[i].sequence == batch.sequence)
    {
      return true;
    }
  }
  return false;
}

void rememberBatch(const AirQuality::BatchPacket &batch)
{
  dedupCache[dedupWriteIndex].active = true;
  dedupCache[dedupWriteIndex].nodeId = batch.nodeId;
  dedupCache[dedupWriteIndex].bootId = batch.bootId;
  dedupCache[dedupWriteIndex].sequence = batch.sequence;
  dedupWriteIndex = (dedupWriteIndex + 1) % DEDUP_CACHE_SIZE;
}

bool appendGatewayBatchLog(const AirQuality::BatchPacket &batch, uint32_t receivedMs, int16_t rssi, int8_t snr)
{
  Serial.printf("Gateway SD append stub: rx_ms=%lu node=%u boot=%lu seq=%lu rssi=%d snr=%d\r\n",
                receivedMs, batch.nodeId, batch.bootId, batch.sequence, rssi, snr);
  return true;
}

bool appendForwardedGatewayLog(const GatewayBridge::BridgeRecord &record)
{
  Serial.printf("Forwarded SD append stub: src=%u rx_ms=%lu node=%u boot=%lu seq=%lu rssi=%d snr=%d bytes=%u\r\n",
                record.sourceNetwork, record.receivedMs, record.nodeId, record.bootId,
                record.sequence, record.rssi, record.snr, record.payloadLength);
  return true;
}

void handleAcceptedLocalBatch(const AirQuality::BatchPacket &batch)
{
  GatewayBridge::BridgeRecord record;
  if (!buildBridgeRecordFromBatch(batch, record))
  {
    Serial.printf("Could not build bridge record for node=%u seq=%lu\r\n",
                  batch.nodeId, batch.sequence);
    return;
  }

  if (CELLULAR_UPLINK_GATEWAY)
  {
    stageCellularRecord(record);
  }

  if (FORWARD_LOCAL_RECORDS_TO_CELL_GATEWAY)
  {
    forwardBridgeRecord(record);
  }
}

bool buildBridgeRecordFromBatch(const AirQuality::BatchPacket &batch, GatewayBridge::BridgeRecord &record)
{
  memset(&record, 0, sizeof(record));
  record.sourceNetwork = LOCAL_SOURCE_NETWORK;
  record.receivedMs = millis();
  record.nodeId = batch.nodeId;
  record.bootId = batch.bootId;
  record.sequence = batch.sequence;
  record.rssi = rxRssi;
  record.snr = rxSnr;
  record.payloadLength = AirQuality::encodeBatch(batch, record.payload, sizeof(record.payload));
  return record.payloadLength > 0;
}

bool forwardBridgeRecord(const GatewayBridge::BridgeRecord &record)
{
  size_t len = GatewayBridge::encodeRecord(record, bridgeTxBuffer, sizeof(bridgeTxBuffer));
  if (len == 0)
  {
    Serial.printf("Bridge encode failed for src=%u node=%u seq=%lu\r\n",
                  record.sourceNetwork, record.nodeId, record.sequence);
    return false;
  }

  // Future wiring: write bridgeTxBuffer[0..len) to Serial1, I2C, or SPI.
  forwardedCount++;
  Serial.printf("Gateway bridge TX stub: src=%u node=%u seq=%lu bytes=%u forwarded=%lu\r\n",
                record.sourceNetwork, record.nodeId, record.sequence,
                (unsigned)len, forwardedCount);
  return true;
}

bool stageCellularRecord(const GatewayBridge::BridgeRecord &record)
{
  cellularStageCount++;
  Serial.printf("Cellular stage stub: src=%u node=%u boot=%lu seq=%lu payload_bytes=%u staged=%lu\r\n",
                record.sourceNetwork, record.nodeId, record.bootId, record.sequence,
                record.payloadLength, cellularStageCount);
  return true;
}

bool uploadStagedCellularRecords(void)
{
  cellularUploadAttemptCount++;
  Serial.printf("Cellular upload stub: attempt=%lu staged_total=%lu bridge_rx=%lu local_saved=%lu\r\n",
                cellularUploadAttemptCount, cellularStageCount, bridgeReceivedCount, savedCount);
  return false;
}

void queueAck(const AirQuality::BatchPacket &batch)
{
  pendingAck.networkId = NETWORK_ID;
  pendingAck.nodeId = batch.nodeId;
  pendingAck.bootId = batch.bootId;
  pendingAck.sequence = batch.sequence;
  pendingAck.gatewayFrameCounter = currentGatewayFrame();
  pendingAck.gatewayMs = millis();
  pendingAck.rssi = rxRssi;
  pendingAck.snr = rxSnr;
  pendingAck.backfillHint = AirQuality::isCatchupFrame(pendingAck.gatewayFrameCounter) ? 1 : 0;
  pendingAckReadyMs = millis();
  hasPendingAck = true;
}

void sendAck(void)
{
  size_t len = AirQuality::encodeAck(pendingAck, txBuffer, sizeof(txBuffer));
  if (len == 0)
  {
    Serial.println("ACK encode failed");
    hasPendingAck = false;
    return;
  }

  radioBusy = true;
  digitalWrite(LED_BLUE, HIGH);
  Serial.printf("TX ACK node=%u boot=%lu seq=%lu frame=%lu bytes=%u\r\n",
                pendingAck.nodeId, pendingAck.bootId, pendingAck.sequence,
                pendingAck.gatewayFrameCounter, (unsigned)len);
  Radio.Sleep();
  Radio.Send(txBuffer, len);
  ackedCount++;
  hasPendingAck = false;
}

uint32_t currentGatewayFrame(void)
{
  return millis() / AirQuality::FRAME_MS;
}

uint32_t frameElapsedMs(void)
{
  return millis() % AirQuality::FRAME_MS;
}

void printBatchSummary(const AirQuality::BatchPacket &batch, bool duplicate)
{
  Serial.println();
  Serial.printf("%s BATCH node=%u boot=%lu seq=%lu start_ms=%lu interval=%u battery=%u status=0x%04X rssi=%d snr=%d\r\n",
                duplicate ? "Duplicate" : "Received",
                batch.nodeId, batch.bootId, batch.sequence, batch.sampleStartMs,
                batch.sampleIntervalMs, batch.batteryMv, batch.statusFlags, rxRssi, rxSnr);

  for (uint8_t channel = 0; channel < AirQuality::CHANNEL_COUNT; channel++)
  {
    Serial.printf("  %s:", CHANNEL_NAMES[channel]);
    for (uint8_t sample = 0; sample < AirQuality::SAMPLE_COUNT; sample++)
    {
      Serial.print(' ');
      if (batch.samples[sample][channel] == AirQuality::MISSING_SAMPLE)
      {
        Serial.print("NA");
      }
      else
      {
        Serial.print(batch.samples[sample][channel]);
      }
    }
    Serial.println();
  }
}

void startReceive(void)
{
  Radio.Sleep();
  Radio.Rx(0);
}

void blinkWakeupTest(void)
{
  for (uint8_t i = 0; i < 6; i++)
  {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);
    delay(120);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH);
    delay(120);
  }
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
}

void flashPacketReceived(void)
{
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  delay(60);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
}

void onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  rxSize = min(size, (uint16_t)AirQuality::MAX_PACKET_LEN);
  memcpy(rxBuffer, payload, rxSize);
  rxRssi = rssi;
  rxSnr = snr;
  rxDone = true;
}

void onTxDone(void)
{
  txDone = true;
}

void onTxTimeout(void)
{
  txTimeout = true;
}

void onRxTimeout(void)
{
  rxTimeout = true;
}

void onRxError(void)
{
  rxError = true;
}
