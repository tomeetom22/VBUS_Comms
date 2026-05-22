#include <Arduino.h>
#include <SX126x-Arduino.h>
#include "air_quality_protocol.h"

// ---------------------------------------------------------------------------
// LoRa P2P radio settings
// ---------------------------------------------------------------------------
static const uint32_t RF_FREQUENCY = 916000000;
static const int8_t TX_OUTPUT_POWER = 22;
static const uint8_t LORA_BANDWIDTH = 0;
static const uint8_t LORA_SPREADING_FACTOR = 7;
static const uint8_t LORA_CODING_RATE = 1;
static const uint16_t LORA_PREAMBLE_LENGTH = 8;
static const uint16_t LORA_SYMBOL_TIMEOUT = 0;
static const bool LORA_FIXED_LENGTH_PAYLOAD = false;
static const bool LORA_IQ_INVERSION = false;
static const uint32_t TX_TIMEOUT_MS = 5000;

// ---------------------------------------------------------------------------
// Node configuration
// ---------------------------------------------------------------------------
static const uint16_t NETWORK_ID = AirQuality::DEFAULT_NETWORK_ID;
static const uint8_t LOCAL_NODE_ID = 1; // Set each deployed node to 1..10.
static const uint32_t SAMPLE_INTERVAL_MS = 1000;
static const uint32_t ACK_WAIT_TIMEOUT_MS = 1500;
static const uint8_t MAX_PENDING_BATCHES = 12;
static const bool REQUIRE_NODE_SD_BEFORE_TX = false; // Enable after SD wiring is known.

static const char *CHANNEL_NAMES[AirQuality::CHANNEL_COUNT] = {
    "pm1", "pm25", "pm10", "temp_c_x100", "rh_x100", "gas_raw"};

static RadioEvents_t RadioEvents;

static volatile bool txDone = false;
static volatile bool txTimeout = false;
static volatile bool rxDone = false;
static volatile bool rxTimeout = false;
static volatile bool rxError = false;
static bool radioBusy = false;

static uint8_t rxBuffer[AirQuality::MAX_PACKET_LEN + 1];
static uint16_t rxSize = 0;
static int16_t rxRssi = 0;
static int8_t rxSnr = 0;
static uint8_t txBuffer[AirQuality::MAX_PACKET_LEN];

static uint32_t bootId = 0;
static uint32_t nextSequence = 1;
static uint32_t lastSampleMs = 0;
static uint32_t currentBatchStartMs = 0;
static uint8_t currentSampleCount = 0;
static int16_t currentSamples[AirQuality::SAMPLE_COUNT][AirQuality::CHANNEL_COUNT];
static uint16_t currentStatusFlags = 0;

static bool synced = false;
static uint32_t syncLocalMs = 0;
static uint32_t syncGatewayMs = 0;
static uint32_t lastKnownFrameCounter = 0;
static uint32_t lastSlotTxFrame = UINT32_MAX;
static uint32_t outstandingSequence = 0;
static uint32_t outstandingSinceMs = 0;
static uint32_t acknowledgedCount = 0;
static uint32_t badPacketCount = 0;

struct PendingBatch
{
  bool active;
  bool saved;
  bool sent;
  bool backfill;
  uint32_t sequence;
  uint8_t attempts;
  uint32_t lastSentMs;
  AirQuality::BatchPacket packet;
};

static PendingBatch pendingBatches[MAX_PENDING_BATCHES];

void onTxDone(void);
void onTxTimeout(void);
void onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void onRxTimeout(void);
void onRxError(void);
void startReceive(void);
void pollRadioEvents(void);
void pollSampling(void);
void finishCurrentBatch(void);
void readSensorSample(int16_t values[AirQuality::CHANNEL_COUNT], uint16_t &statusFlags);
uint16_t readBatteryMillivolts(void);
bool saveNodeBatchToStorage(const AirQuality::BatchPacket &packet);
int findFreePendingBatch(void);
int findPendingBySequence(uint32_t sequence);
int findNewestPendingBatch(void);
int findOldestPendingBatch(void);
uint8_t pendingBatchCount(void);
bool markSequenceAcked(uint32_t sequence);
void pollTdmaTransmit(void);
void sendPendingBatch(uint8_t index, bool asBackfill);
void handleBeacon(const AirQuality::BeaconPacket &beacon);
void handleAck(const AirQuality::AckPacket &ack);
bool getCurrentFrame(uint32_t nowMs, uint32_t &frameCounter, uint32_t &frameElapsedMs);
uint32_t currentGatewayMs(uint32_t nowMs);
void printBatchSummary(const AirQuality::BatchPacket &packet);

void setup()
{
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);

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
  Serial.println("RAK4630 air-quality TDMA node");
  Serial.println("========================================");

  bootId = millis() ^ ((uint32_t)LOCAL_NODE_ID << 24) ^ 0xA15E0001UL;

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

  Serial.printf("Network=0x%04X node=%u slot=%u frame=%lu ms slot=%lu ms\r\n",
                NETWORK_ID, LOCAL_NODE_ID, AirQuality::nodeSlot(LOCAL_NODE_ID),
                AirQuality::FRAME_MS, AirQuality::SLOT_MS);
  Serial.printf("Batch payload: %u samples x %u channels, binary bytes=%u\r\n",
                AirQuality::SAMPLE_COUNT, AirQuality::CHANNEL_COUNT,
                (unsigned)AirQuality::BATCH_PACKET_LEN);
  Serial.println("Node waits for gateway BEACON/ACK sync before transmitting.");
  Serial.println("Node SD append is currently a stub; set REQUIRE_NODE_SD_BEFORE_TX after SD wiring is implemented.");

  lastSampleMs = millis() - SAMPLE_INTERVAL_MS;
  startReceive();
}

void loop()
{
  pollRadioEvents();
  pollSampling();
  pollTdmaTransmit();
}

void pollRadioEvents(void)
{
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
    Serial.println("TX timeout");
    startReceive();
  }

  if (rxDone)
  {
    rxDone = false;
    digitalWrite(LED_GREEN, HIGH);

    AirQuality::BeaconPacket beacon;
    AirQuality::AckPacket ack;
    if (AirQuality::decodeBeacon(rxBuffer, rxSize, beacon) && beacon.networkId == NETWORK_ID)
    {
      handleBeacon(beacon);
    }
    else if (AirQuality::decodeAck(rxBuffer, rxSize, ack) && ack.networkId == NETWORK_ID)
    {
      handleAck(ack);
    }
    else
    {
      badPacketCount++;
      Serial.printf("Ignored non-protocol packet len=%u rssi=%d snr=%d bad=%lu\r\n",
                    rxSize, rxRssi, rxSnr, badPacketCount);
    }

    delay(25);
    digitalWrite(LED_GREEN, LOW);
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
    startReceive();
  }
}

void pollSampling(void)
{
  uint32_t nowMs = millis();
  if (nowMs - lastSampleMs < SAMPLE_INTERVAL_MS)
  {
    return;
  }

  lastSampleMs += SAMPLE_INTERVAL_MS;
  if (nowMs - lastSampleMs >= SAMPLE_INTERVAL_MS)
  {
    lastSampleMs = nowMs;
    currentStatusFlags |= AirQuality::STATUS_STALE_SAMPLE;
  }

  if (currentSampleCount == 0)
  {
    currentBatchStartMs = nowMs;
    currentStatusFlags = 0;
  }

  uint16_t sampleStatus = 0;
  readSensorSample(currentSamples[currentSampleCount], sampleStatus);
  currentStatusFlags |= sampleStatus;
  currentSampleCount++;

  if (currentSampleCount >= AirQuality::SAMPLE_COUNT)
  {
    finishCurrentBatch();
  }
}

void finishCurrentBatch(void)
{
  int freeIndex = findFreePendingBatch();
  if (freeIndex < 0)
  {
    int oldestIndex = findOldestPendingBatch();
    if (oldestIndex >= 0)
    {
      Serial.printf("Pending queue full; dropping oldest unACKed seq=%lu\r\n",
                    pendingBatches[oldestIndex].sequence);
      freeIndex = oldestIndex;
    }
  }

  if (freeIndex < 0)
  {
    Serial.println("No pending queue slot available; batch lost");
    currentSampleCount = 0;
    return;
  }

  PendingBatch &pending = pendingBatches[freeIndex];
  memset(&pending, 0, sizeof(pending));
  pending.active = true;
  pending.sequence = nextSequence++;
  pending.packet.networkId = NETWORK_ID;
  pending.packet.nodeId = LOCAL_NODE_ID;
  pending.packet.bootId = bootId;
  pending.packet.sequence = pending.sequence;
  pending.packet.sampleStartMs = currentBatchStartMs;
  pending.packet.sampleIntervalMs = SAMPLE_INTERVAL_MS;
  pending.packet.batteryMv = readBatteryMillivolts();
  pending.packet.statusFlags = currentStatusFlags;
  memcpy(pending.packet.samples, currentSamples, sizeof(currentSamples));

  pending.saved = saveNodeBatchToStorage(pending.packet);
  if (!pending.saved)
  {
    pending.packet.statusFlags |= AirQuality::STATUS_SD_SAVE_FAILED;
  }

  Serial.printf("Queued batch seq=%lu saved=%u pending=%u start_ms=%lu\r\n",
                pending.sequence, pending.saved ? 1 : 0, pendingBatchCount(),
                pending.packet.sampleStartMs);
  printBatchSummary(pending.packet);

  currentSampleCount = 0;
  currentBatchStartMs = 0;
  currentStatusFlags = 0;
}

void readSensorSample(int16_t values[AirQuality::CHANNEL_COUNT], uint16_t &statusFlags)
{
  // Replace this placeholder with real sensor reads. OPC-N3 should contribute
  // PM1/PM2.5/PM10 scalar values here; keep full histograms in node SD logs.
  uint32_t tick = millis() / SAMPLE_INTERVAL_MS;
  values[0] = 80 + (int16_t)(tick % 10);  // PM1 x10
  values[1] = 120 + (int16_t)(tick % 15); // PM2.5 x10
  values[2] = 210 + (int16_t)(tick % 20); // PM10 x10
  values[3] = 2300 + (int16_t)(tick % 6); // temperature C x100
  values[4] = 4200 + (int16_t)(tick % 8); // RH percent x100
  values[5] = 1000 + (int16_t)(tick % 50); // spare gas/raw channel
  statusFlags = AirQuality::STATUS_WINDOWED_SAMPLE;
}

uint16_t readBatteryMillivolts(void)
{
  return 0; // Fill from PMIC/ADC when battery monitoring is wired.
}

bool saveNodeBatchToStorage(const AirQuality::BatchPacket &packet)
{
  Serial.printf("Node SD append stub: node=%u boot=%lu seq=%lu\r\n",
                packet.nodeId, packet.bootId, packet.sequence);
  return true;
}

int findFreePendingBatch(void)
{
  for (uint8_t i = 0; i < MAX_PENDING_BATCHES; i++)
  {
    if (!pendingBatches[i].active)
    {
      return i;
    }
  }
  return -1;
}

int findPendingBySequence(uint32_t sequence)
{
  for (uint8_t i = 0; i < MAX_PENDING_BATCHES; i++)
  {
    if (pendingBatches[i].active && pendingBatches[i].sequence == sequence)
    {
      return i;
    }
  }
  return -1;
}

int findNewestPendingBatch(void)
{
  uint32_t newestSeq = 0;
  int newestIndex = -1;
  for (uint8_t i = 0; i < MAX_PENDING_BATCHES; i++)
  {
    if (pendingBatches[i].active && pendingBatches[i].sequence > newestSeq)
    {
      newestSeq = pendingBatches[i].sequence;
      newestIndex = i;
    }
  }
  return newestIndex;
}

int findOldestPendingBatch(void)
{
  uint32_t oldestSeq = UINT32_MAX;
  int oldestIndex = -1;
  for (uint8_t i = 0; i < MAX_PENDING_BATCHES; i++)
  {
    if (pendingBatches[i].active && pendingBatches[i].sequence < oldestSeq)
    {
      oldestSeq = pendingBatches[i].sequence;
      oldestIndex = i;
    }
  }
  return oldestIndex;
}

uint8_t pendingBatchCount(void)
{
  uint8_t count = 0;
  for (uint8_t i = 0; i < MAX_PENDING_BATCHES; i++)
  {
    if (pendingBatches[i].active)
    {
      count++;
    }
  }
  return count;
}

bool markSequenceAcked(uint32_t sequence)
{
  int index = findPendingBySequence(sequence);
  if (index < 0)
  {
    return false;
  }
  pendingBatches[index].active = false;
  return true;
}

void pollTdmaTransmit(void)
{
  if (radioBusy)
  {
    return;
  }

  uint32_t nowMs = millis();
  if (outstandingSequence != 0)
  {
    if (nowMs - outstandingSinceMs < ACK_WAIT_TIMEOUT_MS)
    {
      return;
    }
    Serial.printf("ACK timeout seq=%lu; will retry in a future slot\r\n", outstandingSequence);
    outstandingSequence = 0;
  }

  uint32_t frameCounter;
  uint32_t frameElapsedMs;
  if (!getCurrentFrame(nowMs, frameCounter, frameElapsedMs))
  {
    static uint32_t lastUnsyncedPrintMs = 0;
    if (nowMs - lastUnsyncedPrintMs >= 5000)
    {
      lastUnsyncedPrintMs = nowMs;
      Serial.printf("Waiting for sync; pending=%u\r\n", pendingBatchCount());
    }
    return;
  }

  uint8_t slot = AirQuality::nodeSlot(LOCAL_NODE_ID);
  uint32_t slotStartMs = slot * AirQuality::SLOT_MS;
  uint32_t slotEndMs = slotStartMs + AirQuality::SLOT_MS - AirQuality::SLOT_GUARD_MS;
  if (frameElapsedMs < slotStartMs || frameElapsedMs >= slotEndMs)
  {
    return;
  }
  if (lastSlotTxFrame == frameCounter)
  {
    return;
  }

  int nextIndex = -1;
  bool asBackfill = false;
  if (AirQuality::isCatchupFrame(frameCounter) && pendingBatchCount() > 1)
  {
    nextIndex = findOldestPendingBatch();
    asBackfill = true;
  }
  else
  {
    nextIndex = findNewestPendingBatch();
  }

  if (nextIndex < 0)
  {
    return;
  }

  PendingBatch &pending = pendingBatches[nextIndex];
  if (REQUIRE_NODE_SD_BEFORE_TX && !pending.saved)
  {
    Serial.printf("Withholding seq=%lu because node SD save failed\r\n", pending.sequence);
    return;
  }

  lastSlotTxFrame = frameCounter;
  sendPendingBatch((uint8_t)nextIndex, asBackfill);
}

void sendPendingBatch(uint8_t index, bool asBackfill)
{
  PendingBatch &pending = pendingBatches[index];
  pending.packet.statusFlags &= ~AirQuality::STATUS_BACKFILL_BATCH;
  if (asBackfill)
  {
    pending.packet.statusFlags |= AirQuality::STATUS_BACKFILL_BATCH;
  }

  size_t len = AirQuality::encodeBatch(pending.packet, txBuffer, sizeof(txBuffer));
  if (len == 0)
  {
    Serial.printf("Encode failed for seq=%lu\r\n", pending.sequence);
    return;
  }

  pending.sent = true;
  pending.backfill = asBackfill;
  pending.attempts++;
  pending.lastSentMs = millis();
  outstandingSequence = pending.sequence;
  outstandingSinceMs = pending.lastSentMs;
  radioBusy = true;
  digitalWrite(LED_BLUE, HIGH);

  Serial.printf("TX BATCH seq=%lu bytes=%u attempt=%u backfill=%u frame=%lu pending=%u\r\n",
                pending.sequence, (unsigned)len, pending.attempts, asBackfill ? 1 : 0,
                lastKnownFrameCounter, pendingBatchCount());
  Radio.Sleep();
  Radio.Send(txBuffer, len);
}

void handleBeacon(const AirQuality::BeaconPacket &beacon)
{
  if (beacon.frameMs != AirQuality::FRAME_MS || beacon.slotMs != AirQuality::SLOT_MS)
  {
    Serial.printf("Ignored BEACON with unexpected frame/slot: %u/%u\r\n",
                  beacon.frameMs, beacon.slotMs);
    return;
  }

  syncLocalMs = millis();
  syncGatewayMs = beacon.gatewayMs;
  lastKnownFrameCounter = beacon.frameCounter;
  synced = true;

  Serial.printf("BEACON frame=%lu gateway_ms=%lu rssi=%d snr=%d pending=%u\r\n",
                beacon.frameCounter, beacon.gatewayMs, rxRssi, rxSnr, pendingBatchCount());
}

void handleAck(const AirQuality::AckPacket &ack)
{
  if (ack.nodeId != LOCAL_NODE_ID || ack.bootId != bootId)
  {
    return;
  }

  syncLocalMs = millis();
  syncGatewayMs = ack.gatewayMs;
  lastKnownFrameCounter = ack.gatewayFrameCounter;
  synced = true;

  if (markSequenceAcked(ack.sequence))
  {
    acknowledgedCount++;
    Serial.printf("ACK seq=%lu acked=%lu gateway_frame=%lu gateway_rssi=%d gateway_snr=%d pending=%u\r\n",
                  ack.sequence, acknowledgedCount, ack.gatewayFrameCounter,
                  ack.rssi, ack.snr, pendingBatchCount());
  }
  else
  {
    Serial.printf("Duplicate/stale ACK seq=%lu pending=%u\r\n",
                  ack.sequence, pendingBatchCount());
  }

  if (outstandingSequence == ack.sequence)
  {
    outstandingSequence = 0;
  }
}

bool getCurrentFrame(uint32_t nowMs, uint32_t &frameCounter, uint32_t &frameElapsedMs)
{
  if (!synced || nowMs - syncLocalMs > AirQuality::SYNC_TIMEOUT_MS)
  {
    synced = false;
    return false;
  }

  uint32_t gatewayNow = currentGatewayMs(nowMs);
  frameCounter = gatewayNow / AirQuality::FRAME_MS;
  frameElapsedMs = gatewayNow % AirQuality::FRAME_MS;
  lastKnownFrameCounter = frameCounter;
  return true;
}

uint32_t currentGatewayMs(uint32_t nowMs)
{
  return syncGatewayMs + (nowMs - syncLocalMs);
}

void printBatchSummary(const AirQuality::BatchPacket &packet)
{
  Serial.printf("  battery=%u status=0x%04X", packet.batteryMv, packet.statusFlags);
  for (uint8_t channel = 0; channel < AirQuality::CHANNEL_COUNT; channel++)
  {
    Serial.printf(" %s=[", CHANNEL_NAMES[channel]);
    for (uint8_t sample = 0; sample < AirQuality::SAMPLE_COUNT; sample++)
    {
      if (sample > 0)
      {
        Serial.print(',');
      }
      Serial.print(packet.samples[sample][channel]);
    }
    Serial.print(']');
  }
  Serial.println();
}

void startReceive(void)
{
  Radio.Sleep();
  Radio.Rx(0);
}

void onTxDone(void)
{
  txDone = true;
}

void onTxTimeout(void)
{
  txTimeout = true;
}

void onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  rxSize = min(size, (uint16_t)AirQuality::MAX_PACKET_LEN);
  memcpy(rxBuffer, payload, rxSize);
  rxRssi = rssi;
  rxSnr = snr;
  rxDone = true;
}

void onRxTimeout(void)
{
  rxTimeout = true;
}

void onRxError(void)
{
  rxError = true;
}
