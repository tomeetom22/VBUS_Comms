#include <Arduino.h>
#include <SPI.h>
#include <string.h>

// Alphasense OPC-N3 SPI notes:
// - SPI mode 1, MSB first, 3.3 V logic.
// - OPC SS goes to the WisBlock breakout CS pin.
// - The OPC itself needs a separate 5 V supply, but its ground must be tied
//   to the WisBlock ground for SPI to work.
// - Reading the histogram returns 86 bytes and clears the accumulated bins.

static const uint8_t OPC_CS_PIN = WB_SPI_CS;
static const uint32_t OPC_SPI_HZ = 500000;
static const uint32_t READ_INTERVAL_MS = 10000;

static const uint8_t OPC_READY = 0xF3;
static const uint8_t OPC_BUSY = 0x31;

static const uint8_t CMD_WRITE_POWER_STATE = 0x03;
static const uint8_t CMD_READ_POWER_STATE = 0x13;
static const uint8_t CMD_READ_HISTOGRAM = 0x30;
static const uint8_t CMD_CHECK_STATUS = 0xCF;

// One SPISettings object keeps all OPC transfers using the same known-good bus
// mode. The OPC-N3 expects mode 1, not the Arduino default mode 0.
static SPISettings opcSpiSettings(OPC_SPI_HZ, MSBFIRST, SPI_MODE1);

// Decoded view of the OPC-N3 86-byte histogram packet.
//
// Byte map:
//   0..47   = 24 little-endian uint16 particle bins
//   48..51  = MToF for bins 1, 3, 5, 7
//   52..59  = sample period, flow, raw temp, raw RH
//   60..71  = PM1, PM2.5, PM10 as little-endian float32 values
//   72..83  = reject/status counters
//   84..85  = packet CRC16
struct OpcHistogram
{
  uint16_t bins[24];
  float binsPerMl[24];
  uint8_t bin1Mtof;
  uint8_t bin3Mtof;
  uint8_t bin5Mtof;
  uint8_t bin7Mtof;
  float samplingPeriodS;
  float sampleFlowRate;
  float temperatureC;
  float humidityPct;
  float pm1;
  float pm25;
  float pm10;
  uint16_t rejectGlitch;
  uint16_t rejectLongTof;
  uint16_t rejectRatio;
  uint16_t rejectOutOfRange;
  uint16_t fanRevCount;
  uint16_t laserStatus;
  uint16_t checksum;
  uint16_t calculatedChecksum;
  bool checksumOk;
};

static uint32_t lastReadMs = 0;
static uint32_t readCounter = 0;

static uint8_t opcTransferByte(uint8_t value, uint16_t postDelayUs = 10);
static bool waitForReady(uint8_t command);
static bool readBytes(uint8_t command, uint8_t *buffer, size_t length);
static bool writeByte(uint8_t command, uint8_t value);
static bool readPowerState();
static bool turnOpcOn();
static uint16_t le16(const uint8_t *buffer, size_t offset);
static float leFloat(const uint8_t *buffer, size_t offset);
static uint16_t opcCrc16(const uint8_t *buffer, size_t length);
static bool decodeHistogram(const uint8_t *buffer, OpcHistogram &histogram);
static void printHistogram(const OpcHistogram &histogram);
static void printHexByte(uint8_t value);

void setup()
{
  // LEDs are only status hints: green blinks while waiting for USB serial,
  // blue turns on during each histogram read.
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);

  Serial.begin(115200);
  uint32_t serialStartMs = millis();
  while (!Serial && (millis() - serialStartMs < 5000))
  {
    digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
    delay(100);
  }
  digitalWrite(LED_GREEN, LOW);

  Serial.println();
  Serial.println("=====================================");
  Serial.println("RAK4630 / Alphasense OPC-N3 bin test");
  Serial.println("=====================================");
  Serial.println("SPI: mode 1, 500 kHz, MSB first");
  Serial.println("Wiring: OPC SS -> WisBlock breakout CS");
  Serial.printf("Pins: CS=%u MOSI/SDI=%u MISO/SDO=%u SCK=%u\r\n",
                OPC_CS_PIN, WB_SPI_MOSI, WB_SPI_MISO, WB_SPI_CLK);
  Serial.println("Important: OPC 5 V supply ground must be common with WisBlock GND.");

  pinMode(OPC_CS_PIN, OUTPUT);
  digitalWrite(OPC_CS_PIN, HIGH);
  SPI.begin();

  // Give the OPC power supply, fan controller, and internal firmware a moment
  // to come up before the first SPI command.
  Serial.println("Waiting 3 seconds for OPC power-up...");
  delay(3000);

  Serial.println("Checking OPC SPI status...");
  if (!waitForReady(CMD_CHECK_STATUS))
  {
    Serial.println("OPC did not respond cleanly. Power-cycle the OPC if it entered autonomous mode or saw a bad command.");
  }

  if (turnOpcOn())
  {
    Serial.println("Fan and laser turn-on commands sent.");
  }
  else
  {
    Serial.println("Could not send fan/laser turn-on commands.");
  }

  readPowerState();

  // Reading the histogram clears the OPC's accumulated histogram. This first
  // throwaway read gives the recurring printout a clean starting point.
  Serial.println("Discarding first histogram so the next reading starts fresh...");
  uint8_t discard[86];
  readBytes(CMD_READ_HISTOGRAM, discard, sizeof(discard));
  lastReadMs = millis();
}

void loop()
{
  // The OPC-N3 is happier when histogram reads are not hammered too quickly.
  // A 10 s interval gives one clean 10-second histogram window per saved row.
  if (millis() - lastReadMs < READ_INTERVAL_MS)
  {
    return;
  }
  lastReadMs = millis();

  uint8_t rawHistogram[86];
  OpcHistogram histogram;

  // Fetch the raw 86-byte packet first, then decode it separately. Keeping
  // those steps split makes SPI problems easier to distinguish from math bugs.
  digitalWrite(LED_BLUE, HIGH);
  bool ok = readBytes(CMD_READ_HISTOGRAM, rawHistogram, sizeof(rawHistogram));
  digitalWrite(LED_BLUE, LOW);

  if (!ok)
  {
    Serial.println("Histogram read failed. Waiting before retrying...");
    delay(2500);
    return;
  }

  if (!decodeHistogram(rawHistogram, histogram))
  {
    Serial.println("Histogram checksum failed. Raw SPI bytes may be corrupted.");
  }

  readCounter++;
  Serial.println();
  Serial.printf("Histogram #%lu, elapsed %lu ms\r\n", readCounter, millis());
  printHistogram(histogram);
}

static uint8_t opcTransferByte(uint8_t value, uint16_t postDelayUs)
{
  // The OPC treats each transferred byte as one command/data byte. CS is pulsed
  // around every byte, matching the byte-at-a-time timing used by common OPC
  // SPI examples.
  SPI.beginTransaction(opcSpiSettings);
  digitalWrite(OPC_CS_PIN, LOW);
  delayMicroseconds(2);
  uint8_t response = SPI.transfer(value);
  delayMicroseconds(2);
  digitalWrite(OPC_CS_PIN, HIGH);
  SPI.endTransaction();

  if (postDelayUs > 0)
  {
    delayMicroseconds(postDelayUs);
  }

  return response;
}

static bool waitForReady(uint8_t command)
{
  uint8_t response = OPC_BUSY;

  // OPC-N3 commands use a small handshake:
  //   master sends command byte
  //   OPC returns 0x31 while busy
  //   OPC returns 0xF3 when the following data bytes may be read/written
  for (uint8_t attempt = 0; attempt < 26; attempt++)
  {
    if (response != OPC_BUSY)
    {
      Serial.print("Unexpected OPC response 0x");
      printHexByte(response);
      Serial.print(" while waiting on command 0x");
      printHexByte(command);
      Serial.println(". Waiting for its SPI buffer to settle.");
      delay(5000);
      return false;
    }

    response = opcTransferByte(command, 0);
    if (response == OPC_READY)
    {
      return true;
    }

    if (response == OPC_BUSY)
    {
      delay(20);
    }
  }

  Serial.print("Timed out waiting for OPC ready on command 0x");
  printHexByte(command);
  Serial.println();
  delay(5000);
  return false;
}

static bool readBytes(uint8_t command, uint8_t *buffer, size_t length)
{
  // Read commands repeat the same command byte for each returned data byte once
  // the OPC reports READY.
  if (!waitForReady(command))
  {
    return false;
  }

  for (size_t i = 0; i < length; i++)
  {
    buffer[i] = opcTransferByte(command);
  }

  return true;
}

static bool writeByte(uint8_t command, uint8_t value)
{
  // Write commands wait until READY, then send the payload byte.
  if (!waitForReady(command))
  {
    return false;
  }

  opcTransferByte(value);
  return true;
}

static bool readPowerState()
{
  uint8_t state[6];
  if (!readBytes(CMD_READ_POWER_STATE, state, sizeof(state)))
  {
    Serial.println("Power-state read failed.");
    return false;
  }

  Serial.printf("Power state: FanON=%u LaserON=%u FanDAC=%u LaserDAC=%u LaserSwitch=%u GainToggle=%u\r\n",
                state[0], state[1], state[2], state[3], state[4], state[5]);
  return true;
}

static bool turnOpcOn()
{
  // OPC-N3 option byte: option index shifted left once, then LSB is the state.
  // Laser switch on = (3 << 1) | 1 = 0x07. Fan pot on = (1 << 1) | 1 = 0x03.
  bool laserOk = writeByte(CMD_WRITE_POWER_STATE, 0x07);
  delay(600);
  bool fanOk = writeByte(CMD_WRITE_POWER_STATE, 0x03);
  delay(1000);
  return laserOk && fanOk;
}

static uint16_t le16(const uint8_t *buffer, size_t offset)
{
  // OPC-N3 multi-byte integer fields are little-endian.
  return (uint16_t)buffer[offset] | ((uint16_t)buffer[offset + 1] << 8);
}

static float leFloat(const uint8_t *buffer, size_t offset)
{
  // The PM values are IEEE-754 float32 fields in little-endian byte order. The
  // RAK4630 is little-endian too, so memcpy gives the expected float.
  float value;
  memcpy(&value, buffer + offset, sizeof(value));
  return value;
}

static uint16_t opcCrc16(const uint8_t *buffer, size_t length)
{
  // CRC used by the OPC-N3 histogram packet. The checksum field itself is not
  // included, so callers pass the first 84 bytes of the 86-byte packet.
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; i++)
  {
    crc ^= buffer[i];
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

static bool decodeHistogram(const uint8_t *buffer, OpcHistogram &histogram)
{
  // The first 48 bytes are the 24 particle-size bins: 2 bytes per bin.
  for (uint8_t i = 0; i < 24; i++)
  {
    histogram.bins[i] = le16(buffer, i * 2);
    histogram.binsPerMl[i] = 0.0f;
  }

  // Remaining fields follow the OPC-N3 histogram model from the Alphasense SPI
  // documentation and py-opc-ng implementation.
  histogram.bin1Mtof = buffer[48];
  histogram.bin3Mtof = buffer[49];
  histogram.bin5Mtof = buffer[50];
  histogram.bin7Mtof = buffer[51];
  histogram.samplingPeriodS = le16(buffer, 52) / 100.0f;
  histogram.sampleFlowRate = le16(buffer, 54) / 100.0f;
  histogram.temperatureC = -45.0f + (175.0f * le16(buffer, 56) / 65535.0f);
  histogram.humidityPct = 100.0f * le16(buffer, 58) / 65535.0f;
  histogram.pm1 = leFloat(buffer, 60);
  histogram.pm25 = leFloat(buffer, 64);
  histogram.pm10 = leFloat(buffer, 68);
  histogram.rejectGlitch = le16(buffer, 72);
  histogram.rejectLongTof = le16(buffer, 74);
  histogram.rejectRatio = le16(buffer, 76);
  histogram.rejectOutOfRange = le16(buffer, 78);
  histogram.fanRevCount = le16(buffer, 80);
  histogram.laserStatus = le16(buffer, 82);
  histogram.checksum = le16(buffer, 84);
  histogram.calculatedChecksum = opcCrc16(buffer, 84);
  histogram.checksumOk = (histogram.checksum == histogram.calculatedChecksum);

  // Convert raw bin counts into concentration. Sample flow rate is mL/s and
  // sampling period is seconds, so SFR * period is mL sampled this histogram.
  float mlPerPeriod = histogram.sampleFlowRate * histogram.samplingPeriodS;
  if (mlPerPeriod > 0.0f)
  {
    for (uint8_t i = 0; i < 24; i++)
    {
      histogram.binsPerMl[i] = histogram.bins[i] / mlPerPeriod;
    }
  }

  return histogram.checksumOk;
}

static void printHistogram(const OpcHistogram &histogram)
{
  // Print status first so wiring/protocol failures are visible before the
  // longer bin listing scrolls by.
  Serial.print("Checksum: received=0x");
  printHexByte(histogram.checksum >> 8);
  printHexByte(histogram.checksum & 0xFF);
  Serial.print(" calculated=0x");
  printHexByte(histogram.calculatedChecksum >> 8);
  printHexByte(histogram.calculatedChecksum & 0xFF);
  Serial.println(histogram.checksumOk ? " OK" : " BAD");

  Serial.print("Sampling period (s): ");
  Serial.println(histogram.samplingPeriodS, 2);
  Serial.print("Sample flow rate: ");
  Serial.println(histogram.sampleFlowRate, 2);
  Serial.print("Temperature (C): ");
  Serial.println(histogram.temperatureC, 2);
  Serial.print("Relative humidity (%): ");
  Serial.println(histogram.humidityPct, 2);
  Serial.print("PM1 / PM2.5 / PM10 (ug/m3): ");
  Serial.print(histogram.pm1, 3);
  Serial.print(" / ");
  Serial.print(histogram.pm25, 3);
  Serial.print(" / ");
  Serial.println(histogram.pm10, 3);
  Serial.printf("Rejects: glitch=%u longTOF=%u ratio=%u outOfRange=%u\r\n",
                histogram.rejectGlitch, histogram.rejectLongTof,
                histogram.rejectRatio, histogram.rejectOutOfRange);
  Serial.printf("Fan rev count=%u Laser status=%u\r\n",
                histogram.fanRevCount, histogram.laserStatus);

  Serial.println("Bins:");
  for (uint8_t i = 0; i < 24; i++)
  {
    Serial.printf("  Bin %02u raw=%5u  count/mL=", i, histogram.bins[i]);
    Serial.println(histogram.binsPerMl[i], 4);
  }

  Serial.print("MToF us: Bin1=");
  Serial.print(histogram.bin1Mtof / 3.0f, 2);
  Serial.print(" Bin3=");
  Serial.print(histogram.bin3Mtof / 3.0f, 2);
  Serial.print(" Bin5=");
  Serial.print(histogram.bin5Mtof / 3.0f, 2);
  Serial.print(" Bin7=");
  Serial.println(histogram.bin7Mtof / 3.0f, 2);
}

static void printHexByte(uint8_t value)
{
  if (value < 0x10)
  {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}
