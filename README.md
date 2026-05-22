# VBUS LoRa Communications Architecture

## Goal

The field system has about 10 sites. Each air-quality node samples local sensors at 1 Hz, batches those readings into 10-second records, and sends them over LoRa to a central WisBlock gateway. A collaborating researcher may run a second sensor suite at the same 10 sites with many more variables, including 2D sonic momentum, radiation, temperature, and RH at two heights.

The main design constraint is LoRa airtime. Cellular upload bandwidth is much less restrictive than LoRa, so the system should split the radio traffic before combining records for cloud upload.

## Recommended Layout

Use two central WisBlock LoRa gateways and one cellular uploader:

```text
Air-quality nodes  ---> LoRa channel A ---> Gateway A ---> Cellular ---> Cloud
                                                ^
                                                |
Micromet nodes     ---> LoRa channel B ---> Gateway B
                              local UART/I2C/SPI bridge
```

Gateway A is the air-quality gateway and also owns the cellular module. Gateway B is the micromet/radiation gateway. Gateway B should not forward records to Gateway A over LoRa, because that would consume more LoRa airtime. Use a short local wired connection between the gateways instead.

## Why Two LoRa Gateways

A RAK4630/WisBlock SX126x LoRa board is a single half-duplex LoRa radio. Adding a second antenna to the same radio does not create a second simultaneous LoRa channel. The radio can transmit or receive on one frequency at a time. It can retune between frequencies, but it cannot listen to two LoRa channels at once.

The air-quality design already uses most of one simple TDMA network:

- 10 known nodes.
- 10-second frame.
- One 900 ms slot per node.
- One binary batch per node per frame.
- Six fixed channels sampled at 1 Hz.

Adding around 20 more raw 1 Hz signals to the same packet stream would push the single-channel design beyond a comfortable margin. Splitting the two sensor families across two LoRa channels keeps the timing understandable and makes each project easier to debug.

## Gateway Roles

### Gateway A: Cellular Uplink Gateway

- Receives air-quality packets over LoRa channel A.
- Saves decoded air-quality records to local SD.
- Receives forwarded micromet/radiation records from Gateway B over a local wired bridge.
- Saves forwarded records to local SD.
- Uploads combined records to the cloud in batches.

### Gateway B: LoRa-Only Peer Gateway

- Receives micromet/radiation packets over LoRa channel B.
- Saves decoded micromet/radiation records to local SD.
- Forwards decoded records to Gateway A over UART, I2C, SPI, or another short wired link.
- Keeps its own SD copy so data is not lost if Gateway A or the cellular link fails.

## Current Firmware Framework

The codebase now has a shared binary air-quality LoRa protocol in `include/air_quality_protocol.h`:

- `BEACON`: gateway frame sync.
- `BATCH`: one node's 10-second, 1 Hz, six-channel data record.
- `ACK`: gateway acknowledgment after save/dedup handling.

The gateway bridge framework is separated in `include/gateway_bridge.h`:

- `BRIDGE_RECORD`: a local wired-link frame for forwarding decoded gateway records.
- Source IDs distinguish air-quality records from micromet/radiation records.
- The bridge frame carries received time, node ID, boot ID, sequence number, RSSI/SNR, and the binary payload.

In `src/rx_test.cpp`, the gateway has placeholder hooks for:

- Forwarding decoded local LoRa records to a cellular gateway.
- Receiving forwarded records from a peer gateway.
- Staging records for cellular upload.

These hooks are intentionally stubs until the physical bridge and cellular module are selected.

## Storage Policy

Use SD or equivalent nonvolatile storage on both gateways.

- Nodes should eventually save each 10-second batch locally before LoRa transmit.
- Each gateway should save every decoded local LoRa record before ACKing in production mode.
- Gateway B should save its own records even though Gateway A also receives forwarded copies.
- Gateway A should upload only after records are safely staged locally.

This gives at least one local copy near each radio receiver and avoids making the cellular gateway the only copy of the collaborator's data.

## Implementation Defaults

- Air-quality LoRa network: current `916 MHz`, SF7, 125 kHz bandwidth baseline.
- Micromet LoRa network: use a different legal frequency/channel and its own gateway.
- Local gateway bridge: start with UART because it is easy to inspect and robust enough for small local batches.
- Cloud upload cadence: start with 1-minute batches, then adjust after field testing.
- Production ACK policy: set gateway firmware to ACK only after SD save succeeds.

## Open Hardware Decisions

- Exact cellular WisBlock module and cloud endpoint.
- SD module/pins and file format.
- Local bridge wiring between Gateway A and Gateway B.
- Whether the micromet payload uses this same binary framing style or its own decoder before bridge forwarding.

## Bench Test Checklist

- Verify air-quality node-to-Gateway-A TDMA timing with one node, then several node IDs.
- Verify Gateway B can forward a synthetic bridge record to Gateway A over the local wired link.
- Verify Gateway A can stage both local LoRa records and forwarded peer records.
- Simulate cellular outage and confirm records remain on SD.
- Simulate Gateway A outage and confirm Gateway B keeps its own SD log.
- Run both LoRa gateways at the same time on separate channels and confirm neither network loses timing because of the other.

## Host Protocol Check

The binary protocol and bridge framing can be checked without PlatformIO:

```sh
g++ -std=c++11 -Itest/arduino_stubs -Iinclude test/protocol_host_roundtrip.cpp -o /tmp/protocol_host_roundtrip
/tmp/protocol_host_roundtrip
```

This verifies BEACON, BATCH, ACK, CRC rejection, and gateway bridge record round trips.
