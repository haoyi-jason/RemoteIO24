# Firmware Communication Change Proposal (Before Any Firmware Edit)

This document explains required firmware-side communication changes for the new host protocol.

## Why this is needed

The new host implementation follows docs/maui_protocol_architecture.md command contract:

- ReadParam: 0x01
- WriteParam: 0x02
- ReadLive: 0x03
- WriteLive: 0x04
- ACK: 0xF0
- NAK: 0xF1
- Frame: 0x02 LEN CMD DATA CRC 0x03

But current RF_RX_V02 runtime path initializes binaryProtocolInit() and uses function code handlers in source/Common/binaryProtocolTask.c:

- FC_DATAFLASH = 0x10
- FC_LIVEDATA = 0x20
- FC_COMMAND = 0x30
- Response frame also includes filter_id field in payload layout.

## Required firmware changes (proposal only)

1. Add host command dispatcher
- Add command decode for 0x01/0x02/0x03/0x04.
- Keep legacy FC_xx handlers for backward compatibility during transition.

2. Add ACK/NAK reply mode
- On success, return ACK (0xF0) with ID + 32-bit value.
- On invalid ID or range check failure, return NAK (0xF1) with ID.

3. ID and value packing
- Use big-endian for ID and VALUE on wire.
- Keep ID width consistent (recommended: 16-bit because firmware DB addressing is 16-bit type+index).

4. CRC and framing consistency
- Verify parser computes CRC over LEN..last DATA byte exactly.
- Keep STX/ETX as 0x02/0x03.

5. Migration guard
- Add a DF flag for protocol version selection or auto-detection by CMD value range.

## Implemented in this step

Firmware host protocol rewrite has been applied in:

- source/Common/binaryProtocolTask.c

Implemented behavior:

- Native host frame parser: STX | LEN | CMD | DATA | CRC | ETX
- Host commands: 0x01/0x02/0x03/0x04
- Replies: 0xF0 (ACK), 0xF1 (NAK)
- Stream data command kept as 0x40 via send_packet()

Behavior summary:

- ReadParam (0x01): read DF by 16-bit ID and reply ACK(ID, value)
- WriteParam (0x02): write DF by 16-bit ID and 32-bit value and reply ACK/NAK
- ReadLive (0x03): read LD by 16-bit ID and reply ACK(ID, value)
- WriteLive (0x04): write LD by 16-bit ID and 32-bit value and reply ACK/NAK

Notes:

- Value is transported as 32-bit big-endian, and firmware converts to/from native DB storage size.
- The previous bin_protocol framing path is no longer used by binaryProtocolTask.c.

## Validation recommendations

1. Verify 0x01..0x04 read/write using a UART test script on RF_RX_V02 target.
2. Confirm ACK/NAK payload and CRC with captured frames.
3. Verify host read/write against DF and LD IDs used by RF_RX_V02.
