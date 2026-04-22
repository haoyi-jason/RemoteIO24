# MAUI Host App — Binary Protocol + DF/LD Architecture

> **Purpose**: This document is a complete, self-contained reference for an AI agent (or developer) starting a new .NET MAUI project that uses the same binary protocol and DF_/LD_ database pattern as DoorControllerX.

---

## 1. Overview

The architecture has three concerns:

| Layer | What it does |
|---|---|
| **Transport** | Bluetooth / UART byte stream, exposes task-based async API |
| **Protocol** | Binary framing, checksum, command dispatch |
| **Database** | DF_ (persistent parameters) and LD_ (live runtime data) register map |

The MAUI host only calls `ITransportClient`; it never touches raw bytes directly.

---

## 2. Binary Frame Format

```
 Byte 0  : STX  = 0x02
 Byte 1  : LEN  = 1 + len(DATA)    (covers CMD + DATA)
 Byte 2  : CMD  (function code)
 Byte 3…N: DATA (0 or more bytes, big-endian for multi-byte values)
 Byte N+1: CRC  = XOR of bytes[1 .. N]  (LEN through last DATA byte)
 Byte N+2: ETX  = 0x03
```

**Total frame size** = `len(DATA) + 5`

### CRC Rule

```csharp
static byte ComputeCrc(ReadOnlySpan<byte> bytes)
{
    byte crc = 0;
    foreach (var b in bytes) crc ^= b;
    return crc;
}
// Range: frame[1] through frame[1 + LEN]  (inclusive)
```

### Command Codes (Function Codes)

| Constant | Value | Direction | Meaning |
|---|---|---|---|
| `CmdReadParam`  | `0x01` | Host → FW | Read one DF_ parameter by ID |
| `CmdWriteParam` | `0x02` | Host → FW | Write one DF_ parameter |
| `CmdReadLive`   | `0x03` | Host → FW | Read one LD_ live datum by ID |
| `CmdWriteLive`  | `0x04` | Host → FW | Write one LD_ live datum |
| `CmdAck`        | `0xF0` | FW → Host | Positive reply carrying value |
| `CmdNak`        | `0xF1` | FW → Host | Negative reply / range error |

---

## 3. Packet Encoding

All values are **uint32, big-endian**.

### Read Request (host → firmware)

```
STX | LEN=3 | CMD | ID(2B) | CRC | ETX
```

### Write Request (host → firmware)

```
STX | LEN=7 | CMD | ID(2B) | VALUE(4B big-endian) | CRC | ETX
```

### ACK Reply (firmware → host)

```
STX | LEN=7 | 0xF0 | ID(2B) | VALUE(4B big-endian) | CRC | ETX
```

### NAK Reply (firmware → host)

```
STX | LEN=3 | 0xF1 | ID(2B) | CRC | ETX
```

---

## 4. Frame Parser (host side)

The parser is streaming; call `TryParseFrame` repeatedly after appending received bytes to a `List<byte>` buffer.

```csharp
List<byte> _rxBuffer = new();

// After appending new bytes:
while (DoorControllerProtocol.TryParseFrame(_rxBuffer, out var frame))
{
    HandleFrame(frame!);
}
```

Parser behavior:
- Silently discards bytes before the next STX (re-sync on noise).
- Returns `false` and leaves buffer intact if a full frame has not yet arrived.
- Returns `false` and discards garbage bytes if ETX or CRC is wrong.
- Never blocks; caller drives the loop.

---

## 5. DF_ Parameter Database

DF_ parameters are **persistent** (stored in firmware Flash/EEPROM). Each parameter has:

- `Id` — `byte`, matches firmware `df_param_id_t` enum **exactly** (zero-based, sequential)
- `Name` — English identifier string (e.g. `"DF_BLOCK_RETRY_DELAY_SEC"`)
- `Min`, `Max`, `Default` — `uint` range enforced by firmware before writing

### C# Model

```csharp
// Models/DfParamId.cs
public enum DfParamId : byte
{
    BlockRetryDelaySec = 0,
    OpenTriggerAngle   = 1,
    // … (one entry per DF_* firmware enum, same order)
}

// Models/DfParameterInfo.cs
public sealed class DfParameterInfo
{
    public required DfParamId Id      { get; init; }
    public required string    Name    { get; init; }  // English; used in logs / ToString()
    public required uint      Min     { get; init; }
    public required uint      Max     { get; init; }
    public required uint      Default { get; init; }

    // Localized display (e.g. Chinese) — **only bind this in the picker UI**
    public string PickerDisplayName => $"{(byte)Id:D2} {GetLocalizedName(Id)}";

    public override string ToString() => $"{(byte)Id:D2} {Name}";

    private static string GetLocalizedName(DfParamId id) => id switch
    {
        DfParamId.BlockRetryDelaySec => "YOUR_LOCALIZED_NAME",
        // … one case per ID
        _ => id.ToString()
    };
}
```

### Static Catalog

```csharp
// Services/DfParameterCatalog.cs
public static class DfParameterCatalog
{
    public static readonly IReadOnlyList<DfParameterInfo> All =
    [
        new() { Id = DfParamId.BlockRetryDelaySec, Name = "DF_BLOCK_RETRY_DELAY_SEC",
                Min = 1, Max = 10, Default = 1 },
        // … one entry per parameter, same order as firmware enum
    ];
}
```

> **Warning**: `DfParamId` enum values **must** match the firmware `df_param_id_t` integer values exactly. Any shift causes silent corruption.

---

## 6. LD_ Live Data Database

LD_ entries are **runtime RAM values** polled from the firmware at regular intervals.  
They are read-only from the host except for command-latch IDs (e.g. `LD_REMOTE_CMD`).

### C# Enum

```csharp
// Models/LiveDataId.cs
public enum LiveDataId : byte
{
    SysState       = 0,
    M1State        = 1,
    // … (same order and values as firmware ld_param_id_t)
}
```

> **Warning**: Same as DF_ — values must match firmware exactly and gap-free unless the firmware also has gaps.

### Polling Pattern

```csharp
// Declare which IDs to poll
private static readonly LiveDataId[] SnapshotIds =
[
    LiveDataId.SysState,
    LiveDataId.M1State,
    // …
];

// Timer callback (non-reentrant, use a _pollBusy guard)
foreach (var id in SnapshotIds)
{
    uint value = await _transport.ReadLiveAsync((byte)id);
    _latestLiveValues[id] = value;
}
```

---

## 7. ITransportClient Interface

This interface decouples protocol logic from the physical transport (Bluetooth, USB-Serial, mock).

```csharp
public interface ITransportClient : IDisposable
{
    bool IsOpen { get; }

    Task<IReadOnlyList<ConnectionEndpoint>> GetAvailableEndpointsAsync(
        CancellationToken cancellationToken = default);

    Task ConnectAsync(string endpoint, int baudRate,
        CancellationToken cancellationToken = default);

    Task DisconnectAsync(
        CancellationToken cancellationToken = default);

    Task<uint> ReadLiveAsync(byte id,
        int timeoutMs = 1000, CancellationToken cancellationToken = default);

    Task<uint> ReadParamAsync(byte id,
        int timeoutMs = 1000, CancellationToken cancellationToken = default);

    Task WriteParamAsync(byte id, uint value,
        int timeoutMs = 1000, CancellationToken cancellationToken = default);

    Task WriteLiveAsync(byte id, uint value,
        int timeoutMs = 1000, CancellationToken cancellationToken = default);
}
```

Implementations:
- **`AndroidBluetoothTransport`** — Bluetooth Classic SPP (Android only, requires `BLUETOOTH_CONNECT` / `BLUETOOTH_SCAN` permissions on Android 31+)
- Substitute a mock implementation for unit tests or desktop preview.

---

## 8. Project Structure Convention

```
YourHost.Core/               ← portable .NET class library (net9.0)
  Abstractions/
    ITransportClient.cs
  Models/
    DfParamId.cs
    DfParameterInfo.cs
    LiveDataId.cs
    ConnectionEndpoint.cs
  Protocol/
    DoorControllerFrame.cs   ← rename to fit your domain
    DoorControllerProtocol.cs
  Services/
    DfParameterCatalog.cs

YourHost.Maui/               ← MAUI application (net9.0-android / -ios / …)
  Platforms/Android/
    AndroidBluetoothTransport.cs
  MainPage.xaml / .cs        ← live data dashboard
  ParametersPage.xaml / .cs  ← DF read/write UI
  MauiProgram.cs             ← DI registration
```

### DI Registration (MauiProgram.cs)

```csharp
builder.Services.AddSingleton<ITransportClient, AndroidBluetoothTransport>();
builder.Services.AddSingleton<MainPage>();
builder.Services.AddSingleton<ParametersPage>();
```

---

## 9. Parameters Page Pattern

The Parameters page is the canonical UI for DF_ read/write.

```xml
<!-- ParametersPage.xaml -->

<!-- Picker: bind to PickerDisplayName, NOT Name -->
<Picker x:Name="ParamPicker"
        ItemsSource="{x:Static svc:DfParameterCatalog.All}"
        ItemDisplayBinding="{Binding PickerDisplayName}" />

<!-- Value entry + range label -->
<Entry x:Name="ValueEntry" Keyboard="Numeric" />
<Label x:Name="RangeLabel" />   <!-- e.g. "Range: 1 – 10  Default: 1" -->

<!-- Buttons -->
<Button Text="Read"     Clicked="OnReadClicked" />
<Button Text="Write"    Clicked="OnWriteClicked" />
<Button Text="Read All" Clicked="OnReadAllClicked" />
```

```csharp
// Read
var info = (DfParameterInfo)ParamPicker.SelectedItem;
uint val = await _transport.ReadParamAsync((byte)info.Id);
ValueEntry.Text = val.ToString();
RangeLabel.Text = $"Range: {info.Min} – {info.Max}  Default: {info.Default}";

// Write
if (!uint.TryParse(ValueEntry.Text, out uint writeVal)) return;
if (writeVal < info.Min || writeVal > info.Max) { /* show error */ return; }
await _transport.WriteParamAsync((byte)info.Id, writeVal);
```

---

## 10. Localization Design Rule

| Property | Purpose | Bind in |
|---|---|---|
| `DfParameterInfo.Name` | English identifier; used in `ToString()`, logs, debug output | Never bind directly to picker |
| `DfParameterInfo.PickerDisplayName` | `"02 Chinese_label"` format; localized | **Only** `ParametersPage` picker `ItemDisplayBinding` |

This keeps localization contained to one page. All other UI elements (charts, status panels, logs) use `ToString()` which stays English.

---

## 11. Porting Checklist for a New Project

- [ ] Copy `DoorControllerProtocol.cs` and `DoorControllerFrame.cs`; rename constants/class names to fit domain
- [ ] Define new `DfParamId` enum matching firmware `df_param_id_t` **exactly** (same integer values)
- [ ] Define new `LiveDataId` enum matching firmware `ld_param_id_t` **exactly**
- [ ] Populate `DfParameterCatalog.All` with correct Min/Max/Default for each ID (confirm with firmware source)
- [ ] Implement `ITransportClient` for target transport (BT Classic, BLE, USB-Serial, TCP)
- [ ] Register transport and pages in `MauiProgram.cs` via DI
- [ ] Bind `ParametersPage` picker to `PickerDisplayName`, not `Name`
- [ ] Add `PickerDisplayName` / `GetLocalizedName()` entries for new language if needed
- [ ] Add `[SupportedOSPlatform]` guard attributes for Android-specific APIs (avoids CA1416)
- [ ] Set `ApplicationDisplayVersion` and `ApplicationVersion` in `.csproj` before first release

---

## 12. Frame Contract Quick Reference

```
Read DF[id]:     02 02 01 <id>                  CRC 03
Write DF[id,v]:  02 06 02 <id> <v3><v2><v1><v0> CRC 03
Read LD[id]:     02 02 03 <id>                  CRC 03
Write LD[id,v]:  02 06 04 <id> <v3><v2><v1><v0> CRC 03

ACK reply:       02 06 F0 <id> <v3><v2><v1><v0> CRC 03
NAK reply:       02 02 F1 <id>                  CRC 03
```

Value bytes are **big-endian** (`v3` = MSB).

CRC = XOR of all bytes from LEN through last DATA byte (inclusive).

---

## 13. Common Pitfalls

| Pitfall | Prevention |
|---|---|
| Enum ID mismatch between host and firmware | Always derive host enum directly from firmware header; add a comment linking to the firmware file |
| Chinese names in `Name` bleed into non-picker UI | Keep `Name` English; add a separate `PickerDisplayName` property |
| Concurrent poll + manual read/write | Use a `_pollBusy` bool guard; cancel poll timer during manual operations |
| Missing `SupportedOSPlatform` on Bluetooth APIs | Wrap Android 31+ APIs with `if (OperatingSystem.IsAndroidVersionAtLeast(31))` |
| ACK/NAK matching wrong transaction | Echo the request ID in the reply and match in the pending-request dictionary |
| Big vs little endian confusion | Always document endianness at the top of the protocol file; use `BinaryPrimitives` helpers |
