namespace RfRxV02Host.Core.Services;

public enum LiveDataId : ushort
{
    // ── U8 live (PARAM_U8 | PARAM_LIVE = 0x8000) ──────────────────────────
    Rssi        = 0x0000,
    UserControl = 0x0001,
    TxMode      = 0x0002,

    // ── U16 live (PARAM_U16 | PARAM_LIVE = 0x9000) ────────────────────────
    DioState    = 0x1000,
    AinCh1      = 0x1001,
    AinCh2      = 0x1002,
    AinCh3      = 0x1003,
    AinCh4      = 0x1004,
    BatMv       = 0x1005,

    // ── U32 live (PARAM_U32 | PARAM_LIVE = 0xA000) ────────────────────────
    ValveControl  = 0x2000,
    ValveControl2 = 0x2001,
    ValveControl3 = 0x2002,
    ValveControl4 = 0x2003,
    ServoControl  = 0x2004,   // was 0x2001 (bug: clashed with ValveControl2)

    // ── I32 live (PARAM_I32 | PARAM_LIVE = 0xD000) ────────────────────────
    Stepper     = 0x5000,
}
