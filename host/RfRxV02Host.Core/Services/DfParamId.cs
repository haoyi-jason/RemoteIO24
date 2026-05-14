namespace RfRxV02Host.Core.Services;

public enum DfParamId : ushort
{
    // ── U8 params (PARAM_U8 = 0x0000) ─────────────────────────────────────
    AppSignature    = 0x0000,
    BoardId         = 0x0004,
    OpMode          = 0x0005,
    TxIntervalMs    = 0x0006,
    RfDataRateCode  = 0x0007,
    RfTxPower       = 0x0008,
    Baudrate        = 0x0009,

    // ── U16 params (PARAM_U16 = 0x1000) ───────────────────────────────────
    DeviceAddr      = 0x1000,
    DestAddr        = 0x1001,
    TimeoutMs       = 0x1002,

    VrRawPt1        = 0x1003,
    VrRawPt2        = 0x1004,
    VrRawPt3        = 0x1005,
    VrRawPt4        = 0x1006,
    VrAnglePt1      = 0x1007,
    VrAnglePt2      = 0x1008,
    VrAnglePt3      = 0x1009,
    VrAnglePt4      = 0x100A,

    StepperMinPps   = 0x100B,
    StepperMaxPps   = 0x100C,

    PwmMapRawPt1    = 0x100D,
    PwmMapRawPt2    = 0x100E,
    PwmMapRawPt3    = 0x100F,
    PwmMapRawPt4    = 0x1010,
    PwmMapCounterPt1 = 0x1011,
    PwmMapCounterPt2 = 0x1012,
    PwmMapCounterPt3 = 0x1013,
    PwmMapCounterPt4 = 0x1014,

    ServoMapRawPt1  = 0x1015,
    ServoMapRawPt2  = 0x1016,
    ServoMapRawPt3  = 0x1017,
    ServoMapRawPt4  = 0x1018,
    ServoMapCounterPt1 = 0x1019,
    ServoMapCounterPt2 = 0x101A,
    ServoMapCounterPt3 = 0x101B,
    ServoMapCounterPt4 = 0x101C,

    TxDioMask       = 0x101D,
    BlinkPeriodMs   = 0x101E,
    TxLbtLowBound   = 0x101F,
    TxLbtHighBound  = 0x1020,
    DioOnMask       = 0x1021,
    PwmMap4xRawPt1  = 0x1022,
    PwmMap4xRawPt2  = 0x1023,
    PwmMap4xRawPt3  = 0x1024,
    PwmMap4xRawPt4  = 0x1025,
    PwmMap4xCounterPt1 = 0x1026,
    PwmMap4xCounterPt2 = 0x1027,
    PwmMap4xCounterPt3 = 0x1028,
    PwmMap4xCounterPt4 = 0x1029,

    // ── U32 board section (PARAM_U32 = 0x2000) ────────────────────────────
    BoardHwVersion  = 0x2000,
    BoardFwVersion  = 0x2001,
    BoardUserId     = 0x2002,
    BoardFunctionCode = 0x2003,
    BoardSerial     = 0x2004,

    // ── I16 params (PARAM_I16 = 0x4000) ───────────────────────────────────
    SpeedMapAngle1  = 0x4000,
    SpeedMapAngle2  = 0x4001,
    SpeedMapAngle3  = 0x4002,
    SpeedMapAngle4  = 0x4003,
    SpeedMapLow1    = 0x4004,
    SpeedMapLow2    = 0x4005,
    SpeedMapLow3    = 0x4006,
    SpeedMapLow4    = 0x4007,
    SpeedMapHigh1   = 0x4008,
    SpeedMapHigh2   = 0x4009,
    SpeedMapHigh3   = 0x400A,
    SpeedMapHigh4   = 0x400B,
    TxVrOffset      = 0x400C,
    TxBatvOffset    = 0x400D,

    // ── I32 params (PARAM_I32 = 0x5000) ───────────────────────────────────
    UserRawOffsetCh1 = 0x5000,

    // ── F32 params (PARAM_F32 = 0x6000) ───────────────────────────────────
    MotorMapPpd     = 0x6000,
    RfBaseFrequency = 0x6001,
    TxVrGain        = 0x6002,
    TxBatvGain      = 0x6003,
}
