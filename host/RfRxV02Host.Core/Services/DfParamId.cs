namespace RfRxV02Host.Core.Services;

public enum DfParamId : ushort
{
    AppSignature = 0,
    BoardId = 4,
    OpMode = 5,
    TxIntervalMs = 6,
    RfDataRateCode = 7,
    RfTxPower = 8,
    Baudrate = 9,
    DeviceAddr = 0x1000,
    DestAddr = 0x1001,
    TimeoutMs = 0x1002,

    VrRawPt1 = 0x1003,
    VrRawPt2 = 0x1004,
    VrRawPt3 = 0x1005,
    VrRawPt4 = 0x1006,
    VrAnglePt1 = 0x1007,
    VrAnglePt2 = 0x1008,
    VrAnglePt3 = 0x1009,
    VrAnglePt4 = 0x100A,

    PwmMapRawPt1 = 0x100D,
    PwmMapRawPt2 = 0x100E,
    PwmMapRawPt3 = 0x100F,
    PwmMapRawPt4 = 0x1010,
    PwmMapCounterPt1 = 0x1011,
    PwmMapCounterPt2 = 0x1012,
    PwmMapCounterPt3 = 0x1013,
    PwmMapCounterPt4 = 0x1014,

    ServoMapRawPt1 = 0x1015,
    ServoMapRawPt2 = 0x1016,
    ServoMapRawPt3 = 0x1017,
    ServoMapRawPt4 = 0x1018,
    ServoMapCounterPt1 = 0x1019,
    ServoMapCounterPt2 = 0x101A,
    ServoMapCounterPt3 = 0x101B,
    ServoMapCounterPt4 = 0x101C,

    TxDioMask = 0x101D,
    BlinkPeriodMs = 0x101E,
    TxLbtLowBound = 0x101F,
    TxLbtHighBound = 0x1020,
    DioOnMask = 0x1021
}
