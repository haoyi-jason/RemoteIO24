namespace RfRxV02Host.Core.Services;

public enum LiveDataId : ushort
{
    Rssi = 0,
    UserControl = 1,
    DioState = 0x1000,
    AinCh1 = 0x1001,
    AinCh2 = 0x1002,
    AinCh3 = 0x1003,
    AinCh4 = 0x1004,
    BatMv = 0x1005,
    ValveControl = 0x2000,
    ServoControl = 0x2001,
    Stepper = 0x5000
}
