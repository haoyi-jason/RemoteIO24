namespace RfRxV02Host.Core.Services;

public static class DfParameterCatalog
{
    public static readonly IReadOnlyList<DfParameterInfo> All =
    [
        new() { Id = DfParamId.BoardId, Name = "BOARDID", Min = 1, Max = 255, Default = 1 },
        new() { Id = DfParamId.OpMode, Name = "OP_MODE", Min = 0, Max = 255, Default = 128 },
        new() { Id = DfParamId.TxIntervalMs, Name = "TX_INTERVAL_MS", Min = 1, Max = 10000, Default = 100 },
        new() { Id = DfParamId.RfDataRateCode, Name = "RF_DATA_RATE_CODE", Min = 0, Max = 255, Default = 0 },
        new() { Id = DfParamId.RfTxPower, Name = "RF_TX_POWER", Min = 0, Max = 255, Default = 0 },
        new() { Id = DfParamId.Baudrate, Name = "BAUDRATE", Min = 0, Max = 4, Default = 4 },
        new() { Id = DfParamId.DeviceAddr, Name = "DEVICE_ADDR", Min = 0, Max = 65535, Default = 1 },
        new() { Id = DfParamId.DestAddr, Name = "DEST_ADDR", Min = 0, Max = 65535, Default = 1 },
        new() { Id = DfParamId.TimeoutMs, Name = "TIMEOUT_MS", Min = 1, Max = 60000, Default = 1000 },

        new() { Id = DfParamId.VrRawPt1, Name = "VR_RAW_PT_1", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.VrRawPt2, Name = "VR_RAW_PT_2", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.VrRawPt3, Name = "VR_RAW_PT_3", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.VrRawPt4, Name = "VR_RAW_PT_4", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.VrAnglePt1, Name = "VR_ANGLE_PT_1", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.VrAnglePt2, Name = "VR_ANGLE_PT_2", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.VrAnglePt3, Name = "VR_ANGLE_PT_3", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.VrAnglePt4, Name = "VR_ANGLE_PT_4", Min = 0, Max = 65535, Default = 0 },

        new() { Id = DfParamId.PwmMapRawPt1, Name = "PWM_MAP_RAW_PT_1", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.PwmMapRawPt2, Name = "PWM_MAP_RAW_PT_2", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.PwmMapRawPt3, Name = "PWM_MAP_RAW_PT_3", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.PwmMapRawPt4, Name = "PWM_MAP_RAW_PT_4", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.PwmMapCounterPt1, Name = "PWM_MAP_COUNTER_PT_1", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.PwmMapCounterPt2, Name = "PWM_MAP_COUNTER_PT_2", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.PwmMapCounterPt3, Name = "PWM_MAP_COUNTER_PT_3", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.PwmMapCounterPt4, Name = "PWM_MAP_COUNTER_PT_4", Min = 0, Max = 65535, Default = 0 },

        new() { Id = DfParamId.ServoMapRawPt1, Name = "SERVO_MAP_RAW_PT_1", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.ServoMapRawPt2, Name = "SERVO_MAP_RAW_PT_2", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.ServoMapRawPt3, Name = "SERVO_MAP_RAW_PT_3", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.ServoMapRawPt4, Name = "SERVO_MAP_RAW_PT_4", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.ServoMapCounterPt1, Name = "SERVO_MAP_COUNTER_PT_1", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.ServoMapCounterPt2, Name = "SERVO_MAP_COUNTER_PT_2", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.ServoMapCounterPt3, Name = "SERVO_MAP_COUNTER_PT_3", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.ServoMapCounterPt4, Name = "SERVO_MAP_COUNTER_PT_4", Min = 0, Max = 65535, Default = 0 },

        new() { Id = DfParamId.TxLbtLowBound, Name = "TX_LBT_LOW_BOUND", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.TxLbtHighBound, Name = "TX_LBT_HIGH_BOUND", Min = 0, Max = 65535, Default = 0 },
        new() { Id = DfParamId.DioOnMask, Name = "DIO_ON_MASK", Min = 0, Max = 65535, Default = 0 }
    ];
}
