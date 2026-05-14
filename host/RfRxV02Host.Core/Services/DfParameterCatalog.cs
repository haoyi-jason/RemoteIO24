namespace RfRxV02Host.Core.Services;

public static class DfParameterCatalog
{
    public static readonly IReadOnlyList<DfParameterInfo> All =
    [
        // ── U8 ─────────────────────────────────────────────────────────────
        new() { Id = DfParamId.BoardId,          Name = "BOARDID",             Min = 1,     Max = 255,   Default = 1 },
        new() { Id = DfParamId.OpMode,           Name = "OP_MODE",             Min = 0,     Max = 255,   Default = 128 },
        new() { Id = DfParamId.TxIntervalMs,     Name = "TX_INTERVAL_MS",      Min = 1,     Max = 255,   Default = 50 },
        new() { Id = DfParamId.RfDataRateCode,   Name = "RF_DATA_RATE_CODE",   Min = 0,     Max = 6,     Default = 1 },
        new() { Id = DfParamId.RfTxPower,        Name = "RF_TX_POWER",         Min = 0,     Max = 7,     Default = 1 },
        new() { Id = DfParamId.Baudrate,         Name = "BAUDRATE",            Min = 0,     Max = 4,     Default = 4 },

        // ── U16 ────────────────────────────────────────────────────────────
        new() { Id = DfParamId.DeviceAddr,       Name = "DEVICE_ADDR",         Min = 0,     Max = 65535, Default = 1 },
        new() { Id = DfParamId.DestAddr,         Name = "DEST_ADDR",           Min = 0,     Max = 65535, Default = 1 },
        new() { Id = DfParamId.TimeoutMs,        Name = "TIMEOUT_MS",          Min = 1,     Max = 60000, Default = 2000 },

        new() { Id = DfParamId.VrRawPt1,        Name = "VR_RAW_PT_1",         Min = 0,     Max = 65535, Default = 800 },
        new() { Id = DfParamId.VrRawPt2,        Name = "VR_RAW_PT_2",         Min = 0,     Max = 65535, Default = 2600 },
        new() { Id = DfParamId.VrRawPt3,        Name = "VR_RAW_PT_3",         Min = 0,     Max = 65535, Default = 3600 },
        new() { Id = DfParamId.VrRawPt4,        Name = "VR_RAW_PT_4",         Min = 0,     Max = 65535, Default = 4000 },
        new() { Id = DfParamId.VrAnglePt1,      Name = "VR_ANGLE_PT_1",       Min = 0,     Max = 65535, Default = 0 },
        new() { Id = DfParamId.VrAnglePt2,      Name = "VR_ANGLE_PT_2",       Min = 0,     Max = 65535, Default = 30 },
        new() { Id = DfParamId.VrAnglePt3,      Name = "VR_ANGLE_PT_3",       Min = 0,     Max = 65535, Default = 170 },
        new() { Id = DfParamId.VrAnglePt4,      Name = "VR_ANGLE_PT_4",       Min = 0,     Max = 65535, Default = 200 },

        new() { Id = DfParamId.StepperMinPps,   Name = "STEPPER_MIN_PPS",     Min = 0,     Max = 65535, Default = 500 },
        new() { Id = DfParamId.StepperMaxPps,   Name = "STEPPER_MAX_PPS",     Min = 0,     Max = 65535, Default = 1500 },

        new() { Id = DfParamId.PwmMapRawPt1,    Name = "PWM_MAP_RAW_PT_1",    Min = 0,     Max = 65535, Default = 200 },
        new() { Id = DfParamId.PwmMapRawPt2,    Name = "PWM_MAP_RAW_PT_2",    Min = 0,     Max = 65535, Default = 3300 },
        new() { Id = DfParamId.PwmMapRawPt3,    Name = "PWM_MAP_RAW_PT_3",    Min = 0,     Max = 65535, Default = 3800 },
        new() { Id = DfParamId.PwmMapRawPt4,    Name = "PWM_MAP_RAW_PT_4",    Min = 0,     Max = 65535, Default = 4000 },
        new() { Id = DfParamId.PwmMapCounterPt1, Name = "PWM_MAP_COUNTER_PT_1", Min = 0,   Max = 65535, Default = 125 },
        new() { Id = DfParamId.PwmMapCounterPt2, Name = "PWM_MAP_COUNTER_PT_2", Min = 0,   Max = 65535, Default = 250 },
        new() { Id = DfParamId.PwmMapCounterPt3, Name = "PWM_MAP_COUNTER_PT_3", Min = 0,   Max = 65535, Default = 300 },
        new() { Id = DfParamId.PwmMapCounterPt4, Name = "PWM_MAP_COUNTER_PT_4", Min = 0,   Max = 65535, Default = 500 },

        new() { Id = DfParamId.ServoMapRawPt1,  Name = "SERVO_MAP_RAW_PT_1",  Min = 0,     Max = 65535, Default = 200 },
        new() { Id = DfParamId.ServoMapRawPt2,  Name = "SERVO_MAP_RAW_PT_2",  Min = 0,     Max = 65535, Default = 3300 },
        new() { Id = DfParamId.ServoMapRawPt3,  Name = "SERVO_MAP_RAW_PT_3",  Min = 0,     Max = 65535, Default = 3800 },
        new() { Id = DfParamId.ServoMapRawPt4,  Name = "SERVO_MAP_RAW_PT_4",  Min = 0,     Max = 65535, Default = 4000 },
        new() { Id = DfParamId.ServoMapCounterPt1, Name = "SERVO_MAP_COUNTER_PT_1", Min = 0, Max = 65535, Default = 125 },
        new() { Id = DfParamId.ServoMapCounterPt2, Name = "SERVO_MAP_COUNTER_PT_2", Min = 0, Max = 65535, Default = 250 },
        new() { Id = DfParamId.ServoMapCounterPt3, Name = "SERVO_MAP_COUNTER_PT_3", Min = 0, Max = 65535, Default = 300 },
        new() { Id = DfParamId.ServoMapCounterPt4, Name = "SERVO_MAP_COUNTER_PT_4", Min = 0, Max = 65535, Default = 500 },

        new() { Id = DfParamId.TxDioMask,       Name = "TX_DIO_MASK",         Min = 0,     Max = 65535, Default = 0 },
        new() { Id = DfParamId.BlinkPeriodMs,   Name = "BLINK_PERIOD_MS",     Min = 100,   Max = 65535, Default = 500 },
        new() { Id = DfParamId.TxLbtLowBound,   Name = "TX_LBT_LOW_BOUND",    Min = 0,     Max = 65535, Default = 5000 },
        new() { Id = DfParamId.TxLbtHighBound,  Name = "TX_LBT_HIGH_BOUND",   Min = 0,     Max = 65535, Default = 5100 },
        new() { Id = DfParamId.DioOnMask,       Name = "DIO_ON_MASK",         Min = 0,     Max = 65535, Default = 0 },
        new() { Id = DfParamId.PwmMap4xRawPt1,      Name = "PWM_MAP_4X_RAW_PT_1",      Min = 0, Max = 65535, Default = 200 },
        new() { Id = DfParamId.PwmMap4xRawPt2,      Name = "PWM_MAP_4X_RAW_PT_2",      Min = 0, Max = 65535, Default = 3300 },
        new() { Id = DfParamId.PwmMap4xRawPt3,      Name = "PWM_MAP_4X_RAW_PT_3",      Min = 0, Max = 65535, Default = 3800 },
        new() { Id = DfParamId.PwmMap4xRawPt4,      Name = "PWM_MAP_4X_RAW_PT_4",      Min = 0, Max = 65535, Default = 4000 },
        new() { Id = DfParamId.PwmMap4xCounterPt1,  Name = "PWM_MAP_4X_COUNTER_PT_1",  Min = 0, Max = 65535, Default = 125 },
        new() { Id = DfParamId.PwmMap4xCounterPt2,  Name = "PWM_MAP_4X_COUNTER_PT_2",  Min = 0, Max = 65535, Default = 250 },
        new() { Id = DfParamId.PwmMap4xCounterPt3,  Name = "PWM_MAP_4X_COUNTER_PT_3",  Min = 0, Max = 65535, Default = 300 },
        new() { Id = DfParamId.PwmMap4xCounterPt4,  Name = "PWM_MAP_4X_COUNTER_PT_4",  Min = 0, Max = 65535, Default = 500 },

        // ── U32 board section (read-only info) ─────────────────────────────
        new() { Id = DfParamId.BoardHwVersion,  Name = "BOARD_HW_VERSION",    Min = 0,     Max = uint.MaxValue, Default = 0 },
        new() { Id = DfParamId.BoardFwVersion,  Name = "BOARD_FW_VERSION",    Min = 0,     Max = uint.MaxValue, Default = 0 },
        new() { Id = DfParamId.BoardSerial,     Name = "BOARD_SERIAL",        Min = 0,     Max = uint.MaxValue, Default = 0 },

        // ── I16 speed map ───────────────────────────────────────────────────
        new() { Id = DfParamId.SpeedMapAngle1,  Name = "SPEED_MAP_ANGLE_1",   Min = 0,     Max = 32767, Default = 0 },
        new() { Id = DfParamId.SpeedMapAngle2,  Name = "SPEED_MAP_ANGLE_2",   Min = 0,     Max = 32767, Default = 0 },
        new() { Id = DfParamId.SpeedMapAngle3,  Name = "SPEED_MAP_ANGLE_3",   Min = 0,     Max = 32767, Default = 0 },
        new() { Id = DfParamId.SpeedMapAngle4,  Name = "SPEED_MAP_ANGLE_4",   Min = 0,     Max = 32767, Default = 0 },
        new() { Id = DfParamId.SpeedMapLow1,    Name = "SPEED_MAP_LOW_1",     Min = 0,     Max = 32767, Default = 0 },
        new() { Id = DfParamId.SpeedMapLow2,    Name = "SPEED_MAP_LOW_2",     Min = 0,     Max = 32767, Default = 0 },
        new() { Id = DfParamId.SpeedMapLow3,    Name = "SPEED_MAP_LOW_3",     Min = 0,     Max = 32767, Default = 0 },
        new() { Id = DfParamId.SpeedMapLow4,    Name = "SPEED_MAP_LOW_4",     Min = 0,     Max = 32767, Default = 0 },
        new() { Id = DfParamId.SpeedMapHigh1,   Name = "SPEED_MAP_HIGH_1",    Min = 0,     Max = 32767, Default = 0 },
        new() { Id = DfParamId.SpeedMapHigh2,   Name = "SPEED_MAP_HIGH_2",    Min = 0,     Max = 32767, Default = 0 },
        new() { Id = DfParamId.SpeedMapHigh3,   Name = "SPEED_MAP_HIGH_3",    Min = 0,     Max = 32767, Default = 0 },
        new() { Id = DfParamId.SpeedMapHigh4,   Name = "SPEED_MAP_HIGH_4",    Min = 0,     Max = 32767, Default = 0 },
        new() { Id = DfParamId.TxVrOffset,      Name = "TX_VR_OFFSET",        Min = 0,     Max = 32767, Default = 0 },
        new() { Id = DfParamId.TxBatvOffset,    Name = "TX_BATV_OFFSET",      Min = 0,     Max = 32767, Default = 0 },

        // ── F32 (transmitted as raw uint32 / IEEE754 bits) ─────────────────
        new() { Id = DfParamId.MotorMapPpd,     Name = "MOTOR_MAP_PPD",       Min = 0,     Max = uint.MaxValue, Default = 0 },
        new() { Id = DfParamId.RfBaseFrequency, Name = "RF_BASE_FREQUENCY",   Min = 0,     Max = uint.MaxValue, Default = 0 },
        new() { Id = DfParamId.TxVrGain,        Name = "TX_VR_GAIN",          Min = 0,     Max = uint.MaxValue, Default = 0 },
        new() { Id = DfParamId.TxBatvGain,      Name = "TX_BATV_GAIN",        Min = 0,     Max = uint.MaxValue, Default = 0 },
    ];
}
