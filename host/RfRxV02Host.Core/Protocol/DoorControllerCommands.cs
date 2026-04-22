namespace RfRxV02Host.Core.Protocol;

public static class DoorControllerCommands
{
    public const byte CmdReadParam = 0x01;
    public const byte CmdWriteParam = 0x02;
    public const byte CmdReadLive = 0x03;
    public const byte CmdWriteLive = 0x04;
    public const byte CmdResetMcu = 0x05;
    public const byte CmdAck = 0xF0;
    public const byte CmdNak = 0xF1;
}
