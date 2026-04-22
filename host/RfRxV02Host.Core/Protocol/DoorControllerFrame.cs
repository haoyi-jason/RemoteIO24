namespace RfRxV02Host.Core.Protocol;

public sealed class DoorControllerFrame
{
    public required byte Command { get; init; }
    public required byte[] Data { get; init; }

    public ushort Id => Data.Length >= 2
        ? (ushort)((Data[0] << 8) | Data[1])
        : (ushort)0;

    public uint Value => Data.Length >= 6
        ? ((uint)Data[2] << 24) | ((uint)Data[3] << 16) | ((uint)Data[4] << 8) | Data[5]
        : 0;
}
