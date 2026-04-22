using System.Buffers.Binary;

namespace RfRxV02Host.Core.Protocol;

public static class DoorControllerProtocol
{
    public const byte Stx = 0x02;
    public const byte Etx = 0x03;

    public static byte ComputeCrc(ReadOnlySpan<byte> bytes)
    {
        byte crc = 0;
        foreach (byte b in bytes)
        {
            crc ^= b;
        }

        return crc;
    }

    public static byte[] BuildReadFrame(byte command, ushort id)
    {
        Span<byte> payload = stackalloc byte[3];
        payload[0] = command;
        BinaryPrimitives.WriteUInt16BigEndian(payload[1..3], id);
        return BuildFrame(payload);
    }

    public static byte[] BuildWriteFrame(byte command, ushort id, uint value)
    {
        Span<byte> payload = stackalloc byte[7];
        payload[0] = command;
        BinaryPrimitives.WriteUInt16BigEndian(payload[1..3], id);
        BinaryPrimitives.WriteUInt32BigEndian(payload[3..7], value);
        return BuildFrame(payload);
    }

    public static byte[] BuildCommandFrame(byte command)
    {
        Span<byte> payload = stackalloc byte[1];
        payload[0] = command;
        return BuildFrame(payload);
    }

    public static bool TryParseFrame(List<byte> buffer, out DoorControllerFrame? frame)
    {
        frame = null;
        if (buffer.Count == 0)
        {
            return false;
        }

        int stxIndex = buffer.IndexOf(Stx);
        if (stxIndex < 0)
        {
            buffer.Clear();
            return false;
        }

        if (stxIndex > 0)
        {
            buffer.RemoveRange(0, stxIndex);
        }

        if (buffer.Count < 5)
        {
            return false;
        }

        byte len = buffer[1];
        int totalFrameLength = len + 4;
        if (buffer.Count < totalFrameLength)
        {
            return false;
        }

        byte etx = buffer[totalFrameLength - 1];
        if (etx != Etx)
        {
            buffer.RemoveAt(0);
            return false;
        }

        byte expectedCrc = buffer[totalFrameLength - 2];
        byte actualCrc = 0;
        for (int i = 1; i <= len + 1; i++)
        {
            actualCrc ^= buffer[i];
        }
        if (expectedCrc != actualCrc)
        {
            buffer.RemoveAt(0);
            return false;
        }

        byte cmd = buffer[2];
        int dataLen = len - 1;
        byte[] data = dataLen > 0 ? buffer.GetRange(3, dataLen).ToArray() : Array.Empty<byte>();
        frame = new DoorControllerFrame
        {
            Command = cmd,
            Data = data
        };

        buffer.RemoveRange(0, totalFrameLength);
        return true;
    }

    public static byte[] BuildLegacyFrame(byte command, byte filterId, ReadOnlySpan<byte> payload)
    {
        byte len = (byte)payload.Length;
        byte[] frame = new byte[payload.Length + 6];
        frame[0] = Stx;
        frame[1] = command;
        frame[2] = filterId;
        frame[3] = len;
        payload.CopyTo(frame.AsSpan(4));
        frame[frame.Length - 2] = ComputeCrc(frame.AsSpan(1, len + 3));
        frame[frame.Length - 1] = Etx;
        return frame;
    }

    public static bool TryParseLegacyFrame(List<byte> buffer, out DoorControllerFrame? frame, out byte filterId)
    {
        frame = null;
        filterId = 0;

        if (buffer.Count == 0)
        {
            return false;
        }

        int stxIndex = buffer.IndexOf(Stx);
        if (stxIndex < 0)
        {
            buffer.Clear();
            return false;
        }

        if (stxIndex > 0)
        {
            buffer.RemoveRange(0, stxIndex);
        }

        if (buffer.Count < 6)
        {
            return false;
        }

        byte len = buffer[3];
        int totalFrameLength = len + 6;
        if (buffer.Count < totalFrameLength)
        {
            return false;
        }

        byte etx = buffer[totalFrameLength - 1];
        if (etx != Etx)
        {
            buffer.RemoveAt(0);
            return false;
        }

        byte expectedCrc = buffer[totalFrameLength - 2];
        byte actualCrc = ComputeCrc(buffer.GetRange(1, len + 3).ToArray());
        if (expectedCrc != actualCrc)
        {
            buffer.RemoveAt(0);
            return false;
        }

        byte cmd = buffer[1];
        filterId = buffer[2];
        byte[] data = len > 0 ? buffer.GetRange(4, len).ToArray() : Array.Empty<byte>();
        frame = new DoorControllerFrame
        {
            Command = cmd,
            Data = data
        };

        buffer.RemoveRange(0, totalFrameLength);
        return true;
    }

    private static byte[] BuildFrame(ReadOnlySpan<byte> payload)
    {
        byte len = (byte)payload.Length;
        byte[] frame = new byte[payload.Length + 4];
        frame[0] = Stx;
        frame[1] = len;
        payload.CopyTo(frame.AsSpan(2));
        frame[frame.Length - 2] = ComputeCrc(frame.AsSpan(1, len + 1));
        frame[frame.Length - 1] = Etx;
        return frame;
    }
}
