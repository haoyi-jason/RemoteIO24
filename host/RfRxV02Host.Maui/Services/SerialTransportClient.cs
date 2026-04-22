using RfRxV02Host.Core.Abstractions;
using RfRxV02Host.Core.Models;
using RfRxV02Host.Core.Protocol;
using System.IO.Ports;
using System.Diagnostics;

namespace RfRxV02Host.Maui.Services;

public sealed class SerialTransportClient : ITransportClient
{
    private SerialPort? _serialPort;
    private readonly List<byte> _rxBuffer = new();
    private readonly SemaphoreSlim _txLock = new(1, 1);
    private TaskCompletionSource<DoorControllerFrame>? _pendingReply;
    private ushort? _pendingId;

    public bool IsOpen => _serialPort?.IsOpen == true;

    public Task<IReadOnlyList<ConnectionEndpoint>> GetAvailableEndpointsAsync(CancellationToken cancellationToken = default)
    {
        string[] ports = SerialPort.GetPortNames();
        IReadOnlyList<ConnectionEndpoint> endpoints = ports
            .OrderBy(p => p)
            .Select(p => new ConnectionEndpoint { Id = p, DisplayName = p })
            .ToList();
        return Task.FromResult(endpoints);
    }

    public Task ConnectAsync(string endpoint, int baudRate, CancellationToken cancellationToken = default)
    {
        _serialPort?.Dispose();
        _serialPort = new SerialPort(endpoint, baudRate, Parity.None, 8, StopBits.One)
        {
            ReadTimeout = 100,
            WriteTimeout = 500
        };
        _serialPort.DataReceived += OnDataReceived;
        _serialPort.Open();
        return Task.CompletedTask;
    }

    public Task DisconnectAsync(CancellationToken cancellationToken = default)
    {
        if (_serialPort is not null)
        {
            _serialPort.DataReceived -= OnDataReceived;
            _serialPort.Dispose();
            _serialPort = null;
        }
        return Task.CompletedTask;
    }

    public Task<uint> ReadLiveAsync(ushort id, int timeoutMs = 1000, CancellationToken cancellationToken = default)
        => SendReadAsync(DoorControllerCommands.CmdReadLive, id, timeoutMs, cancellationToken);

    public Task<uint> ReadParamAsync(ushort id, int timeoutMs = 1000, CancellationToken cancellationToken = default)
        => SendReadAsync(DoorControllerCommands.CmdReadParam, id, timeoutMs, cancellationToken);

    public Task WriteParamAsync(ushort id, uint value, int timeoutMs = 1000, CancellationToken cancellationToken = default)
        => SendWriteAsync(DoorControllerCommands.CmdWriteParam, id, value, timeoutMs, cancellationToken);

    public Task WriteLiveAsync(ushort id, uint value, int timeoutMs = 1000, CancellationToken cancellationToken = default)
        => SendWriteAsync(DoorControllerCommands.CmdWriteLive, id, value, timeoutMs, cancellationToken);

    public async Task ResetMcuAsync(int timeoutMs = 1000, CancellationToken cancellationToken = default)
    {
        if (_serialPort is null || !_serialPort.IsOpen)
        {
            throw new InvalidOperationException("Transport is not connected.");
        }

        byte[] request = DoorControllerProtocol.BuildCommandFrame(DoorControllerCommands.CmdResetMcu);
        await _txLock.WaitAsync(cancellationToken);
        try
        {
            _serialPort.Write(request, 0, request.Length);
        }
        finally
        {
            _txLock.Release();
        }
    }

    private async Task<uint> SendReadAsync(byte cmd, ushort id, int timeoutMs, CancellationToken cancellationToken)
    {
        byte[] request = DoorControllerProtocol.BuildReadFrame(cmd, id);
        DoorControllerFrame response = await SendAndWaitAsync(request, id, timeoutMs, cancellationToken);
        ValidateReply(response, id);
        return response.Value;
    }

    private async Task SendWriteAsync(byte cmd, ushort id, uint value, int timeoutMs, CancellationToken cancellationToken)
    {
        byte[] request = DoorControllerProtocol.BuildWriteFrame(cmd, id, value);
        DoorControllerFrame response = await SendAndWaitAsync(request, id, timeoutMs, cancellationToken);
        ValidateReply(response, id);
    }

    private static void ValidateReply(DoorControllerFrame reply, ushort expectedId)
    {
        if (reply.Command == DoorControllerCommands.CmdNak)
        {
            throw new InvalidOperationException($"NAK from device. id={expectedId}");
        }

        if (reply.Command != DoorControllerCommands.CmdAck)
        {
            throw new InvalidOperationException($"Unexpected reply cmd=0x{reply.Command:X2}");
        }

        if (reply.Id != expectedId)
        {
            throw new InvalidOperationException($"Reply id mismatch. expected={expectedId}, actual={reply.Id}");
        }
    }

    private async Task<DoorControllerFrame> SendAndWaitAsync(byte[] request, ushort expectedId, int timeoutMs, CancellationToken cancellationToken)
    {
        if (_serialPort is null || !_serialPort.IsOpen)
        {
            throw new InvalidOperationException("Transport is not connected.");
        }

        await _txLock.WaitAsync(cancellationToken);
        try
        {
            _pendingReply = new TaskCompletionSource<DoorControllerFrame>(TaskCreationOptions.RunContinuationsAsynchronously);
            _pendingId = expectedId;
            _serialPort.Write(request, 0, request.Length);

            Stopwatch sw = Stopwatch.StartNew();
            while (sw.ElapsedMilliseconds < timeoutMs)
            {
                cancellationToken.ThrowIfCancellationRequested();
                DrainIncomingSerialBytes();

                if (_pendingReply.Task.IsCompleted)
                {
                    return await _pendingReply.Task;
                }

                await Task.Delay(5, cancellationToken);
            }

            throw new TimeoutException($"Timeout waiting reply for id={expectedId} cmd frame.");
        }
        finally
        {
            _pendingReply = null;
            _pendingId = null;
            _txLock.Release();
        }
    }

    private void OnDataReceived(object sender, SerialDataReceivedEventArgs e)
    {
        DrainIncomingSerialBytes();
    }

    private void DrainIncomingSerialBytes()
    {
        if (_serialPort is null)
        {
            return;
        }

        int available = _serialPort.BytesToRead;
        if (available <= 0)
        {
            return;
        }

        byte[] chunk = new byte[available];
        int read = _serialPort.Read(chunk, 0, chunk.Length);
        if (read <= 0)
        {
            return;
        }

        lock (_rxBuffer)
        {
            for (int i = 0; i < read; i++)
            {
                _rxBuffer.Add(chunk[i]);
            }

            while (DoorControllerProtocol.TryParseFrame(_rxBuffer, out DoorControllerFrame? frame))
            {
                if (frame is null)
                {
                    continue;
                }

                TaskCompletionSource<DoorControllerFrame>? pending = _pendingReply;
                ushort? pendingId = _pendingId;
                if (pending is null || pendingId is null)
                {
                    continue;
                }

                if (frame.Command != DoorControllerCommands.CmdAck && frame.Command != DoorControllerCommands.CmdNak)
                {
                    continue;
                }

                if (frame.Id != pendingId.Value && frame.Id != 0)
                {
                    continue;
                }

                pending.TrySetResult(frame);
            }
        }
    }

    public void Dispose()
    {
        _txLock.Dispose();
        _serialPort?.Dispose();
    }
}
