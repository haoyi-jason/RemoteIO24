using RfRxV02Host.Core.Abstractions;
using RfRxV02Host.Core.Models;
using RfRxV02Host.Core.Protocol;

#if ANDROID
using Android.Bluetooth;
using Java.Util;
using System.Diagnostics;
using System.IO;

namespace RfRxV02Host.Maui.Services;

public sealed class AndroidBluetoothTransport : ITransportClient
{
    private static readonly UUID SerialPortUuid = UUID.FromString("00001101-0000-1000-8000-00805F9B34FB");

    private BluetoothSocket? _socket;
    private Stream? _inputStream;
    private Stream? _outputStream;
    private CancellationTokenSource? _receiveLoopCts;
    private Task? _receiveLoopTask;
    private readonly List<byte> _rxBuffer = new();
    private readonly SemaphoreSlim _txLock = new(1, 1);
    private TaskCompletionSource<DoorControllerFrame>? _pendingReply;
    private ushort? _pendingId;

    public bool IsOpen => _socket?.IsConnected == true;

    public async Task<IReadOnlyList<ConnectionEndpoint>> GetAvailableEndpointsAsync(CancellationToken cancellationToken = default)
    {
        BluetoothAdapter adapter = await GetReadyAdapterAsync(cancellationToken);
        var bondedDevices = adapter.BondedDevices;
        if (bondedDevices is null || bondedDevices.Count == 0)
        {
            return Array.Empty<ConnectionEndpoint>();
        }

        return bondedDevices
            .Where(device => !string.IsNullOrWhiteSpace(device.Address))
            .OrderBy(device => device.Name ?? device.Address)
            .Select(device => new ConnectionEndpoint
            {
                Id = device.Address!,
                DisplayName = string.IsNullOrWhiteSpace(device.Name)
                    ? device.Address!
                    : $"{device.Name} ({device.Address})"
            })
            .ToList();
    }

    public async Task ConnectAsync(string endpoint, int baudRate, CancellationToken cancellationToken = default)
    {
        _ = baudRate;

        BluetoothAdapter adapter = await GetReadyAdapterAsync(cancellationToken);
        await DisconnectAsync(cancellationToken);

        BluetoothDevice device = adapter.GetRemoteDevice(endpoint);
        if (device is null)
        {
            throw new InvalidOperationException($"Bluetooth device not found: {endpoint}");
        }

        if (adapter.IsDiscovering)
        {
            adapter.CancelDiscovery();
        }

        BluetoothSocket socket = device.CreateRfcommSocketToServiceRecord(SerialPortUuid);
        await Task.Run(() => socket.Connect(), cancellationToken);

        _socket = socket;
        _inputStream = socket.InputStream;
        _outputStream = socket.OutputStream;
        _receiveLoopCts = new CancellationTokenSource();
        _receiveLoopTask = Task.Run(() => ReceiveLoopAsync(_receiveLoopCts.Token));
    }

    public async Task DisconnectAsync(CancellationToken cancellationToken = default)
    {
        _pendingReply?.TrySetCanceled(cancellationToken);
        _pendingReply = null;
        _pendingId = null;

        if (_receiveLoopCts is not null)
        {
            _receiveLoopCts.Cancel();
            _receiveLoopCts.Dispose();
            _receiveLoopCts = null;
        }

        if (_receiveLoopTask is not null)
        {
            try
            {
                await _receiveLoopTask;
            }
            catch (OperationCanceledException)
            {
            }
            catch (IOException)
            {
            }
            finally
            {
                _receiveLoopTask = null;
            }
        }

        _inputStream?.Dispose();
        _inputStream = null;
        _outputStream?.Dispose();
        _outputStream = null;

        _socket?.Close();
        _socket?.Dispose();
        _socket = null;
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
        byte[] request = DoorControllerProtocol.BuildCommandFrame(DoorControllerCommands.CmdResetMcu);
        await SendRawAsync(request, timeoutMs, cancellationToken);
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
        await _txLock.WaitAsync(cancellationToken);
        try
        {
            _pendingReply = new TaskCompletionSource<DoorControllerFrame>(TaskCreationOptions.RunContinuationsAsynchronously);
            _pendingId = expectedId;
            await WriteToSocketAsync(request, cancellationToken);

            Stopwatch sw = Stopwatch.StartNew();
            while (sw.ElapsedMilliseconds < timeoutMs)
            {
                cancellationToken.ThrowIfCancellationRequested();

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

    private async Task SendRawAsync(byte[] request, int timeoutMs, CancellationToken cancellationToken)
    {
        using var timeoutCts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        timeoutCts.CancelAfter(timeoutMs);

        await _txLock.WaitAsync(timeoutCts.Token);
        try
        {
            await WriteToSocketAsync(request, timeoutCts.Token);
        }
        finally
        {
            _txLock.Release();
        }
    }

    private async Task WriteToSocketAsync(byte[] request, CancellationToken cancellationToken)
    {
        if (_outputStream is null || !IsOpen)
        {
            throw new InvalidOperationException("Transport is not connected.");
        }

        await _outputStream.WriteAsync(request, cancellationToken);
        await _outputStream.FlushAsync(cancellationToken);
    }

    private async Task ReceiveLoopAsync(CancellationToken cancellationToken)
    {
        if (_inputStream is null)
        {
            return;
        }

        byte[] chunk = new byte[256];
        while (!cancellationToken.IsCancellationRequested)
        {
            int read = await _inputStream.ReadAsync(chunk.AsMemory(0, chunk.Length), cancellationToken);
            if (read <= 0)
            {
                break;
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
    }

    private static async Task<BluetoothAdapter> GetReadyAdapterAsync(CancellationToken cancellationToken)
    {
        bool granted = await MainActivity.EnsureBluetoothPermissionsAsync(cancellationToken);
        if (!granted)
        {
            throw new UnauthorizedAccessException("Bluetooth permission was denied.");
        }

        BluetoothAdapter? adapter = BluetoothAdapter.DefaultAdapter;
        if (adapter is null)
        {
            throw new PlatformNotSupportedException("Bluetooth adapter is not available on this device.");
        }

        if (!adapter.IsEnabled)
        {
            throw new InvalidOperationException("Bluetooth is turned off.");
        }

        return adapter;
    }

    public void Dispose()
    {
        DisconnectAsync().GetAwaiter().GetResult();
        _txLock.Dispose();
    }
}
#else
namespace RfRxV02Host.Maui.Services;

public sealed class AndroidBluetoothTransport : ITransportClient
{
    public bool IsOpen => false;

    public Task<IReadOnlyList<ConnectionEndpoint>> GetAvailableEndpointsAsync(CancellationToken cancellationToken = default)
        => Task.FromResult<IReadOnlyList<ConnectionEndpoint>>(Array.Empty<ConnectionEndpoint>());

    public Task ConnectAsync(string endpoint, int baudRate, CancellationToken cancellationToken = default)
        => throw new PlatformNotSupportedException();

    public Task DisconnectAsync(CancellationToken cancellationToken = default)
        => Task.CompletedTask;

    public Task<uint> ReadLiveAsync(ushort id, int timeoutMs = 1000, CancellationToken cancellationToken = default)
        => throw new PlatformNotSupportedException();

    public Task<uint> ReadParamAsync(ushort id, int timeoutMs = 1000, CancellationToken cancellationToken = default)
        => throw new PlatformNotSupportedException();

    public Task WriteParamAsync(ushort id, uint value, int timeoutMs = 1000, CancellationToken cancellationToken = default)
        => throw new PlatformNotSupportedException();

    public Task WriteLiveAsync(ushort id, uint value, int timeoutMs = 1000, CancellationToken cancellationToken = default)
        => throw new PlatformNotSupportedException();

    public Task ResetMcuAsync(int timeoutMs = 1000, CancellationToken cancellationToken = default)
        => throw new PlatformNotSupportedException();

    public void Dispose()
    {
    }
}
#endif
