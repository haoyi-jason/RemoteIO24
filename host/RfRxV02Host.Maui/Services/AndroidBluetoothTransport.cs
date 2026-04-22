using RfRxV02Host.Core.Abstractions;
using RfRxV02Host.Core.Models;

namespace RfRxV02Host.Maui.Services;

public sealed class AndroidBluetoothTransport : ITransportClient
{
    public bool IsOpen => false;

    public Task<IReadOnlyList<ConnectionEndpoint>> GetAvailableEndpointsAsync(CancellationToken cancellationToken = default)
        => Task.FromResult<IReadOnlyList<ConnectionEndpoint>>(Array.Empty<ConnectionEndpoint>());

    public Task ConnectAsync(string endpoint, int baudRate, CancellationToken cancellationToken = default)
        => throw new NotImplementedException("Android Bluetooth transport is not implemented yet.");

    public Task DisconnectAsync(CancellationToken cancellationToken = default)
        => Task.CompletedTask;

    public Task<uint> ReadLiveAsync(ushort id, int timeoutMs = 1000, CancellationToken cancellationToken = default)
        => throw new NotImplementedException();

    public Task<uint> ReadParamAsync(ushort id, int timeoutMs = 1000, CancellationToken cancellationToken = default)
        => throw new NotImplementedException();

    public Task WriteParamAsync(ushort id, uint value, int timeoutMs = 1000, CancellationToken cancellationToken = default)
        => throw new NotImplementedException();

    public Task WriteLiveAsync(ushort id, uint value, int timeoutMs = 1000, CancellationToken cancellationToken = default)
        => throw new NotImplementedException();

    public Task ResetMcuAsync(int timeoutMs = 1000, CancellationToken cancellationToken = default)
        => throw new NotImplementedException();

    public void Dispose()
    {
    }
}
