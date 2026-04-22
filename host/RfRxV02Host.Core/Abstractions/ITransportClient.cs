using RfRxV02Host.Core.Models;

namespace RfRxV02Host.Core.Abstractions;

public interface ITransportClient : IDisposable
{
    bool IsOpen { get; }

    Task<IReadOnlyList<ConnectionEndpoint>> GetAvailableEndpointsAsync(
        CancellationToken cancellationToken = default);

    Task ConnectAsync(string endpoint, int baudRate, CancellationToken cancellationToken = default);

    Task DisconnectAsync(CancellationToken cancellationToken = default);

    Task<uint> ReadLiveAsync(ushort id, int timeoutMs = 1000, CancellationToken cancellationToken = default);

    Task<uint> ReadParamAsync(ushort id, int timeoutMs = 1000, CancellationToken cancellationToken = default);

    Task WriteParamAsync(ushort id, uint value, int timeoutMs = 1000, CancellationToken cancellationToken = default);

    Task WriteLiveAsync(ushort id, uint value, int timeoutMs = 1000, CancellationToken cancellationToken = default);

    Task ResetMcuAsync(int timeoutMs = 1000, CancellationToken cancellationToken = default);
}
