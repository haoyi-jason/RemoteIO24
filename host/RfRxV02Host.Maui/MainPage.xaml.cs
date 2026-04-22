using RfRxV02Host.Core.Abstractions;
using RfRxV02Host.Core.Models;
using RfRxV02Host.Core.Services;
using System.Collections.ObjectModel;

namespace RfRxV02Host.Maui;

public partial class MainPage : ContentPage
{
    private readonly ITransportClient _transport;
    private readonly FastLineChartDrawable _trendDrawable = new(120);
    private readonly ObservableCollection<LiveDataOption> _liveDataOptions = new();
    private CancellationTokenSource? _trendCts;

    public MainPage(ITransportClient transport)
    {
        InitializeComponent();
        _transport = transport;
        TrendGraphics.Drawable = _trendDrawable;
        InitializeLiveDataPickers();
    }

    protected override async void OnAppearing()
    {
        base.OnAppearing();
        await RefreshEndpointsAsync();
    }

    private async Task RefreshEndpointsAsync()
    {
        var endpoints = (await _transport.GetAvailableEndpointsAsync()).ToList();
        EndpointPicker.ItemsSource = endpoints;
        EndpointPicker.ItemDisplayBinding = new Binding("DisplayName");
        if (endpoints.Count > 0)
        {
            EndpointPicker.SelectedIndex = 0;
        }
    }

    private async void OnRefreshClicked(object? sender, EventArgs e)
    {
        await RefreshEndpointsAsync();
    }

    private async void OnConnectClicked(object? sender, EventArgs e)
    {
        if (EndpointPicker.SelectedItem is null)
        {
            await DisplayAlert("連線", "請選擇連接埠", "OK");
            return;
        }

        if (!int.TryParse(BaudRateEntry.Text, out int baudRate))
        {
            await DisplayAlert("連線", "參數不合法", "OK");
            return;
        }

        ConnectionEndpoint endpoint = (ConnectionEndpoint)EndpointPicker.SelectedItem;
        await _transport.ConnectAsync(endpoint.Id, baudRate);
        ConnectionStatus.Text = $"已連線:{endpoint.DisplayName}";
        StartTrendPolling();
    }

    private async void OnDisconnectClicked(object? sender, EventArgs e)
    {
        StopTrendPolling();
        await _transport.DisconnectAsync();
        ConnectionStatus.Text = "已斷線";
    }

    private async void OnResetMcuClicked(object? sender, EventArgs e)
    {
        bool confirm = await DisplayAlert("重新開機!", "確定要重新開機?", "是", "否");
        if (!confirm)
        {
            return;
        }

        try
        {
            await _transport.ResetMcuAsync();
            ConnectionStatus.Text = "已送出指令";
        }
        catch (Exception ex)
        {
            await DisplayAlert("重新開機!", $"重新開機失敗:{ex.Message}", "OK");
        }
    }

    private async void OnOpenParametersClicked(object? sender, EventArgs e)
    {
        await Navigation.PushAsync(new ParametersPage(_transport));
    }

    private async void OnReadRssiClicked(object? sender, EventArgs e)
    {
        try
        {
            uint value = await _transport.ReadLiveAsync((ushort)LiveDataId.Rssi);
            LiveValueLabel.Text = $"訊號強度: {value}";
        }
        catch (Exception ex)
        {
            LiveValueLabel.Text = $"訊號強度讀取失敗: {ex.Message}";
        }
    }

    private async void OnReadBatClicked(object? sender, EventArgs e)
    {
        try
        {
            uint value = await _transport.ReadLiveAsync((ushort)LiveDataId.BatMv);
            LiveValueLabel.Text = $"電池電壓: {value}";
        }
        catch (Exception ex)
        {
            LiveValueLabel.Text = $"電池電壓讀取失敗: {ex.Message}";
        }
    }

    private void InitializeLiveDataPickers()
    {
        foreach (LiveDataId id in Enum.GetValues<LiveDataId>())
        {
            _liveDataOptions.Add(new LiveDataOption(id, id.ToString()));
        }

        TrendAPicker.ItemsSource = _liveDataOptions;
        TrendBPicker.ItemsSource = _liveDataOptions;
        TrendAPicker.ItemDisplayBinding = new Binding(nameof(LiveDataOption.DisplayName));
        TrendBPicker.ItemDisplayBinding = new Binding(nameof(LiveDataOption.DisplayName));

        TrendAPicker.SelectedItem = _liveDataOptions.FirstOrDefault(x => x.Id == LiveDataId.Rssi) ?? _liveDataOptions.FirstOrDefault();
        TrendBPicker.SelectedItem = _liveDataOptions.FirstOrDefault(x => x.Id == LiveDataId.BatMv) ?? _liveDataOptions.FirstOrDefault();
    }

    private void OnStartTrendClicked(object? sender, EventArgs e)
    {
        StartTrendPolling();
    }

    private void OnStopTrendClicked(object? sender, EventArgs e)
    {
        StopTrendPolling();
    }

    private void StartTrendPolling()
    {
        if (_trendCts is not null)
        {
            return;
        }

        _trendCts = new CancellationTokenSource();
        _ = PollLiveTrendAsync(_trendCts.Token);
    }

    private void StopTrendPolling()
    {
        _trendCts?.Cancel();
        _trendCts?.Dispose();
        _trendCts = null;
    }

    private async Task PollLiveTrendAsync(CancellationToken token)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromSeconds(1));

        while (await timer.WaitForNextTickAsync(token))
        {
            if (!_transport.IsOpen)
            {
                continue;
            }

            LiveDataOption? optionA = TrendAPicker.SelectedItem as LiveDataOption;
            LiveDataOption? optionB = TrendBPicker.SelectedItem as LiveDataOption;
            if (optionA is null || optionB is null)
            {
                continue;
            }

            try
            {
                uint valueA = await _transport.ReadLiveAsync((ushort)optionA.Id, 1000, token);
                uint valueB = await _transport.ReadLiveAsync((ushort)optionB.Id, 1000, token);

                _trendDrawable.AddPoints(valueA, valueB);
                TrendGraphics.Invalidate();

                TrendAValueLabel.Text = $"A {optionA.DisplayName}: {valueA}";
                TrendBValueLabel.Text = $"B {optionB.DisplayName}: {valueB}";
            }
            catch (OperationCanceledException)
            {
                return;
            }
            catch (Exception ex)
            {
                TrendAValueLabel.Text = $"Trend A error: {ex.Message}";
                TrendBValueLabel.Text = $"Trend B error: {ex.Message}";
            }
        }
    }

    protected override void OnDisappearing()
    {
        StopTrendPolling();
        base.OnDisappearing();
    }

    private sealed record LiveDataOption(LiveDataId Id, string DisplayName);
}
