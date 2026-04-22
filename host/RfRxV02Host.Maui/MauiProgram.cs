using Microsoft.Extensions.Logging;
using RfRxV02Host.Core.Abstractions;
using RfRxV02Host.Maui.Services;

namespace RfRxV02Host.Maui;

public static class MauiProgram
{
    public static MauiApp CreateMauiApp()
    {
        var builder = MauiApp.CreateBuilder();
        builder
            .UseMauiApp<App>()
            .ConfigureFonts(fonts =>
            {
                fonts.AddFont("OpenSans-Regular.ttf", "OpenSansRegular");
                fonts.AddFont("OpenSans-Semibold.ttf", "OpenSansSemibold");
            });

#if ANDROID
        builder.Services.AddSingleton<ITransportClient, AndroidBluetoothTransport>();
#else
        builder.Services.AddSingleton<ITransportClient, SerialTransportClient>();
#endif

        builder.Services.AddSingleton<MainPage>();
        builder.Services.AddSingleton<ParametersPage>();

        return builder.Build();
    }
}
