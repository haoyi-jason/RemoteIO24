using Android;
using Android.App;
using Android.Content.PM;
using Android.OS;
using AndroidX.Core.App;
using AndroidX.Core.Content;
using Microsoft.Maui.ApplicationModel;

namespace RfRxV02Host.Maui;

[Activity(
    Theme = "@style/Maui.SplashTheme",
    MainLauncher = true,
    LaunchMode = LaunchMode.SingleTop,
    ConfigurationChanges = ConfigChanges.ScreenSize
        | ConfigChanges.Orientation
        | ConfigChanges.UiMode
        | ConfigChanges.ScreenLayout
        | ConfigChanges.SmallestScreenSize
        | ConfigChanges.Density)]
public class MainActivity : MauiAppCompatActivity
{
    private const int BluetoothPermissionRequestCode = 4101;
    private static readonly string[] BluetoothPermissions = BuildBluetoothPermissions();
    private static TaskCompletionSource<bool>? _permissionRequest;

    internal static async Task<bool> EnsureBluetoothPermissionsAsync(CancellationToken cancellationToken = default)
    {
        if (Build.VERSION.SdkInt < BuildVersionCodes.S)
        {
            return true;
        }

        Activity? activity = Platform.CurrentActivity;
        if (activity is null)
        {
            return false;
        }

        if (BluetoothPermissions.All(permission => ContextCompat.CheckSelfPermission(activity, permission) == Permission.Granted))
        {
            return true;
        }

        TaskCompletionSource<bool> request = _permissionRequest ??= new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        if (!request.Task.IsCompleted)
        {
            await MainThread.InvokeOnMainThreadAsync(() =>
            {
                ActivityCompat.RequestPermissions(activity, BluetoothPermissions, BluetoothPermissionRequestCode);
            });
        }

        using var registration = cancellationToken.Register(() => request.TrySetCanceled(cancellationToken));
        bool granted = await request.Task;
        _permissionRequest = null;
        return granted;
    }

    public override void OnRequestPermissionsResult(int requestCode, string[] permissions, Permission[] grantResults)
    {
        base.OnRequestPermissionsResult(requestCode, permissions, grantResults);

        if (requestCode != BluetoothPermissionRequestCode || _permissionRequest is null)
        {
            return;
        }

        bool granted = grantResults.Length > 0 && grantResults.All(result => result == Permission.Granted);
        _permissionRequest.TrySetResult(granted);
    }

    private static string[] BuildBluetoothPermissions()
    {
        if (Build.VERSION.SdkInt < BuildVersionCodes.S)
        {
            return Array.Empty<string>();
        }

        return new[]
        {
            Manifest.Permission.BluetoothConnect,
            Manifest.Permission.BluetoothScan
        };
    }
}
