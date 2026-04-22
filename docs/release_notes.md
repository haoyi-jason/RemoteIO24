# RFConfiguration Release Notes

## Release Information
- Product Name: RFConfiguration
- Application ID: com.remoteio.rfrxv02host
- Version: 0.1.0 (ApplicationVersion: 1)
- Frameworks: net9.0-windows10.0.19041.0, net9.0-android

## Artifacts

### Windows Installer (MSIX)
- Main package:
  - host/RfRxV02Host.Maui/bin/Release/net9.0-windows10.0.19041.0/win10-x64/AppPackages/RFConfiguration_Test/RFConfiguration.msix
- Dependencies folder:
  - host/RfRxV02Host.Maui/bin/Release/net9.0-windows10.0.19041.0/win10-x64/AppPackages/RFConfiguration_Test/Dependencies

### Android Installer (APK)
- Signed APK (recommended):
  - host/RfRxV02Host.Maui/bin/Release/net9.0-android/publish/com.remoteio.rfrxv02host-Signed.apk
- Unsiged/standard APK:
  - host/RfRxV02Host.Maui/bin/Release/net9.0-android/publish/com.remoteio.rfrxv02host.apk

## Windows Install Steps
1. Ensure certificate trust is configured for CN=RemoteIO.
2. Install RFConfiguration.msix.
3. If dependency prompts appear, install runtime packages from Dependencies folder.

## Android Install Steps
1. Uninstall old app version if launcher icon is not visible.
2. Install com.remoteio.rfrxv02host-Signed.apk.
3. Open app from app drawer; add launcher shortcut manually if your launcher does not auto-create home icon.

## Notes
- Windows icon and package display name have been updated to RFConfiguration.
- Existing legacy package output folders may still remain from older builds; prefer RFConfiguration_Test path.
