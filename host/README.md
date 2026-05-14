# RF_RX_V02 Host (MAUI + Core)

This folder contains a new host-side application scaffold for RF_RX_V02 based on the architecture in docs/maui_protocol_architecture.md.

## Structure

- RfRxV02Host.Core
  - Abstractions/ITransportClient.cs
  - Protocol/DoorControllerProtocol.cs
  - Protocol/DoorControllerFrame.cs
  - Protocol/DoorControllerCommands.cs
  - Services/DfParameterCatalog.cs
  - Services/LiveDataId.cs
- RfRxV02Host.Maui
  - MainPage.xaml(.cs): connect endpoint and read live values
  - ParametersPage.xaml(.cs): DF read/write and read-all
  - Services/SerialTransportClient.cs (Windows serial)
  - Services/AndroidBluetoothTransport.cs (stub)

## Current status

- Host and firmware are aligned to docs/maui_protocol_architecture.md frame format:
  - STX | LEN | CMD | DATA | CRC | ETX
  - CMD: 0x01/0x02/0x03/0x04
  - ACK: 0xF0
  - NAK: 0xF1
- UI has:
  - endpoint scan
  - connect/disconnect
  - live read sample
  - DF parameter page with read/write/read-all

## Important compatibility note

Firmware parser in source/Common/binaryProtocolTask.c has been rewritten to parse host format directly.

Please review FIRMWARE_COMM_CHANGE_PROPOSAL.md before changing firmware communication behavior.

## Next steps

1. Confirm firmware migration approach (new command set vs dual protocol support).
2. Complete full DF/LD ID table from firmware headers to host catalog.
3. Implement Android Bluetooth transport if Android host is required.

## Windows package signing and one-click build

This repository now supports fixed certificate signing for Windows MSIX packaging.

- Signing certificate subject: `CN=RemoteIO`
- Default certificate thumbprint: `B8D8D01EE5D106F76CD12FA269BDAD4EE1B14B5A`
- Public certificate for installation on target PCs: `host/certs/RemoteIO.cer`

### Prerequisites

1. The signing certificate private key must exist in `Cert:\CurrentUser\My` on the build machine.
2. .NET SDK (matching project target) must be installed.

### Build signed MSIX

Run from `host/`:

```powershell
.\package-windows.ps1
```

Optional parameters:

```powershell
.\package-windows.ps1 -Configuration Release -CertThumbprint <thumbprint> -Clean
```

The script validates certificate presence/private key/expiry and then runs `dotnet publish` with:

- `WindowsPackageType=MSIX`
- `GenerateAppxPackageOnBuild=true`
- `AppxPackageSigningEnabled=true`
- `PackageCertificateThumbprint=<thumbprint>`
