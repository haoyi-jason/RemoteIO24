param(
    [string]$Configuration = "Release",
    [string]$RuntimeIdentifier = "",
    [string]$CertThumbprint = "B8D8D01EE5D106F76CD12FA269BDAD4EE1B14B5A",
    [switch]$Clean
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$project = Join-Path $scriptDir "RfRxV02Host.Maui\RfRxV02Host.Maui.csproj"

if (!(Test-Path $project)) {
    throw "Project not found: $project"
}

$cert = Get-Item "Cert:\CurrentUser\My\$CertThumbprint" -ErrorAction SilentlyContinue
if ($null -eq $cert) {
    throw "Signing certificate not found in Cert:\CurrentUser\My. Thumbprint=$CertThumbprint"
}
if (-not $cert.HasPrivateKey) {
    throw "Certificate exists but has no private key: $CertThumbprint"
}
if ($cert.NotAfter -lt (Get-Date)) {
    throw "Certificate expired at $($cert.NotAfter): $CertThumbprint"
}

if ($Clean) {
    dotnet clean $project -c $Configuration
    if ($LASTEXITCODE -ne 0) {
        throw "dotnet clean failed with exit code $LASTEXITCODE"
    }
}

# Build signed MSIX using fixed certificate thumbprint from CurrentUser\My.
$publishArgs = @(
    "publish", $project,
    "-f", "net9.0-windows10.0.19041.0",
    "-c", $Configuration,
    "-p:WindowsPackageType=MSIX",
    "-p:GenerateAppxPackageOnBuild=true",
    "-p:AppxPackageSigningEnabled=true",
    "-p:PackageCertificateThumbprint=$CertThumbprint"
)

if (-not [string]::IsNullOrWhiteSpace($RuntimeIdentifier)) {
    $publishArgs += @("-r", $RuntimeIdentifier)
}

dotnet @publishArgs

if ($LASTEXITCODE -ne 0) {
        throw "dotnet publish failed with exit code $LASTEXITCODE"
}

$pkgBase = Join-Path $scriptDir "RfRxV02Host.Maui\bin\$Configuration"
$pkgRoots = @()
if (Test-Path $pkgBase) {
    $pkgRoots = Get-ChildItem $pkgBase -Directory -Recurse -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -eq "AppPackages" } |
        Select-Object -ExpandProperty FullName
}

$found = $false
foreach ($pkgRoot in $pkgRoots) {
    if (Test-Path $pkgRoot) {
        $found = $true
        Write-Host "Package output: $pkgRoot"
        Get-ChildItem $pkgRoot -Recurse -File | Where-Object { $_.Extension -in ".msix", ".msixbundle", ".cer", ".ps1" } |
            Select-Object FullName, Length, LastWriteTime |
            Format-Table -AutoSize
    }
}

if (-not $found) {
    Write-Warning "AppPackages folder not found. Check publish output for errors."
}
