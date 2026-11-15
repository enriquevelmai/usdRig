# Sign every PE file in a usdRig bundle with Authenticode.
#
#   pwsh packaging/sign-windows.ps1 -Bundle dist/usdRig-1.2.0-windows-x64 `
#        -Thumbprint <cert-store-thumbprint>
#
# The certificate must already be in the current user's certificate store
# (the release workflow imports it from the WINDOWS_SIGNING_CERT_BASE64 /
# WINDOWS_SIGNING_CERT_PASSWORD secrets). Without -Thumbprint the script
# warns and exits 0, so unsigned workflow runs stay green and visibly
# unsigned. Files that already carry a valid signature (vendor Python /
# Qt binaries) are left untouched; everything built in CI (RigExec, the
# OpenUSD build, the launcher) is unsigned on arrival and gets signed.
param(
    [Parameter(Mandatory = $true)][string]$Bundle,
    [string]$Thumbprint = $env:WINDOWS_SIGNING_THUMBPRINT,
    [string]$TimestampUrl = "http://timestamp.digicert.com"
)

$ErrorActionPreference = "Stop"

if (-not $Thumbprint) {
    Write-Warning "WINDOWS_SIGNING_THUMBPRINT not set: bundle stays UNSIGNED."
    exit 0
}
if (-not (Test-Path $Bundle)) { throw "bundle not found: $Bundle" }

$signtool = (Get-Command signtool.exe -ErrorAction SilentlyContinue).Source
if (-not $signtool) {
    $kits = "${env:ProgramFiles(x86)}/Windows Kits/10/bin"
    $signtool = Get-ChildItem $kits -Recurse -Filter signtool.exe |
        Where-Object { $_.FullName -match "x64" } |
        Sort-Object FullName | Select-Object -Last 1 -ExpandProperty FullName
}
if (-not $signtool) { throw "signtool.exe not found (install the Windows SDK)" }

$files = Get-ChildItem $Bundle -Recurse -File |
    Where-Object { $_.Extension -in ".exe", ".dll", ".pyd" }
$signed = 0; $skipped = 0
foreach ($file in $files) {
    $sig = Get-AuthenticodeSignature $file.FullName
    if ($sig.Status -eq "Valid") { $skipped++; continue }
    & $signtool sign /sha1 $Thumbprint /fd SHA256 /td SHA256 `
        /tr $TimestampUrl $file.FullName | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "signtool failed: $($file.FullName)" }
    $signed++
}
Write-Host "signed $signed file(s), skipped $skipped already-signed file(s)"

foreach ($file in $files) {
    & $signtool verify /pa /q $file.FullName | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "signature does not verify: $($file.FullName)" }
}
Write-Host "all bundle signatures verify"
