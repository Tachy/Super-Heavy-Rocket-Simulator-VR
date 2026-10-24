# Release build of the simulator (Windows):
#   1. the double-precision release export template (built once from external/godot if missing)
#   2. core + GDExtension (tools/build.ps1 -NoTests) and the Godot smoke test
#   3. the exe (PCK embedded) with the version stamped in, plus the GDExtension and D3D12 DLLs
#   4. the package folder: exe, launchers, config/, the base data, CREDITS.md, READMEs; the data parts terrain
#      (data/terrain/earth) and imagery (data/imagery); a self-check of the exported exe with and without the parts
#   5. ZIPs: build/SuperHeavyRocketSimulatorVR-<version>-windows.zip (playable, base data),
#      ...-data-terrain.zip, ...-data-imagery.zip (each unpacks into the game folder); none may reach 2 GB
#   6. build/SuperHeavyRocketSimulatorVR-<version>-setup.exe (Inno Setup, everything; installed once if missing)
#
#   powershell -ExecutionPolicy Bypass -File tools/package.ps1 [-Version 1.2.3] [-SkipTests] [-NoInstaller]
#
# /build (.claude/commands/build.md) publishes these files as the release, /build-local builds a test installer.
# The Godot editor comes from start.cfg (GODOT=...) or external/godot, as for the start scripts. -Version defaults to
# game/util/game_version.gd (release-please keeps it); a suffix like 1.2.3-local is allowed (the exe's file version
# takes the numbers only).

param(
    [string]$Version = "",
    [switch]$SkipTests,
    [switch]$NoInstaller
)

$ErrorActionPreference = "Stop"
$Root = Split-Path $PSScriptRoot -Parent
$Build = Join-Path $Root "build"
$Game = Join-Path $Root "game"
$Data = Join-Path $Root "data"
$ExeName = "SuperHeavyRocketSimulatorVR.exe"
$Prefix = "SuperHeavyRocketSimulatorVR"
$GodotBin = Join-Path $Root "external\godot\bin"
$Template = Join-Path $GodotBin "godot.windows.template_release.double.x86_64.exe"
$AssetLimit = 2GB  # GitHub's limit per release asset

function Step($text) { Write-Host ""; Write-Host "== $text" -ForegroundColor Cyan }

# A native program's output (stdout and stderr) as lines. Godot writes warnings to stderr, which PowerShell 5 would
# turn into errors with ErrorActionPreference Stop: success is judged by the result instead.
function Run([string]$exe, [string[]]$arguments) {
    $old = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        $lines = & $exe @arguments 2>&1 | ForEach-Object { "$_" }
    } finally {
        $ErrorActionPreference = $old
    }
    return $lines
}

# robocopy with its exit codes (0-7 success, 8+ failure).
function Mirror([string]$from, [string]$to, [string[]]$extra) {
    $null = & robocopy $from $to /E /NFL /NDL /NJH /NJS /NP @extra
    if ($LASTEXITCODE -ge 8) { throw "robocopy $from -> $to failed ($LASTEXITCODE)" }
    $global:LASTEXITCODE = 0
}

# Runs the exported exe headless with --self-check and returns its SELF-CHECK line; throws on FAIL or no line.
function SelfCheck([string]$exe) {
    $out = Run $exe @("--headless", "--xr-mode", "off", "--", "--no-xr", "--self-check")
    $line = $out | Where-Object { $_ -match '^SELF-CHECK' } | Select-Object -Last 1
    if (-not $line) { $out | Select-Object -Last 20 | ForEach-Object { Write-Host "   $_" }; throw "Self-check printed nothing: $exe" }
    Write-Host "   $line"
    if ($line -notmatch '^SELF-CHECK OK') { throw "Self-check failed: $exe" }
    return $line
}

function Zip([string]$dir, [string]$zip) {
    if (Test-Path $zip) { Remove-Item $zip -Force }
    $entries = Get-ChildItem $dir | ForEach-Object { $_.Name }
    $null = Run "tar.exe" (@("-a", "-c", "-f", $zip, "-C", $dir) + $entries)
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path $zip)) { throw "tar failed for $zip" }
    $size = (Get-Item $zip).Length
    Write-Host ("zip: {0} ({1:N1} MB)" -f $zip, ($size / 1MB))
    if ($size -ge $AssetLimit) { throw "$zip is $([math]::Round($size / 1GB, 2)) GB: GitHub release assets must stay below 2 GB" }
}

# --- Godot -------------------------------------------------------------------------------------------------------
$Godot = Join-Path $GodotBin "godot.windows.editor.double.x86_64.exe"
$cfg = Join-Path $Root "start.cfg"
if (Test-Path $cfg) {
    $line = Get-Content $cfg | Where-Object { $_ -match '^\s*GODOT\s*=' } | Select-Object -First 1
    if ($line) { $Godot = $line -replace '^\s*GODOT\s*=\s*', '' }
}
if (-not (Test-Path $Godot)) { throw "Godot double-precision editor not found: '$Godot' (build it as in CLAUDE.md or set GODOT in start.cfg)" }
$console = $Godot -replace '\.exe$', '.console.exe'
if (Test-Path $console) { $Godot = $console }
Write-Host "Godot $((Run $Godot @('--version') | Select-Object -First 1).Trim())"

# --- version -----------------------------------------------------------------------------------------------------
if (-not $Version) {
    $line = Get-Content (Join-Path $Game "util\game_version.gd") | Where-Object { $_ -match 'const VERSION' }
    if ($line -notmatch '"([^"]+)"') { throw "No version in game/util/game_version.gd" }
    $Version = $Matches[1]
}
if ($Version -notmatch '^(\d+)\.(\d+)\.(\d+)') { throw "Version must start with x.y.z: $Version" }
$fileVersion = "$($Matches[1]).$($Matches[2]).$($Matches[3]).0"
Write-Host "Version $Version (file version $fileVersion)"

if (-not (Test-Path (Join-Path $Data "spice"))) { throw "data/ is missing: run python tools/data_pipeline/fetch.py first" }
foreach ($part in @("terrain\earth", "imagery\earth\index.toml")) {
    if (-not (Test-Path (Join-Path $Data $part))) { throw "data/$part is missing: run python tools/data_pipeline/fetch.py first" }
}

# --- 1. export template ------------------------------------------------------------------------------------------
Step "Export template"
if (Test-Path $Template) {
    Write-Host "present: $Template"
} else {
    New-Item -ItemType Directory -Force -Path $Build | Out-Null
    $log = Join-Path $Build "template_build.log"
    Write-Host "building it once (scons, ~5 min, log $log) ..."
    Push-Location (Join-Path $Root "external\godot")
    try {
        $null = Run "python" @("-m", "SCons", "platform=windows", "target=template_release", "precision=double", "production=yes") | Set-Content $log
    } finally {
        Pop-Location
    }
    if (-not (Test-Path $Template)) { Get-Content $log -Tail 20 | ForEach-Object { Write-Host $_ }; throw "Template build failed (see $log)" }
}
# The D3D12 Agility SDK runtime the export copies (into <exe dir>\x86_64): it looks for the arch-named file next to the
# template, the engine build (install_d3d12_sdk_windows.py) leaves it unsuffixed.
$agility = Join-Path $GodotBin "D3D12Core.dll"
if (-not (Test-Path $agility)) { throw "D3D12Core.dll missing in $GodotBin (run misc/scripts/install_d3d12_sdk_windows.py and rebuild)" }
Copy-Item $agility (Join-Path $GodotBin "D3D12Core.x86_64.dll") -Force

# --- 2. core, extension, smoke test ------------------------------------------------------------------------------
Step "Core + GDExtension"
& powershell -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "build.ps1") -NoTests
if ($LASTEXITCODE -ne 0) { throw "Core build failed." }
# The atmosphere tables (M8, ~84 MB, ~1 min): precomputed into data/atmosphere, which the smoke test uses and the base
# data mirror below ships. The game finds the file by its input hash (a stale file would be recomputed at runtime).
Step "Atmosphere tables"
$null = Run (Join-Path $Build "core\atmosphere_tables.exe") @("--config", (Join-Path $Root "config"), "--out", (Join-Path $Data "atmosphere"))
if ($LASTEXITCODE -ne 0) { throw "atmosphere_tables failed." }
$null = Run $Godot @("--headless", "--xr-mode", "off", "--path", $Game, "--import")
if (-not $SkipTests) {
    Step "Smoke test"
    $out = Run $Godot @("--headless", "--path", $Game, "--script", "res://tests/smoke_test.gd")
    if (-not ($out -match 'SMOKE TEST PASSED')) { $out | Select-Object -Last 20 | ForEach-Object { Write-Host $_ }; throw "Smoke test failed - no build." }
    Write-Host "SMOKE TEST PASSED"
}

# --- 3. export ---------------------------------------------------------------------------------------------------
Step "Export"
$export = Join-Path $Build "export"
if (Test-Path $export) { Remove-Item $export -Recurse -Force }
New-Item -ItemType Directory -Force -Path $export | Out-Null
$presets = Join-Path $Game "export_presets.cfg"
$presetsText = Get-Content $presets -Raw
try {
    # version and template into the preset (for this export only)
    $templatePath = $Template -replace '\\', '/'
    $stamped = $presetsText -replace 'application/file_version="[^"]*"', "application/file_version=`"$fileVersion`"" `
                            -replace 'application/product_version="[^"]*"', "application/product_version=`"$fileVersion`"" `
                            -replace 'custom_template/release="[^"]*"', "custom_template/release=`"$templatePath`""
    [IO.File]::WriteAllText($presets, $stamped)
    $exe = Join-Path $export $ExeName
    Run $Godot @("--headless", "--path", $Game, "--export-release", "Windows Desktop", $exe) | Where-Object { $_ -match 'ERROR' } | ForEach-Object { Write-Host $_ }
    if (-not (Test-Path $exe)) { throw "Export failed: no $exe" }
} finally {
    [IO.File]::WriteAllText($presets, $presetsText)
}
Get-ChildItem $export -File | ForEach-Object { Write-Host ("  {0} ({1:N1} MB)" -f $_.Name, ($_.Length / 1MB)) }

# --- 4. package folder and data parts ----------------------------------------------------------------------------
Step "Package"
$package = Join-Path $Build "package"
$terrainPart = Join-Path $Build "part_terrain"
$imageryPart = Join-Path $Build "part_imagery"
foreach ($dir in @($package, $terrainPart, $imageryPart)) { if (Test-Path $dir) { Remove-Item $dir -Recurse -Force } }
Mirror $export $package @()
Copy-Item (Join-Path $PSScriptRoot "release\*.cmd") $package
Copy-Item (Join-Path $Root "CREDITS.md"), (Join-Path $Root "README.md"), (Join-Path $Root "README.de.md") $package
Mirror (Join-Path $Root "config") (Join-Path $package "config") @()
# base data: everything the game reads except the two large parts; never the raw downloads, webcasts or validation
Mirror $Data (Join-Path $package "data") @("/XD", (Join-Path $Data "raw"), (Join-Path $Data "webcasts"),
    (Join-Path $Data "validation"), (Join-Path $Data "imagery"), (Join-Path $Data "terrain\earth"), "/XF", "README.md")
if (-not (Test-Path (Join-Path $package "data\atmosphere\earth_atmosphere.bin"))) { throw "The package lacks data/atmosphere/earth_atmosphere.bin" }
Mirror (Join-Path $Data "terrain\earth") (Join-Path $terrainPart "data\terrain\earth") @()
Mirror (Join-Path $Data "imagery") (Join-Path $imageryPart "data\imagery") @()

if (-not $SkipTests) {
    Step "Self-check of the exported build"
    $line = SelfCheck (Join-Path $package $ExeName)
    if ($line -notmatch 'earth_terrain=no imagery=no') { throw "Base package should run without the terrain and imagery parts: $line" }
    # the full installation, in a folder with a space as under %LOCALAPPDATA%\Programs
    $full = Join-Path $Build "package full"
    if (Test-Path $full) { Remove-Item $full -Recurse -Force }
    Mirror $package $full @()
    Mirror $terrainPart $full @()
    Mirror $imageryPart $full @()
    $line = SelfCheck (Join-Path $full $ExeName)
    if ($line -notmatch 'earth_terrain=yes imagery=yes') { throw "Full package should find the terrain and imagery: $line" }
    Remove-Item $full -Recurse -Force
}

# --- 5. ZIPs -----------------------------------------------------------------------------------------------------
Step "ZIPs"
Zip $package (Join-Path $Build "$Prefix-$Version-windows.zip")
Zip $terrainPart (Join-Path $Build "$Prefix-$Version-data-terrain.zip")
Zip $imageryPart (Join-Path $Build "$Prefix-$Version-data-imagery.zip")

# --- 6. installer ------------------------------------------------------------------------------------------------
if (-not $NoInstaller) {
    Step "Installer"
    $isccPaths = @(
        (Join-Path ${env:ProgramFiles(x86)} "Inno Setup 6\ISCC.exe"),
        (Join-Path $env:ProgramFiles "Inno Setup 6\ISCC.exe"),
        (Join-Path $env:LOCALAPPDATA "Programs\Inno Setup 6\ISCC.exe")
    )
    $iscc = $isccPaths | Where-Object { Test-Path $_ } | Select-Object -First 1
    if (-not $iscc) {
        Write-Host "Inno Setup not found: installing it once (winget) ..."
        winget install --id JRSoftware.InnoSetup -e --silent --scope user --accept-package-agreements --accept-source-agreements | Out-Host
        if ($LASTEXITCODE -ne 0) {
            winget install --id JRSoftware.InnoSetup -e --silent --accept-package-agreements --accept-source-agreements | Out-Host
        }
        $iscc = $isccPaths | Where-Object { Test-Path $_ } | Select-Object -First 1
        if (-not $iscc) { throw "Inno Setup could not be installed (ISCC.exe not found)." }
    }
    $isccOut = Run $iscc @("/DAppVersion=$Version", "/DFileVersion=$fileVersion", "/DSourceDir=$package",
        "/DTerrainDir=$terrainPart", "/DImageryDir=$imageryPart", "/DOutputDir=$Build", (Join-Path $PSScriptRoot "installer.iss"))
    if ($LASTEXITCODE -ne 0) { $isccOut | Select-Object -Last 15 | ForEach-Object { Write-Host $_ }; throw "Inno Setup failed ($LASTEXITCODE)." }
    $setup = Join-Path $Build "$Prefix-$Version-setup.exe"
    $size = (Get-Item $setup).Length
    Write-Host ("installer: {0} ({1:N1} MB)" -f $setup, ($size / 1MB))
    if ($size -ge $AssetLimit) { throw "$setup is $([math]::Round($size / 1GB, 2)) GB: GitHub release assets must stay below 2 GB" }
}

Step "Done: $Version"
Get-ChildItem $Build -File | Where-Object { $_.Name -like "$Prefix-$Version-*" } | ForEach-Object { Write-Host ("  {0}  ({1:N1} MB)" -f $_.FullName, ($_.Length / 1MB)) }
