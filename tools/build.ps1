# Configures, builds and tests the C++ core (and the GDExtension) inside an MSVC x64 environment.
# Usage: powershell -ExecutionPolicy Bypass -File tools/build.ps1 [-Config Release|Debug] [-NoTests] [-NoBindings]
param(
    [string]$Config = "Release",
    [switch]$NoTests,
    [switch]$NoBindings
)
$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot

$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsPath) { throw "MSVC build tools not found" }
$vcvars = Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat"

$bindings = if ($NoBindings) { "OFF" } else { "ON" }
$buildDir = Join-Path $repo "build"
$steps = @(
    "cmake -S `"$repo`" -B `"$buildDir`" -G Ninja -DCMAKE_BUILD_TYPE=$Config -DSTARSIM_BUILD_BINDINGS=$bindings",
    "cmake --build `"$buildDir`""
)
if (-not $NoTests) { $steps += "ctest --test-dir `"$buildDir`" --output-on-failure" }

cmd /c "`"$vcvars`" >nul && $($steps -join ' && ')"
if ($LASTEXITCODE -ne 0) { throw "build failed ($LASTEXITCODE)" }
