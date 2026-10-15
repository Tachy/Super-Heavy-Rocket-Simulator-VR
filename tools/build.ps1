# Configures, builds and tests the C++ core (and the GDExtension) inside an MSVC x64 environment.
# Usage: powershell -ExecutionPolicy Bypass -File tools/build.ps1 [-Config Release|Debug] [-NoTests] [-NoBindings]
#            [-Fast] [-TestFilter "<Catch2 test spec>"] [-Jobs N]
#   (default)    full suite, test cases in parallel (ctest -j, one process per test case)
#   -Fast        skips the long campaigns (Catch tag [slow] -> CTest label "slow")
#   -TestFilter  runs only the matching test cases directly, e.g. "[cr3bp]" or "[targeter],[cr3bp]"
#   -Jobs        parallel test processes (default: number of logical processors)
param(
    [string]$Config = "Release",
    [switch]$NoTests,
    [switch]$NoBindings,
    [switch]$Fast,
    [string]$TestFilter = "",
    [int]$Jobs = [Environment]::ProcessorCount
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
if (-not $NoTests) {
    if ($TestFilter) {
        $tests = Join-Path $buildDir "core\tests\starsim_core_tests.exe"
        $steps += "`"$tests`" `"$TestFilter`""
    } else {
        $ctest = "ctest --test-dir `"$buildDir`" --output-on-failure -j $Jobs"
        if ($Fast) { $ctest += " -LE slow" }
        $steps += $ctest
    }
}

cmd /c "`"$vcvars`" >nul && $($steps -join ' && ')"
if ($LASTEXITCODE -ne 0) { throw "build failed ($LASTEXITCODE)" }
