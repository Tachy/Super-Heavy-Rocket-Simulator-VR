# Configures, builds and tests the C++ core (and the GDExtension) inside an MSVC x64 environment.
# Usage: powershell -ExecutionPolicy Bypass -File tools/build.ps1 [-Config Release|Debug] [-NoTests] [-NoBindings]
#            [-Fast] [-TestFilter "<Catch2 test spec>"] [-Jobs N]
#   (default)    full suite, test cases in parallel (ctest -j, one process per test case)
#   -Fast        skips the long campaigns (Catch tag [slow] -> CTest label "slow")
#   -TestFilter  a tag list ("[cr3bp]", "[targeter],[cr3bp]", "[a][b]") runs the UNION of its tags as parallel ctest
#                labels ("[a][b]" is a union here, unlike Catch's AND; [slow] included unless -Fast; output in
#                build/ctest_filter.log). Anything else (names, wildcards, ~) runs the test exe serially.
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
$filterLog = $null
if (-not $NoTests) {
    $tagListRe = '^(\[[A-Za-z0-9_.-]+\])+(,(\[[A-Za-z0-9_.-]+\])+)*$'
    if ($TestFilter -and $TestFilter -match $tagListRe) {
        $tags = [regex]::Matches($TestFilter, '\[([A-Za-z0-9_.-]+)\]') | ForEach-Object { [regex]::Escape($_.Groups[1].Value) }
        $filterLog = Join-Path $buildDir "ctest_filter.log"
        if (Test-Path $filterLog) { Remove-Item $filterLog -Force }  # no stale summary when the build fails
        $ctest = "ctest --test-dir `"$buildDir`" --output-on-failure -j $Jobs -L `"^($($tags -join '|'))$`""
        if ($Fast) { $ctest += " -LE slow" }
        $steps += "$ctest > `"$filterLog`" 2>&1"
    } elseif ($TestFilter) {
        Write-Host "TestFilter is not a tag list: running the test exe serially"
        $tests = Join-Path $buildDir "core\tests\starsim_core_tests.exe"
        $steps += "`"$tests`" `"$TestFilter`""
    } else {
        $ctest = "ctest --test-dir `"$buildDir`" --output-on-failure -j $Jobs"
        if ($Fast) { $ctest += " -LE slow" }
        $steps += $ctest
    }
}

cmd /c "`"$vcvars`" >nul && $($steps -join ' && ')"
$exitCode = $LASTEXITCODE
if ($filterLog -and (Test-Path $filterLog)) {
    $log = Get-Content $filterLog
    $log | Select-String -Pattern 'tests passed|tests failed|The following tests FAILED|\*\*\*Failed|\*\*\*Exception|\*\*\*Timeout|\*\*\*Not Run|No tests were found' | ForEach-Object { Write-Host $_.Line }
    Write-Host "(full ctest output: $filterLog)"
    if ($log -match 'No tests were found') { throw "TestFilter matched no test" }
}
if ($exitCode -ne 0) { throw "build failed ($exitCode)" }
