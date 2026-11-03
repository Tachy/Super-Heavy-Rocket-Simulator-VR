# Configures, builds and tests the C++ core (and the GDExtension) inside an MSVC x64 environment.
# Usage: powershell -ExecutionPolicy Bypass -File tools/build.ps1 [-Config Release|Debug] [-NoTests] [-NoBindings]
#            [-Fast] [-TestFilter "<spec>"] [-Nightly] [-ListOnly] [-Jobs N]
#   (default)    all tests except the label "nightly", test cases in parallel (ctest -j, one process per test case)
#   -Fast        skips the long campaigns (Catch tag [slow] -> CTest label "slow")
#   -TestFilter  a tag spec with Catch semantics: "," = union of groups, "[a][b]" inside a group = intersection
#                ("[a],[b]", "[a][b]", "[a][b],[c]"). [slow] cases are included unless -Fast. Output in
#                build/ctest_filter.log. Anything else (names, wildcards, ~) runs the test exe serially.
#   -Nightly     the label "nightly" (long-running nightly tier) is excluded unless this is set. Alone it runs only the
#                nightly cases; together with -TestFilter it adds the nightly cases of the filtered tags.
#   -ListOnly    prints the ctest command and the number of matched cases (ctest -N); no build, nothing is run
#   -Jobs        parallel test processes (default: number of logical processors)
param(
    [string]$Config = "Release",
    [switch]$NoTests,
    [switch]$NoBindings,
    [switch]$Fast,
    [string]$TestFilter = "",
    [switch]$Nightly,
    [switch]$ListOnly,
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
$tagGroupRe = '^(\[[A-Za-z0-9_.-]+\])+(,(\[[A-Za-z0-9_.-]+\])+)*$'
$isTagSpec = $TestFilter -and ($TestFilter -match $tagGroupRe)

# Runs command lines inside the MSVC environment (ctest comes from there too).
function Invoke-Vc([string[]]$Commands) {
    cmd /c "`"$vcvars`" >nul && $($Commands -join ' && ')" | Out-Host
    return [int]$LASTEXITCODE
}

# Selects the test cases by label: returns @{ Names; Total }.
function Select-Tests {
    $json = Join-Path $buildDir "ctest_list.json"
    $code = Invoke-Vc @("ctest --test-dir `"$buildDir`" -N --show-only=json-v1 > `"$json`"")
    if ($code -ne 0) { throw "ctest -N failed ($code); configure the build first" }
    $all = (Get-Content $json -Raw | ConvertFrom-Json).tests
    $groups = @()
    if ($isTagSpec) {
        foreach ($g in $TestFilter.Split(',')) {
            $groups += ,@([regex]::Matches($g, '\[([A-Za-z0-9_.-]+)\]') | ForEach-Object { $_.Groups[1].Value })
        }
    }
    $names = New-Object System.Collections.Generic.List[string]
    foreach ($t in $all) {
        $labels = @()
        foreach ($p in $t.properties) { if ($p.name -eq "LABELS") { $labels = @($p.value) } }
        $isNightly = $labels -contains "nightly"
        if ($isNightly -and -not $Nightly) { continue }
        if ($Fast -and ($labels -contains "slow")) { continue }
        if ($isTagSpec) {
            $hit = $false
            foreach ($g in $groups) {
                $all_in = $true
                foreach ($tag in $g) { if ($labels -notcontains $tag) { $all_in = $false; break } }
                if ($all_in) { $hit = $true; break }
            }
            if (-not $hit) { continue }
        } elseif ($Nightly -and -not $TestFilter -and -not $isNightly) {
            continue  # -Nightly alone: only the nightly tier
        }
        $names.Add($t.name)
    }
    return @{ Names = $names; Total = @($all).Count }
}

if ($TestFilter -and -not $isTagSpec) {
    if ($ListOnly) { Write-Host "TestFilter is not a tag spec: would run the test exe serially with `"$TestFilter`""; return }
    Write-Host "TestFilter is not a tag spec: running the test exe serially"
}

if ($ListOnly) {
    $sel = Select-Tests
    if ($sel.Names.Count -eq 0) { throw "TestFilter matched no test" }
    $listFile = Join-Path $buildDir "ctest_filter_tests.txt"
    Write-Host "ctest --test-dir `"$buildDir`" --output-on-failure -j $Jobs --tests-from-file `"$listFile`""
    Write-Host "matched: $($sel.Names.Count) of $($sel.Total)"
    return
}

$steps = @(
    "cmake -S `"$repo`" -B `"$buildDir`" -G Ninja -DCMAKE_BUILD_TYPE=$Config -DSTARSIM_BUILD_BINDINGS=$bindings",
    "cmake --build `"$buildDir`""
)
$code = Invoke-Vc $steps
if ($code -ne 0) { throw "build failed ($code)" }
if ($NoTests) { return }

if ($TestFilter -and -not $isTagSpec) {
    $tests = Join-Path $buildDir "core\tests\starsim_core_tests.exe"
    $code = Invoke-Vc @("`"$tests`" `"$TestFilter`"")
    if ($code -ne 0) { throw "build failed ($code)" }
    return
}

$sel = Select-Tests
if ($sel.Names.Count -eq 0) { throw "TestFilter matched no test" }
$listFile = Join-Path $buildDir "ctest_filter_tests.txt"
[System.IO.File]::WriteAllLines($listFile, [string[]]$sel.Names, (New-Object System.Text.UTF8Encoding($false)))
$filterLog = Join-Path $buildDir "ctest_filter.log"
if (Test-Path $filterLog) { Remove-Item $filterLog -Force }
Write-Host "running $($sel.Names.Count) of $($sel.Total) test cases"
$code = Invoke-Vc @("ctest --test-dir `"$buildDir`" --output-on-failure -j $Jobs --tests-from-file `"$listFile`" > `"$filterLog`" 2>&1")
$log = Get-Content $filterLog
$log | Select-String -Pattern 'tests passed|tests failed|The following tests FAILED|\*\*\*Failed|\*\*\*Exception|\*\*\*Timeout|\*\*\*Not Run|No tests were found' | ForEach-Object { Write-Host $_.Line }
Write-Host "(full ctest output: $filterLog)"
if ($log -match 'No tests were found') { throw "TestFilter matched no test" }
if ($code -ne 0) { throw "tests failed ($code)" }
