# Plain-assert test of the tag filter in tools/build.ps1 (uses -ListOnly; needs a configured build/).
# Usage: powershell -ExecutionPolicy Bypass -File tools/tests/test_build_filter.ps1   (exit code 0 = pass)
$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$build = Join-Path $repo "tools\build.ps1"
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
$vcvars = Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat"
$script:fails = 0

function Assert([bool]$cond, [string]$msg) {
    if ($cond) { Write-Host "ok   $msg" } else { Write-Host "FAIL $msg"; $script:fails++ }
}

# Number of cases ctest itself selects by label regex (reference for the grouping logic).
function Ctest-Count([string]$args_) {
    $out = cmd /c "`"$vcvars`" >nul && ctest --test-dir `"$repo\build`" -N $args_"
    $line = $out | Select-String -Pattern 'Total Tests:\s*(\d+)'
    return [int]$line.Matches[0].Groups[1].Value
}

function List-Count([string[]]$extra) {
    $ErrorActionPreference = "Continue"  # vcvars writes a harmless vswhere notice to stderr
    $out = & powershell -NoProfile -ExecutionPolicy Bypass -File $build -ListOnly @extra 2>&1 | Out-String
    $ErrorActionPreference = "Stop"
    $m = [regex]::Match($out, 'matched: (\d+) of')
    if (-not $m.Success) { return -1 }
    return [int]$m.Groups[1].Value
}

# Intersection: ctest has no AND on labels, so count via two -L passes is impossible; compare with the JSON instead.
$jsonPath = Join-Path $env:TEMP "test_build_filter_$PID.json"
cmd /c "`"$vcvars`" >nul && ctest --test-dir `"$repo\build`" -N --show-only=json-v1 > `"$jsonPath`"" | Out-Null
$tests = (Get-Content $jsonPath -Raw | ConvertFrom-Json).tests
Remove-Item $jsonPath -Force
function Labels($t) { foreach ($p in $t.properties) { if ($p.name -eq "LABELS") { return @($p.value) } }; return @() }
function Count-With([string[]]$tags) {
    @($tests | Where-Object { $l = Labels $_; -not ($tags | Where-Object { $l -notcontains $_ }) -and ($l -notcontains "nightly") }).Count
}

$inter = Count-With @("gnc", "validation", "ascent")
Assert ($inter -gt 0) "reference intersection [gnc][validation][ascent] is non-empty ($inter)"
Assert ((List-Count @("-TestFilter", "[gnc][validation][ascent]")) -eq $inter) "[gnc][validation][ascent] is the intersection ($inter)"

Assert ($inter -eq 5) "absolute: [gnc][validation][ascent] matches 5 cases ($inter)"

# The union reference comes from the JSON too: without the nightly cases (ctest -L would count them).
function Count-Any([string[]]$tags) {
    @($tests | Where-Object { $l = Labels $_; ($l -notcontains "nightly") -and @($tags | Where-Object { $l -contains $_ }).Count -gt 0 }).Count
}
$legs = Count-With @("legs")
$asds = Count-With @("asds")
$union = Count-Any @("legs", "asds")
Assert ($union -le $legs + $asds) "reference union sane ($union of $legs + $asds)"
Assert ((List-Count @("-TestFilter", "[legs],[asds]")) -eq $union) "[legs],[asds] is the union ($union)"

$threw = $false
$ErrorActionPreference = "Continue"
$nomatchOut = & powershell -NoProfile -ExecutionPolicy Bypass -File $build -ListOnly -TestFilter "[nosuchtag]" 2>&1 | Out-String
if ($LASTEXITCODE -ne 0) { $threw = $true }
$ErrorActionPreference = "Stop"
Assert $threw "[nosuchtag] fails with no match"
Assert ($nomatchOut -match 'TestFilter matched no test') "[nosuchtag] reports 'TestFilter matched no test'"

$f9 = Ctest-Count '-L "^falcon9$"'
$f9noNight = Count-With @("falcon9")
$lunar = Ctest-Count '-L "^lunar$"'
$lunarNoNight = Count-With @("lunar")
Assert ($lunarNoNight -lt $lunar) "[lunar] holds nightly cases, excluded by default ($lunarNoNight of $lunar)"
Assert ((List-Count @("-TestFilter", "[lunar]")) -eq $lunarNoNight) "[lunar] without -Nightly excludes nightly cases ($lunarNoNight)"
Assert ((List-Count @("-TestFilter", "[falcon9]")) -eq $f9noNight) "[falcon9] without -Nightly excludes nightly cases ($f9noNight of $f9)"
$f9Night = @($tests | Where-Object { $l = Labels $_; $l -contains "falcon9" }).Count
Assert ((List-Count @("-TestFilter", "[falcon9]", "-Nightly")) -eq $f9Night) "[falcon9] -Nightly adds the nightly cases ($f9Night)"

$nightly = @($tests | Where-Object { (Labels $_) -contains "nightly" }).Count
if ($nightly -gt 0) {
    Assert ((List-Count @("-Nightly")) -eq $nightly) "-Nightly alone runs only nightly ($nightly)"
} else {
    Write-Host "skip -Nightly alone (no nightly cases tagged yet)"
}

if ($script:fails -gt 0) { Write-Host "$($script:fails) failure(s)"; exit 1 }
Write-Host "all passed"
exit 0
