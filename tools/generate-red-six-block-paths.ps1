$ErrorActionPreference = 'Stop'

$invariant = [System.Globalization.CultureInfo]::InvariantCulture
$repositoryRoot = Split-Path -Parent $PSScriptRoot

function Get-BezierPoint {
    param(
        [double] $X0, [double] $Y0,
        [double] $X1, [double] $Y1,
        [double] $X2, [double] $Y2,
        [double] $X3, [double] $Y3,
        [double] $T
    )
    $u = 1.0 - $T
    $x = $u * $u * $u * $X0 + 3.0 * $u * $u * $T * $X1 +
        3.0 * $u * $T * $T * $X2 + $T * $T * $T * $X3
    $y = $u * $u * $u * $Y0 + 3.0 * $u * $u * $T * $Y1 +
        3.0 * $u * $T * $T * $Y2 + $T * $T * $T * $Y3
    @($x, $y)
}

function Get-Curvature {
    param([object[]] $Points, [int] $Index)
    if ($Index -eq 0 -or $Index -eq $Points.Count - 1) { return 0.0 }
    $a = $Points[$Index - 1]
    $b = $Points[$Index]
    $c = $Points[$Index + 1]
    $ab = [Math]::Sqrt([Math]::Pow($b[0] - $a[0], 2) + [Math]::Pow($b[1] - $a[1], 2))
    $bc = [Math]::Sqrt([Math]::Pow($c[0] - $b[0], 2) + [Math]::Pow($c[1] - $b[1], 2))
    $ac = [Math]::Sqrt([Math]::Pow($c[0] - $a[0], 2) + [Math]::Pow($c[1] - $a[1], 2))
    if ($ab * $bc * $ac -eq 0.0) { return 0.0 }
    $twiceArea = [Math]::Abs(($b[0] - $a[0]) * ($c[1] - $a[1]) -
                            ($b[1] - $a[1]) * ($c[0] - $a[0]))
    2.0 * $twiceArea / ($ab * $bc * $ac)
}

function New-Path {
    param(
        [string] $Name,
        [string] $Output,
        [double[]] $Controls,
        [int] $Count,
        [int] $MaximumSpeed,
        [int] $MinimumSpeed,
        [double] $TailLength,
        [int] $TailSpeed
    )
    $points = @()
    for ($index = 0; $index -lt $Count; ++$index) {
        $t = $index / [double]($Count - 1)
        $points += ,(Get-BezierPoint `
            -X0 ($Controls[0]) -Y0 ($Controls[1]) -X1 ($Controls[2]) -Y1 ($Controls[3]) `
            -X2 ($Controls[4]) -Y2 ($Controls[5]) -X3 ($Controls[6]) -Y3 ($Controls[7]) `
            -T $t)
    }

    $remaining = New-Object double[] $Count
    for ($index = $Count - 2; $index -ge 0; --$index) {
        $segment = [Math]::Sqrt(
            [Math]::Pow($points[$index + 1][0] - $points[$index][0], 2) +
            [Math]::Pow($points[$index + 1][1] - $points[$index][1], 2))
        $remaining[$index] = $remaining[$index + 1] + $segment
    }

    $speeds = New-Object int[] $Count
    for ($index = 0; $index -lt $Count; ++$index) {
        if ($index -eq $Count - 1) {
            $speeds[$index] = 0
        } else {
            $curvature = Get-Curvature $points $index
            $curveCap = $MaximumSpeed / (1.0 + [Math]::Abs($curvature) * 6.25)
            $tailCap = [Math]::Max($TailSpeed,
                $MaximumSpeed * $remaining[$index] / $TailLength)
            $speeds[$index] = [Math]::Round([Math]::Max($MinimumSpeed,
                [Math]::Min($MaximumSpeed, [Math]::Min($curveCap, $tailCap))))
        }
    }
    for ($index = $Count - 3; $index -ge 0; --$index) {
        $speeds[$index] = [Math]::Min($speeds[$index], $speeds[$index + 1] + 15)
    }
    for ($index = 1; $index -lt $Count - 1; ++$index) {
        $speeds[$index] = [Math]::Min($speeds[$index], $speeds[$index - 1] + 15)
    }

    $lines = New-Object System.Collections.Generic.List[string]
    for ($index = 0; $index -lt $Count; ++$index) {
        $x = $points[$index][0].ToString('F5', $invariant)
        $y = $points[$index][1].ToString('F5', $invariant)
        $speed = $speeds[$index].ToString($invariant)
        $lines.Add("$x, $y, $speed")
    }
    $lines.Add('endData')
    $lines.Add('100')
    $lines.Add('50')
    $lines.Add('200')
    $lines.Add(('#PATH.JERRYIO-DATA {{"appVersion":"0.11.0","format":"LemLib v0.5","name":"{0}","generator":"tools/generate-red-six-block-paths.ps1"}}' -f $Name))
    [System.IO.File]::WriteAllLines($Output, $lines)
    $points
}

$loaderControls = @(0.0, 0.0, 0.0, 8.0, -4.0, 17.0, 0.0, 24.0)
$goalControls = @(-9.0, 31.0, -8.77, 29.67, -8.53, 28.33, -8.3, 27.0)
$loader = New-Path -Name 'AON Red Six Loader Approach' `
    -Output (Join-Path $repositoryRoot 'static/red-six-loader-approach.jerryio.txt') `
    -Controls $loaderControls -Count 25 -MaximumSpeed 90 -MinimumSpeed 25 `
    -TailLength 10.0 -TailSpeed 45
$goal = New-Path -Name 'AON Red Six Goal Transfer' `
    -Output (Join-Path $repositoryRoot 'static/red-six-goal-transfer.jerryio.txt') `
    -Controls $goalControls -Count 13 -MaximumSpeed 45 -MinimumSpeed 20 `
    -TailLength 4.0 -TailSpeed 20

$generated = @"
#pragma once

#include <cstddef>

namespace aon::auton {

struct RedSixBlockGenerated {
  static constexpr double loaderStartX = $($loader[0][0].ToString('F1', $invariant));
  static constexpr double loaderStartY = $($loader[0][1].ToString('F1', $invariant));
  static constexpr double loaderEndX = $($loader[-1][0].ToString('F1', $invariant));
  static constexpr double loaderEndY = $($loader[-1][1].ToString('F1', $invariant));
  static constexpr double goalStartX = $($goal[0][0].ToString('F1', $invariant));
  static constexpr double goalStartY = $($goal[0][1].ToString('F1', $invariant));
  static constexpr double goalStageX = $($goal[-1][0].ToString('F1', $invariant));
  static constexpr double goalStageY = $($goal[-1][1].ToString('F1', $invariant));
  static constexpr double goalStageHeading = 171.0;
  static constexpr std::size_t loaderPointCount = $($loader.Count);
  static constexpr std::size_t goalPointCount = $($goal.Count);
};

}  // namespace aon::auton
"@
[System.IO.File]::WriteAllText(
    (Join-Path $repositoryRoot 'include/aon/auton/red-six-block-generated.hpp'),
    $generated.Replace("`r`n", "`n"))
