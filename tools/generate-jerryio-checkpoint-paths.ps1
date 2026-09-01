$ErrorActionPreference = 'Stop'

$invariant = [System.Globalization.CultureInfo]::InvariantCulture
$repositoryRoot = Split-Path -Parent $PSScriptRoot
$sourcePath = Join-Path $repositoryRoot 'static/path.jerryio.txt'

function Get-Distance {
    param([object[]] $First, [object[]] $Second)
    [Math]::Sqrt([Math]::Pow($Second[0] - $First[0], 2) +
                 [Math]::Pow($Second[1] - $First[1], 2))
}

function Get-ClosestSegment {
    param([object[]] $Points, [double] $X, [double] $Y, [int] $StartIndex)
    $bestIndex = -1
    $bestDistance = [double]::PositiveInfinity
    for ($index = $StartIndex; $index -lt $Points.Count - 1; ++$index) {
        $start = $Points[$index]
        $finish = $Points[$index + 1]
        [double]$dx = [double]$finish[0] - [double]$start[0]
        [double]$dy = [double]$finish[1] - [double]$start[1]
        [double]$lengthSquared = $dx * $dx + $dy * $dy
        [double]$t = 0.0
        if ($lengthSquared -ne 0.0) {
            $t = (($X - [double]$start[0]) * $dx +
                ($Y - [double]$start[1]) * $dy) / $lengthSquared
        }
        $t = [Math]::Max(0.0, [Math]::Min(1.0, $t))
        [double]$projectedX = [double]$start[0] + $t * $dx
        [double]$projectedY = [double]$start[1] + $t * $dy
        $projected = @($projectedX, $projectedY)
        $distance = Get-Distance -First $projected -Second @($X, $Y)
        if ($distance -lt $bestDistance) {
            $bestDistance = $distance
            $bestIndex = $index
        }
    }
    if ($bestIndex -lt 0 -or $bestDistance -gt 1.0) {
        throw "Checkpoint ($X, $Y) is not on the generated JerryIO path"
    }
    $bestIndex
}

function Set-StopProfile {
    param([object[]] $Points)
    $remaining = New-Object double[] $Points.Count
    for ($index = $Points.Count - 2; $index -ge 0; --$index) {
        $remaining[$index] = $remaining[$index + 1] +
            (Get-Distance -First $Points[$index] -Second $Points[$index + 1])
    }
    for ($index = 0; $index -lt $Points.Count - 1; ++$index) {
        $tailCap = [Math]::Max(20.0, 127.0 * $remaining[$index] / 12.0)
        $Points[$index][2] = [Math]::Round(
            [Math]::Min($Points[$index][2], $tailCap))
    }
    $Points[-1][2] = 0.0
}

function Write-Path {
    param([string] $Name, [string] $Filename, [object[]] $Points)
    Set-StopProfile $Points
    $lines = New-Object System.Collections.Generic.List[string]
    foreach ($point in $Points) {
        $x = ([double]$point[0]).ToString('F5', $invariant)
        $y = ([double]$point[1]).ToString('F5', $invariant)
        $speed = ([int]$point[2]).ToString($invariant)
        $lines.Add("$x, $y, $speed")
    }
    $lines.Add('endData')
    $lines.Add('127')
    $lines.Add('100')
    $lines.Add('200')
    $lines.Add(('#PATH.JERRYIO-DATA {{"appVersion":"0.11.0",' +
        '"format":"LemLib v0.5","name":"{0}",' +
        '"source":"static/path.jerryio.txt"}}' -f $Name))
    [System.IO.File]::WriteAllLines(
        (Join-Path $repositoryRoot "static/$Filename"), $lines)
}

$points = New-Object System.Collections.Generic.List[object]
foreach ($line in [System.IO.File]::ReadLines($sourcePath)) {
    if ($line -eq 'endData') { break }
    $values = $line.Split(',')
    if ($values.Count -ne 3) { throw "Invalid generated path row: $line" }
    $points.Add(@(
        [double]::Parse($values[0], $invariant),
        [double]::Parse($values[1], $invariant),
        [double]::Parse($values[2], $invariant)))
}

$checkpoints = @(
    @(-12.547, -21.611),
    @(-39.91, -23.221),
    @(-60.298, -58.81)
)
$split1 = Get-ClosestSegment $points $checkpoints[0][0] $checkpoints[0][1] 0
$split2 = Get-ClosestSegment $points $checkpoints[1][0] $checkpoints[1][1] ($split1 + 1)
$split3 = Get-ClosestSegment $points $checkpoints[2][0] $checkpoints[2][1] ($split2 + 1)

$leg1 = @($points[0..$split1]) + ,@($checkpoints[0][0], $checkpoints[0][1], 0.0)
$leg2 = ,@($checkpoints[0][0], $checkpoints[0][1], 20.0) +
    @($points[($split1 + 1)..$split2]) +
    ,@($checkpoints[1][0], $checkpoints[1][1], 0.0)
$leg3 = ,@($checkpoints[1][0], $checkpoints[1][1], 20.0) +
    @($points[($split2 + 1)..$split3]) +
    ,@($checkpoints[2][0], $checkpoints[2][1], 0.0)

Write-Path 'JerryIO to intake' 'path-jerryio-intake.txt' $leg1
Write-Path 'JerryIO to outtake' 'path-jerryio-outtake.txt' $leg2
Write-Path 'JerryIO to pistons' 'path-jerryio-pistons.txt' $leg3
