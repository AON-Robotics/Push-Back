$ErrorActionPreference = 'Stop'
$culture = [System.Globalization.CultureInfo]::InvariantCulture
$steps = 120
$lines = [System.Collections.Generic.List[string]]::new()
$xValues = [System.Collections.Generic.List[double]]::new()
$yValues = [System.Collections.Generic.List[double]]::new()
$speeds = [System.Collections.Generic.List[double]]::new()

for ($index = 0; $index -le $steps; ++$index) {
  $t = [Math]::PI + 2.0 * [Math]::PI * $index / $steps
  $x = 20.0 * [Math]::Cos($t)
  $y = 22.0 + 18.0 * [Math]::Sin(2.0 * $t)
  if ([Math]::Abs($x) -lt 0.0005) { $x = 0.0 }
  if ([Math]::Abs($y - 22.0) -lt 0.0005) { $y = 22.0 }

  $dx = -20.0 * [Math]::Sin($t)
  $dy = 36.0 * [Math]::Cos(2.0 * $t)
  $ddx = -20.0 * [Math]::Cos($t)
  $ddy = -72.0 * [Math]::Sin(2.0 * $t)
  $denominator = [Math]::Pow($dx * $dx + $dy * $dy, 1.5)
  $curvature = if ($denominator -gt 0.000001) {
    [Math]::Abs($dx * $ddy - $dy * $ddx) / $denominator
  } else { 0.0 }
  $radius = if ($curvature -gt 0.000001) { 1.0 / $curvature } else { 1000.0 }
  $speed = [Math]::Min(65.0, 24.0 + 7.0 * [Math]::Sqrt($radius))
  if ([Math]::Abs($x) -le 4.0 -or $radius -le 4.0) {
    $speed = [Math]::Min($speed, 35.0)
  }
  if ($index -eq 0) { $speed = [Math]::Min($speed, 20.0) }
  if ($index -eq $steps) { $speed = 0.0 }

  $xValues.Add($x)
  $yValues.Add($y)
  $speeds.Add($speed)
}

# Bound adjacent speed changes in both directions so the drivetrain accelerates
# and decelerates continuously through bends, crossings, and the final stop.
$maximumStep = 3.0
for ($index = 1; $index -le $steps; ++$index) {
  $speeds[$index] = [Math]::Min(
      $speeds[$index], $speeds[$index - 1] + $maximumStep)
}
for ($index = $steps - 1; $index -ge 0; --$index) {
  $speeds[$index] = [Math]::Min(
      $speeds[$index], $speeds[$index + 1] + $maximumStep)
}

for ($index = 0; $index -le $steps; ++$index) {
  $lines.Add([string]::Format(
      $culture, "{0:F3}, {1:F3}, {2:F3}",
      $xValues[$index], $yValues[$index], $speeds[$index]))
}

$lines.Add('endData')
$lines.Add('65')
$lines.Add('35')
$lines.Add('200')
$lines.Add('#PATH.JERRYIO-DATA {"appVersion":"0.11.0","format":"LemLib v0.5","name":"AON 2x2 Figure Eight","generator":"tools/generate-figure-eight-path.ps1","bounds":{"minX":-22,"maxX":22,"minY":0,"maxY":44}}')

$projectRoot = Split-Path -Parent $PSScriptRoot
$output = Join-Path $projectRoot 'static\figure-eight.jerryio.txt'
$lines | Set-Content -LiteralPath $output -Encoding ascii
Write-Output "Generated $output with $($steps + 1) points"
