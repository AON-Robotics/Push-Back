$ErrorActionPreference = 'Stop'
$culture = [System.Globalization.CultureInfo]::InvariantCulture
$samples = 60
$halfTrackWidth = 6.25
$maximumPathSpeed = 100.0
$projectRoot = Split-Path -Parent $PSScriptRoot
$points = [System.Collections.Generic.List[object]]::new()

function Add-Point([double]$x, [double]$y) {
  $points.Add([pscustomobject]@{x=$x; y=$y; speed=0.0})
}

function Add-Bezier($p0, $p1, $p2, $p3, [bool]$skipFirst) {
  for ($index = 0; $index -le $samples; ++$index) {
    if ($skipFirst -and $index -eq 0) { continue }
    $t = $index / $samples
    $u = 1.0 - $t
    Add-Point `
      ($u*$u*$u*$p0[0] + 3*$u*$u*$t*$p1[0] +
       3*$u*$t*$t*$p2[0] + $t*$t*$t*$p3[0]) `
      ($u*$u*$u*$p0[1] + 3*$u*$u*$t*$p1[1] +
       3*$u*$t*$t*$p2[1] + $t*$t*$t*$p3[1])
  }
}

function Add-Arc([double]$centerX, [double]$centerY, [double]$radius,
                 [double]$startAngle, [double]$endAngle, [bool]$skipFirst) {
  for ($index = 0; $index -le $samples; ++$index) {
    if ($skipFirst -and $index -eq 0) { continue }
    $angle = $startAngle + ($endAngle - $startAngle) * $index / $samples
    Add-Point ($centerX + $radius * [Math]::Cos($angle)) `
              ($centerY + $radius * [Math]::Sin($angle))
  }
}

# A separated-crossover figure eight. The 13-inch spacing at both outside
# turns permits 6.5-inch-radius semicircles; the two center passes remain six
# inches apart, eliminating LemLib's global-nearest branch ambiguity.
Add-Bezier @(0.0,18.0) @(4.0,18.0) @(2.75,17.5) @(6.75,17.5) $false
Add-Arc 6.75 24.0 6.5 (-[Math]::PI/2.0) ([Math]::PI/2.0) $true
Add-Bezier @(6.75,30.5) @(2.75,30.5) @(4.0,30.0) @(0.0,30.0) $true
Add-Bezier @(0.0,30.0) @(-4.0,30.0) @(-2.75,30.5) @(-6.75,30.5) $true
Add-Arc -6.75 24.0 6.5 ([Math]::PI/2.0) (3.0*[Math]::PI/2.0) $true
Add-Bezier @(-6.75,17.5) @(-5.5,17.5) @(-4.25,17.5) @(-3.0,17.5) $true

function Distance($a, $b) {
  [Math]::Sqrt(($a.x-$b.x)*($a.x-$b.x) + ($a.y-$b.y)*($a.y-$b.y))
}

for ($index = 0; $index -lt $points.Count; ++$index) {
  $radius = 1000.0
  if ($index -gt 0 -and $index + 1 -lt $points.Count) {
    $before = $points[$index-1]
    $point = $points[$index]
    $after = $points[$index+1]
    $a = Distance $point $before
    $b = Distance $after $point
    $c = Distance $after $before
    $twiceArea = [Math]::Abs(
        ($point.x-$before.x)*($after.y-$before.y) -
        ($point.y-$before.y)*($after.x-$before.x))
    if ($twiceArea -gt 0.000001) {
      $radius = $a*$b*$c/(2.0*$twiceArea)
    }
  }
  $points[$index].speed = [Math]::Min(
      $maximumPathSpeed,
      $maximumPathSpeed/(1.0+$halfTrackWidth/$radius))
}

$points[$points.Count-1].speed = 0.0
for ($index = $points.Count-2; $index -ge 0; --$index) {
  $points[$index].speed = [Math]::Min(
      $points[$index].speed, $points[$index+1].speed+3.0)
}

$lines = [System.Collections.Generic.List[string]]::new()
foreach ($point in $points) {
  $x = if ([Math]::Abs($point.x) -lt 0.000005) { 0.0 } else { $point.x }
  $lines.Add([string]::Format(
      $culture, "{0:F5}, {1:F5}, {2:F3}", $x, $point.y, $point.speed))
}
$lines.Add('endData')
$lines.Add('100')
$lines.Add('50')
$lines.Add('200')
$lines.Add(
    '#PATH.JERRYIO-DATA {"appVersion":"0.11.0","format":"LemLib v0.5","name":"AON Figure Eight","generator":"tools/generate-figure-eight-path.ps1"}')

$output = Join-Path $projectRoot 'static\figure-eight.jerryio.txt'
$lines | Set-Content -LiteralPath $output -Encoding ascii
Write-Output "Generated $output with $($points.Count) points"
