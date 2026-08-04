param(
  [Parameter(Mandatory = $true)]
  [string]$Nm,

  [Parameter(Mandatory = $true)]
  [string]$NativeTestsObject,

  [Parameter(Mandatory = $true)]
  [string]$NativeRoutinesObject,

  [switch]$ExpectXDrive
)

$ErrorActionPreference = 'Stop'

function Get-DefinedSymbols {
  param([string]$ObjectPath)

  if (-not (Test-Path -LiteralPath $ObjectPath -PathType Leaf)) {
    throw "Required object file does not exist: $ObjectPath"
  }

  $output = & $Nm -C --defined-only $ObjectPath
  if ($LASTEXITCODE -ne 0) {
    throw "nm failed for $ObjectPath with exit code $LASTEXITCODE"
  }

  return @($output | ForEach-Object {
      if ($_ -match '^[0-9A-Fa-f]+\s+\S\s+(.+)$') {
        $Matches[1]
      }
    })
}

$expectedSymbols = @(
  'aon::alignRobotTo(aon::Colors const&)',
  'aon::getDistanceToRing(aon::Colors const&)',
  'aon::driveTillPickUp(double const&)',
  'aon::alignAndIntake(aon::Colors const&)',
  'aon::driveIntoRing(aon::Colors const&)',
  'aon::tests::gpsOctagon()',
  'aon::tests::distanceSensorSpeed(double)',
  'aon::tests::odom()',
  'aon::tests::concurrency()',
  'aon::tests::alignment()',
  'aon::tests::visionSensorDistance()',
  'aon::tests::gyroWithEKF()',
  'aon::tests::adjustable()',
  'aon::tests::multiple()',
  'aon::tests::turns()',
  'aon::tests::square()',
  'aon::tests::continuity()',
  'aon::tests::colorSorting()',
  'aon::tests::purePursuitPoint()',
  'aon::tests::purePursuitSimpleFollow()',
  'aon::tests::purePursuitPath()'
)
if ($ExpectXDrive) {
  $expectedSymbols += 'aon::tests::xDriveRoutine()'
}

$testSymbols = Get-DefinedSymbols -ObjectPath $NativeTestsObject
$routeSymbols = Get-DefinedSymbols -ObjectPath $NativeRoutinesObject
$missing = @($expectedSymbols | Where-Object { $_ -notin $testSymbols })
$leftInRoutes = @($expectedSymbols | Where-Object { $_ -in $routeSymbols })

if ($missing.Count -ne 0) {
  throw "Native test object is missing symbols: $($missing -join ', ')"
}
if ($leftInRoutes.Count -ne 0) {
  throw "Native route object still defines test symbols: $($leftInRoutes -join ', ')"
}

Write-Output "native test symbol boundary passed ($($expectedSymbols.Count) symbols)"
