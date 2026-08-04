param(
  [Parameter(Mandatory = $true)]
  [string]$Nm,

  [Parameter(Mandatory = $true)]
  [ValidateSet('Big', 'Small')]
  [string]$Robot,

  [Parameter(Mandatory = $true)]
  [string]$LegacyObject,

  [Parameter(Mandatory = $true)]
  [string]$BigObject,

  [Parameter(Mandatory = $true)]
  [string]$SmallObject,

  [Parameter(Mandatory = $true)]
  [string]$SkillsObject,

  [ValidateSet('Big', 'Small', 'Skills')]
  [string[]]$ExpectedSplits = @()
)

$ErrorActionPreference = 'Stop'

function Get-DefinedSymbols {
  param([string]$ObjectPath)

  if (-not (Test-Path -LiteralPath $ObjectPath -PathType Leaf)) {
    return @()
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

$bigRoutes = @(
  'aon::routines::safeBigBotRoutine()',
  'aon::routines::bigBotCurves()',
  'aon::routines::bigBotContinuity()',
  'aon::routines::bigBotStayThere()',
  'aon::routines::bigBotLongGoalThenPark()',
  'aon::routines::bigBotPark()'
)
$smallRoutes = @(
  'aon::routines::smallBotRoutine()',
  'aon::routines::blackBeard()',
  'aon::routines::jackSparrow()',
  'aon::routines::smallBotRoutineWorlds()',
  'aon::routines::smallBotCurves()',
  'aon::routines::smallBotPark()'
)
$skillsRoutes = if ($Robot -eq 'Big') {
  @('aon::routines::BigBotSkillsRoutine()')
} else {
  @('aon::routines::smallbotjorgeg()')
}

$objects = [ordered]@{
  Legacy = Get-DefinedSymbols -ObjectPath $LegacyObject
  Big = Get-DefinedSymbols -ObjectPath $BigObject
  Small = Get-DefinedSymbols -ObjectPath $SmallObject
  Skills = Get-DefinedSymbols -ObjectPath $SkillsObject
}

$expectations = @()
if ($Robot -eq 'Big') {
  $bigOwner = if ('Big' -in $ExpectedSplits) { 'Big' } else { 'Legacy' }
  $expectations += $bigRoutes | ForEach-Object { [pscustomobject]@{ Symbol = $_; Owner = $bigOwner } }
} else {
  $smallOwner = if ('Small' -in $ExpectedSplits) { 'Small' } else { 'Legacy' }
  $expectations += $smallRoutes | ForEach-Object { [pscustomobject]@{ Symbol = $_; Owner = $smallOwner } }
}
$skillsOwner = if ('Skills' -in $ExpectedSplits) { 'Skills' } else { 'Legacy' }
$expectations += $skillsRoutes | ForEach-Object { [pscustomobject]@{ Symbol = $_; Owner = $skillsOwner } }

foreach ($expectation in $expectations) {
  $owners = @($objects.Keys | Where-Object { $expectation.Symbol -in $objects[$_] })
  if ($owners.Count -ne 1) {
    throw "Expected exactly one definition of $($expectation.Symbol); found $($owners.Count) in: $($owners -join ', ')"
  }
  if ($owners[0] -ne $expectation.Owner) {
    throw "Expected $($expectation.Symbol) in $($expectation.Owner); found it in $($owners[0])"
  }
}

Write-Output "native $($Robot.ToLowerInvariant()) route symbol boundary passed ($($expectations.Count) symbols)"
