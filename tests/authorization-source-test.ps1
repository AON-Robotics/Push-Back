$ErrorActionPreference = 'Stop'

$repositoryRoot = Split-Path -Parent $PSScriptRoot
$configHeaderPath = Join-Path $repositoryRoot 'include/aon/config/robot-config.hpp'
$configSourcePath = Join-Path $repositoryRoot 'src/aon/config/robot-config.cpp'
$selectorPath = Join-Path $repositoryRoot 'src/aon/auton/routine-selectors.cpp'
$shadowChecklistPath = Join-Path $repositoryRoot 'docs/testing/2026-07-30-shadow-playback-checklist.md'
$roadmapChecklistPath = Join-Path $repositoryRoot 'docs/testing/2026-08-10-roadmap-baseline-checklist.md'

$configHeader = Get-Content -LiteralPath $configHeaderPath -Raw
$configSource = Get-Content -LiteralPath $configSourcePath -Raw
$selectors = Get-Content -LiteralPath $selectorPath -Raw
$shadowChecklist = Get-Content -LiteralPath $shadowChecklistPath -Raw
$roadmapChecklist = Get-Content -LiteralPath $roadmapChecklistPath -Raw

if ($configHeader -notmatch 'baselineAuthorizations\s*\(') {
  throw 'The fail-closed baseline authorization policy is missing'
}

foreach ($mapping in @(
    'config\.localization\.gps\.enabled',
    'config\.localization\.gps\.headingUpdateEnabled',
    'config\.localization\.fusedLemLibAuthorized',
    'config\.localization\.fusedNavigationAuthorized')) {
  if ($configHeader -notmatch $mapping) {
    throw "Authorization snapshot does not map localization gate: $mapping"
  }
}

foreach ($gate in @(
    'authorizations\.gpsHardware',
    'authorizations\.gpsHeadingFusion',
    'authorizations\.fusedLemLib',
    'authorizations\.fusedNavigation')) {
  if ($configSource -notmatch $gate) {
    throw "Localization config bypasses baseline authorization: $gate"
  }
}
$lockedBaselinePattern =
  'baselineAuthorizations\s*\([^)]*\)\s*noexcept\s*\{\s*return\s*\{\s*\}\s*;\s*\}'
$baselineIsEntirelyLocked = $configHeader -match $lockedBaselinePattern

foreach ($identity in @('Big', 'Small')) {
  $usage = "baselineAuthorizations\s*\(\s*RobotIdentity::$identity\s*\)"
  if ($configSource -notmatch $usage) {
    throw "The $identity robot initializer does not consume baselineAuthorizations"
  }
}

foreach ($route in @('RunRedSixBlockHybridFull', 'RunJerryIoPathAuton', 'RunShadowPlayback')) {
  if ($selectors -notmatch [regex]::Escape($route)) {
    throw "Expected gated route registration is missing: $route"
  }
}

$shadowResults = ($shadowChecklist -split '## Results', 2)[-1]
$shadowResultsIncomplete = $shadowResults -match '\|\s*Not run\s*\|'
$roadmapResultsIncomplete = $roadmapChecklist -match '\|[^\r\n]*\|\s*Not run\s*\|'
if (-not $baselineIsEntirelyLocked -and
    ($shadowResultsIncomplete -or $roadmapResultsIncomplete)) {
  throw 'A baseline authorization is enabled while physical Results remain Not run'
}

if ($roadmapChecklist -notmatch 'Approved to begin Phase 1\s*\|\s*\*\*No\*\*') {
  throw 'The roadmap physical gate does not explicitly block Phase 1'
}

Write-Output 'authorization source policy passed'
