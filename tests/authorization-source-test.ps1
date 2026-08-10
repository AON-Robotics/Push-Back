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

foreach ($identity in @('Big', 'Small')) {
  $usage = "baselineAuthorizations\s*\(\s*RobotIdentity::$identity\s*\)"
  if ($configSource -notmatch $usage) {
    throw "The $identity robot initializer does not consume baselineAuthorizations"
  }
}

if ($configSource -match '(?m)^\s*true\s*,\s*//\s*shadowPlaybackAuthorized') {
  throw 'A production robot configuration enables Shadow playback'
}

foreach ($route in @('RunRedSixBlockHybridFull', 'RunJerryIoPathAuton', 'RunShadowPlayback')) {
  if ($selectors -notmatch [regex]::Escape($route)) {
    throw "Expected gated route registration is missing: $route"
  }
}

if ($shadowChecklist -notmatch 'Not run' -or $roadmapChecklist -notmatch 'Not run') {
  throw 'Expected incomplete physical-gate evidence was not found'
}

if ($roadmapChecklist -notmatch 'Approved to begin Phase 1\s*\|\s*\*\*No\*\*') {
  throw 'The roadmap physical gate does not explicitly block Phase 1'
}

Write-Output 'authorization source policy passed'
