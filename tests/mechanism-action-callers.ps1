param(
  [Parameter(Mandatory = $true)]
  [string]$Nm,

  [Parameter(Mandatory = $true)]
  [string]$Object,

  [Parameter(Mandatory = $true)]
  [string[]]$ExpectedActions
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path -LiteralPath $Object -PathType Leaf)) {
  throw "Required object file does not exist: $Object"
}

$output = & $Nm -C --undefined-only $Object
if ($LASTEXITCODE -ne 0) {
  throw "nm failed for $Object with exit code $LASTEXITCODE"
}

$symbols = @($output | ForEach-Object {
    if ($_ -match '^\s+U\s+(.+)$') {
      $Matches[1]
    }
  })
$mechanismSymbols = @($symbols | Where-Object {
    $_ -like 'aon::auton::mechanisms::*'
  })

foreach ($action in $ExpectedActions) {
  $escapedAction = [regex]::Escape($action)
  $explicitPattern =
      "^aon::auton::mechanisms::$escapedAction\(aon::core::Hardware&(?:, .+)?\)$"
  if (-not ($mechanismSymbols -match $explicitPattern)) {
    throw "Missing explicit Hardware& call to mechanism action: $action"
  }
}

$compatibilityCalls = @($mechanismSymbols | Where-Object {
    $_ -notmatch '\(aon::core::Hardware&(?:, .+)?\)$'
  })
if ($compatibilityCalls.Count -ne 0) {
  throw "Compatibility mechanism calls remain: $($compatibilityCalls -join ', ')"
}

Write-Output "explicit mechanism caller boundary passed ($($ExpectedActions.Count) actions)"
