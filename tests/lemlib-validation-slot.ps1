param(
  [Parameter(Mandatory = $true)]
  [string]$Nm,

  [Parameter(Mandatory = $true)]
  [string]$Strings,

  [Parameter(Mandatory = $true)]
  [string]$SelectorObject,

  [Parameter(Mandatory = $true)]
  [string]$GuiObject
)

$ErrorActionPreference = 'Stop'

foreach ($object in @($SelectorObject, $GuiObject)) {
  if (-not (Test-Path -LiteralPath $object -PathType Leaf)) {
    throw "Required object file does not exist: $object"
  }
}

$undefined = & $Nm -C --undefined-only $SelectorObject
if ($LASTEXITCODE -ne 0) {
  throw "nm failed for $SelectorObject with exit code $LASTEXITCODE"
}

$required = 'aon::routines::RunLemLibForwardValidation()'
if (-not ($undefined -match [regex]::Escape($required))) {
  throw "Skills selector does not call the isolated forward validation"
}

$forbidden = @(
  'aon::routines::RunNativeForwardReverseTest()',
  'aon::routines::RunLemLibReverseValidation()',
  'aon::routines::RunLemLibClockwiseTurnValidation()',
  'aon::routines::RunLemLibCounterclockwiseTurnValidation()',
  'aon::routines::RunLemLibCombinedValidation()'
)
foreach ($symbol in $forbidden) {
  if ($undefined -match [regex]::Escape($symbol)) {
    throw "Skills selector exposes a forbidden validation dependency: $symbol"
  }
}

$labels = & $Strings $GuiObject
if ($LASTEXITCODE -ne 0) {
  throw "strings failed for $GuiObject with exit code $LASTEXITCODE"
}
if ('TEST LemLib Forward 12in' -notin $labels) {
  throw 'GUI object does not contain the isolated forward validation label'
}

Write-Output 'isolated LemLib forward skills slot passed'
