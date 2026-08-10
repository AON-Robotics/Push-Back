$ErrorActionPreference = 'Stop'

$compiler = 'C:\Users\jojur\AppData\Local\Programs\CLion\bin\mingw\bin\g++.exe'
if (-not (Test-Path -LiteralPath $compiler)) {
  $compiler = (Get-Command g++ -ErrorAction Stop).Source
}

$compilerDirectory = Split-Path -Parent $compiler
$env:Path = "$compilerDirectory;$env:Path"
$outputDirectory = 'bin\host-tests'
New-Item -ItemType Directory -Force $outputDirectory | Out-Null

function Invoke-HostTest {
  param(
    [Parameter(Mandatory = $true)][string]$Name,
    [Parameter(Mandatory = $true)][string[]]$Sources,
    [switch]$WithoutIncludes,
    [string]$Standard = 'c++17',
    [string[]]$ExtraFlags = @()
  )

  $executable = Join-Path $outputDirectory "$Name.exe"
  $arguments = @("-std=$Standard", '-Wall', '-Wextra', '-Werror')
  $arguments += $ExtraFlags
  if (-not $WithoutIncludes) {
    $arguments += '-Iinclude'
  }
  $arguments += $Sources
  $arguments += @('-o', $executable)
  & $compiler @arguments
  if ($LASTEXITCODE -ne 0) {
    throw "$Name compile failed"
  }
  & $executable
  if ($LASTEXITCODE -ne 0) {
    throw "$Name failed"
  }
}

Invoke-HostTest 'localization-math-test' @(
  'tests\localization-math-test.cpp',
  'src\aon\pose-estimator.cpp',
  'src\aon\ekf.cpp'
)
Invoke-HostTest 'localization-confidence-test' @(
  'tests\localization-confidence-test.cpp',
  'src\aon\localization\confidence.cpp'
)
Invoke-HostTest 'pose-history-test' @(
  'tests\pose-history-test.cpp',
  'src\aon\localization\pose-history.cpp',
  'src\aon\pose-estimator.cpp'
)
Invoke-HostTest 'velocity-estimator-test' @(
  'tests\velocity-estimator-test.cpp',
  'src\aon\localization\velocity-estimator.cpp'
)
Invoke-HostTest 'wall-observation-test' @(
  'tests\wall-observation-test.cpp',
  'src\aon\localization\wall-observation.cpp',
  'src\aon\ekf.cpp',
  'src\aon\pose-estimator.cpp'
)
Invoke-HostTest 'field-model-test' @(
  'tests\field-model-test.cpp',
  'src\aon\field\field-map.cpp',
  'src\aon\field\push-back-field.cpp'
)
Invoke-HostTest 'navigation-test' @(
  'tests\navigation-test.cpp',
  'src\aon\navigation\dynamic-obstacles.cpp',
  'src\aon\navigation\path-planner.cpp',
  'src\aon\navigation\replanner.cpp',
  'src\aon\navigation\path-follower.cpp',
  'src\aon\field\field-map.cpp',
  'src\aon\field\push-back-field.cpp'
)
Invoke-HostTest 'pi-protocol-test' @(
  'tests\pi-protocol-test.cpp',
  'src\aon\communication\pi-protocol.cpp'
)
Invoke-HostTest 'pi-route-assembler-test' @(
  'tests\pi-route-assembler-test.cpp',
  'src\aon\navigation\pi-route-assembler.cpp'
)
Invoke-HostTest 'pi-navigation-gateway-test' @(
  'tests\pi-navigation-gateway-test.cpp',
  'src\aon\communication\pi-navigation-gateway.cpp',
  'src\aon\communication\pi-protocol.cpp',
  'src\aon\navigation\dynamic-obstacles.cpp',
  'src\aon\navigation\pi-route-assembler.cpp'
)
Invoke-HostTest 'lidar-scan-test' @(
  'tests\lidar-scan-test.cpp',
  'src\aon\lidar\scan-processor.cpp',
  'src\aon\field\field-map.cpp',
  'src\aon\field\push-back-field.cpp'
)
Invoke-HostTest 'localization-config-test' @(
  '-DUSING_BIG_ROBOT=0',
  'tests\localization-config-test.cpp',
  'src\aon\config\robot-config.cpp',
  'src\aon\config\hardware-map.cpp'
)
Invoke-HostTest 'localization-config-big-test' @(
  '-DUSING_BIG_ROBOT=1',
  'tests\localization-config-test.cpp',
  'src\aon\config\robot-config.cpp',
  'src\aon\config\hardware-map.cpp'
)
Invoke-HostTest -Name 'localization-integration-test' -Sources @(
  'tests\localization-integration-test.cpp'
) -WithoutIncludes
Invoke-HostTest -Name 'resource-policy-test' -Sources @(
  'tests\resource-policy-test.cpp'
)
Invoke-HostTest -Name 'pros-resource-policy-test' -Sources @(
  'tests\pros-resource-policy-test.cpp'
) -Standard 'gnu++20' -ExtraFlags @(
  '-Wno-deprecated-declarations',
  '-Wno-sign-compare',
  '-Wno-unused-parameter'
)

Write-Host 'All fused localization and navigation host tests passed'
