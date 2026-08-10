$ErrorActionPreference = 'Stop'

$compiler = 'C:\Users\jojur\AppData\Local\Programs\CLion\bin\mingw\bin\g++.exe'
if (-not (Test-Path -LiteralPath $compiler)) {
  $compiler = (Get-Command g++ -ErrorAction Stop).Source
}

# MinGW's driver starts sibling compiler and linker processes. Keep its bin
# directory visible so the same script works outside CLion's configured shell.
$compilerDirectory = Split-Path -Parent $compiler
$env:Path = "$compilerDirectory;$env:Path"

New-Item -ItemType Directory -Force 'bin\host-tests' | Out-Null
& $compiler -std=c++17 -Wall -Wextra -Werror -Iinclude `
  tests\localization-math-test.cpp `
  src\aon\pose-estimator.cpp `
  src\aon\ekf.cpp `
  -o bin\host-tests\localization-math-test.exe
if ($LASTEXITCODE -ne 0) {
  throw 'localization host compile failed'
}

& '.\bin\host-tests\localization-math-test.exe'
if ($LASTEXITCODE -ne 0) {
  throw 'localization host tests failed'
}

& $compiler -std=c++17 -Wall -Wextra -Werror -Iinclude `
  tests\localization-config-test.cpp `
  src\aon\config\robot-config.cpp `
  src\aon\config\hardware-map.cpp `
  -o bin\host-tests\localization-config-test.exe
if ($LASTEXITCODE -ne 0) {
  throw 'localization config compile failed'
}

& '.\bin\host-tests\localization-config-test.exe'
if ($LASTEXITCODE -ne 0) {
  throw 'localization config tests failed'
}
