param(
    [ValidateSet('Debug', 'Release', 'RelWithDebInfo', 'MinSizeRel')]
    [string]$Configuration = 'Debug',
    [ValidateSet('ON', 'OFF')]
    [string]$BuildProto = 'OFF',
    [switch]$CleanCache
)

$ErrorActionPreference = 'Stop'

$cmakeExe = 'C:/Program Files (x86)/Microsoft Visual Studio/18/BuildTools/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe'

if (-not (Test-Path $cmakeExe)) {
    throw "cmake.exe not found at: $cmakeExe"
}

$repoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$sourceDir = Join-Path $repoRoot 'cpp'
$buildDir = Join-Path $repoRoot 'build'

if ($CleanCache) {
    $cacheFile = Join-Path $buildDir 'CMakeCache.txt'
    $cmakeFilesDir = Join-Path $buildDir 'CMakeFiles'

    if (Test-Path $cacheFile) {
        Remove-Item $cacheFile -Force
    }
    if (Test-Path $cmakeFilesDir) {
        Remove-Item $cmakeFilesDir -Recurse -Force
    }
}

& $cmakeExe -S $sourceDir -B $buildDir -G 'Visual Studio 18 2026' -A x64 -DBUILD_PROTO=$BuildProto
& $cmakeExe --build $buildDir --config $Configuration -- /m