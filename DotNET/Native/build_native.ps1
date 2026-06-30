# Build haul_kdtree native library (Windows / cross-platform via CMake).
param(
    [String] $Configuration = "Release",
    [String] $OutDir = ""
)

$ErrorActionPreference = "Stop"
$NativeRoot = $PSScriptRoot
$BuildDir = Join-Path $NativeRoot "build"

if (-not (Test-Path (Join-Path $NativeRoot "third_party\nanoflann\nanoflann.hpp"))) {
    throw "Missing nanoflann.hpp. Download to Native/third_party/nanoflann/nanoflann.hpp"
}

$Generator = $null
if ($IsWindows -or $env:OS -match "Windows") {
    if (Get-Command cl.exe -ErrorAction SilentlyContinue) {
        $Generator = "NMake Makefiles"
    }
    elseif (Test-Path "${env:ProgramFiles}\Microsoft Visual Studio\2022\Community\Common7\IDE\devenv.exe") {
        $Generator = "Visual Studio 17 2022"
    }
    elseif (Test-Path "${env:ProgramFiles}\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat") {
        $Generator = "Visual Studio 17 2022"
    }
    else {
        $Generator = "Visual Studio 17 2022"
    }
}

if ($Generator -eq "Visual Studio 17 2022") {
    cmake -S $NativeRoot -B $BuildDir -G $Generator -A x64
    cmake --build $BuildDir --config $Configuration --target haul_kdtree
    $LibDir = Join-Path $BuildDir $Configuration
}
else {
    cmake -S $NativeRoot -B $BuildDir -DCMAKE_BUILD_TYPE=$Configuration
    cmake --build $BuildDir --target haul_kdtree
    $LibDir = $BuildDir
}

$Dll = Join-Path $LibDir "haul_kdtree.dll"
$So = Join-Path $LibDir "libhaul_kdtree.so"
$Dylib = Join-Path $LibDir "libhaul_kdtree.dylib"

$Artifact = $null
if (Test-Path $Dll) { $Artifact = $Dll }
elseif (Test-Path $So) { $Artifact = $So }
elseif (Test-Path $Dylib) { $Artifact = $Dylib }
else {
    throw "Native library not found under $LibDir"
}

if ($OutDir -ne "") {
    New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
    Copy-Item -Force $Artifact $OutDir
    Write-Host "Copied $(Split-Path $Artifact -Leaf) -> $OutDir"
}

Write-Host "Built $Artifact"
