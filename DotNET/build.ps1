# HaulBuilder — PowerShell build script (Windows when GNU Make is not installed).
param(
    [Parameter(Position = 0)]
    [ValidateSet("all", "restore", "build", "debug", "test", "test-verbose", "verify", "clean", "publish")]
    [String] $Target = "verify",
    [String] $Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$Sln = Join-Path $PSScriptRoot "HaulBuilder.sln"

function Invoke-DotNet {
    param([String[]] $Arguments)
    # Snapshot arguments so paths with spaces stay intact for the dotnet child process.
    & dotnet $Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "dotnet failed with exit code $LASTEXITCODE"
    }
}

switch ($Target) {
    "restore" {
        Invoke-DotNet @("restore", $Sln, "--nologo", "-v", "m")
    }
    "build" {
        Invoke-DotNet @("restore", $Sln, "--nologo", "-v", "m")
        & (Join-Path $PSScriptRoot "Native\build_native.ps1") -Configuration $Configuration
        Invoke-DotNet @("build", $Sln, "-c", $Configuration, "--nologo", "-v", "m", "--no-restore")
    }
    "debug" {
        Invoke-DotNet @("build", $Sln, "-c", "Debug", "--nologo", "-v", "m")
    }
    "test" {
        Invoke-DotNet @("restore", $Sln, "--nologo", "-v", "m")
        Invoke-DotNet @("build", $Sln, "-c", $Configuration, "--nologo", "-v", "m", "--no-restore")
        Invoke-DotNet @("test", $Sln, "-c", $Configuration, "--nologo", "-v", "m", "--no-build")
    }
    "test-verbose" {
        Invoke-DotNet @("restore", $Sln, "--nologo", "-v", "m")
        Invoke-DotNet @("build", $Sln, "-c", $Configuration, "--nologo", "-v", "m", "--no-restore")
        Invoke-DotNet @("test", $Sln, "-c", $Configuration, "--nologo", "-v", "n", "--no-build")
    }
    "verify" {
        & $PSScriptRoot\build.ps1 -Target test -Configuration $Configuration
    }
    "clean" {
        Invoke-DotNet @("clean", $Sln, "-c", "Debug", "--nologo")
        Invoke-DotNet @("clean", $Sln, "-c", "Release", "--nologo")
        $Artifacts = Join-Path $PSScriptRoot "artifacts"
        if (Test-Path $Artifacts) {
            Remove-Item -Recurse -Force $Artifacts
        }
    }
    "publish" {
        & $PSScriptRoot\build.ps1 -Target build -Configuration $Configuration
        $OutDir = Join-Path $PSScriptRoot "artifacts\HaulBuilder"
        Invoke-DotNet @(
            "publish", "HaulBuilder/HaulBuilder.csproj",
            "-c", $Configuration, "--nologo", "-v", "m", "--no-restore",
            "-o", $OutDir
        )
    }
    "all" {
        & $PSScriptRoot\build.ps1 -Target verify -Configuration $Configuration
    }
}
