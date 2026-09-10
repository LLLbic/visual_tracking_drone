param(
    [string]$SdkRoot = "D:\NVToolKits\Optical_Flow_SDK_5.0.7\Optical_Flow_SDK_5.0.7",
    [string]$CudaRoot = "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6"
)

$ErrorActionPreference = "Stop"
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$source = Join-Path $repoRoot "native\nvof_fruc_bridge.cpp"
$outputDir = Join-Path $repoRoot ".runtime\nvof-fruc"
$sdkInclude = Join-Path $SdkRoot "NvOFFRUC\Interface"
$frucDll = Join-Path $SdkRoot "NvOFFRUC\NvOFFRUCSample\bin\win64\NvOFFRUC.dll"
$cudaInclude = Join-Path $CudaRoot "include"
$cudaLibrary = Join-Path $CudaRoot "lib\x64"

foreach ($required in @($source, $sdkInclude, $frucDll, $cudaInclude, (Join-Path $cudaLibrary "cuda.lib"))) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "Required path not found: $required"
    }
}

$vswhere = "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path -LiteralPath $vswhere)) {
    throw "Visual Studio Installer's vswhere.exe was not found."
}
$visualStudio = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $visualStudio) {
    throw "Visual Studio C++ build tools were not found."
}
$vcvars = Join-Path $visualStudio "VC\Auxiliary\Build\vcvars64.bat"
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null

$dll = Join-Path $outputDir "nvof_fruc_bridge.dll"
$object = Join-Path $outputDir "nvof_fruc_bridge.obj"
$importLibrary = Join-Path $outputDir "nvof_fruc_bridge.lib"
$programDatabase = Join-Path $outputDir "nvof_fruc_bridge.pdb"
$compilerCommand = @(
    'cl.exe /nologo /utf-8 /std:c++17 /O2 /EHsc /LD',
    ('/I"{0}"' -f $sdkInclude),
    ('/I"{0}"' -f $cudaInclude),
    ('/Fo:"{0}"' -f $object),
    ('"{0}"' -f $source),
    '/link',
    ('/LIBPATH:"{0}"' -f $cudaLibrary),
    'cuda.lib',
    ('/IMPLIB:"{0}"' -f $importLibrary),
    ('/PDB:"{0}"' -f $programDatabase),
    ('/OUT:"{0}"' -f $dll)
) -join ' '

$commandLine = 'call "{0}" && {1}' -f $vcvars, $compilerCommand
& $env:ComSpec /d /c $commandLine
if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $dll)) {
    throw "Native bridge build failed with exit code $LASTEXITCODE"
}

Write-Output "Built: $dll"
Write-Output "SDK runtime remains external: $frucDll"
