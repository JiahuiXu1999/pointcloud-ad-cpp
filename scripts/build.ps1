[CmdletBinding()]
param(
  [ValidateSet('windows-msvc-debug', 'windows-msvc-release')]
  [string]$Preset = 'windows-msvc-debug',

  [switch]$Fresh,

  [switch]$VerifyInstall,

  [switch]$RunBenchmark,

  [switch]$VerifyPackage
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot

function Find-VisualStudioInstallation {
  $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
  if (Test-Path -LiteralPath $vswhere) {
    $result = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if ($result) {
      return $result.Trim()
    }
  }

  $visualStudioRoot = Join-Path $env:ProgramFiles 'Microsoft Visual Studio'
  if (Test-Path -LiteralPath $visualStudioRoot) {
    $candidate = Get-ChildItem -LiteralPath $visualStudioRoot -Directory |
      Sort-Object Name -Descending |
      ForEach-Object {
        Get-ChildItem -LiteralPath $_.FullName -Directory -ErrorAction SilentlyContinue
      } |
      Where-Object {
        Test-Path -LiteralPath (Join-Path $_.FullName 'VC\Auxiliary\Build\vcvars64.bat')
      } |
      Select-Object -First 1
    if ($candidate) {
      return $candidate.FullName
    }
  }

  throw 'A Visual Studio installation with the Desktop development with C++ workload was not found.'
}

$vsInstallation = Find-VisualStudioInstallation
$vcvars = Join-Path $vsInstallation 'VC\Auxiliary\Build\vcvars64.bat'
$bundledCmake = Join-Path $vsInstallation 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
$bundledNinjaDirectory = Join-Path $vsInstallation 'Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja'
$bundledNinja = Join-Path $bundledNinjaDirectory 'ninja.exe'

$cmakeCommand = Get-Command cmake -ErrorAction SilentlyContinue
if ($cmakeCommand) {
  $cmake = $cmakeCommand.Source
} elseif (Test-Path -LiteralPath $bundledCmake) {
  $cmake = $bundledCmake
} else {
  throw 'CMake 3.25 or newer was not found.'
}

if (-not $env:VCPKG_ROOT) {
  $bundledVcpkg = Join-Path $vsInstallation 'VC\vcpkg'
  if (Test-Path -LiteralPath (Join-Path $bundledVcpkg 'vcpkg.exe')) {
    $env:VCPKG_ROOT = $bundledVcpkg
  }
}

# vcvars64 may repoint VCPKG_ROOT at the Visual Studio bundled vcpkg; remember the caller's
# explicit value so it is restored after the developer environment is imported below.
$userVcpkgRoot = $env:VCPKG_ROOT

$environmentLines = & cmd.exe /d /s /c "call `"$vcvars`" >nul && set"
if ($LASTEXITCODE -ne 0) {
  throw 'Failed to initialize the Visual Studio x64 developer environment.'
}
foreach ($line in $environmentLines) {
  $separator = $line.IndexOf('=')
  if ($separator -gt 0) {
    $name = $line.Substring(0, $separator)
    $value = $line.Substring($separator + 1)
    [Environment]::SetEnvironmentVariable($name, $value, 'Process')
  }
}

$env:VCPKG_ROOT = $userVcpkgRoot
$env:VCPKG_VISUAL_STUDIO_PATH = $vsInstallation
$env:POINTCLOUDAD_NINJA = $bundledNinja
$env:VSLANG = '1033'
$vcpkgRuntime = Join-Path $projectRoot 'vcpkg_installed\x64-windows\bin'
$env:Path = "$vcpkgRuntime;$bundledNinjaDirectory;$([IO.Path]::GetDirectoryName($cmake));$env:Path"

$dumpbinCommand = Get-Command dumpbin.exe -ErrorAction SilentlyContinue
if (-not $dumpbinCommand) {
  throw 'dumpbin.exe was not found after initializing the Visual Studio environment.'
}
$dumpbin = $dumpbinCommand.Source

function Reset-GeneratedDirectory {
  param([Parameter(Mandatory)][string]$Path)

  $generatedRoot = [IO.Path]::GetFullPath((Join-Path $projectRoot 'out'))
  $resolvedPath = [IO.Path]::GetFullPath($Path)
  $requiredPrefix = $generatedRoot.TrimEnd([IO.Path]::DirectorySeparatorChar) +
    [IO.Path]::DirectorySeparatorChar
  if (-not $resolvedPath.StartsWith($requiredPrefix, [StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to reset a directory outside '$generatedRoot': $resolvedPath"
  }

  if (Test-Path -LiteralPath $resolvedPath) {
    Remove-Item -LiteralPath $resolvedPath -Recurse -Force
  }
}

Push-Location $projectRoot
try {
  if ($Fresh) {
    & $cmake --fresh --preset $Preset
    if ($LASTEXITCODE -ne 0) {
      throw "Configure preset '$Preset' failed."
    }
    & $cmake --build --preset $Preset
    if ($LASTEXITCODE -ne 0) {
      throw "Build preset '$Preset' failed."
    }
    & (Join-Path ([IO.Path]::GetDirectoryName($cmake)) 'ctest.exe') --preset $Preset
    if ($LASTEXITCODE -ne 0) {
      throw "Test preset '$Preset' failed."
    }
  } else {
    & $cmake --workflow --preset $Preset
    if ($LASTEXITCODE -ne 0) {
      throw "Build workflow '$Preset' failed."
    }
  }

  $buildDirectory = Join-Path $projectRoot "out\build\$Preset"
  & (Join-Path $PSScriptRoot 'verify-windows-artifacts.ps1') `
    -BuildDirectory $buildDirectory `
    -Dumpbin $dumpbin

  if ($RunBenchmark) {
    if ($Preset -ne 'windows-msvc-release') {
      throw '-RunBenchmark requires the windows-msvc-release preset.'
    }
    & $cmake --build --preset $Preset --target pointcloud_ad_staged_benchmark
    if ($LASTEXITCODE -ne 0) {
      throw 'Staged benchmark build failed.'
    }
    & (Join-Path $buildDirectory 'bin\pointcloud_ad_staged_benchmark.exe')
    if ($LASTEXITCODE -ne 0) {
      throw 'Staged benchmark limits failed.'
    }
  }

  if ($VerifyInstall) {
    if ($Preset -ne 'windows-msvc-release') {
      throw '-VerifyInstall requires the windows-msvc-release preset.'
    }

    $installDirectory = Join-Path $projectRoot "out\install\$Preset"
    $consumerDirectory = Join-Path $projectRoot "out\consumer\$Preset"

    Reset-GeneratedDirectory $installDirectory
    Reset-GeneratedDirectory $consumerDirectory

    & $cmake --install $buildDirectory --prefix $installDirectory
    if ($LASTEXITCODE -ne 0) {
      throw 'Project installation failed.'
    }

    & $cmake -S tests/consumer -B $consumerDirectory -G Ninja `
      -DCMAKE_BUILD_TYPE=Release `
      "-DCMAKE_PREFIX_PATH=$installDirectory"
    if ($LASTEXITCODE -ne 0) {
      throw 'Consumer project configuration failed.'
    }

    & $cmake --build $consumerDirectory
    if ($LASTEXITCODE -ne 0) {
      throw 'Consumer project build failed.'
    }

    $consumerExecutable = Join-Path $consumerDirectory 'pointcloud_ad_consumer.exe'
    & (Join-Path $PSScriptRoot 'verify-windows-artifacts.ps1') `
      -BuildDirectory $buildDirectory `
      -Dumpbin $dumpbin `
      -InstallDirectory $installDirectory `
      -ConsumerExecutable $consumerExecutable

    & (Join-Path $installDirectory 'bin\pcad.exe') --version
    if ($LASTEXITCODE -ne 0) {
      throw 'Installed CLI smoke test failed.'
    }

    & $consumerExecutable
    if ($LASTEXITCODE -ne 0) {
      throw 'Consumer smoke test failed.'
    }
  }

  if ($VerifyPackage) {
    if ($Preset -ne 'windows-msvc-release') {
      throw '-VerifyPackage requires the windows-msvc-release preset.'
    }

    $cpack = Join-Path ([IO.Path]::GetDirectoryName($cmake)) 'cpack.exe'
    $packageDirectory = Join-Path $projectRoot "out\package\$Preset"
    $extractDirectory = Join-Path $projectRoot "out\package-extract\$Preset"
    $packageConsumerDirectory = Join-Path $projectRoot "out\package-consumer\$Preset"
    Reset-GeneratedDirectory $packageDirectory
    Reset-GeneratedDirectory $extractDirectory
    Reset-GeneratedDirectory $packageConsumerDirectory
    New-Item -ItemType Directory -Path $packageDirectory, $extractDirectory | Out-Null

    & $cpack -G ZIP -C Release --config (Join-Path $buildDirectory 'CPackConfig.cmake') `
      -B $packageDirectory
    if ($LASTEXITCODE -ne 0) {
      throw 'CPack SDK archive generation failed.'
    }

    $archives = @(Get-ChildItem -LiteralPath $packageDirectory -Filter '*.zip' -File)
    if ($archives.Count -ne 1) {
      throw "Expected exactly one SDK ZIP archive, found $($archives.Count)."
    }
    Expand-Archive -LiteralPath $archives[0].FullName -DestinationPath $extractDirectory
    $sdkRoots = @(Get-ChildItem -LiteralPath $extractDirectory -Directory)
    if ($sdkRoots.Count -ne 1) {
      throw "Expected exactly one SDK root directory, found $($sdkRoots.Count)."
    }
    $sdkRoot = $sdkRoots[0].FullName
    $sdkBin = Join-Path $sdkRoot 'bin'
    $requiredPackageFiles = @(
      (Join-Path $sdkBin 'pointcloud_ad.dll'),
      (Join-Path $sdkBin 'pcad.exe'),
      (Join-Path $sdkRoot 'lib\pointcloud_ad.lib'),
      (Join-Path $sdkRoot 'include\pointcloud_ad\inspection_pipeline.hpp'),
      (Join-Path $sdkRoot 'lib\cmake\PointCloudAD\PointCloudADConfig.cmake'),
      (Join-Path $sdkRoot 'examples\sdk_consumer\CMakeLists.txt'),
      (Join-Path $sdkRoot 'LICENSE'),
      (Join-Path $sdkRoot 'THIRD_PARTY_NOTICES.md')
    )
    foreach ($requiredPackageFile in $requiredPackageFiles) {
      if (-not (Test-Path -LiteralPath $requiredPackageFile -PathType Leaf)) {
        throw "SDK package file is missing: $requiredPackageFile"
      }
    }

    $installedCmakeFiles = Get-ChildItem -LiteralPath (Join-Path $sdkRoot 'lib\cmake') `
      -Filter '*.cmake' -File -Recurse
    $pathLeakPattern = [regex]::Escape($projectRoot) + '|vcpkg_installed'
    if ($installedCmakeFiles | Select-String -Pattern $pathLeakPattern) {
      throw 'Installed CMake package contains a build-tree or vcpkg path leak.'
    }

    & $cmake -S (Join-Path $sdkRoot 'examples\sdk_consumer') `
      -B $packageConsumerDirectory -G Ninja `
      -DCMAKE_BUILD_TYPE=Release `
      "-DCMAKE_PREFIX_PATH=$sdkRoot"
    if ($LASTEXITCODE -ne 0) {
      throw 'Packaged SDK example configuration failed.'
    }
    & $cmake --build $packageConsumerDirectory
    if ($LASTEXITCODE -ne 0) {
      throw 'Packaged SDK example build failed.'
    }

    $originalPath = $env:Path
    try {
      $env:Path = "$sdkBin;$env:SystemRoot\System32;$env:SystemRoot"
      & (Join-Path $sdkBin 'pcad.exe') --version
      if ($LASTEXITCODE -ne 0) {
        throw 'Packaged CLI failed with only the SDK and Windows system paths available.'
      }
      & (Join-Path $packageConsumerDirectory 'pointcloud_ad_sdk_consumer.exe')
      if ($LASTEXITCODE -ne 0) {
        throw 'Packaged SDK example failed with only the SDK and Windows system paths available.'
      }
    } finally {
      $env:Path = $originalPath
    }

    $archiveHash = (Get-FileHash -LiteralPath $archives[0].FullName -Algorithm SHA256).Hash
    $checksumPath = "$($archives[0].FullName).sha256"
    "$archiveHash  $($archives[0].Name)" | Set-Content -LiteralPath $checksumPath -Encoding ascii
    Write-Host "Verified SDK package: $($archives[0].FullName)"
    Write-Host "SHA256: $archiveHash"
  }
} finally {
  Pop-Location
}
