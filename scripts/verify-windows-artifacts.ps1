[CmdletBinding()]
param(
  [Parameter(Mandatory)]
  [string]$BuildDirectory,

  [Parameter(Mandatory)]
  [string]$Dumpbin,

  [string]$InstallDirectory,

  [string]$ConsumerExecutable
)

$ErrorActionPreference = 'Stop'

function Assert-File {
  param([Parameter(Mandatory)][string]$Path)
  if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
    throw "Required artifact is missing: $Path"
  }
}

function Assert-DynamicDependency {
  param(
    [Parameter(Mandatory)][string]$Executable,
    [Parameter(Mandatory)][string]$Dependency
  )
  $dependents = & $Dumpbin /nologo /dependents $Executable
  $dependentText = $dependents -join "`n"
  if ($LASTEXITCODE -ne 0 -or $dependentText -notmatch [regex]::Escape($Dependency)) {
    throw "'$Executable' does not dynamically depend on '$Dependency'."
  }
}

$buildBin = Join-Path $BuildDirectory 'bin'
$buildLib = Join-Path $BuildDirectory 'lib'
$buildDll = Join-Path $buildBin 'pointcloud_ad.dll'
$buildImportLibrary = Join-Path $buildLib 'pointcloud_ad.lib'
$buildCli = Join-Path $buildBin 'pcad.exe'
$buildUnitTests = Join-Path $buildBin 'pointcloud_ad_unit_tests.exe'

Assert-File $buildDll
Assert-File $buildImportLibrary
Assert-File $buildCli
Assert-File $buildUnitTests

$exports = & $Dumpbin /nologo /exports $buildDll
$exportText = $exports -join "`n"
if ($LASTEXITCODE -ne 0 -or $exportText -notmatch '\?version_string@pointcloud_ad@@') {
  throw "The public pointcloud_ad::version_string symbol is not exported by '$buildDll'."
}
if ($exportText -notmatch 'create@InspectionPipeline@pointcloud_ad@@' -or
    $exportText -notmatch 'run@InspectionPipeline@pointcloud_ad@@') {
  throw "The public InspectionPipeline create/run symbols are not exported by '$buildDll'."
}
if ($exportText -match '@backends@|@comparison@|@detection@|@preprocess@|@registration@|@pcl_backend@') {
  throw "Internal backend or module symbols must not be exported by '$buildDll'."
}

Assert-DynamicDependency $buildCli 'pointcloud_ad.dll'
Assert-DynamicDependency $buildUnitTests 'pointcloud_ad.dll'

if ($InstallDirectory) {
  $installDll = Join-Path $InstallDirectory 'bin\pointcloud_ad.dll'
  $installCli = Join-Path $InstallDirectory 'bin\pcad.exe'
  $installImportLibrary = Join-Path $InstallDirectory 'lib\pointcloud_ad.lib'
  $installHeaders = @(
    (Join-Path $InstallDirectory 'include\pointcloud_ad\config.hpp'),
    (Join-Path $InstallDirectory 'include\pointcloud_ad\geometry.hpp'),
    (Join-Path $InstallDirectory 'include\pointcloud_ad\inspection_pipeline.hpp'),
    (Join-Path $InstallDirectory 'include\pointcloud_ad\inspection_result.hpp'),
    (Join-Path $InstallDirectory 'include\pointcloud_ad\normalization.hpp'),
    (Join-Path $InstallDirectory 'include\pointcloud_ad\registration.hpp'),
    (Join-Path $InstallDirectory 'include\pointcloud_ad\result.hpp'),
    (Join-Path $InstallDirectory 'include\pointcloud_ad\status.hpp'),
    (Join-Path $InstallDirectory 'include\pointcloud_ad\surface.hpp'),
    (Join-Path $InstallDirectory 'include\pointcloud_ad\version.hpp')
  )
  $installExportHeader = Join-Path $InstallDirectory 'include\pointcloud_ad\export.hpp'
  $installConfig = Join-Path $InstallDirectory 'lib\cmake\PointCloudAD\PointCloudADConfig.cmake'

  $installArtifacts = @(
      $installDll,
      $installCli,
      $installImportLibrary
    ) + $installHeaders + @(
      $installExportHeader,
      $installConfig
    )
  foreach ($artifact in $installArtifacts) {
    Assert-File $artifact
  }

  if (Test-Path -LiteralPath (Join-Path $InstallDirectory 'lib\pointcloud_ad.dll')) {
    throw 'The installed runtime DLL must be in bin, not lib.'
  }
  Assert-DynamicDependency $installCli 'pointcloud_ad.dll'

  if (-not $ConsumerExecutable) {
    throw 'ConsumerExecutable is required when InstallDirectory is provided.'
  }

  Assert-File $ConsumerExecutable
  $consumerDll = Join-Path (Split-Path -Parent $ConsumerExecutable) 'pointcloud_ad.dll'
  Assert-File $consumerDll
  if ((Get-FileHash -LiteralPath $consumerDll -Algorithm SHA256).Hash -ne
      (Get-FileHash -LiteralPath $installDll -Algorithm SHA256).Hash) {
    throw 'The consumer-local DLL does not match the installed PointCloudAD runtime.'
  }
  Assert-DynamicDependency $ConsumerExecutable 'pointcloud_ad.dll'
}
