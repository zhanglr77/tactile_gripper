param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$OutputRoot = '',
    [string[]]$CaseId = @(),
    [double]$FrictionCoefficient = -1.0,
    [switch]$NoMagnetCavities
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = (Resolve-Path -LiteralPath $ProjectRoot).Path
if (-not $OutputRoot) {
    $OutputRoot = Join-Path $ProjectRoot 'abaqus_output\analytic_rigid_cavities_mu05_cpus4'
}
elseif (-not [System.IO.Path]::IsPathRooted($OutputRoot)) {
    $OutputRoot = Join-Path $ProjectRoot $OutputRoot
}
$OutputRoot = [System.IO.Path]::GetFullPath($OutputRoot)
$config = Get-Content -LiteralPath (Join-Path $ProjectRoot 'config\indenter_cases.json') `
    -Raw -Encoding UTF8 | ConvertFrom-Json
$configuredCases = @($config.cases)
if ($CaseId.Count -gt 0) {
    $unknown = @($CaseId | Where-Object { $_ -notin $configuredCases.case_id })
    if ($unknown.Count -gt 0) { throw "Unknown case_id(s): $($unknown -join ', ')" }
    $configuredCases = @($configuredCases | Where-Object { $CaseId -contains $_.case_id })
    if ($configuredCases.Count -ne $CaseId.Count) { throw 'Duplicate case_id values were provided.' }
}
$shared = $config.shared_loading
$caseFriction = [double]$shared.friction_coefficient
if ($FrictionCoefficient -ge 0.0) { $caseFriction = $FrictionCoefficient }
if ($caseFriction -lt 0.0) { throw 'FrictionCoefficient must be nonnegative.' }
$checkpoints = @($shared.compression_checkpoints_mm)
$depth = [double]$shared.compression_depth_mm
if ($checkpoints.Count -eq 0 -or [math]::Abs([double]$checkpoints[-1] - $depth) -gt 1e-8) {
    throw 'Compression checkpoints must end at compression_depth_mm.'
}
$previous = 0.0
foreach ($checkpoint in $checkpoints) {
    if ([double]$checkpoint -le $previous) { throw 'Checkpoints must strictly increase from zero.' }
    $previous = [double]$checkpoint
}
$seen = @{}
foreach ($case in $configuredCases) {
    if ($case.case_id -notmatch '^[A-Za-z0-9_-]+$' -or $seen.ContainsKey($case.case_id)) {
        throw "Invalid or duplicate case_id: $($case.case_id)"
    }
    $seen[$case.case_id] = $true
    $caseDir = Join-Path $OutputRoot $case.case_id
    if (Test-Path -LiteralPath $caseDir) {
        throw "Output already exists; choose a new OutputRoot: $caseDir"
    }
}
New-Item -ItemType Directory -Path $OutputRoot -Force | Out-Null
$utf8 = New-Object System.Text.UTF8Encoding($false)
foreach ($case in $configuredCases) {
    $caseDir = Join-Path $OutputRoot $case.case_id
    New-Item -ItemType Directory -Path $caseDir | Out-Null
    $caseConfig = [ordered]@{
        case_id = $case.case_id
        indenter = $case.indenter
        surface_position = $case.surface_position
        initial_gap_mm = $shared.initial_gap_mm
        friction = $caseFriction
        depth_mm = $depth
        compression_checkpoints_mm = $checkpoints
        compression_only = $true
        shear_mm = 0.0
        shear_branches = @($shared.shear_cases | Where-Object { $_.case_id -eq $case.case_id })
        indenter_representation = 'analytical_rigid_surface'
        include_magnet_cavities = (-not $NoMagnetCavities.IsPresent)
        geometry_builder_ready = $true
    }
    [System.IO.File]::WriteAllText((Join-Path $caseDir 'case.json'),
        ($caseConfig | ConvertTo-Json -Depth 12), $utf8)
    Write-Host "Prepared compression configuration: $($case.case_id)"
}
Write-Host "Prepared $($configuredCases.Count) analytical-rigid compression configuration(s); cavities enabled: $(-not $NoMagnetCavities.IsPresent). Run build_case.py from each case directory to generate CAE/INP; no jobs submitted."
