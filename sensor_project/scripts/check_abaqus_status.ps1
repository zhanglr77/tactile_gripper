param(
    [string]$Root = (Join-Path (Split-Path -Parent $PSScriptRoot) 'abaqus_output')
)

$ErrorActionPreference = 'Stop'
if (-not [System.IO.Path]::IsPathRooted($Root)) {
    $Root = Join-Path (Split-Path -Parent $PSScriptRoot) $Root
}
$rootPath = (Resolve-Path -LiteralPath $Root).Path
$extensions = @('.sta', '.odb', '.lck')
$jobPaths = @{}

Get-ChildItem -LiteralPath $rootPath -Recurse -File -ErrorAction SilentlyContinue |
    Where-Object { $extensions -contains $_.Extension.ToLowerInvariant() } |
    ForEach-Object {
        $basePath = $_.FullName.Substring(0, $_.FullName.Length - $_.Extension.Length)
        if (-not $jobPaths.ContainsKey($basePath)) { $jobPaths[$basePath] = @{} }
        $jobPaths[$basePath][$_.Extension.ToLowerInvariant()] = $_.FullName
    }

if ($jobPaths.Count -eq 0) {
    Write-Host "No Abaqus jobs found under $rootPath"
    exit 0
}

$results = foreach ($basePath in ($jobPaths.Keys | Sort-Object)) {
    $files = $jobPaths[$basePath]
    $status = 'NO STATUS FILE'
    $stageText = '-'
    $incrementText = '-'
    $progressText = '-'
    $odbText = '-'
    $warningCount = '-'
    $errorCount = '-'
    $staText = ''

    if ($files.ContainsKey('.sta')) {
        $staText = Get-Content -LiteralPath $files['.sta'] -Raw
    }
    $msgText = ''
    if ($files.ContainsKey('.msg')) {
        $msgText = Get-Content -LiteralPath $files['.msg'] -Raw
        if ($msgText -match '(?im)^\s*(\d+)\s+WARNING MESSAGES DURING ANALYSIS') {
            $warningCount = $Matches[1]
        }
        if ($msgText -match '(?im)^\s*(\d+)\s+ERROR MESSAGES') {
            $errorCount = $Matches[1]
        }
    }

    if ($staText -match '(?i)ANALYSIS HAS COMPLETED SUCCESSFULLY' -or
        $msgText -match '(?i)ANALYSIS HAS COMPLETED SUCCESSFULLY') {
        $status = 'COMPLETED'
    } elseif ($staText -match '(?i)ANALYSIS HAS NOT BEEN COMPLETED|ANALYSIS TERMINATED' -or
              $msgText -match '(?i)ANALYSIS HAS NOT BEEN COMPLETED|ANALYSIS TERMINATED|ANALYSIS EXITED WITH AN ERROR') {
        $status = 'FAILED / TERMINATED'
    } elseif ($files.ContainsKey('.lck')) {
        $status = 'RUNNING'
    } elseif ($files.ContainsKey('.sta')) {
        $status = 'STOPPED / INCOMPLETE'
    }

    if ($files.ContainsKey('.sta')) {
        $lastProgress = Get-Content -LiteralPath $files['.sta'] |
            Where-Object { $_ -match '^\s*\d+\s+\d+\s+\d+\s+' } |
            Select-Object -Last 1
        if ($lastProgress) {
            $fields = $lastProgress.Trim() -split '\s+'
            $stepNumber = [int]$fields[0]
            $stepName = "Step $stepNumber"
            $inpPath = $basePath + '.inp'
            if (Test-Path -LiteralPath $inpPath) {
                $stepNames = [regex]::Matches(
                    (Get-Content -LiteralPath $inpPath -Raw),
                    '(?im)^\*Step\s*,\s*name\s*=\s*([^,\r\n]+)'
                )
                if ($stepNumber -le $stepNames.Count) {
                    $stepName = $stepNames[$stepNumber - 1].Groups[1].Value.Trim()
                }
            }
            $stepTime = [double]$fields[7]
            $stageText = $stepName
            $incrementText = $fields[1]
            $progressText = '{0:N1}%' -f ([Math]::Min(100, [Math]::Max(0, $stepTime * 100)))
        }
    }

    if ($files.ContainsKey('.odb')) {
        $odb = Get-Item -LiteralPath $files['.odb']
        $odbText = '{0:N1} MB, {1:HH:mm:ss}' -f ($odb.Length / 1MB), $odb.LastWriteTime
    }

    [pscustomobject]@{
        Job = $basePath.Substring($rootPath.Length).TrimStart('\')
        Status = $status
        Stage = $stageText
        Increment = $incrementText
        StepProgress = $progressText
        ODB = $odbText
        Warnings = $warningCount
        Errors = $errorCount
    }
}

Write-Host "Abaqus jobs under $rootPath"
$results | Format-Table -AutoSize
