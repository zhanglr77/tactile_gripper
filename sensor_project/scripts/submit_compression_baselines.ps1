param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$RunRoot = ''
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = (Resolve-Path -LiteralPath $ProjectRoot).Path
if (-not $RunRoot) {
    $RunRoot = Join-Path $ProjectRoot 'abaqus_output\analytic_rigid_cavities_mu05_cpus4'
}
elseif (-not [System.IO.Path]::IsPathRooted($RunRoot)) {
    $RunRoot = Join-Path $ProjectRoot $RunRoot
}
$RunRoot = (Resolve-Path -LiteralPath $RunRoot).Path
$cases = Get-ChildItem -LiteralPath $RunRoot -Directory | Sort-Object Name
if ($cases.Count -eq 0) { throw 'No prepared cases found.' }

$jobs = foreach ($caseDir in $cases) {
    $id = $caseDir.Name
    $job = "Job-$id"
    $inp = Join-Path $caseDir.FullName "$job.inp"
    if (-not (Test-Path -LiteralPath $inp)) { throw "Missing input: $inp" }
    $deck = Get-Content -LiteralPath $inp -Raw
    if ($deck -notmatch '(?im)^\*Surface,\s*type=(REVOLUTION|CYLINDER)' -or
        $deck -notmatch '(?im)^\*Rigid Body,.*analytical surface=') {
        throw "$id does not contain an analytical rigid indenter surface"
    }
    $compressionSteps = [regex]::Matches($deck, '(?im)^\*Step,\s*name=Step-compression-\d+').Count
    $restartWrites = [regex]::Matches($deck, '(?im)^\*Restart,\s*write,\s*frequency=0').Count
    if ($compressionSteps -ne 6 -or $restartWrites -lt 6) {
        throw "$id must have 6 compression steps and 6 restart writes"
    }
    if ($deck -match '(?im)^\*Step,\s*name=Step-shear') {
        throw "$id compression baseline unexpectedly contains a shear step"
    }
    if (Test-Path -LiteralPath (Join-Path $caseDir.FullName "$job.lck")) {
        throw "$id already has an Abaqus lock file"
    }
    [pscustomobject]@{ Id=$id; Dir=$caseDir.FullName; Job=$job; Input=$inp }
}

$launched = @()
foreach ($item in $jobs) {
    $staPath = Join-Path $item.Dir "$($item.Job).sta"
    if ((Test-Path -LiteralPath $staPath) -and
        (Select-String -LiteralPath $staPath -Pattern 'ANALYSIS HAS COMPLETED SUCCESSFULLY' -Quiet)) {
        Write-Host "Already completed: $($item.Id)"
        continue
    }
    $command = "abaqus job=$($item.Job) cpus=4 memory=10% ask_delete=OFF interactive > run.log 2>&1"
    $launcher = Start-Process -WindowStyle Hidden -PassThru `
        -WorkingDirectory $item.Dir `
        -FilePath (Join-Path $env:SystemRoot 'System32\cmd.exe') `
        -ArgumentList '/c', $command `
        -RedirectStandardError (Join-Path $item.Dir 'run.err')
    [pscustomobject]@{
        case_id = $item.Id
        job = $item.Job
        launcher_pid = $launcher.Id
        submitted_at = (Get-Date).ToString('s')
        cpus = 4
        memory = '10%'
    } | ConvertTo-Json | Set-Content `
        -LiteralPath (Join-Path $item.Dir 'submission.json') -Encoding UTF8
    $launched += $item.Id
    Write-Host "Submitted: $($item.Id), launcher PID $($launcher.Id)"
}

Write-Host "Submitted $($launched.Count) analytical-rigid compression jobs. No shear jobs were submitted."
