param(
    [string]$VivadoBat = 'C:\vivado2018.3\vivado\Vivado\2018.3\bin\vivado.bat',
    [string]$StageDir = 'D:\vivado2018.3\fpga_sandox\fastbev_part2_ascii'
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$RtlSource = Join-Path $ProjectRoot 'Vivado_Project\rtl'
$NetlistSource = Join-Path $ProjectRoot 'Vivado_Project\netlist'
$ConstrSource = Join-Path $ProjectRoot 'Vivado_Project\constraints'
$StageRtl = Join-Path $StageDir 'rtl'
$StageConstr = Join-Path $StageDir 'constrs'

if (-not (Test-Path -LiteralPath $VivadoBat -PathType Leaf)) {
    throw "Vivado batch launcher not found: $VivadoBat"
}

New-Item -ItemType Directory -Force -Path $StageDir | Out-Null
New-Item -ItemType Directory -Force -Path $StageRtl | Out-Null
New-Item -ItemType Directory -Force -Path $StageConstr | Out-Null

Get-ChildItem -LiteralPath $RtlSource -Filter '*.v' -File |
    Copy-Item -Destination $StageRtl -Force
Copy-Item -LiteralPath (Join-Path $NetlistSource 'ps_ai_wrap_demo.edf') `
    -Destination $StageRtl -Force
Copy-Item -LiteralPath (Join-Path $ConstrSource 'impl_constraints.xdc') `
    -Destination $StageConstr -Force
Copy-Item -LiteralPath (Join-Path $ConstrSource 'ai7030.xdc') `
    -Destination $StageConstr -Force
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'check_full_ascii.tcl') `
    -Destination (Join-Path $StageDir 'check_full_ascii.tcl') -Force

Push-Location $StageDir
try {
    & $VivadoBat -mode batch -notrace -source check_full_ascii.tcl `
        -log vivado.log -journal vivado.jou
    if ($LASTEXITCODE -ne 0) {
        throw "Vivado implementation failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

Write-Host "Vivado reports: $(Join-Path $StageDir 'out')"
