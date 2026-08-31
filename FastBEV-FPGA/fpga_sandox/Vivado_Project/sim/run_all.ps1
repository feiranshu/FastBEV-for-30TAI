param(
    [Parameter(Mandatory = $false)]
    [string]$DataRoot = $env:FASTBEV_DATA_ROOT,
    [string]$IverilogRoot = '',
    [switch]$SkipFullFrame
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$RtlDir = Join-Path $ProjectRoot 'Vivado_Project\rtl'
$Build = Join-Path $PSScriptRoot 'build'

if ([string]::IsNullOrWhiteSpace($DataRoot)) {
    throw 'Specify -DataRoot or set FASTBEV_DATA_ROOT. The directory must contain part1, part2_lut and part3.'
}
$DataRoot = (Resolve-Path -LiteralPath $DataRoot).Path
foreach ($name in @('part1', 'part2_lut', 'part3')) {
    if (-not (Test-Path -LiteralPath (Join-Path $DataRoot $name) -PathType Container)) {
        throw "DataRoot is missing required directory: $name"
    }
}

if (-not [string]::IsNullOrWhiteSpace($IverilogRoot)) {
    $Iverilog = Join-Path $IverilogRoot 'bin\iverilog.exe'
    $Vvp = Join-Path $IverilogRoot 'bin\vvp.exe'
} else {
    $IverilogCmd = Get-Command 'iverilog' -ErrorAction SilentlyContinue
    $VvpCmd = Get-Command 'vvp' -ErrorAction SilentlyContinue
    if ($IverilogCmd -and $VvpCmd) {
        $Iverilog = $IverilogCmd.Source
        $Vvp = $VvpCmd.Source
    } else {
        $bundled = Join-Path $DataRoot 'part2_fpga\tools\iverilog'
        $Iverilog = Join-Path $bundled 'bin\iverilog.exe'
        $Vvp = Join-Path $bundled 'bin\vvp.exe'
    }
}
if (-not (Test-Path -LiteralPath $Iverilog -PathType Leaf) -or
    -not (Test-Path -LiteralPath $Vvp -PathType Leaf)) {
    throw 'Icarus Verilog was not found. Install it, add it to PATH, or pass -IverilogRoot.'
}

$env:FASTBEV_DATA_ROOT = $DataRoot
$env:FASTBEV_SIM_BUILD = $Build
New-Item -ItemType Directory -Force -Path $Build | Out-Null

$TopSources = @(
    (Join-Path $RtlDir 'bev_accel_top.v'),
    (Join-Path $RtlDir 'bev_reg_ctrl.v'),
    (Join-Path $RtlDir 'dma_arbiter.v'),
    (Join-Path $RtlDir 'pulse_cross.v'),
    (Join-Path $RtlDir 'lut_engine_fp32.v'),
    (Join-Path $RtlDir 'sa_engine_fp32.v')
)

Push-Location $ProjectRoot
try {
    python (Join-Path $PSScriptRoot 'verify_network_interfaces.py')
    if ($LASTEXITCODE -ne 0) { throw 'Network-interface verification failed.' }
    python (Join-Path $PSScriptRoot 'verify_lut_for_rtl.py')
    if ($LASTEXITCODE -ne 0) { throw 'RTL LUT verification failed.' }
    Push-Location (Join-Path $DataRoot 'part2_lut')
    try {
        python 'verify_fastbev_v2_lut.py'
        if ($LASTEXITCODE -ne 0) { throw 'Strict LUT/tensor verification failed.' }
    } finally {
        Pop-Location
    }

    & $Iverilog -g2012 -s tb_fp32_tf32_fp16 `
        -o (Join-Path $Build 'tb_fp32_tf32_fp16.vvp') `
        (Join-Path $RtlDir 'lut_engine_fp32.v') `
        (Join-Path $PSScriptRoot 'tb_fp32_tf32_fp16.v')
    if ($LASTEXITCODE -ne 0) { throw 'FP conversion test compile failed.' }
    & $Vvp (Join-Path $Build 'tb_fp32_tf32_fp16.vvp')
    if ($LASTEXITCODE -ne 0) { throw 'FP conversion test failed.' }

    & $Iverilog -g2012 -s tb_fp32_add `
        -o (Join-Path $Build 'tb_fp32_add.vvp') `
        (Join-Path $RtlDir 'sa_engine_fp32.v') `
        (Join-Path $PSScriptRoot 'tb_fp32_add.v')
    if ($LASTEXITCODE -ne 0) { throw 'FP32 adder test compile failed.' }
    & $Vvp (Join-Path $Build 'tb_fp32_add.vvp')
    if ($LASTEXITCODE -ne 0) { throw 'FP32 adder test failed.' }

    & $Iverilog -g2012 -s tb_lut_engine_nhwc_fp16 `
        -o (Join-Path $Build 'tb_lut_engine_nhwc_fp16.vvp') `
        (Join-Path $RtlDir 'lut_engine_fp32.v') `
        (Join-Path $PSScriptRoot 'tb_lut_engine_nhwc_fp16.v')
    if ($LASTEXITCODE -ne 0) { throw 'LUT engine test compile failed.' }
    & $Vvp (Join-Path $Build 'tb_lut_engine_nhwc_fp16.vvp')
    if ($LASTEXITCODE -ne 0) { throw 'LUT engine test failed.' }

    & $Iverilog -g2012 -s tb_bev_accel_top_cdc `
        -o (Join-Path $Build 'tb_bev_accel_top_cdc.vvp') `
        @TopSources (Join-Path $PSScriptRoot 'tb_bev_accel_top_cdc.v')
    if ($LASTEXITCODE -ne 0) { throw 'Top CDC test compile failed.' }
    & $Vvp (Join-Path $Build 'tb_bev_accel_top_cdc.vvp')
    if ($LASTEXITCODE -ne 0) { throw 'Top CDC test failed.' }

    & $Iverilog -g2012 -Wall -Wimplicit -s bev_accel_top -t null @TopSources
    if ($LASTEXITCODE -ne 0) { throw 'bev_accel_top elaboration failed.' }
    & $Iverilog -g2012 -Wall -Wimplicit -Wno-timescale -s bev_edif_top -t null `
        (Join-Path $RtlDir 'bev_edif_top.v') `
        (Join-Path $RtlDir 'ps_ai_wrap_demo.v') @TopSources
    if ($LASTEXITCODE -ne 0) { throw 'bev_edif_top elaboration failed.' }

    if (-not $SkipFullFrame) {
        python (Join-Path $PSScriptRoot 'prepare_lut_hex.py')
        if ($LASTEXITCODE -ne 0) { throw 'LUT hex preparation failed.' }
        & $Iverilog -g2012 -s tb_lut_engine_full_frame `
            -o (Join-Path $Build 'tb_lut_engine_full_frame.vvp') `
            (Join-Path $RtlDir 'lut_engine_fp32.v') `
            (Join-Path $PSScriptRoot 'tb_lut_engine_full_frame.v')
        if ($LASTEXITCODE -ne 0) { throw 'Full-frame test compile failed.' }
        & $Vvp (Join-Path $Build 'tb_lut_engine_full_frame.vvp')
        if ($LASTEXITCODE -ne 0) { throw 'Full-frame test failed.' }
    }

    Write-Host 'ALL_RTL_CHECKS_PASS'
} finally {
    Pop-Location
}
