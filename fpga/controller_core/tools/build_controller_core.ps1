<#
.SYNOPSIS
Build and verify the custom-PCB FT Explorer controller-core bitstream.

.EXAMPLE
Set-ExecutionPolicy -Scope Process Bypass
& '.\fpga\controller_core\tools\build_controller_core.ps1'

.DESCRIPTION
Generates deterministic numeric vectors, simulates the fixed-point controller,
requires an exact RTL/reference match, synthesizes and places/routes for the
iCE40HX1K-TQ144 at 12 MHz, packs a programmer image, and writes a concise build
report under out/fpga_controller_core.

This builds the controller core behind a synchronous scan verification wrapper.
It does not implement sensor acquisition, estimation, PWM, ESC output, or an
FPGA programming/upload operation.
#>

$ErrorActionPreference = 'Stop'

$toolDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
$controllerDirectory = Split-Path -Parent $toolDirectory
$workbenchDirectory = (Resolve-Path (Join-Path $controllerDirectory '..\..')).Path
$outputDirectory = Join-Path $workbenchDirectory 'out\fpga_controller_core'
$ossCadDirectory = 'C:\oss-cad-suite'

$python = (Get-Command python -ErrorAction Stop).Source
$iverilog = Join-Path $ossCadDirectory 'bin\iverilog.exe'
$vvp = Join-Path $ossCadDirectory 'bin\vvp.exe'
$yosys = Join-Path $ossCadDirectory 'bin\yosys.exe'
$nextpnr = Join-Path $ossCadDirectory 'bin\nextpnr-ice40.exe'
$icepack = Join-Path $ossCadDirectory 'bin\icepack.exe'

foreach ($requiredTool in @($iverilog, $vvp, $yosys, $nextpnr, $icepack)) {
    if (-not (Test-Path -LiteralPath $requiredTool)) {
        throw "Required tool not found: $requiredTool"
    }
}

New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
$env:PATH = "$(Join-Path $ossCadDirectory 'bin');$(Join-Path $ossCadDirectory 'lib');$env:PATH"

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)][string]$Executable,
        [Parameter(Mandatory = $true)][string[]]$Arguments
    )

    & $Executable @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code $LASTEXITCODE`: $Executable"
    }
}

$relativeOutput = 'out/fpga_controller_core'
$vectors = Join-Path $outputDirectory 'vectors.txt'
$expected = Join-Path $outputDirectory 'expected.txt'
$actual = Join-Path $outputDirectory 'actual.txt'
$numericReport = Join-Path $outputDirectory 'NUMERIC_VALIDATION.txt'
$simulationLog = Join-Path $outputDirectory 'simulation.log'
$testbench = Join-Path $outputDirectory 'controller_tb.vvp'
$wrapperTestbench = Join-Path $outputDirectory 'controller_wrapper_tb.vvp'
$yosysLog = Join-Path $outputDirectory 'yosys.log'
$nextpnrLog = Join-Path $outputDirectory 'nextpnr.log'
$json = Join-Path $outputDirectory 'controller_core.json'
$asc = Join-Path $outputDirectory 'controller_core.asc'
$bitstream = Join-Path $outputDirectory 'controller_core.bin'
$buildReport = Join-Path $outputDirectory 'BUILD_REPORT.txt'

Push-Location $workbenchDirectory
try {
    Invoke-Checked $python @(
        'fpga/controller_core/tools/verify_controller_core.py',
        'generate', $vectors, $expected, $numericReport
    )

    Invoke-Checked $iverilog @(
        '-g2012', '-Wall', '-s', 'ftx_controller_core_tb', '-o', $testbench,
        'fpga/controller_core/rtl/ftx_serial_multiplier.v',
        'fpga/controller_core/rtl/ftx_controller_core.v',
        'fpga/controller_core/tb/ftx_controller_core_tb.v'
    )

    & $vvp $testbench "+VECTORS=$vectors" "+ACTUAL=$actual" 2>&1 |
        Tee-Object -FilePath $simulationLog
    if ($LASTEXITCODE -ne 0) {
        throw "RTL simulation failed with exit code $LASTEXITCODE"
    }
    Invoke-Checked $python @(
        'fpga/controller_core/tools/verify_controller_core.py',
        'check', $expected, $actual
    )

    Invoke-Checked $iverilog @(
        '-g2012', '-Wall', '-s', 'ftx_controller_board_top_tb',
        '-o', $wrapperTestbench,
        'fpga/controller_core/rtl/ftx_serial_multiplier.v',
        'fpga/controller_core/rtl/ftx_controller_core.v',
        'fpga/controller_core/rtl/ftx_controller_board_top.v',
        'fpga/controller_core/tb/ftx_controller_board_top_tb.v'
    )
    & $vvp $wrapperTestbench 2>&1 | Tee-Object -FilePath $simulationLog -Append
    if ($LASTEXITCODE -ne 0) {
        throw "Scan-wrapper simulation failed with exit code $LASTEXITCODE"
    }

    $yosysCommand = 'read_verilog ' +
        'fpga/controller_core/rtl/ftx_serial_multiplier.v ' +
        'fpga/controller_core/rtl/ftx_controller_core.v ' +
        'fpga/controller_core/rtl/ftx_controller_board_top.v; ' +
        "synth_ice40 -top ftx_controller_board_top -json $relativeOutput/controller_core.json"
    Invoke-Checked $yosys @('-q', '-l', $yosysLog, '-p', $yosysCommand)

    Invoke-Checked $nextpnr @(
        '--hx1k', '--package', 'tq144', '--json', $json,
        '--pcf', 'fpga/controller_core/constraints/custom_pcb_tq144.pcf',
        '--asc', $asc, '--freq', '12', '--log', $nextpnrLog, '--quiet'
    )
    Invoke-Checked $icepack @($asc, $bitstream)

    $numericLines = Get-Content -LiteralPath $numericReport
    $simulationLines = Get-Content -LiteralPath $simulationLog
    $utilizationLines = Select-String -LiteralPath $nextpnrLog -Pattern 'ICESTORM_LC:|ICESTORM_RAM:' |
        ForEach-Object { $_.Line.Trim() }
    $timingLine = Select-String -LiteralPath $nextpnrLog -Pattern 'Max frequency for clock' |
        Select-Object -Last 1 -ExpandProperty Line
    $bitstreamFile = Get-Item -LiteralPath $bitstream
    $bitstreamHash = Get-FileHash -LiteralPath $bitstream -Algorithm SHA256

    $reportLines = @(
        'FTX CONTROLLER CORE BUILD',
        "generated=$((Get-Date).ToString('s'))",
        'target=iCE40HX1K-TQ144',
        'board_clock_mhz=12',
        'boundary=estimated state and commands in; aileron/elevator/rudder commands out',
        'excluded=sensor acquisition, estimator, PWM, ESC, physical upload',
        '',
        'NUMERIC VALIDATION'
    ) + $numericLines + @(
        '',
        'RTL SIMULATION'
    ) + $simulationLines + @(
        'RTL_FIXED_POINT_MATCH=PASS',
        '',
        'PLACE AND ROUTE'
    ) + $utilizationLines + @(
        $timingLine.Trim(),
        '',
        'BITSTREAM',
        "file=$bitstream",
        "bytes=$($bitstreamFile.Length)",
        "sha256=$($bitstreamHash.Hash)",
        '',
        'PHYSICAL_UPLOAD=NOT_RUN'
    )
    Set-Content -LiteralPath $buildReport -Value $reportLines -Encoding ascii

    Write-Host "PASS: controller-core bitstream built at $bitstream"
    Write-Host "Report: $buildReport"
} finally {
    Pop-Location
}
