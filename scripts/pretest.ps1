# 현장 셋업 — conda 환경을 켜고 preflight를 돌린다.
#
#   .\scripts\pretest.ps1
#   .\scripts\pretest.ps1 -VtdHost 192.168.50.11 -Lidar
#
# 판단 로직은 전부 src/preflight.py 에 있다. 이 파일은 얇게 유지한다.

param(
    [string]$VtdHost = '192.168.50.11',
    [int]$Port = 9910,
    [string]$EnvName = 'hlfma',
    [switch]$Lidar
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

Write-Host ''
Write-Host '=== HLFMA 현장 셋업 ===' -ForegroundColor Cyan
Write-Host ("작업 디렉터리: {0}" -f $root)

# --- conda 환경 -------------------------------------------------------
if ($env:CONDA_DEFAULT_ENV -eq $EnvName) {
    Write-Host ("conda 환경: {0} (이미 활성)" -f $EnvName) -ForegroundColor Green
}
else {
    $conda = (Get-Command conda -ErrorAction SilentlyContinue)
    if ($null -eq $conda) {
        Write-Host 'conda를 찾을 수 없다. Anaconda Prompt에서 실행하거나' -ForegroundColor Yellow
        Write-Host '  conda activate hlfma  를 먼저 하고 다시 실행할 것' -ForegroundColor Yellow
    }
    else {
        Write-Host ("conda activate {0}" -f $EnvName)
        conda activate $EnvName
        if (-not $?) {
            Write-Host ("환경 {0} 활성화 실패" -f $EnvName) -ForegroundColor Red
            exit 1
        }
    }
}

# --- 네트워크 요약 (실패해도 진행한다. 판정은 preflight가 한다) -------
Write-Host ''
Write-Host '--- 이 PC의 IPv4 주소 ---'
Get-NetIPAddress -AddressFamily IPv4 |
    Where-Object { $_.IPAddress -notlike '127.*' } |
    Select-Object InterfaceAlias, IPAddress |
    Format-Table -AutoSize

# --- preflight --------------------------------------------------------
$args = @('src/preflight.py', '--host', $VtdHost, '--port', $Port)
if ($Lidar) { $args += '--lidar' }

Write-Host ("실행: python {0}" -f ($args -join ' '))
Write-Host ''
python @args
$code = $LASTEXITCODE

if ($code -ne 0) {
    Write-Host ''
    Write-Host 'preflight 실패. 위 항목을 해결하고 다시 실행할 것.' -ForegroundColor Red
    Write-Host 'VTD PC 실행 순서: 랜선 -> nmcli con up hlfma -> lmgrd -> vtdStart.sh' -ForegroundColor Yellow
    Write-Host 'Setup은 반드시 00_HL_VTD (창 3개: mainRS / mainRS_Broadcast / mainRS_LiDAR)' -ForegroundColor Yellow
}

exit $code
