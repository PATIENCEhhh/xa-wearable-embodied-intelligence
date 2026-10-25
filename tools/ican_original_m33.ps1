param(
    [switch]$Build,
    [switch]$Package,
    [switch]$Program,
    [int]$Jobs = 4
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$firmware = Join-Path $repo 'firmware/m33'
$localPython = Join-Path $repo '.restore-tools/Scripts/python.exe'
$packer = Join-Path $repo '.restore-tools/Scripts/edgeprotecttools.exe'
$openocdHome = Join-Path $repo '.restore-tools/openocd/openocd'
$image = Join-Path $firmware 'build/rtthread.hex'
$xip = Join-Path $firmware 'build/rtthread_xip_verify.hex'
$manifest = Join-Path $firmware 'build/ican-original-manifest.json'

function Invoke-Checked([string]$Executable, [string[]]$Arguments) {
    & $Executable @Arguments
    if ($LASTEXITCODE -ne 0) { throw "工具执行失败，退出码：$LASTEXITCODE" }
}
function Require-File([string]$File) {
    if (!(Test-Path -LiteralPath $File -PathType Leaf)) { throw "缺少文件：$File" }
}

# 默认仅报告准备状态；只有显式Program参数才会访问并改写目标板。
if (!$Build -and !$Package -and !$Program) {
    foreach ($file in @($localPython, $packer, (Join-Path $openocdHome 'bin/openocd.exe'), $manifest)) {
        Write-Output ('{0}: {1}' -f (Test-Path -LiteralPath $file), $file)
    }
    Write-Output '准备：-Build -Package；由现场人员烧录：-Program。默认不访问硬件。'
    return
}
if ($Program -and ($Build -or $Package)) { throw '烧录必须单独执行；先检查构建与镜像记录。' }

$oldPythonPath = $env:PYTHONPATH
$oldCompilerPath = $env:RTT_EXEC_PATH
Push-Location $firmware
try {
    if ($Build) {
        # 复用本机已有SCons依赖；构建原SConstruct，不引用SDK源或桥接转换。
        $sconsRoot = Join-Path (Split-Path $repo -Parent) 'A_ROBOT_ARM-submit/build/tools-python'
        Require-File (Join-Path $sconsRoot 'SCons/__init__.py')
        $env:PYTHONPATH = $sconsRoot
        $env:RTT_EXEC_PATH = Split-Path (Get-Command arm-none-eabi-gcc -ErrorAction Stop).Source -Parent
        Invoke-Checked 'python' @('-X', 'utf8', '-B', '-m', 'SCons', "-j$Jobs", 'rt-thread.elf')
    }
    if ($Package) {
        Require-File $packer
        Require-File 'rtthread.hex'
        Require-File 'tools/edgeprotecttools/cm33_s_signed_fw/proj_cm33_s_signed.hex'
        # 每次从未重定位的NS输入重新生成；不对已组合镜像重复合并。
        Invoke-Checked $packer @('run-config', '-i', 'config/boot_with_extended_boot_scons.json')
        Require-File $image
        Invoke-Checked $packer @('hex-relocate', '--input', $image, '--region', '0x60000000', '0x01000000', '0x08000000', '--region', '0x70000000', '0x01000000', '0x18000000', '--output', $xip)
        $record = [ordered]@{
            created = (Get-Date -Format o)
            purpose = 'iCAN原工程M33恢复；不接SDK；尚未实物验收'
            elf_sha256 = (Get-FileHash 'rt-thread.elf').Hash
            ns_sha256 = (Get-FileHash 'rtthread.hex').Hash
            secure_sha256 = (Get-FileHash 'tools/edgeprotecttools/cm33_s_signed_fw/proj_cm33_s_signed.hex').Hash
            image_sha256 = (Get-FileHash $image).Hash
            xip_sha256 = (Get-FileHash $xip).Hash
        }
        # 此文件是本次构建的生成证据，不是源码配置。
        $record | ConvertTo-Json | Set-Content -LiteralPath $manifest -Encoding utf8
        Write-Output "组合镜像：$image"
        Write-Output "生成记录：$manifest"
    }
    if ($Program) {
        Require-File $manifest
        $record = Get-Content -LiteralPath $manifest -Raw | ConvertFrom-Json
        if ((Get-FileHash $image).Hash -ne $record.image_sha256 -or
            (Get-FileHash $xip).Hash -ne $record.xip_sha256) { throw '镜像哈希不符，重新打包核对。' }
        $openocd = Join-Path $openocdHome 'bin/openocd.exe'
        $loader = Join-Path $openocdHome 'flm/infineon/pse8x6/PSE84_SMIF.FLM'
        $flow = Join-Path $firmware 'tools/openocd/pse84_m33_verified_flash.tcl'
        $qspiConfig = Join-Path $firmware 'libs/TARGET_APP_KIT_PSE84_EVAL_EPC2/config/GeneratedSource/qspi_config.cfg'
        foreach ($file in @($openocd, $loader, $flow, $qspiConfig)) { Require-File $file }
        Write-Output '本操作将擦写M33镜像区域并启动原固件；现场应关闭电机动力电源。'
        # 沿用原有raw及XIP校验流程；不执行整片擦除、不改安全生命周期。
        Invoke-Checked $openocd @('-s', (Join-Path $openocdHome 'scripts'),
            '-s', (Split-Path $qspiConfig -Parent),
            '-s', (Split-Path $loader -Parent), '-f', 'interface/kitprog3.cfg',
            '-c', 'transport select swd', '-c', "set QSPI_FLASHLOADER {$($loader.Replace('\','/'))}",
            '-c', 'set ENABLE_CM55 0; set DEVICE PSE84xGxS2', '-f', 'target/infineon/pse84.cfg',
            '-c', "set M33_IMAGE {$($image.Replace('\','/'))}",
            '-c', "set M33_XIP_IMAGE {$($xip.Replace('\','/'))}",
            '-c', 'set M33_STANDALONE 1', '-f', $flow)
    }
} finally {
    Pop-Location
    $env:PYTHONPATH = $oldPythonPath
    $env:RTT_EXEC_PATH = $oldCompilerPath
}
