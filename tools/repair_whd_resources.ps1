param([switch]$Program)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$backup = Join-Path $repo 'original-whd-layout-backup.bin'
$output = Join-Path $repo 'original-whd-resources-relocated.bin'
$bytes = [IO.File]::ReadAllBytes($backup)
if ($bytes.Length -ne 0xC0000) { throw '无线资源备份长度不匹配，停止。' }
$backupHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $backup).Hash
if ($backupHash -ne '4DA38FD4B5D9C704199C14EF3FCD39D8CAFE1F9A8F22E4FEB8B97EE00E3F8DA1') {
    throw '备份不是本次已检查的设备资源快照，停止。'
}

# 板端旧资源位于 0x60E00000，而当前 M55 FAL 从 0x60DC0000 读取。
# 只迁移三个无线资源分区；完整保留原始备份，不碰应用、蓝牙和用户配置。
$parts = @(
    @{ Name='whd_firmware'; Offset=0; Capacity=0x60000 },
    @{ Name='whd_clm'; Offset=0x60000; Capacity=0x10000 },
    @{ Name='whd_nvram'; Offset=0x70000; Capacity=0x10000 }
)
foreach ($part in $parts) {
    $position = 0x40000 + $part.Offset
    $location = [BitConverter]::ToUInt32($bytes, $position)
    $size = [BitConverter]::ToUInt32($bytes, $position + 4)
    if ($location -ne 2 -or $size -eq 0 -or ($size + 16) -gt $part.Capacity) {
        throw "资源头无效：$($part.Name)，停止。"
    }
    Write-Output "$($part.Name): 有效资源头，负载 $size 字节"
}
$image = New-Object byte[] 0x80000
[Array]::Copy($bytes, 0x40000, $image, 0, $image.Length)
[IO.File]::WriteAllBytes($output, $image)
Get-FileHash -Algorithm SHA256 -LiteralPath $backup, $output | Format-List
if (-not $Program) {
    Write-Output '迁移镜像已准备；尚未烧录。电机动力断电后才能使用 -Program。'
    exit 0
}

# 写入范围固定为 0x60DC0000..0x60E3FFFF，按擦除边界对齐。
# 同时停止双核以避免外部 Flash 擦写期间另一核心继续执行 XIP。
$openocd = Join-Path $repo '.restore-tools/openocd/openocd'
$generated = Join-Path $repo 'firmware/m33/libs/TARGET_APP_KIT_PSE84_EVAL_EPC2/config/GeneratedSource'
$imagePath = $output.Replace('\', '/')
Push-Location $repo
try {
    & "$openocd/bin/openocd.exe" -s "$openocd/scripts" -s $generated `
        -f interface/kitprog3.cfg -c 'transport select swd' `
        -c 'set ENABLE_CM55 1; set DEVICE PSE84xGxS2' `
        -f target/infineon/pse84.cfg `
        -c "init; reset init; targets cat1d.cm55; halt; targets cat1d.cm33; halt; flash write_image erase {$imagePath} 0x60dc0000 bin; flash verify_image {$imagePath} 0x60dc0000 bin; reset run; shutdown"
    if ($LASTEXITCODE -ne 0) { throw "无线资源写入或校验失败：$LASTEXITCODE" }
} finally {
    Pop-Location
}
