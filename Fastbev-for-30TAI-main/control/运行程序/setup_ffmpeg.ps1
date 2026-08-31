$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$runtime = Join-Path $root 'runtime\ffmpeg'
$zip = Join-Path $runtime 'ffmpeg-release-essentials.zip'
$unpack = Join-Path $runtime '_unpack'
$ffmpeg = Join-Path $runtime 'bin\ffmpeg.exe'

New-Item -ItemType Directory -Force -Path $runtime | Out-Null
if (Test-Path $ffmpeg) {
    & $ffmpeg -version | Select-Object -First 1
    Write-Host "FFmpeg is already ready: $ffmpeg"
    exit 0
}
if (Test-Path $unpack) { Remove-Item -Recurse -Force $unpack }
Write-Host 'Downloading the FFmpeg Windows essentials build to this project folder...'
Invoke-WebRequest -Uri 'https://www.gyan.dev/ffmpeg/builds/ffmpeg-release-essentials.zip' -OutFile $zip
Expand-Archive -LiteralPath $zip -DestinationPath $unpack -Force
$extracted = Get-ChildItem -LiteralPath $unpack -Directory | Select-Object -First 1
if (-not $extracted) { throw 'The FFmpeg archive did not contain the expected directory.' }
Get-ChildItem -LiteralPath $extracted.FullName | Move-Item -Destination $runtime -Force
Remove-Item -Recurse -Force $unpack
Remove-Item -Force $zip
if (-not (Test-Path $ffmpeg)) { throw "FFmpeg is missing after extraction: $ffmpeg" }
& $ffmpeg -version | Select-Object -First 1
Write-Host "Ready: $ffmpeg"
