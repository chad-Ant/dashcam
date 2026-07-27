@echo off
setlocal

where arduino-cli >nul 2>&1
if errorlevel 1 (
    echo arduino-cli is not available on PATH.
    exit /b 1
)

for %%I in ("%~dp0.") do set "SKETCH_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\..\lib\commLink") do set "LIB_DIR=%%~fI"

rem Optimized ESP32-C3 config:
rem  - CDCOnBoot=cdc      : Serial over native USB, frees UART0 GPIO20/21 for Serial1
rem  - PartitionScheme    : huge_app (3MB app, no OTA/FS bloat for a single resident sketch)
rem  - 160MHz / QIO / 80MHz flash : max performance defaults
rem  - DebugLevel=none    : no core debug logging overhead
set "FQBN=esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=huge_app,CPUFreq=160,FlashMode=qio,FlashFreq=80,FlashSize=4M,DebugLevel=none,UploadSpeed=921600"

if "%~1"=="" (
    arduino-cli compile --fqbn "%FQBN%" --library "%LIB_DIR%" "%SKETCH_DIR%"
) else (
    arduino-cli compile --upload --port "%~1" --fqbn "%FQBN%" --library "%LIB_DIR%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
