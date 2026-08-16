@echo off
setlocal

rem BridgeCoalesceVerify - regression tests for High-G delivery across the hop.
rem
rem   BuildAndUpload.cmd            compile only
rem   BuildAndUpload.cmd COM7       compile and upload to COM7
rem
rem Then open a serial monitor at 115200. Every test reports PASS, FAIL or SKIP
rem and the sketch prints a summary.
rem
rem NO WIRING AND NO SECOND BOARD. The sketch puts Serial1 into internal loopback
rem and transmits master frames to itself, so the real decoder runs on the real
rem frames. If the chip refuses loopback the sketch says so and asks for a jumper
rem between GPIO21 and GPIO20, which gives the identical path.
rem
rem NOTE: keep this file plain ASCII - cmd.exe parses it in the OEM codepage.

where arduino-cli >nul 2>&1
if errorlevel 1 (
    echo arduino-cli is not available on PATH.
    exit /b 1
)

for %%I in ("%~dp0.") do set "SKETCH_DIR=%%~fI"
rem SKETCH_DIR is peripherals/esp32-c3/tests/BridgeCoalesceVerify, so the shared
rem library lives two levels up (..\..\lib\commLink), not one.
for %%I in ("%SKETCH_DIR%\..\..\lib\commLink") do set "LIB_DIR=%%~fI"

rem Matches the CommReceiver test and the production bridge:
rem  - CDCOnBoot=cdc      : Serial over native USB, frees UART0 GPIO20/21 for Serial1
rem  - PartitionScheme    : huge_app (3MB app, no OTA/FS bloat)
rem  - DebugLevel=none    : no core debug logging injected into the report
set "FQBN=esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=huge_app,CPUFreq=160,FlashMode=qio,FlashFreq=80,FlashSize=4M,DebugLevel=none,UploadSpeed=921600"

if "%~1"=="" (
    arduino-cli compile --fqbn "%FQBN%" --library "%LIB_DIR%" "%SKETCH_DIR%"
) else (
    arduino-cli compile --upload --port "%~1" --fqbn "%FQBN%" --library "%LIB_DIR%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
