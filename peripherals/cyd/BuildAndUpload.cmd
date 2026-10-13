@echo off
setlocal

rem CYD dashboard - compile / upload with arduino-cli.
rem   BuildAndUpload.cmd          compile only
rem   BuildAndUpload.cmd COM7     compile and upload to COM7

where arduino-cli >nul 2>&1
if errorlevel 1 (
    echo arduino-cli is not available on PATH.
    exit /b 1
)

for %%I in ("%~dp0.") do set "SKETCH_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\lib\cydGraphics") do set "GRAPHICS_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\lib\cydStorage") do set "STORAGE_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\lib\cydAudio") do set "AUDIO_DIR=%%~fI"

rem The CYD uses a classic ESP32 and a CH340 serial adapter, not ESP32-C3 USB CDC.
set "FQBN=esp32:esp32:esp32"
set "WARN=--warnings all"

if "%~1"=="" (
    arduino-cli compile %WARN% --fqbn "%FQBN%" --library "%STORAGE_DIR%" --library "%GRAPHICS_DIR%" --library "%AUDIO_DIR%" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% --upload --port "%~1" --fqbn "%FQBN%" --library "%STORAGE_DIR%" --library "%GRAPHICS_DIR%" --library "%AUDIO_DIR%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
