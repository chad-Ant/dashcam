@echo off
setlocal

rem CANDiscovery - passive CAN census, standalone helper sketch.
rem
rem   BuildAndUpload.cmd            compile only
rem   BuildAndUpload.cmd COM5       compile and upload to COM5
rem
rem No --library flag: this sketch is self-contained and deliberately does NOT
rem pull in peripherals/mkr_zero/lib. It must stay independent of the production
rem firmware so it can never disturb the working telemetry chain.
rem
rem NOTE: keep this file plain ASCII - cmd.exe parses it in the OEM codepage.

where arduino-cli >nul 2>&1
if errorlevel 1 (
    echo arduino-cli is not available on PATH.
    exit /b 1
)

for %%I in ("%~dp0.") do set "SKETCH_DIR=%%~fI"

set "FQBN=arduino:samd:mkrzero"
set "WARN=--warnings all"

if "%~1"=="" (
    arduino-cli compile %WARN% --fqbn "%FQBN%" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% --upload --port "%~1" --fqbn "%FQBN%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
