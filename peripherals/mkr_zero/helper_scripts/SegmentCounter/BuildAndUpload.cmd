@echo off
setlocal

rem SegmentCounter - counts 0..255 on an HT16K33 backpack daisy-chained to the IMU.
rem
rem   BuildAndUpload.cmd            compile only
rem   BuildAndUpload.cmd COM5       compile and upload to COM5
rem
rem The display shares the IMU's I2C bus, so a stepping counter means the bus is
rem alive and a frozen one means it has wedged - visible from across the room,
rem with no console attached. That is the whole reason this sketch exists.
rem
rem Only vendor\Wire is needed: the HT16K33 is driven directly, with no display
rem library, and nothing from lib\ is compiled in. The Wire flag is NOT optional
rem - stock TwoWire::requestFrom() reads an uninitialised busOwner on 1-byte
rem transfers, and the sketch fails the build on a missing marker rather than
rem silently linking the buggy copy. See vendor\Wire\README.md.
rem
rem NOTE: keep this file plain ASCII - cmd.exe parses it in the OEM codepage.

where arduino-cli >nul 2>&1
if errorlevel 1 (
    echo arduino-cli is not available on PATH.
    exit /b 1
)

for %%I in ("%~dp0.") do set "SKETCH_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\..\..\vendor\Wire") do set "WIRE_DIR=%%~fI"

set "FQBN=arduino:samd:mkrzero"
set "CORE_REQUIRED=1.8.14"
set "WARN=--warnings all"

set "CORE_FOUND="
for /f "tokens=2" %%V in ('arduino-cli core list ^| findstr /b /c:"arduino:samd"') do set "CORE_FOUND=%%V"
if not "%CORE_FOUND%"=="%CORE_REQUIRED%" (
    echo.
    echo ERROR: arduino:samd %CORE_REQUIRED% is required, found "%CORE_FOUND%".
    exit /b 1
)

if "%~1"=="" (
    arduino-cli compile %WARN% --fqbn "%FQBN%" --library "%WIRE_DIR%" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% --upload --port "%~1" --fqbn "%FQBN%" --library "%WIRE_DIR%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
