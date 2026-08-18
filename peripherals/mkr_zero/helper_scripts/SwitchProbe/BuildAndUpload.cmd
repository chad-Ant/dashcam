@echo off
setlocal

rem SwitchProbe - bring-up for the 2x 74HC165 switch chain on D0/D1/D2.
rem
rem   BuildAndUpload.cmd            compile only
rem   BuildAndUpload.cmd COM5       compile and upload to COM5
rem
rem NO --library FLAGS AT ALL, and that is deliberate. This sketch drives the
rem three pins directly and links nothing from lib\ - the thing being
rem commissioned is the WIRING, and a probe sharing its transport with the
rem driver under test cannot tell a bad harness from a bad driver. It also
rem touches neither I2C nor SPI, so vendor\Wire and vendor\CANBus are genuinely
rem not needed here rather than merely omitted.
rem
rem NOTE: keep this file plain ASCII - cmd.exe parses it in the OEM codepage.

where arduino-cli >nul 2>&1
if errorlevel 1 (
    echo arduino-cli is not available on PATH.
    exit /b 1
)

for %%I in ("%~dp0.") do set "SKETCH_DIR=%%~fI"

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
    arduino-cli compile %WARN% --fqbn "%FQBN%" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% --upload --port "%~1" --fqbn "%FQBN%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
