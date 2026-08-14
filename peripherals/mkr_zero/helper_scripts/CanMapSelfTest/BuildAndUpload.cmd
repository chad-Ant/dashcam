@echo off
setlocal

rem CanMapSelfTest - bench proof of the CAN map parser and bit extractor.
rem
rem   BuildAndUpload.cmd            compile only
rem   BuildAndUpload.cmd COM5       compile and upload to COM5
rem
rem No SD card, no CAN hardware and no vehicle are needed to run it. It exists
rem because the generic decoder REPLACES code known to work on a real car, so
rem the bar is "reproduces the old numbers", not "runs".
rem
rem The --library flags match the production build because lib\CANMap.cpp is
rem compiled here and pulls in SDFunctions.h (for canMapLoad) and
rem VehicleSignals.h. See the production BuildAndUpload.cmd for what each is for.
rem
rem NOTE: keep this file plain ASCII - cmd.exe parses it in the OEM codepage.

where arduino-cli >nul 2>&1
if errorlevel 1 (
    echo arduino-cli is not available on PATH.
    exit /b 1
)

for %%I in ("%~dp0.") do set "SKETCH_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\..\..\lib") do set "LIB_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\..\..\vendor\Wire") do set "WIRE_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\..\..\vendor\CANBus") do set "CAN_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\..\..\vendor\SdFat") do set "SDFAT_DIR=%%~fI"
for %%I in ("%SKETCH_DIR%\..\..\vendor\BNO055") do set "BNO_DIR=%%~fI"

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
    arduino-cli compile %WARN% --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%BNO_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
) else (
    arduino-cli compile %WARN% --upload --port "%~1" --fqbn "%FQBN%" --library "%WIRE_DIR%" --library "%CAN_DIR%" --library "%SDFAT_DIR%" --library "%BNO_DIR%" --library "%LIB_DIR%" "%SKETCH_DIR%"
)

exit /b %ERRORLEVEL%
